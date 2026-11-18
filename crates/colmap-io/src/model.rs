//! Text and binary model I/O (cameras, images, points3D).

use crate::error::{IoError, Result};
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use colmap_core::camera::{Camera, CameraModelKind};
use colmap_core::geometry::Rigid3;
use colmap_core::ids::{CameraId, ImageId, Point2DIdx, Point3DId};
use colmap_core::reconstruction::{Image, Point2D, Point3D, Reconstruction};
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

const INVALID_POINT3D: u64 = u64::MAX;

// ---------- text ----------

pub fn read_cameras_text<R: BufRead>(r: R, rec: &mut Reconstruction) -> Result<()> {
    for line in r.lines() {
        let line = line.map_err(IoError::Io)?;
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        let mut parts = t.split_whitespace();
        let cam_id: u32 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing camera id".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let model_name = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing model".into()))?;
        let model = CameraModelKind::from_name(model_name)
            .ok_or_else(|| IoError::Invalid(format!("unknown model {}", model_name)))?;
        let width: u32 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing width".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let height: u32 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing height".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let mut params = Vec::new();
        for tok in parts {
            params.push(
                tok.parse::<f64>()
                    .map_err(|e| IoError::Invalid(format!("{}", e)))?,
            );
        }
        if params.len() != model.num_params() {
            return Err(IoError::Invalid(format!(
                "param count {} != {}",
                params.len(),
                model.num_params()
            )));
        }
        let cam = Camera::new(model, params, width, height)
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        rec.cameras.insert(CameraId::new(cam_id), cam);
    }
    Ok(())
}

pub fn write_cameras_text<W: Write>(w: &mut W, rec: &Reconstruction) -> Result<()> {
    writeln!(w, "# Camera list with one line of data per camera:").ok();
    writeln!(w, "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]").ok();
    for (id, cam) in &rec.cameras {
        write!(
            w,
            "{} {} {} {}",
            id.get(),
            cam.model.name(),
            cam.width,
            cam.height
        )
        .unwrap();
        for p in &cam.params {
            write!(w, " {:.17}", p).unwrap();
        }
        writeln!(w).unwrap();
    }
    Ok(())
}

pub fn read_images_text<R: BufRead>(r: R, rec: &mut Reconstruction) -> Result<()> {
    // First pass: need legacy handling? We assume legacy (one rig per camera not needed for our data model).
    // Format: line with ID qw qx qy qz tx ty tz camera_id name
    // then next line with 2D points: x y point3D_id ...
    let lines = r.lines();
    // We'll iterate manually
    let buf_lines: Vec<String> = lines
        .collect::<std::result::Result<_, _>>()
        .map_err(IoError::Io)?;
    let mut i = 0;
    while i < buf_lines.len() {
        let line = buf_lines[i].trim().to_string();
        i += 1;
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        // parse image header
        let mut parts = line.split_whitespace();
        let image_id: u32 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing image id".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let qw: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing qw".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let qx: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing qx".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let qy: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing qy".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let qz: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing qz".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let tx: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing tx".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let ty: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing ty".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let tz: f64 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing tz".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let camera_id: u32 = parts
            .next()
            .ok_or_else(|| IoError::Invalid("missing camera_id".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        // name is remainder of line
        let name: String = parts.collect::<Vec<_>>().join(" ");
        // trim leading ws already consumed; name may have leading? We collected rest tokens; join with space. COLMAP name may contain spaces (rare). Accept.
        let name = name.trim().to_string();

        // rotation quaternion (w,x,y,z) -> UnitQuaternion
        use nalgebra::{Quaternion, UnitQuaternion};
        let q = Quaternion::new(qw, qx, qy, qz);
        let rot = UnitQuaternion::from_quaternion(q);
        let pose = Rigid3 {
            rotation: rot,
            translation: nalgebra::Vector3::new(tx, ty, tz),
        };

        // read next non? Actually next line is points2D line (could be empty)
        // Ensure we read next line even if EOF
        let pts_line = if i < buf_lines.len() {
            buf_lines[i].clone()
        } else {
            String::new()
        };
        i += 1;
        let pts_line = pts_line.trim().to_string();

        let mut points2d: Vec<Point2D> = Vec::new();
        if !pts_line.is_empty() && !pts_line.starts_with('#') {
            let toks = pts_line.split_whitespace();
            // parse pairs: x y id
            // We'll collect tokens
            let toks_vec: Vec<&str> = toks.collect();
            // process in chunks of 3
            let mut k = 0;
            while k + 2 < toks_vec.len() || (k + 2 <= toks_vec.len() && k < toks_vec.len()) {
                if k + 2 >= toks_vec.len() {
                    break;
                }
                let x: f64 = toks_vec[k]
                    .parse()
                    .map_err(|e| IoError::Invalid(format!("{}", e)))?;
                let y: f64 = toks_vec[k + 1]
                    .parse()
                    .map_err(|e| IoError::Invalid(format!("{}", e)))?;
                let id_signed: i64 = toks_vec[k + 2]
                    .parse()
                    .map_err(|e| IoError::Invalid(format!("{}", e)))?;
                let pid = if id_signed == -1 {
                    None
                } else {
                    Some(Point3DId::new(id_signed as u64))
                };
                points2d.push(Point2D {
                    xy: [x, y],
                    point3d_id: pid,
                });
                k += 3;
            }
        }

        let img = Image {
            image_id: ImageId::new(image_id),
            name,
            camera_id: CameraId::new(camera_id),
            pose,
            points2d,
        };
        rec.images.insert(img.image_id, img);
    }
    Ok(())
}

pub fn write_images_text<W: Write>(w: &mut W, rec: &Reconstruction) -> Result<()> {
    writeln!(w, "# Image list with one line of data per image:").ok();
    writeln!(
        w,
        "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME"
    )
    .ok();
    writeln!(w, "#   POINTS2D[] as (X, Y, POINT3D_ID)").ok();
    for (id, img) in &rec.images {
        let q = img.pose.rotation.quaternion();
        write!(
            w,
            "{} {:.17} {:.17} {:.17} {:.17} {:.17} {:.17} {:.17} {} {}",
            id.get(),
            q.w,
            q.i,
            q.j,
            q.k,
            img.pose.translation.x,
            img.pose.translation.y,
            img.pose.translation.z,
            img.camera_id.get(),
            img.name
        )
        .unwrap();
        writeln!(w).unwrap();
        // points2D line
        let mut line = String::new();
        for p in &img.points2d {
            let pid = match p.point3d_id {
                Some(x) => x.get() as i64,
                None => -1,
            };
            line.push_str(&format!(" {:.17} {:.17} {}", p.xy[0], p.xy[1], pid));
        }
        if !line.is_empty() {
            writeln!(w, "{}", line.trim_start()).unwrap();
        } else {
            writeln!(w).unwrap();
        }
    }
    Ok(())
}

pub fn read_points3d_text<R: BufRead>(r: R, rec: &mut Reconstruction) -> Result<()> {
    for line in r.lines() {
        let line = line.map_err(IoError::Io)?;
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        let mut toks = t.split_whitespace();
        let pid: u64 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing id".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let x: f64 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing x".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let y: f64 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing y".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let z: f64 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing z".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let r: u8 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing r".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let g: u8 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing g".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let b: u8 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing b".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let err: f64 = toks
            .next()
            .ok_or_else(|| IoError::Invalid("missing err".into()))?
            .parse()
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        let mut track = Vec::new();
        // remaining pairs
        let rest: Vec<&str> = toks.collect();
        let mut k = 0;
        while k + 1 < rest.len() {
            let iid: u32 = rest[k]
                .parse()
                .map_err(|e| IoError::Invalid(format!("{}", e)))?;
            let idx: u32 = rest[k + 1]
                .parse()
                .map_err(|e| IoError::Invalid(format!("{}", e)))?;
            track.push((ImageId::new(iid), Point2DIdx::new(idx)));
            k += 2;
        }
        let pt = Point3D {
            xyz: nalgebra::Vector3::new(x, y, z),
            color: [r, g, b],
            error: err,
            track,
        };
        rec.points3d.insert(Point3DId::new(pid), pt);
    }
    Ok(())
}

pub fn write_points3d_text<W: Write>(w: &mut W, rec: &Reconstruction) -> Result<()> {
    writeln!(w, "# 3D point list with one line of data per point:").ok();
    writeln!(
        w,
        "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)"
    )
    .ok();
    for (id, pt) in &rec.points3d {
        write!(
            w,
            "{} {:.17} {:.17} {:.17} {} {} {} {:.17}",
            id.get(),
            pt.xyz.x,
            pt.xyz.y,
            pt.xyz.z,
            pt.color[0],
            pt.color[1],
            pt.color[2],
            pt.error
        )
        .unwrap();
        for (iid, idx) in &pt.track {
            write!(w, " {} {}", iid.get(), idx.get()).unwrap();
        }
        writeln!(w).unwrap();
    }
    Ok(())
}

// ---------- binary ----------

pub fn read_cameras_binary<R: Read>(r: &mut R, rec: &mut Reconstruction) -> Result<()> {
    let num = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
    for _ in 0..num {
        let cam_id = r.read_u32::<LittleEndian>().map_err(IoError::Io)?;
        let model_i = r.read_i32::<LittleEndian>().map_err(IoError::Io)?;
        let model =
            CameraModelKind::from_i32(model_i).ok_or(IoError::UnknownCameraModel(model_i))?;
        let width = r.read_u64::<LittleEndian>().map_err(IoError::Io)? as u32;
        let height = r.read_u64::<LittleEndian>().map_err(IoError::Io)? as u32;
        let nparams = model.num_params();
        let mut params = vec![0.0f64; nparams];
        for p in &mut params {
            *p = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        }
        let cam = Camera::new(model, params, width, height)
            .map_err(|e| IoError::Invalid(format!("{}", e)))?;
        rec.cameras.insert(CameraId::new(cam_id), cam);
    }
    Ok(())
}

// Hmm camera_id read above as u32 then cast to u64 variable named cam_id; okay.
// But we read camera_id as u32? We did read_u32. Good. But code uses cam_id as u64 from u32. Fine.

pub fn write_cameras_binary<W: Write>(w: &mut W, rec: &Reconstruction) -> Result<()> {
    w.write_u64::<LittleEndian>(rec.cameras.len() as u64)
        .map_err(IoError::Io)?;
    for (id, cam) in &rec.cameras {
        w.write_u32::<LittleEndian>(id.get()).map_err(IoError::Io)?;
        w.write_i32::<LittleEndian>(cam.model as i32)
            .map_err(IoError::Io)?;
        w.write_u64::<LittleEndian>(cam.width as u64)
            .map_err(IoError::Io)?;
        w.write_u64::<LittleEndian>(cam.height as u64)
            .map_err(IoError::Io)?;
        for p in &cam.params {
            w.write_f64::<LittleEndian>(*p).map_err(IoError::Io)?;
        }
    }
    Ok(())
}

pub fn read_images_binary<R: Read>(r: &mut R, rec: &mut Reconstruction) -> Result<()> {
    let num = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
    for _ in 0..num {
        let image_id = r.read_u32::<LittleEndian>().map_err(IoError::Io)?; // image_t uint32
        let qw = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let qx = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let qy = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let qz = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let tx = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let ty = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let tz = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let camera_id = r.read_u32::<LittleEndian>().map_err(IoError::Io)?;
        // name null-terminated
        let mut name = String::new();
        loop {
            let mut buf = [0u8; 1];
            r.read_exact(&mut buf).map_err(IoError::Io)?;
            if buf[0] == 0 {
                break;
            }
            name.push(buf[0] as char);
        }
        let num_pts = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
        let mut points2d = Vec::with_capacity(num_pts as usize);
        for _ in 0..num_pts {
            let x = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
            let y = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
            let pid_raw = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
            let pid = if pid_raw == INVALID_POINT3D {
                None
            } else {
                Some(Point3DId::new(pid_raw))
            };
            points2d.push(Point2D {
                xy: [x, y],
                point3d_id: pid,
            });
        }
        use nalgebra::{Quaternion, UnitQuaternion};
        let q = Quaternion::new(qw, qx, qy, qz);
        let rot = UnitQuaternion::from_quaternion(q);
        let pose = Rigid3 {
            rotation: rot,
            translation: nalgebra::Vector3::new(tx, ty, tz),
        };
        let img = Image {
            image_id: ImageId::new(image_id),
            name,
            camera_id: CameraId::new(camera_id),
            pose,
            points2d,
        };
        rec.images.insert(img.image_id, img);
    }
    Ok(())
}

pub fn write_images_binary<W: Write>(w: &mut W, rec: &Reconstruction) -> Result<()> {
    w.write_u64::<LittleEndian>(rec.images.len() as u64)
        .map_err(IoError::Io)?;
    for (id, img) in &rec.images {
        w.write_u32::<LittleEndian>(id.get()).map_err(IoError::Io)?;
        let q = img.pose.rotation.quaternion();
        w.write_f64::<LittleEndian>(q.w).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(q.i).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(q.j).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(q.k).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(img.pose.translation.x)
            .map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(img.pose.translation.y)
            .map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(img.pose.translation.z)
            .map_err(IoError::Io)?;
        w.write_u32::<LittleEndian>(img.camera_id.get())
            .map_err(IoError::Io)?;
        // name null-terminated
        w.write_all(img.name.as_bytes()).map_err(IoError::Io)?;
        w.write_all(&[0u8]).map_err(IoError::Io)?;
        w.write_u64::<LittleEndian>(img.points2d.len() as u64)
            .map_err(IoError::Io)?;
        for p in &img.points2d {
            w.write_f64::<LittleEndian>(p.xy[0]).map_err(IoError::Io)?;
            w.write_f64::<LittleEndian>(p.xy[1]).map_err(IoError::Io)?;
            let pid = match p.point3d_id {
                Some(x) => x.get(),
                None => INVALID_POINT3D,
            };
            w.write_u64::<LittleEndian>(pid).map_err(IoError::Io)?;
        }
    }
    Ok(())
}

pub fn read_points3d_binary<R: Read>(r: &mut R, rec: &mut Reconstruction) -> Result<()> {
    let num = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
    for _ in 0..num {
        let pid = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
        let x = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let y = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let z = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let mut color = [0u8; 3];
        r.read_exact(&mut color).map_err(IoError::Io)?;
        let err = r.read_f64::<LittleEndian>().map_err(IoError::Io)?;
        let track_len = r.read_u64::<LittleEndian>().map_err(IoError::Io)?;
        let mut track = Vec::with_capacity(track_len as usize);
        for _ in 0..track_len {
            let iid = r.read_u32::<LittleEndian>().map_err(IoError::Io)?;
            let idx = r.read_u32::<LittleEndian>().map_err(IoError::Io)?;
            track.push((ImageId::new(iid), Point2DIdx::new(idx)));
        }
        let pt = Point3D {
            xyz: nalgebra::Vector3::new(x, y, z),
            color,
            error: err,
            track,
        };
        rec.points3d.insert(Point3DId::new(pid), pt);
    }
    Ok(())
}

pub fn write_points3d_binary<W: Write>(w: &mut W, rec: &Reconstruction) -> Result<()> {
    w.write_u64::<LittleEndian>(rec.points3d.len() as u64)
        .map_err(IoError::Io)?;
    for (id, pt) in &rec.points3d {
        w.write_u64::<LittleEndian>(id.get()).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(pt.xyz.x).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(pt.xyz.y).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(pt.xyz.z).map_err(IoError::Io)?;
        w.write_all(&pt.color).map_err(IoError::Io)?;
        w.write_f64::<LittleEndian>(pt.error).map_err(IoError::Io)?;
        w.write_u64::<LittleEndian>(pt.track.len() as u64)
            .map_err(IoError::Io)?;
        for (iid, idx) in &pt.track {
            w.write_u32::<LittleEndian>(iid.get())
                .map_err(IoError::Io)?;
            w.write_u32::<LittleEndian>(idx.get())
                .map_err(IoError::Io)?;
        }
    }
    Ok(())
}

// ---------- dir helpers ----------

pub fn read_model_text_dir(dir: &Path, rec: &mut Reconstruction) -> Result<()> {
    // cameras
    let cams = dir.join("cameras.txt");
    if cams.exists() {
        let f = File::open(&cams).map_err(IoError::Io)?;
        read_cameras_text(BufReader::new(f), rec)?;
    }
    let imgs = dir.join("images.txt");
    if imgs.exists() {
        let f = File::open(&imgs).map_err(IoError::Io)?;
        read_images_text(BufReader::new(f), rec)?;
    }
    let pts = dir.join("points3D.txt");
    if pts.exists() {
        let f = File::open(&pts).map_err(IoError::Io)?;
        read_points3d_text(BufReader::new(f), rec)?;
    }
    Ok(())
}

pub fn write_model_text_dir(dir: &Path, rec: &Reconstruction) -> Result<()> {
    fs::create_dir_all(dir).map_err(IoError::Io)?;
    {
        let f = File::create(dir.join("cameras.txt")).map_err(IoError::Io)?;
        let mut w = BufWriter::new(f);
        write_cameras_text(&mut w, rec)?;
    }
    {
        let f = File::create(dir.join("images.txt")).map_err(IoError::Io)?;
        let mut w = BufWriter::new(f);
        write_images_text(&mut w, rec)?;
    }
    {
        let f = File::create(dir.join("points3D.txt")).map_err(IoError::Io)?;
        let mut w = BufWriter::new(f);
        write_points3d_text(&mut w, rec)?;
    }
    Ok(())
}

pub fn read_model_binary_dir(dir: &Path, rec: &mut Reconstruction) -> Result<()> {
    let cams = dir.join("cameras.bin");
    if cams.exists() {
        let mut f = File::open(&cams).map_err(IoError::Io)?;
        // binary files start with num? cameras.bin format: first u64 count then entries (no header count? Actually ReadCamerasBinary reads count). Good.
        read_cameras_binary(&mut f, rec)?;
    }
    let imgs = dir.join("images.bin");
    if imgs.exists() {
        let mut f = File::open(&imgs).map_err(IoError::Io)?;
        read_images_binary(&mut f, rec)?;
    }
    let pts = dir.join("points3D.bin");
    if pts.exists() {
        let mut f = File::open(&pts).map_err(IoError::Io)?;
        read_points3d_binary(&mut f, rec)?;
    }
    Ok(())
}

pub fn write_model_binary_dir(dir: &Path, rec: &Reconstruction) -> Result<()> {
    fs::create_dir_all(dir).map_err(IoError::Io)?;
    {
        let f = File::create(dir.join("cameras.bin")).map_err(IoError::Io)?;
        let mut w = BufWriter::new(f);
        write_cameras_binary(&mut w, rec)?;
    }
    {
        let f = File::create(dir.join("images.bin")).map_err(IoError::Io)?;
        let mut w = BufWriter::new(f);
        write_images_binary(&mut w, rec)?;
    }
    {
        let f = File::create(dir.join("points3D.bin")).map_err(IoError::Io)?;
        let mut w = BufWriter::new(f);
        write_points3d_binary(&mut w, rec)?;
    }
    Ok(())
}
