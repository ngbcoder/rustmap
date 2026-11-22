//! SQLite database I/O matching COLMAP schema (4.3.0.dev0).

use crate::error::{IoError, Result};
use colmap_core::camera::{Camera, CameraModelKind};
use colmap_core::ids::CameraId;
use colmap_core::reconstruction::Reconstruction;
use rusqlite::{params, Connection};
use std::path::Path;

pub struct Database {
    conn: Connection,
}

impl Database {
    pub fn open(path: &Path) -> Result<Self> {
        let conn = Connection::open(path).map_err(IoError::Sqlite)?;
        let ver: i64 = conn
            .query_row("PRAGMA user_version", [], |r| r.get(0))
            .unwrap_or(0);
        if ver != 0 && ver != 4030001 {
            return Err(IoError::Invalid(format!("unsupported db version {}", ver)));
        }
        Ok(Self { conn })
    }

    pub fn create(path: &Path) -> Result<Self> {
        // remove existing
        let _ = std::fs::remove_file(path);
        let conn = Connection::open(path).map_err(IoError::Sqlite)?;
        let db = Self { conn };
        db.create_tables()?;
        Ok(db)
    }

    fn create_tables(&self) -> Result<()> {
        self.conn.execute_batch(
            "CREATE TABLE cameras (camera_id INTEGER NOT NULL, model INTEGER NOT NULL, width INTEGER NOT NULL, height INTEGER NOT NULL, params BLOB, source INTEGER NOT NULL, PRIMARY KEY(camera_id, source));
             CREATE TABLE images (image_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, name TEXT NOT NULL UNIQUE, camera_id INTEGER NOT NULL, CONSTRAINT image_id_check CHECK(image_id >= 0 and image_id < 2147483647));
             CREATE UNIQUE INDEX index_name ON images(name);
             CREATE TABLE keypoints (image_id INTEGER PRIMARY KEY NOT NULL, rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB, FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);
             CREATE TABLE descriptors (image_id INTEGER PRIMARY KEY NOT NULL, type INTEGER NOT NULL, rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB, FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);
             CREATE TABLE matches (pair_id INTEGER PRIMARY KEY NOT NULL, rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB);
             CREATE TABLE two_view_geometries (pair_id INTEGER PRIMARY KEY NOT NULL, rows INTEGER NOT NULL, cols INTEGER NOT NULL, data BLOB, config INTEGER NOT NULL, F BLOB, E BLOB, H BLOB, qvec BLOB, tvec BLOB, camera1 BLOB, camera2 BLOB);
             CREATE TABLE pose_priors (pose_prior_id INTEGER PRIMARY KEY NOT NULL, corr_data_id INTEGER NOT NULL, corr_sensor_id INTEGER NOT NULL, corr_sensor_type INTEGER NOT NULL, position BLOB, position_covariance BLOB, gravity BLOB, coordinate_system INTEGER NOT NULL);
             CREATE UNIQUE INDEX pose_prior_data_assignment ON pose_priors(corr_data_id, corr_sensor_id, corr_sensor_type);
             CREATE TABLE rigs (rig_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, ref_sensor_id INTEGER NOT NULL, ref_sensor_type INTEGER NOT NULL);
             CREATE UNIQUE INDEX rig_ref_sensor_assignment ON rigs(ref_sensor_id, ref_sensor_type);
             CREATE TABLE rig_sensors (rig_id INTEGER NOT NULL, sensor_id INTEGER NOT NULL, sensor_type INTEGER NOT NULL, sensor_from_rig BLOB, FOREIGN KEY(rig_id) REFERENCES rigs(rig_id) ON DELETE CASCADE);
             CREATE UNIQUE INDEX rig_sensor_assignment ON rig_sensors(sensor_id, sensor_type);
             CREATE TABLE frames (frame_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, rig_id INTEGER NOT NULL, FOREIGN KEY(rig_id) REFERENCES rigs(rig_id) ON DELETE CASCADE);
             CREATE TABLE frame_data (frame_id INTEGER NOT NULL, data_id INTEGER NOT NULL, sensor_id INTEGER NOT NULL, sensor_type INTEGER NOT NULL, FOREIGN KEY(frame_id) REFERENCES frames(frame_id) ON DELETE CASCADE);
             CREATE UNIQUE INDEX frame_sensor_assignment ON frame_data(data_id, sensor_type);"
        ).map_err(IoError::Sqlite)?;
        self.conn
            .execute("PRAGMA user_version = 4030001", [])
            .map_err(IoError::Sqlite)?;
        Ok(())
    }

    // cameras
    pub fn read_cameras(&self) -> Result<Vec<(i64, Camera)>> {
        let mut stmt = self
            .conn
            .prepare("SELECT camera_id, model, width, height, params, source FROM cameras")
            .map_err(IoError::Sqlite)?;
        let rows = stmt
            .query_map([], |row| {
                let camera_id: i64 = row.get(0)?;
                let model_i: i64 = row.get(1)?;
                let width: i64 = row.get(2)?;
                let height: i64 = row.get(3)?;
                let params_blob: Vec<u8> = row.get(4)?;
                let _source: i64 = row.get(5).unwrap_or(0);
                Ok((camera_id, model_i, width, height, params_blob))
            })
            .map_err(IoError::Sqlite)?;
        let mut out = Vec::new();
        for r in rows {
            let (camera_id, model_i, width, height, params_blob) = r.map_err(IoError::Sqlite)?;
            let model = CameraModelKind::from_i32(model_i as i32)
                .ok_or(IoError::UnknownCameraModel(model_i as i32))?;
            let nparams = model.num_params();
            // params stored as array of f64 little-endian
            let mut params = vec![0.0f64; nparams];
            if params_blob.len() >= nparams * 8 {
                for (i, p) in params.iter_mut().enumerate().take(nparams) {
                    let off = i * 8;
                    *p = f64::from_le_bytes(params_blob[off..off + 8].try_into().unwrap());
                }
            }
            let cam = Camera::new(model, params, width as u32, height as u32)
                .map_err(|e| IoError::Invalid(format!("{}", e)))?;
            out.push((camera_id, cam));
        }
        Ok(out)
    }

    pub fn write_camera(&self, camera_id: i64, cam: &Camera) -> Result<()> {
        // serialize params as f64 LE array
        let mut blob = Vec::with_capacity(cam.params.len() * 8);
        for p in &cam.params {
            blob.extend_from_slice(&p.to_le_bytes());
        }
        self.conn.execute(
            "INSERT OR REPLACE INTO cameras (camera_id, model, width, height, params, source) VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
            params![camera_id, cam.model as i32, cam.width as i64, cam.height as i64, blob, 0i64],
        ).map_err(IoError::Sqlite)?;
        Ok(())
    }

    // images
    pub fn read_images(&self) -> Result<Vec<(i64, String, i64)>> {
        let mut stmt = self
            .conn
            .prepare("SELECT image_id, name, camera_id FROM images")
            .map_err(IoError::Sqlite)?;
        let rows = stmt
            .query_map([], |row| {
                Ok((
                    row.get::<_, i64>(0)?,
                    row.get::<_, String>(1)?,
                    row.get::<_, i64>(2)?,
                ))
            })
            .map_err(IoError::Sqlite)?;
        let mut out = Vec::new();
        for r in rows {
            out.push(r.map_err(IoError::Sqlite)?);
        }
        Ok(out)
    }

    pub fn write_image(&self, image_id: Option<i64>, name: &str, camera_id: i64) -> Result<i64> {
        if let Some(id) = image_id {
            self.conn
                .execute(
                    "INSERT OR REPLACE INTO images (image_id, name, camera_id) VALUES (?1, ?2, ?3)",
                    params![id, name, camera_id],
                )
                .map_err(IoError::Sqlite)?;
            Ok(id)
        } else {
            self.conn
                .execute(
                    "INSERT INTO images (name, camera_id) VALUES (?1, ?2)",
                    params![name, camera_id],
                )
                .map_err(IoError::Sqlite)?;
            Ok(self.conn.last_insert_rowid())
        }
    }

    // keypoints
    pub fn read_keypoints(&self, image_id: i64) -> Result<(usize, usize, Vec<u8>)> {
        let mut stmt = self
            .conn
            .prepare("SELECT rows, cols, data FROM keypoints WHERE image_id = ?1")
            .map_err(IoError::Sqlite)?;
        let mut rows = stmt.query(params![image_id]).map_err(IoError::Sqlite)?;
        if let Some(row) = rows.next().map_err(IoError::Sqlite)? {
            let r: i64 = row.get(0).map_err(IoError::Sqlite)?;
            let c: i64 = row.get(1).map_err(IoError::Sqlite)?;
            let data: Vec<u8> = row.get(2).map_err(IoError::Sqlite)?;
            Ok((r as usize, c as usize, data))
        } else {
            Ok((0, 0, Vec::new()))
        }
    }

    pub fn write_keypoints(
        &self,
        image_id: i64,
        rows: usize,
        cols: usize,
        data: &[u8],
    ) -> Result<()> {
        self.conn.execute(
            "INSERT OR REPLACE INTO keypoints (image_id, rows, cols, data) VALUES (?1, ?2, ?3, ?4)",
            params![image_id, rows as i64, cols as i64, data],
        ).map_err(IoError::Sqlite)?;
        Ok(())
    }

    // descriptors
    pub fn read_descriptors(&self, image_id: i64) -> Result<(i64, usize, usize, Vec<u8>)> {
        let mut stmt = self
            .conn
            .prepare("SELECT type, rows, cols, data FROM descriptors WHERE image_id = ?1")
            .map_err(IoError::Sqlite)?;
        let mut rows = stmt.query(params![image_id]).map_err(IoError::Sqlite)?;
        if let Some(row) = rows.next().map_err(IoError::Sqlite)? {
            let t: i64 = row.get(0).map_err(IoError::Sqlite)?;
            let r: i64 = row.get(1).map_err(IoError::Sqlite)?;
            let c: i64 = row.get(2).map_err(IoError::Sqlite)?;
            let data: Vec<u8> = row.get(3).map_err(IoError::Sqlite)?;
            Ok((t, r as usize, c as usize, data))
        } else {
            Ok((0, 0, 0, Vec::new()))
        }
    }

    pub fn write_descriptors(
        &self,
        image_id: i64,
        dtype: i64,
        rows: usize,
        cols: usize,
        data: &[u8],
    ) -> Result<()> {
        self.conn.execute(
            "INSERT OR REPLACE INTO descriptors (image_id, type, rows, cols, data) VALUES (?1, ?2, ?3, ?4, ?5)",
            params![image_id, dtype, rows as i64, cols as i64, data],
        ).map_err(IoError::Sqlite)?;
        Ok(())
    }

    // matches: pair_id, rows, cols, data (point2D_t ints)
    pub fn write_matches(&self, pair_id: i64, rows: usize, cols: usize, data: &[u8]) -> Result<()> {
        self.conn.execute(
            "INSERT OR REPLACE INTO matches (pair_id, rows, cols, data) VALUES (?1, ?2, ?3, ?4)",
            params![pair_id, rows as i64, cols as i64, data],
        ).map_err(IoError::Sqlite)?;
        Ok(())
    }

    pub fn read_matches(&self, pair_id: i64) -> Result<(usize, usize, Vec<u8>)> {
        let mut stmt = self
            .conn
            .prepare("SELECT rows, cols, data FROM matches WHERE pair_id = ?1")
            .map_err(IoError::Sqlite)?;
        let mut rows = stmt.query(params![pair_id]).map_err(IoError::Sqlite)?;
        if let Some(row) = rows.next().map_err(IoError::Sqlite)? {
            let r: i64 = row.get(0).map_err(IoError::Sqlite)?;
            let c: i64 = row.get(1).map_err(IoError::Sqlite)?;
            let data: Vec<u8> = row.get(2).map_err(IoError::Sqlite)?;
            Ok((r as usize, c as usize, data))
        } else {
            Ok((0, 0, Vec::new()))
        }
    }

    // two_view_geometries
    pub fn write_two_view_geometry(
        &self,
        pair_id: i64,
        rows: usize,
        cols: usize,
        data: &[u8],
        config: i64,
    ) -> Result<()> {
        self.conn.execute(
            "INSERT OR REPLACE INTO two_view_geometries (pair_id, rows, cols, data, config, F, E, H, qvec, tvec, camera1, camera2) VALUES (?1, ?2, ?3, ?4, ?5, NULL, NULL, NULL, NULL, NULL, NULL, NULL)",
            params![pair_id, rows as i64, cols as i64, data, config],
        ).map_err(IoError::Sqlite)?;
        Ok(())
    }

    pub fn read_two_view_geometry(&self, pair_id: i64) -> Result<(usize, usize, Vec<u8>, i64)> {
        let mut stmt = self
            .conn
            .prepare("SELECT rows, cols, data, config FROM two_view_geometries WHERE pair_id = ?1")
            .map_err(IoError::Sqlite)?;
        let mut rows = stmt.query(params![pair_id]).map_err(IoError::Sqlite)?;
        if let Some(row) = rows.next().map_err(IoError::Sqlite)? {
            let r: i64 = row.get(0).map_err(IoError::Sqlite)?;
            let c: i64 = row.get(1).map_err(IoError::Sqlite)?;
            let data: Vec<u8> = row.get(2).map_err(IoError::Sqlite)?;
            let cfg: i64 = row.get(3).map_err(IoError::Sqlite)?;
            Ok((r as usize, c as usize, data, cfg))
        } else {
            Ok((0, 0, Vec::new(), 0))
        }
    }

    // Convenience: load cameras into Reconstruction
    pub fn load_cameras_into(&self, rec: &mut Reconstruction) -> Result<()> {
        for (cid, cam) in self.read_cameras()? {
            rec.cameras.insert(CameraId::new(cid as u32), cam);
        }
        Ok(())
    }

    pub fn save_cameras_from(&self, rec: &Reconstruction) -> Result<()> {
        for (id, cam) in &rec.cameras {
            self.write_camera(id.get() as i64, cam)?;
        }
        Ok(())
    }
}
