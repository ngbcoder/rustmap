//! PLY point-cloud export.

use crate::error::Result;
use colmap_core::reconstruction::Reconstruction;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

pub fn write_ply(path: &Path, rec: &Reconstruction) -> Result<()> {
    let f = File::create(path).map_err(crate::error::IoError::Io)?;
    let mut w = BufWriter::new(f);
    let n = rec.points3d.len();
    writeln!(w, "ply").unwrap();
    writeln!(w, "format ascii 1.0").unwrap();
    writeln!(w, "element vertex {}", n).unwrap();
    writeln!(w, "property float x").unwrap();
    writeln!(w, "property float y").unwrap();
    writeln!(w, "property float z").unwrap();
    writeln!(w, "property uchar red").unwrap();
    writeln!(w, "property uchar green").unwrap();
    writeln!(w, "property uchar blue").unwrap();
    writeln!(w, "end_header").unwrap();
    for pt in rec.points3d.values() {
        writeln!(
            w,
            "{} {} {} {} {} {}",
            pt.xyz.x, pt.xyz.y, pt.xyz.z, pt.color[0], pt.color[1], pt.color[2]
        )
        .unwrap();
    }
    Ok(())
}
