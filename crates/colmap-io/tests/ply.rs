use colmap_core::ids::Point3DId;
use colmap_core::reconstruction::{Point3D, Reconstruction};
use colmap_io::ply::write_ply;
use std::fs;

#[test]
fn ply_writes_header_and_points() {
    let mut rec = Reconstruction::new();
    rec.add_point3d(
        Point3DId::new(1),
        Point3D {
            xyz: nalgebra::Vector3::new(1.0, 2.0, 3.0),
            color: [255, 0, 0],
            error: 0.0,
            track: Vec::new(),
        },
    );
    let path = std::env::temp_dir().join("test.ply");
    write_ply(&path, &rec).unwrap();
    let s = fs::read_to_string(&path).unwrap();
    assert!(s.contains("element vertex 1"));
    assert!(s.contains("1 2 3 255 0 0"));
    let _ = fs::remove_file(&path);
}
