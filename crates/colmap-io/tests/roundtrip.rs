use colmap_core::camera::{Camera, CameraModelKind};
use colmap_core::geometry::Rigid3;
use colmap_core::ids::{CameraId, ImageId, Point3DId};
use colmap_core::reconstruction::{Image, Point2D, Point3D, Reconstruction};
use colmap_io::model;
use nalgebra::Vector3;
use std::path::PathBuf;
use tempfile::tempdir;

#[test]
fn binary_roundtrip() {
    let mut rec = Reconstruction::default();
    let cam = Camera::new(CameraModelKind::Pinhole, vec![100.0, 100.0, 50.0, 60.0], 100, 100).unwrap();
    rec.cameras.insert(CameraId::new(1), cam);
    let mut img = Image {
        image_id: ImageId::new(1),
        name: "a.jpg".to_string(),
        camera_id: CameraId::new(1),
        pose: Rigid3::identity(),
        points2d: vec![Point2D { xy: [1.0, 2.0], point3d_id: Some(Point3DId::new(1)) }],
    };
    rec.images.insert(img.image_id, img);
    rec.points3d.insert(Point3DId::new(1), Point3D {
        xyz: Vector3::new(1.0, 2.0, 3.0),
        color: [1,2,3],
        error: 0.5,
        track: vec![(ImageId::new(1), colmap_core::ids::Point2DIdx::new(0))],
    });

    let dir = tempdir().unwrap();
    model::write_model_binary_dir(dir.path(), &rec).unwrap();
    let mut rec2 = Reconstruction::default();
    model::read_model_binary_dir(dir.path(), &mut rec2).unwrap();
    assert_eq!(rec2.cameras.len(), 1);
    assert_eq!(rec2.images.len(), 1);
    assert_eq!(rec2.points3d.len(), 1);
    let p = rec2.points3d.get(&Point3DId::new(1)).unwrap();
    assert!((p.xyz.x - 1.0).abs() < 1e-12);
    let im = rec2.images.get(&ImageId::new(1)).unwrap();
    assert_eq!(im.points2d[0].point3d_id, Some(Point3DId::new(1)));
}

#[test]
fn text_roundtrip() {
    let mut rec = Reconstruction::default();
    let cam = Camera::new(CameraModelKind::SimplePinhole, vec![100.0, 50.0, 60.0], 100, 100).unwrap();
    rec.cameras.insert(CameraId::new(1), cam);
    let img = Image {
        image_id: ImageId::new(1),
        name: "a.jpg".to_string(),
        camera_id: CameraId::new(1),
        pose: Rigid3::identity(),
        points2d: vec![],
    };
    rec.images.insert(img.image_id, img);

    let dir = tempdir().unwrap();
    model::write_model_text_dir(dir.path(), &rec).unwrap();
    let mut rec2 = Reconstruction::default();
    model::read_model_text_dir(dir.path(), &mut rec2).unwrap();
    assert_eq!(rec2.cameras.len(), 1);
    assert_eq!(rec2.images.len(), 1);
}
