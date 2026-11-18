use colmap_core::camera::{Camera, CameraModelKind};
use colmap_core::geometry::Rigid3;
use colmap_core::ids::{CameraId, ImageId, Point2DIdx, Point3DId};
use colmap_core::reconstruction::{Image, Point2D, Point3D, Reconstruction};
use colmap_io::model;
use nalgebra::Vector3;
use std::env;

fn main() {
    let out = env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/opencode/gen_model".to_string());
    let mut rec = Reconstruction::default();
    let cam = Camera::new(
        CameraModelKind::SimplePinhole,
        vec![100.0, 50.0, 60.0],
        100,
        100,
    )
    .unwrap();
    rec.cameras.insert(CameraId::new(1), cam);
    let img = Image {
        image_id: ImageId::new(1),
        name: "a.jpg".to_string(),
        camera_id: CameraId::new(1),
        pose: Rigid3::identity(),
        points2d: vec![Point2D {
            xy: [10.0, 20.0],
            point3d_id: Some(Point3DId::new(1)),
        }],
    };
    rec.images.insert(img.image_id, img);
    rec.points3d.insert(
        Point3DId::new(1),
        Point3D {
            xyz: Vector3::new(1.0, 2.0, 3.0),
            color: [255, 0, 0],
            error: 0.5,
            track: vec![(ImageId::new(1), Point2DIdx::new(0))],
        },
    );
    model::write_model_text_dir(std::path::Path::new(&out), &rec).unwrap();
    println!("wrote {}", out);
}
