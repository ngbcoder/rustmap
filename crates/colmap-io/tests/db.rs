use colmap_core::camera::{Camera, CameraModelKind};
use colmap_io::database::Database;
use tempfile::tempdir;

#[test]
fn db_camera_roundtrip() {
    let dir = tempdir().unwrap();
    let db = Database::create(&dir.path().join("t.db")).unwrap();
    let cam = Camera::new(
        CameraModelKind::Pinhole,
        vec![100.0, 100.0, 50.0, 60.0],
        100,
        100,
    )
    .unwrap();
    db.write_camera(1, &cam).unwrap();
    let cams = db.read_cameras().unwrap();
    assert_eq!(cams.len(), 1);
    assert_eq!(cams[0].0, 1);
    assert_eq!(cams[0].1.width, 100);
}
