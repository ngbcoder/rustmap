use colmap_io::model;
use std::path::PathBuf;

#[test]
fn read_golden_model() {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("tests/data/simple_model");
    let mut rec = colmap_core::reconstruction::Reconstruction::default();
    model::read_model_text_dir(&p, &mut rec).unwrap();
    assert_eq!(rec.cameras.len(), 1);
    assert_eq!(rec.images.len(), 1);
    assert_eq!(rec.points3d.len(), 1);
    // binary roundtrip
    let tmp = tempfile::tempdir().unwrap();
    model::write_model_binary_dir(tmp.path(), &rec).unwrap();
    let mut rec2 = colmap_core::reconstruction::Reconstruction::default();
    model::read_model_binary_dir(tmp.path(), &mut rec2).unwrap();
    assert_eq!(rec2.points3d.len(), 1);
}
