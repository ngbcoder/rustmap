use colmap_io::model;
use std::path::Path;

// Ignored by default: requires an upstream-generated model directory.
// Run with: UPSTREAM_MODEL=/path/to/sparse/0 cargo test -- --ignored
#[test]
#[ignore]
fn read_upstream_model() {
    let p = std::env::var("UPSTREAM_MODEL").unwrap_or_else(|_| "/tmp/opencode/deckcol_sparse/0".to_string());
    let path = Path::new(&p);
    if !path.exists() { return; }
    let mut rec = colmap_core::reconstruction::Reconstruction::default();
    model::read_model_text_dir(path, &mut rec).unwrap();
    assert!(rec.cameras.len() > 0);
    // write back and read
    let tmp = tempfile::tempdir().unwrap();
    model::write_model_text_dir(tmp.path(), &rec).unwrap();
    let mut rec2 = colmap_core::reconstruction::Reconstruction::default();
    model::read_model_text_dir(tmp.path(), &mut rec2).unwrap();
    assert_eq!(rec.cameras.len(), rec2.cameras.len());
    assert_eq!(rec.images.len(), rec2.images.len());
    assert_eq!(rec.points3d.len(), rec2.points3d.len());
}
