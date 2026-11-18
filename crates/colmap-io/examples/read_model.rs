use colmap_io::model;
use std::env;
fn main() {
    let dir = env::args().nth(1).unwrap();
    let mut rec = colmap_core::reconstruction::Reconstruction::default();
    model::read_model_binary_dir(std::path::Path::new(&dir), &mut rec).unwrap();
    println!(
        "cameras={} images={} points={}",
        rec.cameras.len(),
        rec.images.len(),
        rec.points3d.len()
    );
}
