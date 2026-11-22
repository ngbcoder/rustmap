#![forbid(unsafe_code)]

//! Incremental structure-from-motion — skeleton.

use colmap_core::reconstruction::Reconstruction;

pub struct CorrespondenceGraph {
    pub num_images: usize,
}

pub struct Mapper {
    pub reconstruction: Reconstruction,
}

impl Mapper {
    pub fn new() -> Self {
        Self {
            reconstruction: Reconstruction::new(),
        }
    }
}
