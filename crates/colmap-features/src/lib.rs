#![forbid(unsafe_code)]

//! Feature extraction (SIFT, etc.) — skeleton.

#[derive(Clone, Debug)]
pub struct Keypoint {
    pub x: f64,
    pub y: f64,
    pub scale: f64,
    pub orientation: f64,
}

#[derive(Clone, Debug)]
pub struct Descriptors {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<u8>,
}

pub trait FeatureExtractor {
    fn extract(&self, image: &[u8], width: u32, height: u32) -> (Vec<Keypoint>, Descriptors);
}

pub struct NoopExtractor;
impl FeatureExtractor for NoopExtractor {
    fn extract(&self, _image: &[u8], _width: u32, _height: u32) -> (Vec<Keypoint>, Descriptors) {
        (
            Vec::new(),
            Descriptors {
                rows: 0,
                cols: 0,
                data: Vec::new(),
            },
        )
    }
}
