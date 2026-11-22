#![forbid(unsafe_code)]

//! Feature matching and pair generation — skeleton.

pub trait PairGenerator {
    fn generate(&self, num_images: usize) -> Vec<(usize, usize)>;
}

pub struct ExhaustivePairGenerator;
impl PairGenerator for ExhaustivePairGenerator {
    fn generate(&self, num_images: usize) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for i in 0..num_images {
            for j in (i + 1)..num_images {
                out.push((i, j));
            }
        }
        out
    }
}

pub trait Matcher {
    fn match_descriptors(&self, a: &[u8], b: &[u8], cols: usize) -> Vec<(usize, usize)>;
}
