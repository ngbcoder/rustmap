#![forbid(unsafe_code)]

//! Geometry primitives: RANSAC, minimal solvers, triangulation, two-view geometry.

pub mod essential;
pub mod fundamental;
pub mod homography;
pub mod ransac;
pub mod triangulation;

pub use essential::{count_cheiral, decompose_essential, essential_from_fundamental};
pub use fundamental::fundamental_8point;
pub use homography::homography_dlt;
pub use ransac::{ransac, Estimator, RansacOptions, RansacResult};
pub use triangulation::triangulate_dlt;
