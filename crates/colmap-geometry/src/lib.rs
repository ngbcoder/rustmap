#![forbid(unsafe_code)]

//! Geometry primitives: RANSAC, minimal solvers, triangulation, two-view geometry.

pub mod essential;
pub mod fundamental;
pub mod homography;
pub mod pnp;
pub mod ransac;
pub mod triangulation;
pub mod two_view;

pub use essential::{count_cheiral, decompose_essential, essential_from_fundamental};
pub use fundamental::fundamental_8point;
pub use homography::homography_dlt;
pub use pnp::{pnp_dlt, pnp_ransac, PnpEstimator};
pub use ransac::{ransac, Estimator, RansacOptions, RansacResult};
pub use triangulation::triangulate_dlt;
pub use two_view::{
    estimate_two_view_geometry, fundamental_ransac, homography_ransac, TwoViewGeometry, TwoViewType,
};
