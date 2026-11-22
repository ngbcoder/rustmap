#![forbid(unsafe_code)]

//! Optimization: Levenberg-Marquardt and bundle adjustment.

pub mod ba;
pub mod lm;
pub mod robust;

pub use ba::{solve_ba, BaProblem, BaResult, Observation};
pub use lm::{lm_solve, LmOptions, LmResult};
