#![forbid(unsafe_code)]

//! Optimization: Levenberg-Marquardt and (later) bundle adjustment.

pub mod lm;

pub use lm::{lm_solve, LmOptions, LmResult};
