#![forbid(unsafe_code)]

//! I/O for COLMAP-compatible databases and model files.

pub mod database;
pub mod error;
pub mod model;
pub mod ply;

pub use error::{IoError, Result};
