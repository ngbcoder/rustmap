//! Error types.

use thiserror::Error;

/// Errors returned by colmap-core.
#[derive(Debug, Error)]
pub enum ColmapError {
    #[error("invalid camera model: {0}")]
    InvalidCameraModel(String),
    #[error("invalid parameter count: expected {expected}, got {got}")]
    InvalidParamCount { expected: usize, got: usize },
    #[error("degenerate geometry: {0}")]
    Degenerate(String),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
}

/// Convenience alias.
pub type Result<T> = std::result::Result<T, ColmapError>;
