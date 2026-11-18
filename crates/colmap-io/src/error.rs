use thiserror::Error;

#[derive(Debug, Error)]
pub enum IoError {
    #[error("sqlite error: {0}")]
    Sqlite(#[from] rusqlite::Error),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid data: {0}")]
    Invalid(String),
    #[error("unknown camera model id: {0}")]
    UnknownCameraModel(i32),
}

pub type Result<T> = std::result::Result<T, IoError>;
