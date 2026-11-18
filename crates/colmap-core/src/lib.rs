#![forbid(unsafe_code)]

//! Core types for `colmap-rs`: IDs, geometry primitives, camera models,
//! and the reconstruction data model.

pub mod camera;
pub mod error;
pub mod geometry;
pub mod ids;
pub mod reconstruction;
pub mod types;

pub use camera::{Camera, CameraModelKind};
pub use error::{ColmapError, Result};
pub use geometry::{Rigid3, Sim3};
pub use ids::{CameraId, ImageId, Point2DIdx, Point3DId, RigId, SensorId};
pub use reconstruction::{Image, Point2D, Point3D, Reconstruction};
pub use types::{Point2, Point3};
