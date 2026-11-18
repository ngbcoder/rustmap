//! Basic vector/point aliases.

use nalgebra::{Vector2, Vector3};

/// 2D point (x, y).
pub type Point2 = Vector2<f64>;
/// 3D point (x, y, z).
pub type Point3 = Vector3<f64>;

/// Convert degrees to radians.
pub fn deg2rad(d: f64) -> f64 {
    d * std::f64::consts::PI / 180.0
}
/// Convert radians to degrees.
pub fn rad2deg(r: f64) -> f64 {
    r * 180.0 / std::f64::consts::PI
}
