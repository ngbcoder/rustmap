//! Rigid and similarity transforms.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::ops::Mul;

/// A 3D rigid transform: `x' = R * x + t`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Rigid3 {
    /// Rotation as a unit quaternion.
    pub rotation: UnitQuaternion<f64>,
    /// Translation.
    pub translation: Vector3<f64>,
}

impl Rigid3 {
    /// Identity transform.
    pub fn identity() -> Self {
        Self {
            rotation: UnitQuaternion::identity(),
            translation: Vector3::zeros(),
        }
    }

    /// Inverse transform.
    pub fn inverse(&self) -> Self {
        let r_inv = self.rotation.inverse();
        let t_inv = -(r_inv * self.translation);
        Self {
            rotation: r_inv,
            translation: t_inv,
        }
    }

    /// Apply to a point.
    pub fn transform_point(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.rotation * p + self.translation
    }
}

pub type Pose = Rigid3;

impl Default for Rigid3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Mul for Rigid3 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            rotation: self.rotation * rhs.rotation,
            translation: self.rotation * rhs.translation + self.translation,
        }
    }
}

impl Mul<Vector3<f64>> for Rigid3 {
    type Output = Vector3<f64>;
    fn mul(self, rhs: Vector3<f64>) -> Vector3<f64> {
        self.transform_point(&rhs)
    }
}

/// A 3D similarity transform: `x' = s * R * x + t`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Sim3 {
    pub scale: f64,
    pub rotation: UnitQuaternion<f64>,
    pub translation: Vector3<f64>,
}

impl Sim3 {
    pub fn identity() -> Self {
        Self {
            scale: 1.0,
            rotation: UnitQuaternion::identity(),
            translation: Vector3::zeros(),
        }
    }

    pub fn inverse(&self) -> Self {
        let r_inv = self.rotation.inverse();
        let s_inv = 1.0 / self.scale;
        let t_inv = -(r_inv * self.translation) * s_inv;
        Self {
            scale: s_inv,
            rotation: r_inv,
            translation: t_inv,
        }
    }

    pub fn transform_point(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.scale * (self.rotation * p) + self.translation
    }
}

impl Default for Sim3 {
    fn default() -> Self {
        Self::identity()
    }
}

// Serde helpers: store rotation as [w,x,y,z] + translation + scale.
#[derive(Serialize, Deserialize)]
struct Rigid3Serde {
    rotation: [f64; 4],
    translation: [f64; 3],
}

impl Serialize for Rigid3 {
    fn serialize<S: serde::Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        let q = self.rotation.quaternion();
        Rigid3Serde {
            rotation: [q.w, q.i, q.j, q.k],
            translation: [self.translation.x, self.translation.y, self.translation.z],
        }
        .serialize(s)
    }
}

impl<'de> Deserialize<'de> for Rigid3 {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Self, D::Error> {
        let r: Rigid3Serde = Rigid3Serde::deserialize(d)?;
        let q = Quaternion::new(r.rotation[0], r.rotation[1], r.rotation[2], r.rotation[3]);
        let rot = UnitQuaternion::from_quaternion(q);
        Ok(Self {
            rotation: rot,
            translation: Vector3::new(r.translation[0], r.translation[1], r.translation[2]),
        })
    }
}

#[derive(Serialize, Deserialize)]
struct Sim3Serde {
    scale: f64,
    rotation: [f64; 4],
    translation: [f64; 3],
}

impl Serialize for Sim3 {
    fn serialize<S: serde::Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        let q = self.rotation.quaternion();
        Sim3Serde {
            scale: self.scale,
            rotation: [q.w, q.i, q.j, q.k],
            translation: [self.translation.x, self.translation.y, self.translation.z],
        }
        .serialize(s)
    }
}

impl<'de> Deserialize<'de> for Sim3 {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Self, D::Error> {
        let r: Sim3Serde = Sim3Serde::deserialize(d)?;
        let q = Quaternion::new(r.rotation[0], r.rotation[1], r.rotation[2], r.rotation[3]);
        Ok(Self {
            scale: r.scale,
            rotation: UnitQuaternion::from_quaternion(q),
            translation: Vector3::new(r.translation[0], r.translation[1], r.translation[2]),
        })
    }
}

// Avoid unused import warning
#[allow(dead_code)]
fn _use_matrix3() -> Matrix3<f64> {
    Matrix3::identity()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_transform() {
        let r = Rigid3::identity();
        let p = Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(r.transform_point(&p), p);
    }

    #[test]
    fn inverse_roundtrip() {
        let r = Rigid3 {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5),
            translation: Vector3::new(1.0, -2.0, 3.0),
        };
        let p = Vector3::new(0.5, 1.5, -0.5);
        let q = r.transform_point(&p);
        let rinv = r.inverse();
        let p2 = rinv.transform_point(&q);
        for i in 0..3 {
            assert!((p[i] - p2[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn composition() {
        let a = Rigid3 {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3),
            translation: Vector3::new(1.0, 0.0, 0.0),
        };
        let b = Rigid3 {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -0.2),
            translation: Vector3::new(0.0, 2.0, 0.0),
        };
        let ab = a * b;
        let p = Vector3::new(1.0, 1.0, 1.0);
        let left = ab.transform_point(&p);
        let right = a.transform_point(&b.transform_point(&p));
        for i in 0..3 {
            assert!((left[i] - right[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn serde_roundtrip() {
        let r = Rigid3 {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 1.2),
            translation: Vector3::new(3.0, 4.0, 5.0),
        };
        let s = serde_json::to_string(&r).unwrap();
        let back: Rigid3 = serde_json::from_str(&s).unwrap();
        assert_eq!(r, back);
    }
}
