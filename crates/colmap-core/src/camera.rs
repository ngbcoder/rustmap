//! Camera models.

use crate::error::{ColmapError, Result};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::str::FromStr;

/// Camera model kind, with numeric ids matching COLMAP `CameraModelId`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(i32)]
pub enum CameraModelKind {
    Invalid = -1,
    SimplePinhole = 0,
    Pinhole = 1,
    SimpleRadial = 2,
    Radial = 3,
    OpenCv = 4,
    OpenCvFisheye = 5,
    FullOpenCv = 6,
    Fov = 7,
    SimpleRadialFisheye = 8,
    RadialFisheye = 9,
    ThinPrismFisheye = 10,
    RadTanThinPrismFisheye = 11,
    SimpleDivision = 12,
    Division = 13,
    SimpleFisheye = 14,
    Fisheye = 15,
    Eucm = 16,
    Equirectangular = 17,
    SkewedPinhole = 18,
}

impl CameraModelKind {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            -1 => Some(Self::Invalid),
            0 => Some(Self::SimplePinhole),
            1 => Some(Self::Pinhole),
            2 => Some(Self::SimpleRadial),
            3 => Some(Self::Radial),
            4 => Some(Self::OpenCv),
            5 => Some(Self::OpenCvFisheye),
            6 => Some(Self::FullOpenCv),
            7 => Some(Self::Fov),
            8 => Some(Self::SimpleRadialFisheye),
            9 => Some(Self::RadialFisheye),
            10 => Some(Self::ThinPrismFisheye),
            11 => Some(Self::RadTanThinPrismFisheye),
            12 => Some(Self::SimpleDivision),
            13 => Some(Self::Division),
            14 => Some(Self::SimpleFisheye),
            15 => Some(Self::Fisheye),
            16 => Some(Self::Eucm),
            17 => Some(Self::Equirectangular),
            18 => Some(Self::SkewedPinhole),
            _ => None,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::SimplePinhole => "SIMPLE_PINHOLE",
            Self::Pinhole => "PINHOLE",
            Self::SimpleRadial => "SIMPLE_RADIAL",
            Self::Radial => "RADIAL",
            Self::OpenCv => "OPENCV",
            Self::OpenCvFisheye => "OPENCV_FISHEYE",
            Self::FullOpenCv => "FULL_OPENCV",
            Self::Fov => "FOV",
            Self::SimpleRadialFisheye => "SIMPLE_RADIAL_FISHEYE",
            Self::RadialFisheye => "RADIAL_FISHEYE",
            Self::ThinPrismFisheye => "THIN_PRISM_FISHEYE",
            Self::RadTanThinPrismFisheye => "RAD_TAN_THIN_PRISM_FISHEYE",
            Self::SimpleDivision => "SIMPLE_DIVISION",
            Self::Division => "DIVISION",
            Self::SimpleFisheye => "SIMPLE_FISHEYE",
            Self::Fisheye => "FISHEYE",
            Self::Eucm => "EUCM",
            Self::Equirectangular => "EQUIRECTANGULAR",
            Self::SkewedPinhole => "SKEWED_PINHOLE",
            Self::Invalid => "INVALID",
        }
    }

    pub fn from_name(s: &str) -> Option<Self> {
        match s {
            "SIMPLE_PINHOLE" => Some(Self::SimplePinhole),
            "PINHOLE" => Some(Self::Pinhole),
            "SIMPLE_RADIAL" => Some(Self::SimpleRadial),
            "RADIAL" => Some(Self::Radial),
            "OPENCV" => Some(Self::OpenCv),
            "OPENCV_FISHEYE" => Some(Self::OpenCvFisheye),
            "FULL_OPENCV" => Some(Self::FullOpenCv),
            "FOV" => Some(Self::Fov),
            "SIMPLE_RADIAL_FISHEYE" => Some(Self::SimpleRadialFisheye),
            "RADIAL_FISHEYE" => Some(Self::RadialFisheye),
            "THIN_PRISM_FISHEYE" => Some(Self::ThinPrismFisheye),
            "RAD_TAN_THIN_PRISM_FISHEYE" => Some(Self::RadTanThinPrismFisheye),
            "SIMPLE_DIVISION" => Some(Self::SimpleDivision),
            "DIVISION" => Some(Self::Division),
            "SIMPLE_FISHEYE" => Some(Self::SimpleFisheye),
            "FISHEYE" => Some(Self::Fisheye),
            "EUCM" => Some(Self::Eucm),
            "EQUIRECTANGULAR" => Some(Self::Equirectangular),
            "SKEWED_PINHOLE" => Some(Self::SkewedPinhole),
            "INVALID" => Some(Self::Invalid),
            _ => None,
        }
    }

    pub fn num_params(&self) -> usize {
        match self {
            Self::SimplePinhole => 3,
            Self::Pinhole => 4,
            Self::SimpleRadial => 4,
            Self::Radial => 5,
            Self::OpenCv => 8,
            Self::OpenCvFisheye => 8,
            Self::FullOpenCv => 12,
            Self::Fov => 5,
            Self::SimpleRadialFisheye => 4,
            Self::RadialFisheye => 5,
            Self::ThinPrismFisheye => 12,
            Self::RadTanThinPrismFisheye => 16,
            Self::SimpleDivision => 4,
            Self::Division => 5, // f,cx,cy,k1,k2? Actually division has 4? check: SIMPLE_DIVISION 4; DIVISION maybe 5? Let's set 5 placeholder
            Self::SimpleFisheye => 3,
            Self::Fisheye => 4,
            Self::Eucm => 6,
            Self::Equirectangular => 0,
            Self::SkewedPinhole => 5,
            Self::Invalid => 0,
        }
    }
}

/// A camera instance.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Camera {
    pub model: CameraModelKind,
    pub params: Vec<f64>,
    pub width: u32,
    pub height: u32,
}

impl Camera {
    pub fn new(model: CameraModelKind, params: Vec<f64>, width: u32, height: u32) -> Result<Self> {
        if params.len() != model.num_params() {
            return Err(ColmapError::InvalidParamCount {
                expected: model.num_params(),
                got: params.len(),
            });
        }
        Ok(Self {
            model,
            params,
            width,
            height,
        })
    }

    /// Project a 3D point in camera coordinates to pixel (x, y). Returns None if not projectable.
    pub fn project(&self, p_cam: &Vector3<f64>) -> Option<(f64, f64)> {
        let p = self.params.as_slice();
        match self.model {
            CameraModelKind::SimplePinhole => {
                if p_cam.z <= 0.0 {
                    return None;
                } // cheirality handled by HasProjectableDepth (w>0)
                  // HasProjectableDepth: w > 0 if check_cheirality; COLMAP default true
                if p_cam[2] <= 0.0 {
                    return None;
                }
                let f = p[0];
                let cx = p[1];
                let cy = p[2];
                let x = f * p_cam.x / p_cam.z + cx;
                let y = f * p_cam.y / p_cam.z + cy;
                Some((x, y))
            }
            CameraModelKind::Pinhole => {
                if p_cam[2] <= 0.0 {
                    return None;
                }
                let fx = p[0];
                let fy = p[1];
                let cx = p[2];
                let cy = p[3];
                Some((fx * p_cam.x / p_cam.z + cx, fy * p_cam.y / p_cam.z + cy))
            }
            CameraModelKind::SimpleRadial => proj_simple_radial(p, p_cam),
            CameraModelKind::Radial => proj_radial(p, p_cam),
            CameraModelKind::OpenCv => proj_opencv(p, p_cam),
            CameraModelKind::FullOpenCv => proj_full_opencv(p, p_cam),
            CameraModelKind::Fov => proj_fov(p, p_cam),
            CameraModelKind::SimpleRadialFisheye => proj_simple_radial_fisheye(p, p_cam),
            CameraModelKind::RadialFisheye => proj_radial_fisheye(p, p_cam),
            CameraModelKind::ThinPrismFisheye => proj_thin_prism_fisheye(p, p_cam),
            CameraModelKind::OpenCvFisheye => proj_opencv_fisheye(p, p_cam),
            CameraModelKind::SimpleFisheye => proj_simple_fisheye(p, p_cam),
            CameraModelKind::Fisheye => proj_fisheye(p, p_cam),
            _ => None,
        }
    }

    /// Jacobian of projection (x,y) w.r.t. 3D point in camera coordinates.
    /// Returns [[dx/dX, dx/dY, dx/dZ], [dy/dX, dy/dY, dy/dZ]] or None.
    pub fn jacobian_point(&self, p_cam: &Vector3<f64>) -> Option<[[f64; 3]; 2]> {
        let p = self.params.as_slice();
        match self.model {
            CameraModelKind::SimplePinhole => {
                if p_cam[2] <= 0.0 {
                    return None;
                }
                let f = p[0];
                let z = p_cam[2];
                let x = p_cam[0];
                let y = p_cam[1];
                Some([
                    [f / z, 0.0, -f * x / (z * z)],
                    [0.0, f / z, -f * y / (z * z)],
                ])
            }
            CameraModelKind::Pinhole => {
                if p_cam[2] <= 0.0 {
                    return None;
                }
                let fx = p[0];
                let fy = p[1];
                let z = p_cam[2];
                let x = p_cam[0];
                let y = p_cam[1];
                Some([
                    [fx / z, 0.0, -fx * x / (z * z)],
                    [0.0, fy / z, -fy * y / (z * z)],
                ])
            }
            _ => None,
        }
    }

    /// Unproject a pixel to a normalized camera ray (x, y, z) with z=1 for perspective.
    pub fn unproject(&self, x: f64, y: f64) -> Option<Vector3<f64>> {
        let p = self.params.as_slice();
        match self.model {
            CameraModelKind::SimplePinhole => {
                let f = p[0];
                let cx = p[1];
                let cy = p[2];
                Some(Vector3::new((x - cx) / f, (y - cy) / f, 1.0))
            }
            CameraModelKind::Pinhole => {
                let fx = p[0];
                let fy = p[1];
                let cx = p[2];
                let cy = p[3];
                Some(Vector3::new((x - cx) / fx, (y - cy) / fy, 1.0))
            }
            CameraModelKind::SimpleRadial => unproj_simple_radial(p, x, y),
            CameraModelKind::Radial => unproj_radial(p, x, y),
            CameraModelKind::OpenCv => unproj_opencv(p, x, y),
            CameraModelKind::FullOpenCv => unproj_full_opencv(p, x, y),
            CameraModelKind::Fov => unproj_fov(p, x, y),
            CameraModelKind::SimpleRadialFisheye => unproj_simple_radial_fisheye(p, x, y),
            CameraModelKind::RadialFisheye => unproj_radial_fisheye(p, x, y),
            CameraModelKind::ThinPrismFisheye => unproj_thin_prism_fisheye(p, x, y),
            CameraModelKind::OpenCvFisheye => unproj_opencv_fisheye(p, x, y),
            CameraModelKind::SimpleFisheye => unproj_simple_fisheye(p, x, y),
            CameraModelKind::Fisheye => unproj_fisheye(p, x, y),
            _ => None,
        }
    }
}

// ---- perspective helpers ----

fn has_depth(w: f64) -> bool {
    w > 0.0
}

fn proj_simple_radial(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let k = p[3];
    let uu = pc.x / pc.z;
    let vv = pc.y / pc.z;
    let r2 = uu * uu + vv * vv;
    let radial = k * r2;
    let x = uu * (1.0 + radial);
    let y = vv * (1.0 + radial);
    Some((f * x + cx, f * y + cy))
}
// Wait: COLMAP SIMPLE_RADIAL Distortion: du = u * k*r2; *x = u + du = u*(1+k*r2). Yes.

fn proj_radial(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let k1 = p[3];
    let k2 = p[4];
    let uu = pc.x / pc.z;
    let vv = pc.y / pc.z;
    let r2 = uu * uu + vv * vv;
    let radial = k1 * r2 + k2 * r2 * r2;
    let x = uu * (1.0 + radial);
    let y = vv * (1.0 + radial);
    Some((f * x + cx, f * y + cy))
}

fn proj_opencv(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let k1 = p[4];
    let k2 = p[5];
    let pp1 = p[6];
    let pp2 = p[7];
    let uu = pc.x / pc.z;
    let vv = pc.y / pc.z;
    let u2 = uu * uu;
    let v2 = vv * vv;
    let uv = uu * vv;
    let r2 = u2 + v2;
    let radial = k1 * r2 + k2 * r2 * r2;
    let dx = uu * radial + 2.0 * pp1 * uv + pp2 * (r2 + 2.0 * u2);
    let dy = vv * radial + 2.0 * pp2 * uv + pp1 * (r2 + 2.0 * v2);
    Some((f1 * (uu + dx) + c1, f2 * (vv + dy) + c2))
}

fn proj_full_opencv(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let k1 = p[4];
    let k2 = p[5];
    let pp1 = p[6];
    let pp2 = p[7];
    let k3 = p[8];
    let k4 = p[9];
    let k5 = p[10];
    let k6 = p[11];
    let uu = pc.x / pc.z;
    let vv = pc.y / pc.z;
    let u2 = uu * uu;
    let v2 = vv * vv;
    let uv = uu * vv;
    let r2 = u2 + v2;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) / (1.0 + k4 * r2 + k5 * r4 + k6 * r6);
    // du = u*radial + tangential - u  (COLMAP form)
    let dx = uu * radial + 2.0 * pp1 * uv + pp2 * (r2 + 2.0 * u2) - uu;
    let dy = vv * radial + 2.0 * pp2 * uv + pp1 * (r2 + 2.0 * v2) - vv;
    Some((f1 * (uu + dx) + c1, f2 * (vv + dy) + c2))
}

fn proj_fov(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let omega = p[4];
    let uu = pc.x / pc.z;
    let vv = pc.y / pc.z;
    // Distortion returns (du,dv) = (u*factor, v*factor)
    let r2 = uu * uu + vv * vv;
    let omega2 = omega * omega;
    let r = r2.sqrt();
    let tan_half = (omega / 2.0).tan();
    let factor = if omega2 < 1e-4 {
        omega2 / 12.0 - (omega2 * r2) / 3.0 + 1.0
    } else if r2 < 1e-4 {
        (-2.0 * tan_half * (4.0 * r2 * tan_half * tan_half - 3.0)) / (3.0 * omega)
    } else {
        (r * 2.0 * tan_half).atan() / (r * omega)
    };
    let dx = uu * factor;
    let dy = vv * factor;
    Some((f1 * dx + c1, f2 * dy + c2))
}

// fisheye helpers
fn fisheye_from_normal(u: f64, v: f64) -> (f64, f64) {
    let r = (u * u + v * v).sqrt();
    if r > f64::EPSILON {
        let theta = r.atan();
        (u * theta / r, v * theta / r)
    } else {
        (u, v)
    }
}
fn normal_from_fisheye(uu: f64, vv: f64) -> (f64, f64) {
    let theta = (uu * uu + vv * vv).sqrt();
    let tc = theta * theta.cos();
    if tc > f64::EPSILON {
        let scale = theta.sin() / tc;
        (uu * scale, vv * scale)
    } else {
        (uu, vv)
    }
}

fn proj_simple_fisheye(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let (uu, vv) = fisheye_from_normal(pc.x / pc.z, pc.y / pc.z);
    Some((f * uu + cx, f * vv + cy))
}
fn proj_fisheye(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let (uu, vv) = fisheye_from_normal(pc.x / pc.z, pc.y / pc.z);
    Some((f1 * uu + c1, f2 * vv + c2))
}

fn proj_simple_radial_fisheye(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let k = p[3];
    let (uu, vv) = fisheye_from_normal(pc.x / pc.z, pc.y / pc.z);
    let theta2 = uu * uu + vv * vv;
    let radial = k * theta2;
    let x = uu * (1.0 + radial);
    let y = vv * (1.0 + radial);
    Some((f * x + cx, f * y + cy))
}
fn proj_radial_fisheye(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let k1 = p[3];
    let k2 = p[4];
    let (uu, vv) = fisheye_from_normal(pc.x / pc.z, pc.y / pc.z);
    let theta2 = uu * uu + vv * vv;
    let theta4 = theta2 * theta2;
    let radial = k1 * theta2 + k2 * theta4;
    let x = uu * (1.0 + radial);
    let y = vv * (1.0 + radial);
    Some((f * x + cx, f * y + cy))
}
fn proj_opencv_fisheye(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let k1 = p[4];
    let k2 = p[5];
    let k3 = p[6];
    let k4 = p[7];
    let (uu, vv) = fisheye_from_normal(pc.x / pc.z, pc.y / pc.z);
    let theta2 = uu * uu + vv * vv;
    let theta4 = theta2 * theta2;
    let theta6 = theta4 * theta2;
    let theta8 = theta4 * theta4;
    let radial = k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8;
    let x = uu * (1.0 + radial);
    let y = vv * (1.0 + radial);
    Some((f1 * x + c1, f2 * y + c2))
}
fn proj_thin_prism_fisheye(p: &[f64], pc: &Vector3<f64>) -> Option<(f64, f64)> {
    if pc[2] <= 0.0 {
        return None;
    }
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let k1 = p[4];
    let k2 = p[5];
    let pp1 = p[6];
    let pp2 = p[7];
    let k3 = p[8];
    let k4 = p[9];
    let sx1 = p[10];
    let sy1 = p[11];
    let (uu, vv) = fisheye_from_normal(pc.x / pc.z, pc.y / pc.z);
    let u2 = uu * uu;
    let v2 = vv * vv;
    let uv = uu * vv;
    let r2 = u2 + v2;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let r8 = r6 * r2;
    let radial = k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
    let dx = uu * radial + 2.0 * pp1 * uv + pp2 * (r2 + 2.0 * u2) + sx1 * r2;
    let dy = vv * radial + 2.0 * pp2 * uv + pp1 * (r2 + 2.0 * v2) + sy1 * r2;
    Some((f1 * (uu + dx) + c1, f2 * (vv + dy) + c2))
}

// ---- unproject helpers ----

fn iterative_undistort<F>(extra: &[f64], u: f64, v: f64, dist: F) -> Option<(f64, f64)>
where
    F: Fn(&[f64], f64, f64) -> (f64, f64),
{
    // solve u_dist = u + du(u,v); v_dist = v + dv(u,v)
    // given distorted (u,v) input, find undistorted (uu,vv) such that uu+du(uu,vv)=u
    let mut x = u;
    let mut y = v;
    for _ in 0..100 {
        let (du, dv) = dist(extra, x, y);
        // residual
        let rx = (x + du) - u;
        let ry = (y + dv) - v;
        // jacobian via finite differences
        let eps = 1e-6;
        let (du_px, dv_px) = dist(extra, x + eps, y);
        let (du_mx, dv_mx) = dist(extra, x - eps, y);
        let (du_py, dv_py) = dist(extra, x, y + eps);
        let (du_my, dv_my) = dist(extra, x, y - eps);
        let j00 = (du_px - du_mx) / (2.0 * eps) + 1.0;
        let j01 = (du_py - du_my) / (2.0 * eps);
        let j10 = (dv_px - dv_mx) / (2.0 * eps);
        let j11 = (dv_py - dv_my) / (2.0 * eps) + 1.0;
        let det = j00 * j11 - j01 * j10;
        if det.abs() < 1e-15 {
            return None;
        }
        let inv00 = j11 / det;
        let inv01 = -j01 / det;
        let inv10 = -j10 / det;
        let inv11 = j00 / det;
        let sx = inv00 * rx + inv01 * ry;
        let sy = inv10 * rx + inv11 * ry;
        // trust region
        let radius_sqr = (x * x + y * y) * 0.01_f64.max(0.01).max(0.01 * 0.01); // approx
        let step_norm = sx * sx + sy * sy;
        let (sx, sy) = if step_norm > radius_sqr {
            let s = (radius_sqr / step_norm).sqrt();
            (sx * s, sy * s)
        } else {
            (sx, sy)
        };
        x -= sx;
        y -= sy;
        if sx * sx + sy * sy < 1e-20 {
            break;
        }
    }
    Some((x, y))
}

fn unproj_simple_radial(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let u = (x - cx) / f;
    let v = (y - cy) / f;
    let extra = &p[3..];
    let dist = |extra: &[f64], uu: f64, vv: f64| {
        let k = extra[0];
        let r2 = uu * uu + vv * vv;
        (uu * k * r2, vv * k * r2)
    };
    let (uu, vv) = iterative_undistort(extra, u, v, dist)?;
    Some(Vector3::new(uu, vv, 1.0))
}
fn unproj_radial(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let u = (x - cx) / f;
    let v = (y - cy) / f;
    let extra = &p[3..];
    let dist = |extra: &[f64], uu: f64, vv: f64| {
        let k1 = extra[0];
        let k2 = extra[1];
        let r2 = uu * uu + vv * vv;
        let radial = k1 * r2 + k2 * r2 * r2;
        (uu * radial, vv * radial)
    };
    let (uu, vv) = iterative_undistort(extra, u, v, dist)?;
    Some(Vector3::new(uu, vv, 1.0))
}
fn unproj_opencv(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let u = (x - c1) / f1;
    let v = (y - c2) / f2;
    let extra = &p[4..];
    let dist = |extra: &[f64], uu: f64, vv: f64| {
        let k1 = extra[0];
        let k2 = extra[1];
        let pp1 = extra[2];
        let pp2 = extra[3];
        let u2 = uu * uu;
        let v2 = vv * vv;
        let uv = uu * vv;
        let r2 = u2 + v2;
        let radial = k1 * r2 + k2 * r2 * r2;
        let dx = uu * radial + 2.0 * pp1 * uv + pp2 * (r2 + 2.0 * u2);
        let dy = vv * radial + 2.0 * pp2 * uv + pp1 * (r2 + 2.0 * v2);
        (dx, dy)
    };
    let (uu, vv) = iterative_undistort(extra, u, v, dist)?;
    Some(Vector3::new(uu, vv, 1.0))
}
fn unproj_full_opencv(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let u = (x - c1) / f1;
    let v = (y - c2) / f2;
    let extra = &p[4..];
    let dist = |extra: &[f64], uu: f64, vv: f64| {
        let k1 = extra[0];
        let k2 = extra[1];
        let pp1 = extra[2];
        let pp2 = extra[3];
        let k3 = extra[4];
        let k4 = extra[5];
        let k5 = extra[6];
        let k6 = extra[7];
        let u2 = uu * uu;
        let v2 = vv * vv;
        let uv = uu * vv;
        let r2 = u2 + v2;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        let radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) / (1.0 + k4 * r2 + k5 * r4 + k6 * r6);
        let dx = uu * radial + 2.0 * pp1 * uv + pp2 * (r2 + 2.0 * u2) - uu;
        let dy = vv * radial + 2.0 * pp2 * uv + pp1 * (r2 + 2.0 * v2) - vv;
        (dx, dy)
    };
    let (uu, vv) = iterative_undistort(extra, u, v, dist)?;
    Some(Vector3::new(uu, vv, 1.0))
}

fn unproj_fov(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let uu = (x - c1) / f1;
    let vv = (y - c2) / f2;
    let omega = p[4];
    // inverse of Distortion factor
    let r2 = uu * uu + vv * vv;
    let omega2 = omega * omega;
    let factor = if omega2 < 1e-4 {
        (omega2 * r2) / 3.0 - omega2 / 12.0 + 1.0
    } else if r2 < 1e-4 {
        (omega * (omega * omega * r2 + 3.0)) / (6.0 * (omega / 2.0).tan())
    } else {
        let r = r2.sqrt();
        (r * 2.0 * (omega / 2.0).tan()).tan() / (r * omega)
    };
    Some(Vector3::new(uu * factor, vv * factor, 1.0))
}

// fisheye unproject: convert pixel to fisheye coords then NormalFromFisheye inverse?

fn unproj_simple_fisheye(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let uu = (x - cx) / f;
    let vv = (y - cy) / f;
    let (nx, ny) = normal_from_fisheye(uu, vv);
    Some(Vector3::new(nx, ny, 1.0))
}
fn unproj_fisheye(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let uu = (x - c1) / f1;
    let vv = (y - c2) / f2;
    let (nx, ny) = normal_from_fisheye(uu, vv);
    Some(Vector3::new(nx, ny, 1.0))
}
fn unproj_simple_radial_fisheye(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let uu = (x - cx) / f;
    let vv = (y - cy) / f;
    let extra = &p[3..];
    let dist = |extra: &[f64], a: f64, b: f64| {
        let k = extra[0];
        let t2 = a * a + b * b;
        (a * k * t2, b * k * t2)
    };
    let (a, b) = iterative_undistort(extra, uu, vv, dist)?;
    let (nx, ny) = normal_from_fisheye(a, b);
    Some(Vector3::new(nx, ny, 1.0))
}
fn unproj_radial_fisheye(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f = p[0];
    let cx = p[1];
    let cy = p[2];
    let uu = (x - cx) / f;
    let vv = (y - cy) / f;
    let extra = &p[3..];
    let dist = |extra: &[f64], a: f64, b: f64| {
        let k1 = extra[0];
        let k2 = extra[1];
        let t2 = a * a + b * b;
        let t4 = t2 * t2;
        let radial = k1 * t2 + k2 * t4;
        (a * radial, b * radial)
    };
    let (a, b) = iterative_undistort(extra, uu, vv, dist)?;
    let (nx, ny) = normal_from_fisheye(a, b);
    Some(Vector3::new(nx, ny, 1.0))
}
fn unproj_opencv_fisheye(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let uu = (x - c1) / f1;
    let vv = (y - c2) / f2;
    let extra = &p[4..];
    let dist = |extra: &[f64], a: f64, b: f64| {
        let k1 = extra[0];
        let k2 = extra[1];
        let k3 = extra[2];
        let k4 = extra[3];
        let t2 = a * a + b * b;
        let t4 = t2 * t2;
        let t6 = t4 * t2;
        let t8 = t4 * t4;
        let radial = k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8;
        (a * radial, b * radial)
    };
    let (a, b) = iterative_undistort(extra, uu, vv, dist)?;
    let (nx, ny) = normal_from_fisheye(a, b);
    Some(Vector3::new(nx, ny, 1.0))
}
fn unproj_thin_prism_fisheye(p: &[f64], x: f64, y: f64) -> Option<Vector3<f64>> {
    let f1 = p[0];
    let f2 = p[1];
    let c1 = p[2];
    let c2 = p[3];
    let uu = (x - c1) / f1;
    let vv = (y - c2) / f2;
    let extra = &p[4..];
    let dist = |extra: &[f64], a: f64, b: f64| {
        let k1 = extra[0];
        let k2 = extra[1];
        let pp1 = extra[2];
        let pp2 = extra[3];
        let k3 = extra[4];
        let k4 = extra[5];
        let sx1 = extra[6];
        let sy1 = extra[7];
        let u2 = a * a;
        let v2 = b * b;
        let uv = a * b;
        let r2 = u2 + v2;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        let r8 = r6 * r2;
        let radial = k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
        let dx = a * radial + 2.0 * pp1 * uv + pp2 * (r2 + 2.0 * u2) + sx1 * r2;
        let dy = b * radial + 2.0 * pp2 * uv + pp1 * (r2 + 2.0 * v2) + sy1 * r2;
        (dx, dy)
    };
    let (a, b) = iterative_undistort(extra, uu, vv, dist)?;
    let (nx, ny) = normal_from_fisheye(a, b);
    Some(Vector3::new(nx, ny, 1.0))
}

// unused helper to silence
#[allow(dead_code)]
fn _has_depth(w: f64) -> bool {
    has_depth(w)
}

impl fmt::Display for CameraModelKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl FromStr for CameraModelKind {
    type Err = String;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        Self::from_name(s).ok_or_else(|| format!("unknown camera model: {}", s))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn simple_pinhole_roundtrip() {
        let cam = Camera::new(
            CameraModelKind::SimplePinhole,
            vec![100.0, 50.0, 60.0],
            100,
            100,
        )
        .unwrap();
        let p = Vector3::new(0.3, -0.2, 2.0);
        let (x, y) = cam.project(&p).unwrap();
        let back = cam.unproject(x, y).unwrap();
        assert!((back.x - p.x / p.z).abs() < 1e-9);
        assert!((back.y - p.y / p.z).abs() < 1e-9);
    }
    #[test]
    fn opencv_roundtrip() {
        let cam = Camera::new(
            CameraModelKind::OpenCv,
            vec![100.0, 100.0, 50.0, 60.0, 0.1, -0.05, 0.01, -0.02],
            100,
            100,
        )
        .unwrap();
        let p = Vector3::new(0.2, 0.1, 1.5);
        let (x, y) = cam.project(&p).unwrap();
        let back = cam.unproject(x, y).unwrap();
        let orig = (p.x / p.z, p.y / p.z);
        assert!((back.x - orig.0).abs() < 1e-6, "x {} vs {}", back.x, orig.0);
        assert!((back.y - orig.1).abs() < 1e-6);
    }

    #[test]
    fn jacobian_finite_diff() {
        let cam = Camera::new(
            CameraModelKind::Pinhole,
            vec![100.0, 90.0, 50.0, 60.0],
            100,
            100,
        )
        .unwrap();
        let p = nalgebra::Vector3::new(0.3, -0.2, 2.0);
        let jac = cam.jacobian_point(&p).unwrap();
        let eps = 1e-6;
        for k in 0..3 {
            let mut pp = p;
            let mut pm = p;
            pp[k] += eps;
            pm[k] -= eps;
            let (xp, yp) = cam.project(&pp).unwrap();
            let (xm, ym) = cam.project(&pm).unwrap();
            let dx = (xp - xm) / (2.0 * eps);
            let dy = (yp - ym) / (2.0 * eps);
            assert!(
                (jac[0][k] - dx).abs() < 1e-4,
                "dx k{}: {} vs {}",
                k,
                jac[0][k],
                dx
            );
            assert!(
                (jac[1][k] - dy).abs() < 1e-4,
                "dy k{}: {} vs {}",
                k,
                jac[1][k],
                dy
            );
        }
    }
}
