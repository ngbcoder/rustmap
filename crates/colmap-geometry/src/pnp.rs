//! Absolute pose (PnP) solvers.

use crate::ransac::{ransac, Estimator, RansacOptions, RansacResult};
use colmap_core::geometry::Rigid3;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

/// DLT-based absolute pose from 3D-2D correspondences.
pub fn pnp_dlt(pts3d: &[[f64; 3]], pts2d: &[[f64; 2]], k: &Matrix3<f64>) -> Option<Rigid3> {
    if pts3d.len() < 6 || pts3d.len() != pts2d.len() {
        return None;
    }
    let n = pts3d.len();
    let kinv = k.try_inverse()?;
    let mut norm2d = Vec::with_capacity(n);
    for p in pts2d {
        let v = kinv * Vector3::new(p[0], p[1], 1.0);
        norm2d.push([v[0] / v[2], v[1] / v[2]]);
    }

    let mut a = nalgebra::DMatrix::zeros(2 * n, 12);
    for i in 0..n {
        let x = pts3d[i][0];
        let y = pts3d[i][1];
        let z = pts3d[i][2];
        let u = norm2d[i][0];
        let v = norm2d[i][1];
        a[(2 * i, 0)] = x;
        a[(2 * i, 1)] = y;
        a[(2 * i, 2)] = z;
        a[(2 * i, 3)] = 1.0;
        a[(2 * i, 8)] = -u * x;
        a[(2 * i, 9)] = -u * y;
        a[(2 * i, 10)] = -u * z;
        a[(2 * i, 11)] = -u;
        a[(2 * i + 1, 4)] = x;
        a[(2 * i + 1, 5)] = y;
        a[(2 * i + 1, 6)] = z;
        a[(2 * i + 1, 7)] = 1.0;
        a[(2 * i + 1, 8)] = -v * x;
        a[(2 * i + 1, 9)] = -v * y;
        a[(2 * i + 1, 10)] = -v * z;
        a[(2 * i + 1, 11)] = -v;
    }
    let svd = a.svd(false, true);
    let vt = svd.v_t?;
    let pvec = vt.row(vt.nrows() - 1);
    let mut m = nalgebra::DMatrix::zeros(3, 4);
    for i in 0..3 {
        for j in 0..4 {
            m[(i, j)] = pvec[i * 4 + j];
        }
    }
    let mut m3m = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            m3m[(i, j)] = m[(i, j)];
        }
    }
    let mut vt3 = Vector3::zeros();
    for i in 0..3 {
        vt3[i] = m[(i, 3)];
    }
    if m3m.determinant() < 0.0 {
        m3m = -m3m;
        vt3 = -vt3;
    }
    let svd_m = m3m.svd(true, true);
    let u = svd_m.u?;
    let vt = svd_m.v_t?;
    let r = u * vt;
    let trace = (r.transpose() * m3m).trace();
    let s = trace / 3.0;
    if s.abs() < 1e-12 {
        return None;
    }
    let t = vt3 / s;
    let rot = UnitQuaternion::from_matrix(&r);
    Some(Rigid3 {
        rotation: rot,
        translation: t,
    })
}

pub struct PnpEstimator<'a> {
    pub pts3d: &'a [[f64; 3]],
    pub pts2d: &'a [[f64; 2]],
    pub k: &'a Matrix3<f64>,
}

impl Estimator<usize> for PnpEstimator<'_> {
    type Model = Rigid3;

    fn min_sample_size(&self) -> usize {
        6
    }

    fn estimate(&self, _data: &[usize], sample: &[usize]) -> Vec<Self::Model> {
        let mut p3 = Vec::with_capacity(sample.len());
        let mut p2 = Vec::with_capacity(sample.len());
        for &i in sample {
            p3.push(self.pts3d[i]);
            p2.push(self.pts2d[i]);
        }
        match pnp_dlt(&p3, &p2, self.k) {
            Some(m) => vec![m],
            None => vec![],
        }
    }

    fn residuals(&self, model: &Self::Model, data: &[usize]) -> Vec<f64> {
        data.iter()
            .map(|&i| {
                let x = Vector3::new(self.pts3d[i][0], self.pts3d[i][1], self.pts3d[i][2]);
                let pc = model.rotation.to_rotation_matrix() * x + model.translation;
                if pc[2] <= 0.0 {
                    return 1e6;
                }
                let p = self.k * pc;
                let px = p[0] / p[2];
                let py = p[1] / p[2];
                let dx = px - self.pts2d[i][0];
                let dy = py - self.pts2d[i][1];
                (dx * dx + dy * dy).sqrt()
            })
            .collect()
    }
}

pub fn pnp_ransac(
    pts3d: &[[f64; 3]],
    pts2d: &[[f64; 2]],
    k: &Matrix3<f64>,
    opts: &RansacOptions,
) -> Option<RansacResult<Rigid3>> {
    let est = PnpEstimator { pts3d, pts2d, k };
    let data: Vec<usize> = (0..pts3d.len()).collect();
    ransac(&est, &data, opts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ransac::RansacOptions;

    #[test]
    fn pnp_identity() {
        let k = Matrix3::identity();
        let pts3d: Vec<[f64; 3]> = vec![
            [1.0, 2.0, 5.0],
            [-1.0, 0.5, 4.0],
            [0.3, -1.2, 6.0],
            [2.0, 1.0, 3.0],
            [-0.5, -0.5, 7.0],
            [1.5, -2.0, 4.5],
        ];
        let pts2d: Vec<[f64; 2]> = pts3d.iter().map(|p| [p[0] / p[2], p[1] / p[2]]).collect();
        let pose = pnp_dlt(&pts3d, &pts2d, &k).unwrap();
        assert!(pose.translation.norm() < 1e-6, "t={}", pose.translation);
        let r = pose.rotation.to_rotation_matrix();
        assert!((r.matrix()[(0, 0)] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn pnp_ransac_basic() {
        let k = Matrix3::identity();
        let pts3d: Vec<[f64; 3]> = vec![
            [1.0, 2.0, 5.0],
            [-1.0, 0.5, 4.0],
            [0.3, -1.2, 6.0],
            [2.0, 1.0, 3.0],
            [-0.5, -0.5, 7.0],
            [1.5, -2.0, 4.5],
            [0.0, 0.0, 10.0],
        ];
        let mut pts2d: Vec<[f64; 2]> = pts3d.iter().map(|p| [p[0] / p[2], p[1] / p[2]]).collect();
        pts2d[6] = [10.0, 10.0];
        let opts = RansacOptions {
            threshold: 1e-2,
            min_inliers: 5,
            ..Default::default()
        };
        let res = pnp_ransac(&pts3d, &pts2d, &k, &opts).unwrap();
        assert!(res.inliers.len() >= 5);
    }
}
