//! Two-view geometry classification (simplified).

use crate::fundamental::fundamental_8point;
use crate::homography::homography_dlt;
use crate::ransac::{ransac, Estimator, RansacOptions, RansacResult};
use colmap_core::camera::Camera;
use nalgebra::Matrix3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TwoViewType {
    Calibrated,
    Uncalibrated,
    Planar,
    Panoramic,
    Watermark,
    Invalid,
}

#[derive(Clone, Debug)]
pub struct TwoViewGeometry {
    pub ty: TwoViewType,
    pub inliers: Vec<usize>,
    pub f: Option<Matrix3<f64>>,
    pub h: Option<Matrix3<f64>>,
}

// ---- Fundamental RANSAC ----

pub struct FundamentalEstimator<'a> {
    pub pts1: &'a [[f64; 2]],
    pub pts2: &'a [[f64; 2]],
}

impl Estimator<usize> for FundamentalEstimator<'_> {
    type Model = Matrix3<f64>;

    fn min_sample_size(&self) -> usize {
        8
    }

    fn estimate(&self, _data: &[usize], sample: &[usize]) -> Vec<Self::Model> {
        let mut p1 = Vec::with_capacity(sample.len());
        let mut p2 = Vec::with_capacity(sample.len());
        for &i in sample {
            p1.push(self.pts1[i]);
            p2.push(self.pts2[i]);
        }
        match fundamental_8point(&p1, &p2) {
            Some(f) => vec![f],
            None => vec![],
        }
    }

    fn residuals(&self, model: &Self::Model, data: &[usize]) -> Vec<f64> {
        data.iter()
            .map(|&i| {
                let x1 = nalgebra::Vector3::new(self.pts1[i][0], self.pts1[i][1], 1.0);
                let x2 = nalgebra::Vector3::new(self.pts2[i][0], self.pts2[i][1], 1.0);
                let fx1 = model * x1;
                let ftx2 = model.transpose() * x2;
                let num = x2.dot(&(model * x1));
                let den =
                    (fx1[0] * fx1[0] + fx1[1] * fx1[1] + ftx2[0] * ftx2[0] + ftx2[1] * ftx2[1])
                        .sqrt();
                if den < 1e-12 {
                    return 1e6;
                }
                (num / den).abs()
            })
            .collect()
    }
}

pub fn fundamental_ransac(
    pts1: &[[f64; 2]],
    pts2: &[[f64; 2]],
    opts: &RansacOptions,
) -> Option<RansacResult<Matrix3<f64>>> {
    let est = FundamentalEstimator { pts1, pts2 };
    let data: Vec<usize> = (0..pts1.len()).collect();
    ransac(&est, &data, opts)
}

// ---- Homography RANSAC ----

pub struct HomographyEstimator<'a> {
    pub pts1: &'a [[f64; 2]],
    pub pts2: &'a [[f64; 2]],
}

impl Estimator<usize> for HomographyEstimator<'_> {
    type Model = Matrix3<f64>;

    fn min_sample_size(&self) -> usize {
        4
    }

    fn estimate(&self, _data: &[usize], sample: &[usize]) -> Vec<Self::Model> {
        let mut p1 = Vec::with_capacity(sample.len());
        let mut p2 = Vec::with_capacity(sample.len());
        for &i in sample {
            p1.push(self.pts1[i]);
            p2.push(self.pts2[i]);
        }
        let hh = homography_dlt(&p1, &p2);
        match hh {
            Some(h) => vec![h],
            None => vec![],
        }
    }

    fn residuals(&self, model: &Self::Model, data: &[usize]) -> Vec<f64> {
        data.iter()
            .map(|&i| {
                let x1 = nalgebra::Vector3::new(self.pts1[i][0], self.pts1[i][1], 1.0);
                let hp = model * x1;
                if hp[2].abs() < 1e-12 {
                    return 1e6;
                }
                let px = hp[0] / hp[2];
                let py = hp[1] / hp[2];
                let dx = px - self.pts2[i][0];
                let dy = py - self.pts2[i][1];
                (dx * dx + dy * dy).sqrt()
            })
            .collect()
    }
}

pub fn homography_ransac(
    pts1: &[[f64; 2]],
    pts2: &[[f64; 2]],
    opts: &RansacOptions,
) -> Option<RansacResult<Matrix3<f64>>> {
    let est = HomographyEstimator { pts1, pts2 };
    let data: Vec<usize> = (0..pts1.len()).collect();
    ransac(&est, &data, opts)
}

pub fn estimate_two_view_geometry(
    pts1: &[[f64; 2]],
    pts2: &[[f64; 2]],
    _cam1: &Camera,
    _cam2: &Camera,
    opts: &RansacOptions,
) -> TwoViewGeometry {
    let f_res = fundamental_ransac(pts1, pts2, opts);
    let h_res = homography_ransac(pts1, pts2, opts);

    let n_f = f_res.as_ref().map(|r| r.inliers.len()).unwrap_or(0);
    let n_h = h_res.as_ref().map(|r| r.inliers.len()).unwrap_or(0);

    let ty = if n_f == 0 && n_h == 0 {
        TwoViewType::Invalid
    } else if n_h as f64 > 0.9 * (n_f.max(n_h) as f64) && n_h > n_f {
        TwoViewType::Planar
    } else {
        // assume intrinsics available -> calibrated
        TwoViewType::Calibrated
    };

    // inliers: prefer F inliers if calibrated, else H
    let inliers = if ty == TwoViewType::Planar {
        h_res
            .as_ref()
            .map(|r| r.inliers.clone())
            .unwrap_or_default()
    } else {
        f_res
            .as_ref()
            .map(|r| r.inliers.clone())
            .unwrap_or_default()
    };

    TwoViewGeometry {
        ty,
        inliers,
        f: f_res.map(|r| r.model),
        h: h_res.map(|r| r.model),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ransac::RansacOptions;

    #[test]
    fn homography_ransac_planar() {
        // Points on plane z=1, view1 identity, view2 with H
        let h = Matrix3::new(1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        let pts1: Vec<[f64; 2]> = (0..20)
            .map(|i| [i as f64 * 0.1, (i as f64 * 0.37).sin()])
            .collect();
        let pts2: Vec<[f64; 2]> = pts1
            .iter()
            .map(|p| {
                let v = h * nalgebra::Vector3::new(p[0], p[1], 1.0);
                [v[0] / v[2], v[1] / v[2]]
            })
            .collect();
        let opts = RansacOptions {
            threshold: 1e-3,
            min_inliers: 10,
            ..Default::default()
        };
        let res = homography_ransac(&pts1, &pts2, &opts).unwrap();
        assert!(res.inliers.len() >= 10);
    }
}
