//! Essential matrix and relative pose recovery.

use nalgebra::{Matrix3, Vector3};

/// Essential matrix from fundamental matrix and intrinsics.
/// E = K2^T F K1. Here assume K1,K2 as 3x3.
pub fn essential_from_fundamental(
    f: &Matrix3<f64>,
    k1: &Matrix3<f64>,
    k2: &Matrix3<f64>,
) -> Matrix3<f64> {
    k2.transpose() * f * k1
}

/// Recover the four (R,t) candidates from an essential matrix.
/// Returns list of (R, t) where t is unit vector.
pub fn decompose_essential(e: &Matrix3<f64>) -> Vec<(Matrix3<f64>, Vector3<f64>)> {
    let svd = e.svd(true, true);
    let u = match svd.u {
        Some(x) => x,
        None => return Vec::new(),
    };
    let vt = match svd.v_t {
        Some(x) => x,
        None => return Vec::new(),
    };
    // Ensure det(U)=+1, det(V)=+1
    let mut u = u;
    let mut v = vt.transpose();
    if u.determinant() < 0.0 {
        u = -u;
    }
    if v.determinant() < 0.0 {
        v = -v;
    }
    let w = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    let w_t = w.transpose();
    let t = Vector3::new(u[(0, 2)], u[(1, 2)], u[(2, 2)]);
    let r1 = u * w * v.transpose();
    let r2 = u * w_t * v.transpose();
    vec![(r1, t), (r1, -t), (r2, t), (r2, -t)]
}

/// Cheirality check: count points with positive depth in both views.
pub fn count_cheiral(
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
    pts1: &[[f64; 2]],
    pts2: &[[f64; 2]],
    k1: &Matrix3<f64>,
    k2: &Matrix3<f64>,
) -> usize {
    let k1inv = k1.try_inverse().unwrap_or(Matrix3::identity());
    let k2inv = k2.try_inverse().unwrap_or(Matrix3::identity());
    let mut count = 0usize;
    for i in 0..pts1.len() {
        let p1 = Vector3::new(pts1[i][0], pts1[i][1], 1.0);
        let p2 = Vector3::new(pts2[i][0], pts2[i][1], 1.0);
        let x1 = k1inv * p1;
        let x2 = k2inv * p2;
        // depth in cam1: lambda1 such that X = lambda1 x1 in cam1, and in cam2: lambda2 x2 = R (lambda1 x1) + t
        // Solve lambda1 x1? Use cross product: t x (R x1) . (x2 x R x1)? Standard: lambda2 = (t x R x1) . (x2 x R x1) / |x2 x R x1|^2
        let rx1 = r * x1;
        let cross = x2.cross(&rx1);
        let denom = cross.dot(&cross);
        if denom < 1e-12 {
            continue;
        }
        // lambda1 = (t x x2) . (R x1 x x2) / |R x1 x x2|^2 ?
        let a = rx1.cross(&x2);
        let ad = a.dot(&a);
        if ad < 1e-12 {
            continue;
        }
        let lambda1 = (t.cross(&x2)).dot(&a) / ad;
        let lambda2 = (t.cross(&rx1)).dot(&a) / ad; // approximate
        let _ = lambda2;
        if lambda1 > 0.0 {
            // depth in cam2
            let x_cam2 = rx1 * lambda1 + t;
            if x_cam2[2] > 0.0 {
                count += 1;
            }
        }
    }
    count
}
