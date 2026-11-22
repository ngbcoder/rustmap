//! Fundamental matrix solvers.

use nalgebra::Matrix3;

/// Normalize a set of 2D points: translate centroid to origin, scale so mean
/// distance to origin is sqrt(2). Returns (normalized points, transform T).
fn normalize_points(pts: &[[f64; 2]]) -> (Vec<[f64; 3]>, Matrix3<f64>) {
    let n = pts.len() as f64;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for p in pts {
        cx += p[0];
        cy += p[1];
    }
    cx /= n;
    cy /= n;
    let mut dist = 0.0;
    for p in pts {
        let dx = p[0] - cx;
        let dy = p[1] - cy;
        dist += (dx * dx + dy * dy).sqrt();
    }
    dist /= n;
    let scale = if dist > 1e-12 {
        (2.0_f64).sqrt() / dist
    } else {
        1.0
    };
    let mut out = Vec::with_capacity(pts.len());
    for p in pts {
        out.push([scale * (p[0] - cx), scale * (p[1] - cy), 1.0]);
    }
    let t = Matrix3::new(
        scale,
        0.0,
        -scale * cx,
        0.0,
        scale,
        -scale * cy,
        0.0,
        0.0,
        1.0,
    );
    (out, t)
}

/// 8-point (or more) algorithm for the fundamental matrix.
/// `pts1`, `pts2`: correspondences (x,y). Returns F (3x3) mapping p2 ~ F p1,
/// or None if degenerate.
pub fn fundamental_8point(pts1: &[[f64; 2]], pts2: &[[f64; 2]]) -> Option<Matrix3<f64>> {
    if pts1.len() < 8 || pts1.len() != pts2.len() {
        return None;
    }
    let (n1, t1) = normalize_points(pts1);
    let (n2, t2) = normalize_points(pts2);

    // Build A: each row [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
    let m = pts1.len();
    let mut a = nalgebra::DMatrix::zeros(m, 9);
    for i in 0..m {
        let x1 = n1[i][0];
        let y1 = n1[i][1];
        let x2 = n2[i][0];
        let y2 = n2[i][1];
        a[(i, 0)] = x2 * x1;
        a[(i, 1)] = x2 * y1;
        a[(i, 2)] = x2;
        a[(i, 3)] = y2 * x1;
        a[(i, 4)] = y2 * y1;
        a[(i, 5)] = y2;
        a[(i, 6)] = x1;
        a[(i, 7)] = y1;
        a[(i, 8)] = 1.0;
    }
    // SVD of A; solution is last row of V^T (smallest singular value)
    let ata = a.transpose() * &a;
    let eig = ata.symmetric_eigen();
    let mut idx = 0usize;
    let mut minval = eig.eigenvalues[0];
    for i in 1..9 {
        if eig.eigenvalues[i] < minval {
            minval = eig.eigenvalues[i];
            idx = i;
        }
    }
    let f_vec = eig.eigenvectors.column(idx);
    let mut f = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            f[(i, j)] = f_vec[i * 3 + j];
        }
    }
    // Enforce rank 2
    let svd_f = f.svd(true, true);
    let u = svd_f.u?;
    let vt = svd_f.v_t?;
    let s = svd_f.singular_values;
    let s2 = nalgebra::Vector3::new(s[0], s[1], 0.0);
    let f_rank2 = u * nalgebra::Matrix3::from_diagonal(&s2) * vt;
    // Convert back to Matrix3
    let mut f2 = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            f2[(i, j)] = f_rank2[(i, j)];
        }
    }
    // Denormalize: F = T2^T F_norm T1
    let f_denorm = t2.transpose() * f2 * t1;
    Some(f_denorm)
}
