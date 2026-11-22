//! Homography solvers.

use nalgebra::Matrix3;

/// DLT homography from point correspondences (>=4). Returns H such that p2 ~ H p1.
pub fn homography_dlt(pts1: &[[f64; 2]], pts2: &[[f64; 2]]) -> Option<Matrix3<f64>> {
    if pts1.len() < 4 || pts1.len() != pts2.len() {
        return None;
    }
    let m = pts1.len();
    // Build 2m x 9 matrix A.
    let mut a = nalgebra::DMatrix::zeros(2 * m, 9);
    for i in 0..m {
        let x1 = pts1[i][0];
        let y1 = pts1[i][1];
        let x2 = pts2[i][0];
        let y2 = pts2[i][1];
        // row 2i: [-x1,-y1,-1, 0,0,0, x2*x1, x2*y1, x2]
        a[(2 * i, 0)] = -x1;
        a[(2 * i, 1)] = -y1;
        a[(2 * i, 2)] = -1.0;
        a[(2 * i, 6)] = x2 * x1;
        a[(2 * i, 7)] = x2 * y1;
        a[(2 * i, 8)] = x2;
        // row 2i+1: [0,0,0, -x1,-y1,-1, y2*x1, y2*y1, y2]
        a[(2 * i + 1, 3)] = -x1;
        a[(2 * i + 1, 4)] = -y1;
        a[(2 * i + 1, 5)] = -1.0;
        a[(2 * i + 1, 6)] = y2 * x1;
        a[(2 * i + 1, 7)] = y2 * y1;
        a[(2 * i + 1, 8)] = y2;
    }
    let svd = a.svd(false, true);
    let vt = svd.v_t?;
    let h_vec = vt.row(vt.nrows() - 1);
    let mut h = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            h[(i, j)] = h_vec[i * 3 + j];
        }
    }
    if h.determinant().abs() < 1e-12 {
        return None;
    }
    Some(h)
}
