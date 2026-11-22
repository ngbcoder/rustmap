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
    let ata = a.transpose() * &a;
    let eig = ata.symmetric_eigen();
    // smallest eigenvalue -> last after sorting? symmetric_eigen returns ascending? Check: eigenvalues are sorted in ascending order.
    let mut idx = 0usize;
    let mut minval = eig.eigenvalues[0];
    for i in 1..9 {
        if eig.eigenvalues[i] < minval {
            minval = eig.eigenvalues[i];
            idx = i;
        }
    }
    let vec = eig.eigenvectors.column(idx);
    let mut h = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            h[(i, j)] = vec[i * 3 + j];
        }
    }
    if h.determinant().abs() < 1e-12 {
        return None;
    }
    Some(h)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn basic() {
        let h = Matrix3::new(1.2, 0.3, 5.0, -0.2, 0.9, -3.0, 0.001, 0.002, 1.0);
        let pts1 = vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 3.0]];
        let pts2: Vec<[f64; 2]> = pts1
            .iter()
            .map(|p| {
                let v = h * nalgebra::Vector3::new(p[0], p[1], 1.0);
                [v[0] / v[2], v[1] / v[2]]
            })
            .collect();
        let he = homography_dlt(&pts1[..4], &pts2[..4]).unwrap();
        // residual on 5th
        let x1 = nalgebra::Vector3::new(pts1[4][0], pts1[4][1], 1.0);
        let hp = he * x1;
        let (px, py) = (hp[0] / hp[2], hp[1] / hp[2]);
        assert!(((px - pts2[4][0]).powi(2) + (py - pts2[4][1]).powi(2)).sqrt() < 1e-6);
    }
}
