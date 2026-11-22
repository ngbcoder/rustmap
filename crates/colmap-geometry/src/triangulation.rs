//! Triangulation helpers.

use colmap_core::camera::Camera;
use colmap_core::geometry::Rigid3;
use nalgebra::{Matrix3x4, Matrix4, Vector3};

/// DLT triangulation from two views. Poses are world->camera (R,t).
/// Returns 3D point in world coordinates, or None if degenerate.
#[allow(clippy::too_many_arguments)]
pub fn triangulate_dlt_two_view(
    pose1: &Rigid3,
    cam1: &Camera,
    px1: f64,
    py1: f64,
    pose2: &Rigid3,
    cam2: &Camera,
    px2: f64,
    py2: f64,
) -> Option<Vector3<f64>> {
    let n1 = cam1.unproject(px1, py1)?;
    let n2 = cam2.unproject(px2, py2)?;

    // Projection matrices P = [R | t] (3x4) in normalized camera space.
    let r1 = pose1.rotation.to_rotation_matrix();
    let r2 = pose2.rotation.to_rotation_matrix();
    let mut p1 = Matrix3x4::<f64>::zeros();
    for i in 0..3 {
        for j in 0..3 {
            p1[(i, j)] = r1.matrix()[(i, j)];
        }
        p1[(i, 3)] = pose1.translation[i];
    }
    let mut p2 = Matrix3x4::<f64>::zeros();
    for i in 0..3 {
        for j in 0..3 {
            p2[(i, j)] = r2.matrix()[(i, j)];
        }
        p2[(i, 3)] = pose2.translation[i];
    }

    // Build 4x4 A: two rows per view from cross product.
    let mut a = Matrix4::<f64>::zeros();
    // view 1
    let u1 = n1;
    // row0: u1[1]*P[2] - u1[2]*P[1]; row1: u1[2]*P[0]-u1[0]*P[2]
    for j in 0..4 {
        a[(0, j)] = u1[1] * p1[(2, j)] - u1[2] * p1[(1, j)];
        a[(1, j)] = u1[2] * p1[(0, j)] - u1[0] * p1[(2, j)];
    }
    for j in 0..4 {
        a[(2, j)] = n2[1] * p2[(2, j)] - n2[2] * p2[(1, j)];
        a[(3, j)] = n2[2] * p2[(0, j)] - n2[0] * p2[(2, j)];
    }

    let svd = a.svd(true, true);
    let v = svd.v_t?; // V^T, rows are right singular vectors
                      // smallest singular value -> last row of V^T
    let x = v.row(3);
    if x[3].abs() < 1e-12 {
        return None;
    }
    let pt = Vector3::new(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
    // cheirality: point should be in front of both cameras
    let c1 = pose1.transform_point(&pt); // world->cam? Actually pose transform_point applies R*p+t
                                         // Wait Rigid3::transform_point does R*p + t (world->cam). So z should be >0.
    if c1[2] <= 0.0 {
        return None;
    }
    let c2 = pose2.transform_point(&pt);
    if c2[2] <= 0.0 {
        return None;
    }
    Some(pt)
}

/// Back-compat alias.
#[allow(clippy::too_many_arguments)]
pub fn triangulate_dlt(
    pose1: &Rigid3,
    cam1: &Camera,
    px1: f64,
    py1: f64,
    pose2: &Rigid3,
    cam2: &Camera,
    px2: f64,
    py2: f64,
) -> Option<Vector3<f64>> {
    triangulate_dlt_two_view(pose1, cam1, px1, py1, pose2, cam2, px2, py2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use colmap_core::camera::{Camera, CameraModelKind};
    use colmap_core::geometry::Rigid3;
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn triangulate_identity() {
        let cam = Camera::new(CameraModelKind::Pinhole, vec![1.0, 1.0, 0.0, 0.0], 10, 10).unwrap();
        let pose1 = Rigid3::identity();
        let pose2 = Rigid3 {
            rotation: UnitQuaternion::identity(),
            translation: Vector3::new(-1.0, 0.0, 0.0),
        };
        let pt = triangulate_dlt_two_view(&pose1, &cam, 0.0, 0.0, &pose2, &cam, -0.2, 0.0).unwrap();
        assert!((pt.x).abs() < 1e-6);
        assert!((pt.y).abs() < 1e-6);
        assert!((pt.z - 5.0).abs() < 1e-6, "z={}", pt.z);
    }
}
