//! Bundle adjustment (dense, finite-difference-friendly analytic residuals).

use colmap_core::camera::Camera;
use colmap_core::geometry::Rigid3;
use nalgebra::{DVector, UnitQuaternion, Vector3};

#[derive(Clone, Debug)]
pub struct Observation {
    pub camera: usize,
    pub point: usize,
    pub xy: [f64; 2],
}

#[derive(Clone, Debug)]
pub struct BaProblem {
    pub cameras: Vec<Camera>,
    pub poses: Vec<Rigid3>, // world->camera, initial
    pub points: Vec<Vector3<f64>>,
    pub observations: Vec<Observation>,
    /// Whether to optimize camera intrinsics (not implemented: fixed).
    pub refine_intrinsics: bool,
}

#[derive(Clone, Debug)]
pub struct BaResult {
    pub poses: Vec<Rigid3>,
    pub points: Vec<Vector3<f64>>,
    pub cost: f64,
    pub iterations: usize,
}

/// Pack params: [pose0 (6: rotvec 3 + t 3), ..., point0 (3), ...].
fn pack(prob: &BaProblem) -> DVector<f64> {
    let ncam = prob.poses.len();
    let npt = prob.points.len();
    let mut x = DVector::zeros(ncam * 6 + npt * 3);
    for i in 0..ncam {
        let p = &prob.poses[i];
        // rotation vector from quaternion
        let rv = p.rotation.scaled_axis();
        x[i * 6] = rv.x;
        x[i * 6 + 1] = rv.y;
        x[i * 6 + 2] = rv.z;
        x[i * 6 + 3] = p.translation.x;
        x[i * 6 + 4] = p.translation.y;
        x[i * 6 + 5] = p.translation.z;
    }
    for (j, pt) in prob.points.iter().enumerate() {
        let base = ncam * 6 + j * 3;
        x[base] = pt.x;
        x[base + 1] = pt.y;
        x[base + 2] = pt.z;
    }
    x
}

fn unpack(x: &DVector<f64>, ncam: usize, npt: usize) -> (Vec<Rigid3>, Vec<Vector3<f64>>) {
    let mut poses = Vec::with_capacity(ncam);
    for i in 0..ncam {
        let rv = Vector3::new(x[i * 6], x[i * 6 + 1], x[i * 6 + 2]);
        let rot = UnitQuaternion::from_scaled_axis(rv);
        let t = Vector3::new(x[i * 6 + 3], x[i * 6 + 4], x[i * 6 + 5]);
        poses.push(Rigid3 {
            rotation: rot,
            translation: t,
        });
    }
    let mut points = Vec::with_capacity(npt);
    for j in 0..npt {
        let base = ncam * 6 + j * 3;
        points.push(Vector3::new(x[base], x[base + 1], x[base + 2]));
    }
    (poses, points)
}

fn residuals_vec(prob: &BaProblem, poses: &[Rigid3], points: &[Vector3<f64>]) -> DVector<f64> {
    let mut r = DVector::zeros(prob.observations.len() * 2);
    for (k, obs) in prob.observations.iter().enumerate() {
        let pc = poses[obs.camera].transform_point(&points[obs.point]);
        let cam = &prob.cameras[obs.camera];
        match cam.project(&pc) {
            Some((px, py)) => {
                r[k * 2] = obs.xy[0] - px;
                r[k * 2 + 1] = obs.xy[1] - py;
            }
            None => {
                r[k * 2] = 1e3;
                r[k * 2 + 1] = 1e3;
            }
        }
    }
    r
}

pub fn solve_ba(prob: &BaProblem, opts: &crate::lm::LmOptions) -> BaResult {
    let ncam = prob.poses.len();
    let npt = prob.points.len();
    let x0 = pack(prob);
    let probc = prob.clone();
    let residual = move |x: &DVector<f64>| -> DVector<f64> {
        let (poses, points) = unpack(x, ncam, npt);
        residuals_vec(&probc, &poses, &points)
    };
    let res = crate::lm::lm_solve(x0, residual, opts);
    let (poses, points) = unpack(&res.x, ncam, npt);
    BaResult {
        poses,
        points,
        cost: res.cost,
        iterations: res.iterations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use colmap_core::camera::{Camera, CameraModelKind};
    use nalgebra::Vector3;

    #[test]
    fn ba_perfect() {
        let cam = Camera::new(
            CameraModelKind::Pinhole,
            vec![100.0, 100.0, 50.0, 50.0],
            100,
            100,
        )
        .unwrap();
        let pose = Rigid3::identity();
        let pts = vec![
            Vector3::new(0.1, 0.2, 2.0),
            Vector3::new(-0.3, 0.1, 3.0),
            Vector3::new(0.2, -0.2, 2.5),
        ];
        // observations: project
        let mut obs = Vec::new();
        for (j, pt) in pts.iter().enumerate() {
            let pc = pose.transform_point(pt);
            let (x, y) = cam.project(&pc).unwrap();
            obs.push(Observation {
                camera: 0,
                point: j,
                xy: [x, y],
            });
        }
        let prob = BaProblem {
            cameras: vec![cam],
            poses: vec![pose],
            points: pts.clone(),
            observations: obs,
            refine_intrinsics: false,
        };
        let opts = crate::lm::LmOptions::default();
        let res = solve_ba(&prob, &opts);
        assert!(res.cost < 1e-6, "cost={}", res.cost);

        #[test]
        fn ba_recovers_perturbation() {
            let cam = Camera::new(
                CameraModelKind::Pinhole,
                vec![100.0, 100.0, 50.0, 50.0],
                100,
                100,
            )
            .unwrap();
            let pose = Rigid3::identity();
            let pts = vec![
                Vector3::new(0.1, 0.2, 2.0),
                Vector3::new(-0.3, 0.1, 3.0),
                Vector3::new(0.2, -0.2, 2.5),
            ];
            let mut obs = Vec::new();
            for (j, pt) in pts.iter().enumerate() {
                let pc = pose.transform_point(pt);
                let (x, y) = cam.project(&pc).unwrap();
                obs.push(Observation {
                    camera: 0,
                    point: j,
                    xy: [x, y],
                });
            }
            // perturb initial pose/points
            let mut pose0 = pose;
            pose0.translation.x += 0.1;
            let mut pts0 = pts.clone();
            pts0[0].x += 0.05;
            let prob = BaProblem {
                cameras: vec![cam],
                poses: vec![pose0],
                points: pts0,
                observations: obs,
                refine_intrinsics: false,
            };
            let cost_before = {
                let r = residuals_vec(&prob, &prob.poses, &prob.points);
                r.dot(&r)
            };
            let opts = crate::lm::LmOptions::default();
            let res = solve_ba(&prob, &opts);
            assert!(
                res.cost < cost_before,
                "cost {} not < {}",
                res.cost,
                cost_before
            );
        }
    }
}
