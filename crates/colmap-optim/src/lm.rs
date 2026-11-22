//! Levenberg-Marquardt solver (dense, finite-difference Jacobian).

use nalgebra::{DMatrix, DVector};

/// Options for LM.
#[derive(Clone, Debug)]
pub struct LmOptions {
    pub max_iterations: usize,
    pub ftol: f64,
    pub xtol: f64,
    pub gtol: f64,
    pub lambda_init: f64,
    /// Finite-difference step for Jacobian.
    pub eps: f64,
}

impl Default for LmOptions {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            ftol: 1e-10,
            xtol: 1e-10,
            gtol: 1e-10,
            lambda_init: 1e-3,
            eps: 1e-6,
        }
    }
}

/// Result of LM.
#[derive(Clone, Debug)]
pub struct LmResult {
    pub x: DVector<f64>,
    pub cost: f64,
    pub iterations: usize,
    pub converged: bool,
}

/// Solve min_x ||r(x)||^2 via Levenberg-Marquardt.
/// `residual(x)` returns the residual vector.
pub fn lm_solve<F>(x0: DVector<f64>, residual: F, opts: &LmOptions) -> LmResult
where
    F: Fn(&DVector<f64>) -> DVector<f64>,
{
    let n = x0.len();
    let mut x = x0;
    let mut r = residual(&x);
    let m = r.len();
    let mut cost = r.dot(&r);
    let mut lambda = opts.lambda_init;

    let mut converged = false;
    let mut iters = 0usize;
    for _ in 0..opts.max_iterations {
        iters += 1;
        // Jacobian via central differences
        let mut j = DMatrix::zeros(m, n);
        for k in 0..n {
            let mut xp = x.clone();
            let mut xm = x.clone();
            xp[k] += opts.eps;
            xm[k] -= opts.eps;
            let rp = residual(&xp);
            let rm = residual(&xm);
            for i in 0..m {
                j[(i, k)] = (rp[i] - rm[i]) / (2.0 * opts.eps);
            }
        }
        let jtj = j.transpose() * &j;
        let jtr = j.transpose() * &r;
        // gradient norm
        let gnorm = jtr.norm();
        if gnorm < opts.gtol {
            converged = true;
            break;
        }

        // Try step
        let mut accepted = false;
        for _ in 0..10 {
            // (J^T J + lambda * diag(J^T J)) delta = -J^T r
            let mut a = jtj.clone();
            for k in 0..n {
                a[(k, k)] += lambda * jtj[(k, k)].max(1e-12);
            }
            let rhs = -&jtr;
            let delta = match a.lu().solve(&rhs) {
                Some(d) => d,
                None => break,
            };
            let x_new = &x + &delta;
            let r_new = residual(&x_new);
            let cost_new = r_new.dot(&r_new);
            // gain ratio
            let pred_red = delta.dot(&(&jtr * 2.0 - &jtj * &delta)); // approx
            let _ = pred_red;
            if cost_new < cost {
                // accept
                x = x_new;
                r = r_new;
                cost = cost_new;
                lambda *= 0.1;
                accepted = true;
                break;
            } else {
                lambda *= 10.0;
            }
        }
        if !accepted {
            // no progress
            break;
        }
        // check convergence on x and cost
        // (simplified) stop if delta small
    }

    // final convergence check (cost)
    LmResult {
        x,
        cost,
        iterations: iters,
        converged,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_scalar() {
        let x0 = DVector::from_vec(vec![0.0]);
        let res = lm_solve(
            x0,
            |x: &DVector<f64>| DVector::from_vec(vec![x[0] - 2.0]),
            &LmOptions::default(),
        );
        assert!((res.x[0] - 2.0).abs() < 1e-6, "got {}", res.x[0]);
    }

    #[test]
    fn solves_linear_least_squares() {
        // fit y = a x + b to points (0,1),(1,3),(2,5) -> a=2,b=1
        let data = vec![(0.0, 1.0), (1.0, 3.0), (2.0, 5.0)];
        let x0 = DVector::from_vec(vec![0.0, 0.0]);
        let res = lm_solve(
            x0,
            |p: &DVector<f64>| {
                let mut r = DVector::zeros(data.len());
                for (i, (xi, yi)) in data.iter().enumerate() {
                    r[i] = p[0] * xi + p[1] - yi;
                }
                r
            },
            &LmOptions::default(),
        );
        assert!((res.x[0] - 2.0).abs() < 1e-6);
        assert!((res.x[1] - 1.0).abs() < 1e-6);
    }
}
