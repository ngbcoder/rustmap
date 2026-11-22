//! Generic RANSAC framework.

use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

#[derive(Clone, Debug)]
pub struct RansacOptions {
    pub max_iterations: usize,
    pub threshold: f64,
    pub min_inliers: usize,
    pub confidence: f64,
    pub seed: u64,
    pub local_optimization: bool,
}

impl Default for RansacOptions {
    fn default() -> Self {
        Self {
            max_iterations: 1000,
            threshold: 1e-3,
            min_inliers: 0,
            confidence: 0.99,
            seed: 0,
            local_optimization: true,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RansacResult<M> {
    pub model: M,
    pub inliers: Vec<usize>,
}

pub trait Estimator<D> {
    type Model: Clone;
    fn min_sample_size(&self) -> usize;
    fn estimate(&self, data: &[D], sample: &[usize]) -> Vec<Self::Model>;
    fn residuals(&self, model: &Self::Model, data: &[D]) -> Vec<f64>;
    fn refine(&self, model: &Self::Model, _inliers: &[usize], _data: &[D]) -> Self::Model {
        model.clone()
    }
}

pub fn ransac<D, E: Estimator<D>>(
    estimator: &E,
    data: &[D],
    opts: &RansacOptions,
) -> Option<RansacResult<E::Model>> {
    if data.len() < estimator.min_sample_size() {
        return None;
    }
    let n = data.len();
    let m = estimator.min_sample_size();
    let mut rng = ChaCha8Rng::seed_from_u64(opts.seed);

    let mut best_model: Option<E::Model> = None;
    let mut best_score: usize = opts.min_inliers.saturating_sub(1);
    let mut best_rsum: f64 = f64::INFINITY;

    let mut iters = opts.max_iterations;
    let mut done = 0usize;
    while done < iters {
        done += 1;
        if done > opts.max_iterations {
            break;
        }

        // Random minimal sample
        let mut indices: Vec<usize> = (0..n).collect();
        for i in 0..m {
            let j = rng.gen_range(i..n);
            indices.swap(i, j);
        }
        let sample = &indices[..m];
        let mut uniq = sample.to_vec();
        uniq.sort();
        uniq.dedup();
        if uniq.len() < m {
            continue;
        }

        let candidates = estimator.estimate(data, sample);
        for cand in candidates {
            let res = estimator.residuals(&cand, data);
            let mut inliers = Vec::new();
            let mut rsum = 0.0f64;
            for (i, r) in res.iter().enumerate() {
                if *r <= opts.threshold {
                    inliers.push(i);
                    rsum += *r;
                }
            }
            if inliers.len() < opts.min_inliers {
                // still consider if no best yet
                if best_model.is_some() {
                    continue;
                }
            }
            let better =
                inliers.len() > best_score || (inliers.len() == best_score && rsum < best_rsum);
            if better {
                best_score = inliers.len();
                best_rsum = rsum;
                let mut model = cand;
                if opts.local_optimization && !inliers.is_empty() {
                    model = estimator.refine(&model, &inliers, data);
                }
                best_model = Some(model);

                // adaptive iters
                let w = inliers.len() as f64 / n as f64;
                if w > 0.0 && w < 1.0 {
                    let p_no = (1.0 - w.powi(m as i32)).max(1e-12_f64);
                    let n_iters = (opts.confidence.ln() / p_no.ln()).ceil() as usize;
                    if n_iters < iters {
                        iters = n_iters.max(1);
                    }
                }
            }
        }
    }

    best_model.map(|model| {
        let res = estimator.residuals(&model, data);
        let inliers: Vec<usize> = res
            .iter()
            .enumerate()
            .filter(|(_, r)| **r <= opts.threshold)
            .map(|(i, _)| i)
            .collect();
        if opts.local_optimization && !inliers.is_empty() {
            let refined = estimator.refine(&model, &inliers, data);
            let res2 = estimator.residuals(&refined, data);
            let inl2: Vec<usize> = res2
                .iter()
                .enumerate()
                .filter(|(_, r)| **r <= opts.threshold)
                .map(|(i, _)| i)
                .collect();
            if inl2.len() >= inliers.len() {
                return RansacResult {
                    model: refined,
                    inliers: inl2,
                };
            }
        }
        RansacResult { model, inliers }
    })
}
