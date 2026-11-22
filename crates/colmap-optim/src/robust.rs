//! Robust loss functions (IRLS weighting).

#[derive(Clone, Copy, Debug)]
pub enum RobustLoss {
    L2,
    Huber { delta: f64 },
    Cauchy { c: f64 },
    SoftL1 { c: f64 },
    Triggs { c: f64 },
}

impl RobustLoss {
    /// Weight for a residual of magnitude `r` (for IRLS).
    pub fn weight(&self, r: f64) -> f64 {
        let a = r.abs();
        match self {
            RobustLoss::L2 => 1.0,
            RobustLoss::Huber { delta } => {
                if a <= *delta {
                    1.0
                } else {
                    *delta / a
                }
            }
            RobustLoss::Cauchy { c } => 1.0 / (1.0 + (a / *c).powi(2)),
            RobustLoss::SoftL1 { c } => 1.0 / (1.0 + a / *c).sqrt(),
            RobustLoss::Triggs { c } => {
                let denom = 1.0 + (a / *c).powi(2);
                1.0 / (denom * denom)
            }
        }
    }
    /// Rho (cost) for residual r.
    pub fn rho(&self, r: f64) -> f64 {
        let a = r.abs();
        match self {
            RobustLoss::L2 => 0.5 * r * r,
            RobustLoss::Huber { delta } => {
                if a <= *delta {
                    0.5 * r * r
                } else {
                    *delta * (a - 0.5 * *delta)
                }
            }
            RobustLoss::Cauchy { c } => 0.5 * *c * *c * (1.0 + (a / *c).powi(2)).ln(),
            RobustLoss::SoftL1 { c } => *c * *c * ((1.0 + (a / *c).powi(2)).sqrt() - 1.0),
            RobustLoss::Triggs { c } => 0.5 * *c * *c * (1.0 - 1.0 / (1.0 + (a / *c).powi(2))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn huber_weight() {
        let h = RobustLoss::Huber { delta: 1.0 };
        assert_eq!(h.weight(0.5), 1.0);
        assert!((h.weight(2.0) - 0.5).abs() < 1e-9);
    }
    #[test]
    fn rho_zero() {
        let h = RobustLoss::Cauchy { c: 1.0 };
        assert_eq!(h.rho(0.0), 0.0);
    }
}
