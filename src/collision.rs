//! Oriented vehicle bounds and separating-axis collision checks.

const EPSILON: f64 = 1.0e-9;

#[derive(Clone, Copy, Debug)]
pub struct OrientedBox {
    pub center: (f64, f64),
    pub angle: f64,
    pub half_length: f64,
    pub half_width: f64,
}

impl OrientedBox {
    pub fn new(center: (f64, f64), angle: f64, length: f64, width: f64) -> OrientedBox {
        OrientedBox {
            center,
            angle,
            half_length: length / 2.0,
            half_width: width / 2.0,
        }
    }

    pub fn axis_aligned(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> OrientedBox {
        OrientedBox {
            center: ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
            angle: 0.0,
            half_length: (max_x - min_x) / 2.0,
            half_width: (max_y - min_y) / 2.0,
        }
    }

    pub fn expanded(self, gap: f64) -> OrientedBox {
        OrientedBox {
            half_length: self.half_length + gap / 2.0,
            half_width: self.half_width + gap / 2.0,
            ..self
        }
    }

    pub fn intersects(self, other: OrientedBox) -> bool {
        let center_distance_squared =
            (self.center.0 - other.center.0).powi(2) + (self.center.1 - other.center.1).powi(2);
        let maximum_distance =
            self.half_length.hypot(self.half_width) + other.half_length.hypot(other.half_width);
        if center_distance_squared > maximum_distance.powi(2) {
            return false;
        }

        let axes = [self.forward(), self.side(), other.forward(), other.side()];
        let self_corners = self.corners();
        let other_corners = other.corners();

        axes.into_iter().all(|axis| {
            let (self_min, self_max) = projection_range(&self_corners, axis);
            let (other_min, other_max) = projection_range(&other_corners, axis);
            self_max + EPSILON >= other_min && other_max + EPSILON >= self_min
        })
    }

    pub fn corners(self) -> [(f64, f64); 4] {
        let forward = self.forward();
        let side = self.side();
        std::array::from_fn(|index| {
            let longitudinal = if index & 1 == 0 { -1.0 } else { 1.0 };
            let lateral = if index & 2 == 0 { -1.0 } else { 1.0 };
            (
                self.center.0
                    + forward.0 * self.half_length * longitudinal
                    + side.0 * self.half_width * lateral,
                self.center.1
                    + forward.1 * self.half_length * longitudinal
                    + side.1 * self.half_width * lateral,
            )
        })
    }

    fn forward(self) -> (f64, f64) {
        (self.angle.cos(), self.angle.sin())
    }

    fn side(self) -> (f64, f64) {
        (-self.angle.sin(), self.angle.cos())
    }
}

fn projection_range(corners: &[(f64, f64); 4], axis: (f64, f64)) -> (f64, f64) {
    corners
        .iter()
        .map(|point| point.0 * axis.0 + point.1 * axis.1)
        .fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(minimum, maximum), value| (minimum.min(value), maximum.max(value)),
        )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_rotated_overlap_and_separation() {
        let first = OrientedBox::new((0.0, 0.0), 0.0, 30.0, 17.0);
        let crossing = OrientedBox::new((0.0, 0.0), std::f64::consts::FRAC_PI_2, 30.0, 17.0);
        let distant = OrientedBox::new((50.0, 0.0), 0.0, 30.0, 17.0);

        assert!(first.intersects(crossing));
        assert!(!first.intersects(distant));
    }

    #[test]
    fn expansion_adds_a_symmetric_safety_gap() {
        let first = OrientedBox::new((0.0, 0.0), 0.0, 30.0, 17.0).expanded(13.0);
        let second = OrientedBox::new((42.0, 0.0), 0.0, 30.0, 17.0).expanded(13.0);

        assert!(first.intersects(second));
    }
}
