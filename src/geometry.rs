//! Geometry constants and the immutable paths followed by vehicles.

pub const W: u32 = 780;
pub const H: u32 = 780;
pub const CX: f64 = 390.0;
pub const CY: f64 = 390.0;
pub const LANE: f64 = 48.0;
pub const OFF: f64 = 24.0;
pub const START: f64 = -40.0;

pub const CAR_LEN: f64 = 30.0;
pub const CAR_W: f64 = 17.0;
pub const GAP: f64 = 13.0;
pub const FOLLOW_DISTANCE: f64 = CAR_LEN + GAP;
pub const CROSSWALK_DEPTH: f64 = 18.0;
pub const FIXED_HZ: u32 = 60;
pub const FIXED_DT: f64 = 1.0 / FIXED_HZ as f64;
pub const VEHICLE_SPEED: f64 = 96.0;

const RIGHT_TURN_RADIUS: f64 = OFF;

/// Distance from spawn to the point where the car's nose reaches the box.
pub fn s_stop() -> f64 {
    (CY - LANE) - START - CAR_LEN / 2.0
}

/// Subject formula: floor(lane length / (vehicle length + safety gap)).
pub fn capacity() -> usize {
    (((CY - LANE) - START) / FOLLOW_DISTANCE).floor() as usize
}

/// A polyline the vehicle follows, with cumulative arc-length for lookup.
#[derive(Clone, Debug)]
pub struct Path {
    pub pts: Vec<(f64, f64)>,
    pub cum: Vec<f64>,
    pub len: f64,
    /// Furthest progress allowed on red; the whole vehicle remains before the crosswalk.
    pub stop_progress: f64,
    /// First progress at which any part of the vehicle enters the conflict box.
    pub conflict_entry: f64,
    /// First progress at which the whole vehicle has left the conflict box.
    pub conflict_exit: f64,
}

impl Path {
    fn new(pts: Vec<(f64, f64)>) -> Self {
        let mut cum = vec![0.0];
        let mut len = 0.0;
        for i in 1..pts.len() {
            len += ((pts[i].0 - pts[i - 1].0).powi(2) + (pts[i].1 - pts[i - 1].1).powi(2)).sqrt();
            cum.push(len);
        }
        let mut path = Path {
            pts,
            cum,
            len,
            stop_progress: (CY - LANE - CROSSWALK_DEPTH) - START - CAR_LEN / 2.0,
            conflict_entry: 0.0,
            conflict_exit: len,
        };
        (path.conflict_entry, path.conflict_exit) = path.measure_conflict_span();
        path
    }

    /// position + heading (radians) at arc-length `s`
    pub fn at(&self, s: f64) -> (f64, f64, f64, bool) {
        let s = s.max(0.0);
        if s >= self.len {
            let n = self.pts.len();
            let (a, b) = (self.pts[n - 2], self.pts[n - 1]);
            return (b.0, b.1, (b.1 - a.1).atan2(b.0 - a.0), true);
        }
        let mut i = 0;
        while i < self.cum.len() - 1 && s > self.cum[i + 1] {
            i += 1;
        }
        let (a, b) = (self.pts[i], self.pts[i + 1]);
        let seg = (self.cum[i + 1] - self.cum[i]).max(1e-6);
        let t = (s - self.cum[i]) / seg;
        (
            a.0 + (b.0 - a.0) * t,
            a.1 + (b.1 - a.1) * t,
            (b.1 - a.1).atan2(b.0 - a.0),
            false,
        )
    }

    fn measure_conflict_span(&self) -> (f64, f64) {
        // Expanding the box by half a car length turns a centre-point lookup
        // into "any part of the car is in the conflict zone".
        let pad = CAR_LEN / 2.0;
        let min_x = CX - LANE - pad;
        let max_x = CX + LANE + pad;
        let min_y = CY - LANE - pad;
        let max_y = CY + LANE + pad;
        let sample_step = 0.25;
        let samples = (self.len / sample_step).ceil() as usize;
        let mut first = None;
        let mut last = None;

        for i in 0..=samples {
            let s = (i as f64 * sample_step).min(self.len);
            let (x, y, _, _) = self.at(s);
            if x >= min_x && x <= max_x && y >= min_y && y <= max_y {
                first.get_or_insert(s);
                last = Some(s);
            }
        }

        (
            first.unwrap_or_else(s_stop),
            last.map(|s| (s + sample_step).min(self.len))
                .unwrap_or_else(s_stop),
        )
    }
}

fn arc(c: (f64, f64), r: f64, d0: f64, d1: f64, steps: usize) -> Vec<(f64, f64)> {
    (1..=steps)
        .map(|i| {
            let a = (d0 + (d1 - d0) * i as f64 / steps as f64).to_radians();
            (c.0 + r * a.cos(), c.1 + r * a.sin())
        })
        .collect()
}

fn rot1(p: (f64, f64)) -> (f64, f64) {
    (CX - (p.1 - CY), CY + (p.0 - CX))
}
fn rot_k(pts: &[(f64, f64)], k: usize) -> Vec<(f64, f64)> {
    let mut v = pts.to_vec();
    for _ in 0..k {
        v = v.iter().map(|&p| rot1(p)).collect();
    }
    v
}

/// `paths[origin][route]`, where routes are straight, left and right.
/// Origins are north, east, south and west.
pub fn build_paths() -> [[Path; 3]; 4] {
    let lane_x = CX - OFF;

    let straight = vec![(lane_x, START), (lane_x, H as f64 + 40.0)];

    let rr = RIGHT_TURN_RADIUS;
    let cr = (lane_x - rr, CY - OFF - rr);
    let mut right = vec![(lane_x, START), (lane_x, CY - OFF - rr)];
    right.extend(arc(cr, rr, 0.0, 90.0, 10));
    right.push((-40.0, CY - OFF));

    let rl = 40.0; // wide left turn
    let cl = (lane_x + rl, CY + OFF - rl);
    let mut left = vec![(lane_x, START), (lane_x, CY + OFF - rl)];
    left.extend(arc(cl, rl, 180.0, 90.0, 10));
    left.push((W as f64 + 40.0, CY + OFF));

    let base = [straight, left, right];
    std::array::from_fn(|k| std::array::from_fn(|r| Path::new(rot_k(&base[r], k))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn right_turns_end_in_the_expected_exit_lane() {
        let paths = build_paths();
        let base_exit = (-40.0, CY - OFF);

        for (origin, origin_paths) in paths.iter().enumerate() {
            let expected = rot_k(&[base_exit], origin)[0];
            let actual = *origin_paths[2].pts.last().unwrap();
            assert!((actual.0 - expected.0).abs() < 0.01);
            assert!((actual.1 - expected.1).abs() < 0.01);
        }
    }

    #[test]
    fn right_turn_vehicle_body_stays_on_paved_road() {
        for path in build_paths().iter().map(|paths| &paths[2]) {
            let samples = (path.len / 0.5).ceil() as usize;
            for sample in 0..=samples {
                let progress = (sample as f64 * 0.5).min(path.len);
                let (x, y, angle, _) = path.at(progress);
                let forward = (angle.cos(), angle.sin());
                let side = (-forward.1, forward.0);

                for longitudinal in [-0.5, 0.5] {
                    for lateral in [-0.5, 0.5] {
                        let corner_x =
                            x + forward.0 * CAR_LEN * longitudinal + side.0 * CAR_W * lateral;
                        let corner_y =
                            y + forward.1 * CAR_LEN * longitudinal + side.1 * CAR_W * lateral;
                        let on_vertical_road = (corner_x - CX).abs() <= LANE + 0.01;
                        let on_horizontal_road = (corner_y - CY).abs() <= LANE + 0.01;
                        assert!(
                            on_vertical_road || on_horizontal_road,
                            "right-turn corner left the road at progress {progress}: \
                             ({corner_x}, {corner_y})"
                        );
                    }
                }
            }
        }
    }
}
