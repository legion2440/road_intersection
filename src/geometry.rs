//! Geometry constants and the immutable paths followed by vehicles.

use crate::collision::OrientedBox;

pub const W: u32 = 780;
pub const H: u32 = 780;
pub const PANEL_W: u32 = 360;
pub const WIN_W: u32 = W + PANEL_W;
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
pub const CROSSWALK_START: f64 = CY - LANE - CROSSWALK_DEPTH;
pub const STOP_LINE_COORD: f64 = CROSSWALK_START - 5.0;
pub const STOP_LINE_THICKNESS: f64 = 5.0;
pub const FIXED_HZ: u32 = 60;
pub const FIXED_DT: f64 = 1.0 / FIXED_HZ as f64;
pub const VEHICLE_SPEED: f64 = 96.0;

const RIGHT_TURN_RADIUS: f64 = OFF;

/// Subject formula: floor(lane length / (vehicle length + safety gap)).
pub fn capacity() -> usize {
    ((STOP_LINE_COORD - START) / FOLLOW_DISTANCE).floor() as usize
}

/// Returns the destination arm for origin `0..=3` and route `0..=2`.
///
/// Origins rotate clockwise as north, east, south and west. Routes are
/// straight, left and right.
pub fn exit_arm(origin: usize, route: usize) -> usize {
    match route {
        0 => (origin + 2) % 4,
        1 => (origin + 1) % 4,
        2 => (origin + 3) % 4,
        _ => origin,
    }
}

/// A polyline the vehicle follows, with cumulative arc-length for lookup.
#[derive(Clone, Debug)]
pub struct Path {
    /// Polyline control points in travel order.
    ///
    /// Turning paths keep the curve start at index 1 and the curve end at the
    /// penultimate index; the final point only extends the outgoing lane.
    pub pts: Vec<(f64, f64)>,
    /// Arc-length at each matching entry in `pts`, with `cum[0] == 0`.
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
            stop_progress: STOP_LINE_COORD - START - CAR_LEN / 2.0,
            conflict_entry: 0.0,
            conflict_exit: len,
        };
        (path.conflict_entry, path.conflict_exit) = path.measure_conflict_span();
        path
    }

    /// position + heading (radians) at arc-length `s`
    pub fn at(&self, s: f64) -> (f64, f64, f64) {
        let s = s.max(0.0);
        if s >= self.len {
            let n = self.pts.len();
            let (a, b) = (self.pts[n - 2], self.pts[n - 1]);
            return (b.0, b.1, (b.1 - a.1).atan2(b.0 - a.0));
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
        )
    }

    pub fn vehicle_bounds(&self, s: f64) -> OrientedBox {
        let (x, y, angle) = self.at(s);
        OrientedBox::new((x, y), angle, CAR_LEN, CAR_W)
    }

    fn measure_conflict_span(&self) -> (f64, f64) {
        let conflict = OrientedBox::axis_aligned(CX - LANE, CY - LANE, CX + LANE, CY + LANE);
        let intersects = |progress: f64| self.vehicle_bounds(progress).intersects(conflict);
        // Coarse quarter-pixel sampling only brackets each transition; the
        // binary refinement below computes the actual entry/exit boundary.
        let sample_step = 0.25;
        let samples = (self.len / sample_step).ceil() as usize;
        let mut entry_bracket = None;
        let mut exit_bracket = None;
        let mut previous_s = 0.0;
        let mut previous_inside = intersects(previous_s);

        for i in 1..=samples {
            let s = (i as f64 * sample_step).min(self.len);
            let inside = intersects(s);
            if !previous_inside && inside && entry_bracket.is_none() {
                entry_bracket = Some((previous_s, s));
            } else if previous_inside && !inside {
                exit_bracket = Some((previous_s, s));
            }
            previous_s = s;
            previous_inside = inside;
        }

        let (entry_low, entry_high) =
            entry_bracket.expect("every vehicle path must enter the conflict zone");
        let (exit_low, exit_high) =
            exit_bracket.expect("every vehicle path must leave the conflict zone");

        (
            refine_transition(entry_low, entry_high, &intersects, true),
            refine_transition(exit_low, exit_high, &intersects, false),
        )
    }
}

fn refine_transition(
    mut low: f64,
    mut high: f64,
    intersects: &impl Fn(f64) -> bool,
    target_state: bool,
) -> f64 {
    for _ in 0..24 {
        let middle = (low + high) / 2.0;
        if intersects(middle) == target_state {
            high = middle;
        } else {
            low = middle;
        }
    }
    high
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

/// Builds `paths[origin][route]`.
///
/// Origins `0..=3` are north, east, south and west. Routes `0..=2` are
/// straight, left and right. For turning paths, `cum[1]` and
/// `cum[cum.len() - 2]` therefore delimit the sampled curve itself.
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
                let (x, y, angle) = path.at(progress);
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

    #[test]
    fn capacity_is_measured_to_the_physical_stop_line() {
        let expected = ((STOP_LINE_COORD - START) / FOLLOW_DISTANCE).floor() as usize;
        assert_eq!(capacity(), expected);
    }

    #[test]
    fn conflict_span_tracks_the_whole_rotated_vehicle() {
        let conflict = OrientedBox::axis_aligned(CX - LANE, CY - LANE, CX + LANE, CY + LANE);
        for path in build_paths().iter().flatten() {
            assert!(path
                .vehicle_bounds(path.conflict_entry + 0.01)
                .intersects(conflict));
            assert!(path
                .vehicle_bounds(path.conflict_exit - 0.01)
                .intersects(conflict));
            assert!(!path
                .vehicle_bounds((path.conflict_exit + 0.01).min(path.len))
                .intersects(conflict));
        }
    }
}
