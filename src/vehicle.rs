//! Vehicle state and the deterministic, fixed-timestep simulation.

use crate::collision::OrientedBox;
use crate::geometry::*;
use crate::lights::Lights;
use rand::{seq::SliceRandom, Rng};

const EPSILON: f64 = 1.0e-6;
// A quarter-pixel sample is substantially finer than the 1.6-pixel fixed-tick
// movement. It captures the rapid OBB rotation at polyline corners without
// pretending that the reservation is a mathematically continuous sweep.
const SWEEP_SAMPLE_DISTANCE: f64 = 0.25;

/// The vehicle's relationship to the stop line and physical conflict zone.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehiclePhase {
    /// Moving toward the stop line and still governed by the signal.
    Approaching,
    /// Stationary before the stop line, due to a red signal or a leader.
    Waiting,
    /// Past the stop line but not yet physically inside the conflict zone.
    ///
    /// A committed vehicle must continue even if its signal changes.
    Committed,
    /// At least part of the vehicle occupies the conflict zone.
    Crossing,
    /// The entire vehicle has cleared the conflict zone and is on an exit arm.
    Leaving,
}

/// A vehicle following one immutable path through the intersection.
#[derive(Clone, Debug)]
pub struct Vehicle {
    /// Entry arm: `0..=3` means north, east, south and west.
    pub origin: usize,
    /// Immutable route: `0..=2` means straight, left and right.
    pub route: usize,
    /// Arc-length travelled along `paths[origin][route]`.
    pub progress: f64,
    pub position: (f64, f64),
    pub angle: f64,
    pub phase: VehiclePhase,
}

impl Vehicle {
    fn is_queued(&self) -> bool {
        matches!(
            self.phase,
            VehiclePhase::Approaching | VehiclePhase::Waiting
        )
    }

    fn requires_clearance(&self) -> bool {
        matches!(self.phase, VehiclePhase::Committed | VehiclePhase::Crossing)
    }
}

/// Complete deterministic simulation state consumed by the renderer.
pub struct Sim {
    /// Immutable geometry indexed by `[origin][route]`.
    pub paths: [[Path; 3]; 4],
    pub vehicles: Vec<Vehicle>,
    pub lights: Lights,
    pub spawned: u32,
    pub passed: u32,
    pub rejected: u32,
    /// Cumulative physical conflict-entry crossings for each origin.
    pub entered: [u32; 4],
}

impl Sim {
    pub fn new() -> Self {
        Self {
            paths: build_paths(),
            vehicles: Vec::new(),
            lights: Lights::new(),
            spawned: 0,
            passed: 0,
            rejected: 0,
            entered: [0; 4],
        }
    }

    /// Spawns a random immutable route from one origin. Returns false when
    /// capacity or physical spacing makes the request unsafe.
    pub fn spawn(&mut self, origin: usize) -> bool {
        let route = rand::thread_rng().gen_range(0..3);
        let spawned = self.spawn_with_route(origin, route);
        if !spawned {
            self.rejected = self.rejected.saturating_add(1);
        }
        spawned
    }

    pub fn spawn_with_route(&mut self, origin: usize, route: usize) -> bool {
        if route >= 3 || !self.can_spawn(origin) {
            return false;
        }

        let path = &self.paths[origin][route];
        let (x, y, angle) = path.at(0.0);
        self.vehicles.push(Vehicle {
            origin,
            route,
            progress: 0.0,
            position: (x, y),
            angle,
            phase: VehiclePhase::Approaching,
        });
        self.spawned += 1;
        true
    }

    pub fn spawn_random(&mut self) -> bool {
        let mut rng = rand::thread_rng();
        let order = Self::random_origin_order(&mut rng);
        self.spawn_random_in_order(order)
    }

    fn random_origin_order<R: Rng + ?Sized>(rng: &mut R) -> [usize; 4] {
        let mut order = [0, 1, 2, 3];
        order.shuffle(rng);
        order
    }

    fn spawn_random_in_order(&mut self, order: [usize; 4]) -> bool {
        let route = rand::thread_rng().gen_range(0..3);
        for origin in order {
            if self.spawn_with_route(origin, route) {
                return true;
            }
        }

        self.rejected = self.rejected.saturating_add(1);
        false
    }

    fn can_spawn(&self, origin: usize) -> bool {
        if origin >= 4 {
            return false;
        }

        self.queue_lengths()[origin] < capacity()
            && !self
                .vehicles
                .iter()
                .filter(|vehicle| vehicle.origin == origin)
                .any(|vehicle| vehicle.progress < FOLLOW_DISTANCE - EPSILON)
    }

    pub fn queue_lengths(&self) -> [usize; 4] {
        let mut queues = [0; 4];
        for vehicle in &self.vehicles {
            if vehicle.is_queued() {
                queues[vehicle.origin] += 1;
            }
        }
        queues
    }

    pub fn capacity(&self) -> usize {
        capacity()
    }

    pub fn clearance_pending(&self) -> bool {
        self.vehicles.iter().any(Vehicle::requires_clearance)
    }

    pub fn step(&mut self) {
        let queues = self.queue_lengths();
        self.lights.update(&queues, self.clearance_pending());

        let mut next_progress: Vec<f64> = self
            .vehicles
            .iter()
            .map(|vehicle| vehicle.progress)
            .collect();

        // Process front-to-back independently for each physical entry lane.
        // Different routes are released from following constraints as soon as
        // their actual positions have separated by the required distance.
        for origin in 0..4 {
            let mut order: Vec<usize> = self
                .vehicles
                .iter()
                .enumerate()
                .filter_map(|(index, vehicle)| (vehicle.origin == origin).then_some(index))
                .collect();
            order.sort_by(|&a, &b| {
                self.vehicles[b]
                    .progress
                    .total_cmp(&self.vehicles[a].progress)
            });

            let mut nearest_by_route = [None; 3];
            for index in order {
                let vehicle = &self.vehicles[index];
                let path = &self.paths[vehicle.origin][vehicle.route];
                let mut target = (vehicle.progress + VEHICLE_SPEED * FIXED_DT).min(path.len);

                if matches!(
                    vehicle.phase,
                    VehiclePhase::Approaching | VehiclePhase::Waiting
                ) && !self.lights.is_green(vehicle.origin)
                {
                    target = target.min(path.stop_progress);
                }

                for leader_index in nearest_by_route.into_iter().flatten() {
                    target = self.limit_behind_leader(
                        index,
                        target,
                        leader_index,
                        next_progress[leader_index],
                    );
                }

                next_progress[index] = target.max(vehicle.progress);
                nearest_by_route[vehicle.route] = Some(index);
            }
        }

        self.apply_exit_lane_following(&mut next_progress);
        self.apply_obb_safety(&mut next_progress);

        for (index, vehicle) in self.vehicles.iter_mut().enumerate() {
            let previous = vehicle.progress;
            vehicle.progress = next_progress[index];
            let path = &self.paths[vehicle.origin][vehicle.route];
            let (x, y, angle) = path.at(vehicle.progress);
            vehicle.position = (x, y);
            vehicle.angle = angle;
            if previous + EPSILON < path.conflict_entry
                && vehicle.progress + EPSILON >= path.conflict_entry
            {
                self.entered[vehicle.origin] = self.entered[vehicle.origin].saturating_add(1);
            }

            vehicle.phase = if vehicle.progress + EPSILON >= path.conflict_exit {
                VehiclePhase::Leaving
            } else if vehicle.progress + EPSILON >= path.conflict_entry {
                VehiclePhase::Crossing
            } else if vehicle.progress > path.stop_progress + EPSILON {
                VehiclePhase::Committed
            } else if (vehicle.progress - previous).abs() <= EPSILON {
                VehiclePhase::Waiting
            } else {
                VehiclePhase::Approaching
            };
        }

        let before = self.vehicles.len();
        self.vehicles.retain(|vehicle| {
            vehicle.progress + EPSILON < self.paths[vehicle.origin][vehicle.route].len
        });
        self.passed += (before - self.vehicles.len()) as u32;
    }

    fn limit_behind_leader(
        &self,
        follower_index: usize,
        mut target: f64,
        leader_index: usize,
        leader_progress: f64,
    ) -> f64 {
        let follower = &self.vehicles[follower_index];
        let leader = &self.vehicles[leader_index];
        if leader.progress <= follower.progress + EPSILON {
            return target;
        }

        if follower.route == leader.route {
            target = target.min(leader_progress - FOLLOW_DISTANCE);
        }

        let separated = |progress: f64| {
            self.separated_from_leader_sweep(
                follower_index,
                progress,
                leader_index,
                leader_progress,
            )
        };

        if separated(target) {
            return target;
        }
        if !separated(follower.progress) {
            return follower.progress;
        }

        // The update distance is small, so a short binary search gives the
        // exact safe point without coupling already-diverged route branches.
        let mut low = follower.progress;
        let mut high = target;
        for _ in 0..20 {
            let middle = (low + high) * 0.5;
            if separated(middle) {
                low = middle;
            } else {
                high = middle;
            }
        }
        low
    }

    fn separated_from_leader_sweep(
        &self,
        follower_index: usize,
        follower_progress: f64,
        leader_index: usize,
        leader_progress: f64,
    ) -> bool {
        let follower_bounds = self.safety_bounds(follower_index, follower_progress);
        if follower_bounds.intersects(self.safety_bounds(leader_index, leader_progress)) {
            return false;
        }

        let leader_path = self.path_for(leader_index);
        if self.vehicles[follower_index].route == self.vehicles[leader_index].route {
            // Same-route vehicles need only one tick of lookahead: they will
            // traverse the same curve in order, so reserving the whole turn
            // would stop the follower long after the leader has moved ahead.
            let sweep_end = (leader_progress + VEHICLE_SPEED * FIXED_DT).min(leader_path.len);
            let mut sample = leader_progress;
            while sample + EPSILON < sweep_end {
                sample = (sample + SWEEP_SAMPLE_DISTANCE).min(sweep_end);
                if follower_bounds.intersects(self.safety_bounds(leader_index, sample)) {
                    return false;
                }
            }
            return true;
        }

        // build_paths stores the curve between the second and penultimate
        // control points; the first and last segments are the approach/exit.
        let Some((&turn_start, &turn_end)) = leader_path
            .cum
            .get(1)
            .zip(leader_path.cum.get(leader_path.cum.len().saturating_sub(2)))
        else {
            return true;
        };
        if turn_end <= turn_start + EPSILON || leader_progress + EPSILON >= turn_end {
            return true;
        }

        // Outside two expanded-OBB bounding radii no future rotation can
        // overlap the follower. One extra fixed step activates the reservation
        // before the leader can enter that range.
        let safety_radius = (CAR_LEN / 2.0 + GAP / 2.0).hypot(CAR_W / 2.0 + GAP / 2.0);
        let activation_distance = 2.0 * safety_radius + VEHICLE_SPEED * FIXED_DT;
        if follower_progress + activation_distance + EPSILON < turn_start {
            return true;
        }

        // Reserve the whole turn, not only the still-untraversed tail. Keeping
        // this envelope stable prevents a stopped follower from creeping into
        // space uncovered behind the rotating leader.
        let mut sample = turn_start;
        loop {
            if follower_bounds.intersects(self.safety_bounds(leader_index, sample)) {
                return false;
            }
            if sample + EPSILON >= turn_end {
                break;
            }
            sample = (sample + SWEEP_SAMPLE_DISTANCE).min(turn_end);
        }
        true
    }

    fn apply_exit_lane_following(&self, next_progress: &mut [f64]) {
        for arm in 0..4 {
            let mut order: Vec<usize> = self
                .vehicles
                .iter()
                .enumerate()
                .filter_map(|(index, vehicle)| {
                    let path = &self.paths[vehicle.origin][vehicle.route];
                    (exit_arm(vehicle.origin, vehicle.route) == arm
                        && next_progress[index] + EPSILON >= path.conflict_exit)
                        .then_some(index)
                })
                .collect();
            order.sort_by(|&a, &b| {
                self.remaining_distance(a, next_progress[a])
                    .total_cmp(&self.remaining_distance(b, next_progress[b]))
            });

            for follower_position in 1..order.len() {
                let follower_index = order[follower_position];
                let leader_index = order[follower_position - 1];
                let leader_remaining =
                    self.remaining_distance(leader_index, next_progress[leader_index]);
                let follower_path = self.path_for(follower_index);
                let safe_progress = follower_path.len - leader_remaining - FOLLOW_DISTANCE;
                next_progress[follower_index] = next_progress[follower_index]
                    .min(safe_progress)
                    .max(self.vehicles[follower_index].progress);
            }
        }
    }

    fn apply_obb_safety(&self, next_progress: &mut [f64]) {
        let candidates: Vec<_> = self
            .vehicles
            .iter()
            .enumerate()
            .filter_map(|(index, _)| {
                let path = self.path_for(index);
                (next_progress[index] + FOLLOW_DISTANCE >= path.stop_progress
                    && next_progress[index] <= path.conflict_exit + FOLLOW_DISTANCE)
                    .then_some(index)
            })
            .collect();

        // One pass resolves direct conflicts; the second catches a follower
        // affected by a vehicle constrained during the first pass. These
        // passes stabilize dependencies; they do not replace the proactive
        // turn reservation above.
        for _ in 0..2 {
            let mut changed = false;
            let mut bounds: Vec<_> = next_progress
                .iter()
                .enumerate()
                .map(|(index, &progress)| self.safety_bounds(index, progress))
                .collect();
            for (position, &first) in candidates.iter().enumerate() {
                for &second in &candidates[(position + 1)..] {
                    if !bounds[first].intersects(bounds[second]) {
                        continue;
                    }

                    let (follower, leader) = self.yielding_pair(first, second, next_progress);
                    let constrained = self.limit_against_vehicle(
                        follower,
                        next_progress[follower],
                        leader,
                        next_progress[leader],
                    );
                    if constrained + EPSILON < next_progress[follower] {
                        next_progress[follower] = constrained;
                        bounds[follower] = self.safety_bounds(follower, constrained);
                        changed = true;
                    }

                    // Emergency fallback for an already-too-close follower:
                    // preserve its current safe position and roll the moving
                    // leader back to the last non-overlapping point. Normal
                    // movement is constrained proactively; moving the leader
                    // is reserved for states where the follower cannot retreat.
                    let follower_current = self.vehicles[follower].progress;
                    if self
                        .safety_bounds(follower, follower_current)
                        .intersects(bounds[leader])
                    {
                        let constrained = self.limit_against_vehicle(
                            leader,
                            next_progress[leader],
                            follower,
                            follower_current,
                        );
                        if constrained + EPSILON < next_progress[leader] {
                            next_progress[leader] = constrained;
                            bounds[leader] = self.safety_bounds(leader, constrained);
                            changed = true;
                        }
                    }
                }
            }
            if !changed {
                break;
            }
        }
    }

    fn yielding_pair(&self, first: usize, second: usize, next_progress: &[f64]) -> (usize, usize) {
        let first_vehicle = &self.vehicles[first];
        let second_vehicle = &self.vehicles[second];
        if first_vehicle.origin == second_vehicle.origin {
            return if first_vehicle.progress <= second_vehicle.progress {
                (first, second)
            } else {
                (second, first)
            };
        }

        let same_exit = exit_arm(first_vehicle.origin, first_vehicle.route)
            == exit_arm(second_vehicle.origin, second_vehicle.route);
        let first_on_exit = next_progress[first] + EPSILON >= self.path_for(first).conflict_exit;
        let second_on_exit = next_progress[second] + EPSILON >= self.path_for(second).conflict_exit;
        if same_exit && first_on_exit && second_on_exit {
            return if self.remaining_distance(first, next_progress[first])
                >= self.remaining_distance(second, next_progress[second])
            {
                (first, second)
            } else {
                (second, first)
            };
        }

        let first_priority = self.motion_priority(first);
        let second_priority = self.motion_priority(second);
        if first_priority != second_priority {
            return if first_priority < second_priority {
                (first, second)
            } else {
                (second, first)
            };
        }

        let first_fraction = next_progress[first] / self.path_for(first).len;
        let second_fraction = next_progress[second] / self.path_for(second).len;
        if first_fraction <= second_fraction {
            (first, second)
        } else {
            (second, first)
        }
    }

    fn limit_against_vehicle(
        &self,
        follower_index: usize,
        target: f64,
        leader_index: usize,
        leader_progress: f64,
    ) -> f64 {
        let leader_bounds = self.safety_bounds(leader_index, leader_progress);
        let separated = |progress: f64| {
            !self
                .safety_bounds(follower_index, progress)
                .intersects(leader_bounds)
        };
        let current = self.vehicles[follower_index].progress;
        if separated(target) {
            return target;
        }
        if !separated(current) {
            return current;
        }

        let mut low = current;
        let mut high = target;
        for _ in 0..20 {
            let middle = (low + high) / 2.0;
            if separated(middle) {
                low = middle;
            } else {
                high = middle;
            }
        }
        low
    }

    fn safety_bounds(&self, index: usize, progress: f64) -> OrientedBox {
        // expanded(GAP) adds GAP/2 per side. Expanding both vehicles therefore
        // turns a physical clearance below GAP into an OBB intersection.
        self.path_for(index).vehicle_bounds(progress).expanded(GAP)
    }

    fn path_for(&self, index: usize) -> &Path {
        let vehicle = &self.vehicles[index];
        &self.paths[vehicle.origin][vehicle.route]
    }

    fn remaining_distance(&self, index: usize, progress: f64) -> f64 {
        self.path_for(index).len - progress
    }

    fn motion_priority(&self, index: usize) -> u8 {
        match self.vehicles[index].phase {
            VehiclePhase::Approaching | VehiclePhase::Waiting => 0,
            VehiclePhase::Committed => 1,
            VehiclePhase::Crossing => 2,
            VehiclePhase::Leaving => 3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lights::{Phase, MAX_GREEN_TICKS, MAX_WAIT_TICKS, MIN_CLEAR_TICKS};
    use rand::{rngs::StdRng, Rng, SeedableRng};

    fn run_steps(sim: &mut Sim, count: usize) {
        for _ in 0..count {
            sim.step();
        }
    }

    fn hold_red(sim: &mut Sim, origin: usize, count: usize) {
        for _ in 0..count {
            sim.lights.phase = Phase::Green;
            sim.lights.green_dir = (origin + 1) % 4;
            sim.lights.green_timer = 0;
            sim.step();
        }
    }

    fn vehicle_at(sim: &Sim, origin: usize, route: usize, progress: f64) -> Vehicle {
        let path = &sim.paths[origin][route];
        let (x, y, angle) = path.at(progress);
        Vehicle {
            origin,
            route,
            progress,
            position: (x, y),
            angle,
            phase: if progress >= path.conflict_exit {
                VehiclePhase::Leaving
            } else if progress >= path.conflict_entry {
                VehiclePhase::Crossing
            } else if progress > path.stop_progress {
                VehiclePhase::Committed
            } else {
                VehiclePhase::Approaching
            },
        }
    }

    fn overlaps(first: &Vehicle, second: &Vehicle) -> bool {
        OrientedBox::new(first.position, first.angle, CAR_LEN, CAR_W).intersects(OrientedBox::new(
            second.position,
            second.angle,
            CAR_LEN,
            CAR_W,
        ))
    }

    fn safety_overlaps(sim: &Sim, first: usize, second: usize) -> bool {
        sim.safety_bounds(first, sim.vehicles[first].progress)
            .intersects(sim.safety_bounds(second, sim.vehicles[second].progress))
    }

    fn spawn_left_leader_and_straight_follower(sim: &mut Sim) {
        assert!(sim.spawn_with_route(0, 1));
        for _ in 0..100 {
            sim.step();
            if sim.spawn_with_route(0, 0) {
                return;
            }
        }
        panic!("straight follower never fit behind the left-turn leader");
    }

    #[test]
    fn vehicle_stays_at_the_stop_line_on_red() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 400);
        let vehicle = &sim.vehicles[0];
        let stop = sim.paths[1][0].stop_progress;
        assert!((vehicle.progress - stop).abs() < 0.01);
        assert_eq!(vehicle.phase, VehiclePhase::Waiting);
    }

    #[test]
    fn stopped_vehicle_remains_fully_before_the_crosswalk() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(0, 0));
        hold_red(&mut sim, 0, 400);

        let vehicle = &sim.vehicles[0];
        let front_y = vehicle.position.1 + vehicle.angle.sin() * CAR_LEN / 2.0;
        assert!(front_y <= STOP_LINE_COORD + EPSILON);
        assert!(sim.paths[0][0].stop_progress < sim.paths[0][0].conflict_entry);
    }

    #[test]
    fn waiting_vehicle_moves_when_green() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 250);
        let stop = sim.paths[1][0].stop_progress;
        assert!(sim.vehicles[0].progress <= stop + EPSILON);

        sim.lights.phase = Phase::Green;
        sim.lights.green_dir = 1;
        sim.lights.green_timer = 0;
        sim.step();
        assert!(sim.vehicles[0].progress > stop);
        assert_eq!(sim.vehicles[0].phase, VehiclePhase::Committed);
        assert_eq!(sim.queue_lengths()[1], 0);
        assert!(sim.clearance_pending());

        while sim.vehicles[0].phase == VehiclePhase::Committed {
            sim.step();
        }
        assert_eq!(sim.vehicles[0].phase, VehiclePhase::Crossing);
    }

    #[test]
    fn committed_vehicle_continues_when_signal_enters_clearing() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 250);

        sim.lights.phase = Phase::Green;
        sim.lights.green_dir = 1;
        sim.lights.green_timer = 0;
        sim.step();
        assert_eq!(sim.vehicles[0].phase, VehiclePhase::Committed);

        sim.lights.phase = Phase::Clearing;
        sim.lights.clear_timer = 0;
        let previous = sim.vehicles[0].progress;
        sim.step();

        assert!(sim.vehicles[0].progress > previous);
        assert_eq!(sim.lights.phase, Phase::Clearing);
    }

    #[test]
    fn clearing_waits_until_committed_vehicle_leaves_conflict_zone() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 250);

        sim.lights.phase = Phase::Green;
        sim.lights.green_dir = 1;
        sim.lights.green_timer = 0;
        sim.step();
        assert_eq!(sim.vehicles[0].phase, VehiclePhase::Committed);
        assert!(sim.spawn_with_route(2, 0));

        sim.lights.phase = Phase::Clearing;
        sim.lights.clear_timer = MIN_CLEAR_TICKS;
        for _ in 0..300 {
            sim.step();
            if sim
                .vehicles
                .iter()
                .find(|vehicle| vehicle.origin == 1)
                .is_some_and(|vehicle| vehicle.phase == VehiclePhase::Leaving)
            {
                break;
            }
            assert_eq!(sim.lights.phase, Phase::Clearing);
        }

        assert_eq!(sim.lights.phase, Phase::Clearing);
        assert!(sim
            .vehicles
            .iter()
            .find(|vehicle| vehicle.origin == 1)
            .is_some_and(|vehicle| vehicle.phase == VehiclePhase::Leaving));

        sim.step();
        assert_eq!(sim.lights.phase, Phase::Green);
        assert_eq!(sim.lights.green_dir, 2);
    }

    #[test]
    fn uncommitted_vehicle_stays_at_stop_progress_during_clearing() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 250);
        let stop = sim.paths[1][0].stop_progress;

        sim.lights.phase = Phase::Clearing;
        sim.lights.clear_timer = 0;
        run_steps(&mut sim, (MIN_CLEAR_TICKS - 1) as usize);

        assert!((sim.vehicles[0].progress - stop).abs() < 0.01);
        assert_eq!(sim.vehicles[0].phase, VehiclePhase::Waiting);
    }

    #[test]
    fn following_distance_is_never_below_car_plus_gap() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        run_steps(&mut sim, 30);
        assert!(sim.spawn_with_route(1, 0));
        run_steps(&mut sim, 400);

        let mut progress: Vec<_> = sim
            .vehicles
            .iter()
            .filter(|vehicle| vehicle.origin == 1)
            .map(|vehicle| vehicle.progress)
            .collect();
        progress.sort_by(f64::total_cmp);
        assert!(progress[1] - progress[0] + 0.001 >= FOLLOW_DISTANCE);
    }

    #[test]
    fn different_origins_keep_distance_on_a_shared_exit_lane() {
        let mut sim = Sim::new();
        let leader_remaining = 90.0;
        let follower_remaining = leader_remaining + FOLLOW_DISTANCE;
        let leader_progress = sim.paths[0][0].len - leader_remaining;
        let follower_progress = sim.paths[1][1].len - follower_remaining;
        assert!(leader_progress > sim.paths[0][0].conflict_exit);
        assert!(follower_progress > sim.paths[1][1].conflict_exit);
        sim.vehicles.push(vehicle_at(&sim, 0, 0, leader_progress));
        sim.vehicles.push(vehicle_at(&sim, 1, 1, follower_progress));

        for _ in 0..30 {
            sim.step();
            if sim.vehicles.len() < 2 {
                break;
            }
            let first = &sim.vehicles[0];
            let second = &sim.vehicles[1];
            assert!(!overlaps(first, second));
            let first_remaining = sim.paths[first.origin][first.route].len - first.progress;
            let second_remaining = sim.paths[second.origin][second.route].len - second.progress;
            assert!((second_remaining - first_remaining).abs() + EPSILON >= FOLLOW_DISTANCE);
        }
    }

    #[test]
    fn diverged_routes_resume_the_full_fixed_speed() {
        let mut sim = Sim::new();
        let straight_progress = sim.paths[0][0].conflict_exit + FOLLOW_DISTANCE;
        let right_progress = sim.paths[0][2].conflict_exit + FOLLOW_DISTANCE;
        sim.vehicles.push(vehicle_at(&sim, 0, 0, straight_progress));
        sim.vehicles.push(vehicle_at(&sim, 0, 2, right_progress));
        let before: Vec<_> = sim
            .vehicles
            .iter()
            .map(|vehicle| vehicle.progress)
            .collect();

        sim.step();

        for (vehicle, previous) in sim.vehicles.iter().zip(before) {
            assert!(
                (vehicle.progress - previous - VEHICLE_SPEED * FIXED_DT).abs() < 0.001,
                "route {} moved by {}",
                vehicle.route,
                vehicle.progress - previous
            );
        }
    }

    #[test]
    fn same_route_pair_is_not_held_by_diverging_reservation() {
        let mut sim = Sim::new();
        let turn_start = sim.paths[0][1].cum[1];
        let leader_progress = turn_start - 1.25;
        let follower_progress = leader_progress - FOLLOW_DISTANCE - 1.2;
        let turn_end = sim.paths[0][1].cum[sim.paths[0][1].cum.len() - 2];
        let step_distance = VEHICLE_SPEED * FIXED_DT;
        let follower_bounds = sim.paths[0][1]
            .vehicle_bounds(follower_progress + step_distance)
            .expanded(GAP);
        let mut sample = turn_start;
        let mut intersects_later_turn = false;
        while sample <= turn_end {
            intersects_later_turn |=
                follower_bounds.intersects(sim.paths[0][1].vehicle_bounds(sample).expanded(GAP));
            sample += SWEEP_SAMPLE_DISTANCE;
        }
        assert!(
            intersects_later_turn,
            "test setup must distinguish the full-turn reservation"
        );
        sim.vehicles.push(vehicle_at(&sim, 0, 1, leader_progress));
        sim.vehicles.push(vehicle_at(&sim, 0, 1, follower_progress));

        sim.step();

        assert!(
            (sim.vehicles[1].progress - follower_progress - VEHICLE_SPEED * FIXED_DT).abs() < 0.001
        );
    }

    #[test]
    fn left_turn_leader_never_enters_straight_follower_safety_envelope() {
        let mut sim = Sim::new();
        spawn_left_leader_and_straight_follower(&mut sim);

        for tick in 0..600 {
            sim.step();
            for first in 0..sim.vehicles.len() {
                for second in (first + 1)..sim.vehicles.len() {
                    assert!(
                        !safety_overlaps(&sim, first, second),
                        "expanded OBB overlap at tick {tick}: {first} and {second}"
                    );
                }
            }
            if sim.passed == 2 {
                return;
            }
        }

        panic!("diverging pair did not leave the simulation");
    }

    #[test]
    fn diverging_pair_does_not_deadlock() {
        let mut sim = Sim::new();
        spawn_left_leader_and_straight_follower(&mut sim);
        let turn_end = *sim.paths[0][1]
            .cum
            .get(sim.paths[0][1].cum.len() - 2)
            .unwrap();
        let mut stopped_progress = None;
        let mut leader_cleared_turn = false;

        for _ in 0..600 {
            let follower_before = sim
                .vehicles
                .iter()
                .find(|vehicle| vehicle.route == 0)
                .map(|vehicle| vehicle.progress);
            sim.step();
            let follower_after = sim
                .vehicles
                .iter()
                .find(|vehicle| vehicle.route == 0)
                .map(|vehicle| vehicle.progress);

            if stopped_progress.is_none()
                && follower_before
                    .zip(follower_after)
                    .is_some_and(|(before, after)| (after - before).abs() <= EPSILON)
            {
                stopped_progress = follower_after;
            }

            leader_cleared_turn = sim
                .vehicles
                .iter()
                .find(|vehicle| vehicle.route == 1)
                .is_none_or(|vehicle| vehicle.progress + EPSILON >= turn_end);
            if leader_cleared_turn {
                break;
            }
        }

        assert!(
            leader_cleared_turn,
            "left-turn leader did not complete its turn within the step limit"
        );
        let stopped_progress =
            stopped_progress.expect("straight follower was never proactively held");

        for _ in 0..120 {
            sim.step();
            if sim
                .vehicles
                .iter()
                .find(|vehicle| vehicle.route == 0)
                .is_none_or(|vehicle| vehicle.progress > stopped_progress + EPSILON)
            {
                return;
            }
        }

        panic!("straight follower did not resume after the leader cleared the turn");
    }

    #[test]
    fn spawn_spam_cannot_overlap_cars() {
        let mut sim = Sim::new();
        for _ in 0..100 {
            sim.spawn_with_route(2, 0);
        }
        assert_eq!(sim.vehicles.len(), 1);
        assert_eq!(sim.spawned, 1);
    }

    #[test]
    fn random_spawn_skips_blocked_origins_without_rejecting_the_request() {
        for blocked_count in 1..=2 {
            let mut sim = Sim::new();
            for origin in 0..blocked_count {
                assert!(sim.spawn_with_route(origin, 0));
            }

            assert!(sim.spawn_random_in_order([0, 1, 2, 3]));

            assert_eq!(sim.rejected, 0);
            assert_eq!(sim.vehicles.len(), blocked_count + 1);
            assert_eq!(sim.vehicles.last().unwrap().origin, blocked_count);
        }
    }

    #[test]
    fn random_spawn_rejects_once_only_when_every_origin_is_blocked() {
        let mut sim = Sim::new();
        for origin in 0..4 {
            assert!(sim.spawn_with_route(origin, 0));
        }

        assert!(!sim.spawn_random_in_order([3, 1, 0, 2]));

        assert_eq!(sim.rejected, 1);
        assert_eq!(sim.vehicles.len(), 4);
    }

    #[test]
    fn random_origin_order_contains_each_direction_once() {
        let mut rng = StdRng::seed_from_u64(0x01_ED00);
        for _ in 0..32 {
            let mut order = Sim::random_origin_order(&mut rng);
            order.sort_unstable();
            assert_eq!(order, [0, 1, 2, 3]);
        }
    }

    #[test]
    fn directional_spawn_rejects_one_request_once() {
        let mut sim = Sim::new();
        assert!(sim.spawn(0));

        assert!(!sim.spawn(0));

        assert_eq!(sim.rejected, 1);
    }

    #[test]
    fn route_never_changes_after_spawn() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(0, 2));
        for _ in 0..200 {
            sim.step();
            if let Some(vehicle) = sim.vehicles.first() {
                assert_eq!(vehicle.route, 2);
            }
        }
    }

    #[test]
    fn capacity_uses_the_subject_formula() {
        let expected = ((STOP_LINE_COORD - START) / (CAR_LEN + GAP)).floor() as usize;
        assert_eq!(capacity(), expected);
        assert_eq!(capacity(), 8);
    }

    #[test]
    fn completed_vehicle_is_removed_and_counted() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(0, 0));
        run_steps(&mut sim, 700);
        assert!(sim.vehicles.is_empty());
        assert_eq!(sim.passed, 1);
    }

    #[test]
    fn sustained_critical_lane_does_not_starve_single_vehicle() {
        let mut sim = Sim::new();
        let spawn_spacing_ticks =
            (FOLLOW_DISTANCE / (VEHICLE_SPEED * FIXED_DT)).ceil() as usize + 1;

        for _ in 0..(capacity() - 1) {
            assert!(sim.spawn_with_route(0, 0));
            hold_red(&mut sim, 0, spawn_spacing_ticks);
        }
        assert!(sim.spawn_with_route(1, 0));
        sim.lights = Lights::new();

        for tick in 0..(120 * FIXED_HZ) {
            if tick % 2 == 0 {
                sim.spawn_with_route(0, 0);
            }

            let entered_before = sim.entered[1];
            sim.step();

            if sim.entered[1] > entered_before {
                return;
            }
        }

        panic!("the single vehicle in direction 1 never entered Crossing");
    }

    #[test]
    fn headless_sixty_second_stress_test() {
        const SEEDS: [u64; 10] = [
            0x01_ED00, 0x01_ED01, 0x01_ED02, 0x01_ED03, 0x01_ED04, 0x01_ED05, 0x01_ED06, 0x01_ED07,
            0x01_ED08, 0x01_ED09,
        ];

        for seed in SEEDS {
            run_stress_seed(seed);
        }
    }

    fn run_stress_seed(seed: u64) {
        let mut sim = Sim::new();
        let mut rng = StdRng::seed_from_u64(seed);
        let mut served = [false; 4];
        let mut had_queue = [false; 4];
        let mut wait_started_at = [None; 4];
        let step_distance = VEHICLE_SPEED * FIXED_DT;
        let approach_ticks = (sim
            .paths
            .iter()
            .flatten()
            .map(|path| path.conflict_entry)
            .fold(0.0, f64::max)
            / step_distance)
            .ceil() as u32;
        let clearance_ticks = ((sim
            .paths
            .iter()
            .flatten()
            .map(|path| path.conflict_exit - path.conflict_entry)
            .fold(0.0, f64::max)
            + FOLLOW_DISTANCE)
            / step_distance)
            .ceil() as u32;
        let starvation_bound = approach_ticks
            + MAX_WAIT_TICKS
            + 3 * (MAX_GREEN_TICKS + clearance_ticks + MIN_CLEAR_TICKS);

        for origin in 0..4 {
            assert!(sim.spawn_with_route(origin, origin % 3));
        }

        for tick in 0..(60 * FIXED_HZ) {
            if tick > 0 && tick % 3 == 0 {
                let origin = rng.gen_range(0..4);
                let route = rng.gen_range(0..3);
                sim.spawn_with_route(origin, route);
            }

            let pending_before = pending_service_counts(&sim);
            let entered_before = sim.entered;
            for dir in 0..4 {
                if pending_before[dir] > 0 {
                    had_queue[dir] = true;
                    if wait_started_at[dir].is_none() {
                        wait_started_at[dir] = Some(tick);
                    }
                }
            }

            sim.step();
            let queues_after = sim.queue_lengths();
            let pending_after = pending_service_counts(&sim);

            for dir in 0..4 {
                let crossed = sim.entered[dir] > entered_before[dir];
                if crossed {
                    served[dir] = true;
                }

                if pending_after[dir] == 0 {
                    wait_started_at[dir] = None;
                } else if crossed || pending_before[dir] == 0 {
                    wait_started_at[dir] = Some(tick);
                }

                if let Some(started_at) = wait_started_at[dir] {
                    assert!(
                        tick - started_at <= starvation_bound,
                        "starvation in direction {dir}, seed {seed:#x}, \
                         tick {tick}, waited {} ticks (bound {starvation_bound})",
                        tick - started_at
                    );
                }
            }

            assert!(
                queues_after.iter().all(|&queue| queue <= capacity()),
                "capacity exceeded for seed {seed:#x} at tick {tick}: \
                 {queues_after:?}"
            );

            for a in 0..sim.vehicles.len() {
                for b in (a + 1)..sim.vehicles.len() {
                    let first = &sim.vehicles[a];
                    let second = &sim.vehicles[b];
                    let distance = (first.position.0 - second.position.0)
                        .hypot(first.position.1 - second.position.1);
                    assert!(
                        !overlaps(first, second),
                        "vehicle collision for seed {seed:#x} at tick {tick}: \
                         {a} and {b}"
                    );
                    assert!(
                        !safety_overlaps(&sim, a, b),
                        "expanded OBB overlap for seed {seed:#x} at tick {tick}: \
                         {a} ({:?}, route {}, progress {:.3}, angle {:.3}) and \
                         {b} ({:?}, route {}, progress {:.3}, angle {:.3})",
                        first.origin,
                        first.route,
                        first.progress,
                        first.angle,
                        second.origin,
                        second.route,
                        second.progress,
                        second.angle
                    );
                    if first.origin == second.origin {
                        assert!(
                            distance + 0.01 >= FOLLOW_DISTANCE,
                            "same-lane gap {distance} for seed {seed:#x} \
                             at tick {tick}"
                        );
                    }
                }
            }
        }

        for dir in 0..4 {
            assert!(
                !had_queue[dir] || served[dir],
                "direction {dir} accumulated a queue but never crossed, \
                 seed {seed:#x}"
            );
        }
        assert!(sim.passed > 0, "no vehicle passed for seed {seed:#x}");
        assert!(
            sim.lights.green_timer <= MAX_GREEN_TICKS || {
                let queues = sim.queue_lengths();
                queues
                    .iter()
                    .enumerate()
                    .all(|(dir, &queue)| dir == sim.lights.green_dir || queue == 0)
            },
            "green exceeded its maximum while another direction waited, \
             seed {seed:#x}"
        );
    }

    fn pending_service_counts(sim: &Sim) -> [usize; 4] {
        let mut pending = [0; 4];
        for vehicle in &sim.vehicles {
            if matches!(
                vehicle.phase,
                VehiclePhase::Approaching | VehiclePhase::Waiting | VehiclePhase::Committed
            ) {
                pending[vehicle.origin] += 1;
            }
        }
        pending
    }
}
