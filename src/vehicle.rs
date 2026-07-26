//! Vehicle state and the deterministic, fixed-timestep simulation.

use crate::geometry::*;
use crate::lights::Lights;
use rand::{seq::SliceRandom, Rng};

const EPSILON: f64 = 1.0e-6;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehiclePhase {
    Approaching,
    Waiting,
    Crossing,
    Leaving,
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub origin: usize,
    pub route: usize,
    pub progress: f64,
    pub position: (f64, f64),
    pub angle: f64,
    pub phase: VehiclePhase,
}

impl Vehicle {
    fn is_before_conflict(&self) -> bool {
        matches!(
            self.phase,
            VehiclePhase::Approaching | VehiclePhase::Waiting
        )
    }
}

pub struct Sim {
    pub paths: [[Path; 3]; 4],
    pub vehicles: Vec<Vehicle>,
    pub lights: Lights,
    pub spawned: u32,
    pub passed: u32,
    pub rejected: u32,
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
        let (x, y, angle, _) = path.at(0.0);
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
            if vehicle.is_before_conflict() {
                queues[vehicle.origin] += 1;
            }
        }
        queues
    }

    pub fn capacity(&self) -> usize {
        capacity()
    }

    pub fn conflict_occupied(&self) -> bool {
        self.vehicles
            .iter()
            .any(|vehicle| matches!(vehicle.phase, VehiclePhase::Crossing))
    }

    pub fn step(&mut self) {
        let queues = self.queue_lengths();
        self.lights.update(&queues, self.conflict_occupied());

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

            let mut leaders = Vec::new();
            for index in order {
                let vehicle = &self.vehicles[index];
                let path = &self.paths[vehicle.origin][vehicle.route];
                let mut target = (vehicle.progress + VEHICLE_SPEED * FIXED_DT).min(path.len);

                if vehicle.is_before_conflict() && !self.lights.is_green(vehicle.origin) {
                    target = target.min(path.conflict_entry);
                }

                for &leader_index in &leaders {
                    target = self.limit_behind_leader(
                        index,
                        target,
                        leader_index,
                        next_progress[leader_index],
                    );
                }

                next_progress[index] = target.max(vehicle.progress);
                leaders.push(index);
            }
        }

        for (index, vehicle) in self.vehicles.iter_mut().enumerate() {
            let previous = vehicle.progress;
            vehicle.progress = next_progress[index];
            let path = &self.paths[vehicle.origin][vehicle.route];
            let (x, y, angle, _) = path.at(vehicle.progress);
            vehicle.position = (x, y);
            vehicle.angle = angle;

            vehicle.phase = if vehicle.progress > path.conflict_exit - EPSILON {
                VehiclePhase::Leaving
            } else if vehicle.progress > path.conflict_entry + EPSILON {
                VehiclePhase::Crossing
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

        let follower_path = &self.paths[follower.origin][follower.route];
        let leader_path = &self.paths[leader.origin][leader.route];
        let (leader_x, leader_y, _, _) = leader_path.at(leader_progress);
        let separated = |progress: f64| {
            let (x, y, _, _) = follower_path.at(progress);
            (x - leader_x).hypot(y - leader_y) + EPSILON >= FOLLOW_DISTANCE
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

    fn overlaps(first: &Vehicle, second: &Vehicle) -> bool {
        let axes = [
            (first.angle.cos(), first.angle.sin()),
            (-first.angle.sin(), first.angle.cos()),
            (second.angle.cos(), second.angle.sin()),
            (-second.angle.sin(), second.angle.cos()),
        ];
        let corners = |vehicle: &Vehicle| {
            let forward = (vehicle.angle.cos(), vehicle.angle.sin());
            let side = (-forward.1, forward.0);
            std::array::from_fn(|index| {
                let longitudinal = if index & 1 == 0 { -0.5 } else { 0.5 };
                let lateral = if index & 2 == 0 { -0.5 } else { 0.5 };
                (
                    vehicle.position.0
                        + forward.0 * CAR_LEN * longitudinal
                        + side.0 * CAR_W * lateral,
                    vehicle.position.1
                        + forward.1 * CAR_LEN * longitudinal
                        + side.1 * CAR_W * lateral,
                )
            })
        };
        let first_corners: [(f64, f64); 4] = corners(first);
        let second_corners: [(f64, f64); 4] = corners(second);

        axes.iter().all(|axis| {
            let projection = |point: &(f64, f64)| point.0 * axis.0 + point.1 * axis.1;
            let (first_min, first_max) = first_corners
                .iter()
                .map(projection)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |range, value| {
                    (range.0.min(value), range.1.max(value))
                });
            let (second_min, second_max) = second_corners
                .iter()
                .map(projection)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |range, value| {
                    (range.0.min(value), range.1.max(value))
                });
            first_max > second_min + EPSILON && second_max > first_min + EPSILON
        })
    }

    #[test]
    fn vehicle_stays_at_the_stop_line_on_red() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 400);
        let vehicle = &sim.vehicles[0];
        let stop = sim.paths[1][0].conflict_entry;
        assert!((vehicle.progress - stop).abs() < 0.01);
        assert_eq!(vehicle.phase, VehiclePhase::Waiting);
    }

    #[test]
    fn waiting_vehicle_moves_when_green() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(1, 0));
        hold_red(&mut sim, 1, 250);
        let stop = sim.paths[1][0].conflict_entry;
        assert!(sim.vehicles[0].progress <= stop + EPSILON);

        sim.lights.phase = Phase::Green;
        sim.lights.green_dir = 1;
        sim.lights.green_timer = 0;
        sim.step();
        assert!(sim.vehicles[0].progress > stop);
        assert_eq!(sim.vehicles[0].phase, VehiclePhase::Crossing);
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
        let expected = (((CY - LANE) - START) / (CAR_LEN + GAP)).floor() as usize;
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

            let queues_before = sim.queue_lengths();
            sim.step();
            let queues_after = sim.queue_lengths();

            if queues_after[1] < queues_before[1] {
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

            let queues_before = sim.queue_lengths();
            for dir in 0..4 {
                if queues_before[dir] > 0 {
                    had_queue[dir] = true;
                    if wait_started_at[dir].is_none() {
                        wait_started_at[dir] = Some(tick);
                    }
                }
            }

            sim.step();
            let queues_after = sim.queue_lengths();

            for dir in 0..4 {
                let crossed = queues_after[dir] < queues_before[dir];
                if crossed {
                    served[dir] = true;
                }

                if queues_after[dir] == 0 {
                    wait_started_at[dir] = None;
                } else if crossed {
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
}
