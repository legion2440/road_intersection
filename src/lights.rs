//! Fair, congestion-aware, single-approach traffic-light controller.

use crate::geometry::{capacity, FIXED_HZ};

pub const MIN_GREEN_TICKS: u32 = 3 * FIXED_HZ;
pub const MAX_GREEN_TICKS: u32 = 8 * FIXED_HZ;
pub const MIN_CLEAR_TICKS: u32 = 45;
pub const MAX_WAIT_TICKS: u32 = 15 * FIXED_HZ;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Phase {
    Green,
    Clearing,
}

#[derive(Debug)]
pub struct Lights {
    pub phase: Phase,
    pub green_dir: usize,
    pub green_timer: u32,
    pub clear_timer: u32,
    pub wait_ticks: [u32; 4],
    round_robin_cursor: usize,
}

impl Lights {
    pub fn new() -> Self {
        Lights {
            phase: Phase::Green,
            green_dir: 0,
            green_timer: 0,
            clear_timer: 0,
            wait_ticks: [0; 4],
            round_robin_cursor: 1,
        }
    }

    pub fn is_green(&self, origin: usize) -> bool {
        matches!(self.phase, Phase::Green) && self.green_dir == origin
    }

    pub fn update(&mut self, queues: &[usize; 4], conflict_occupied: bool) {
        self.update_wait_times(queues);

        match self.phase {
            Phase::Green => {
                self.green_timer = self.green_timer.saturating_add(1);
                let other_demand = queues
                    .iter()
                    .enumerate()
                    .any(|(dir, &queue)| dir != self.green_dir && queue > 0);
                let active_is_critical = queues[self.green_dir] >= Self::critical_threshold();
                let minimum_elapsed = self.green_timer >= MIN_GREEN_TICKS;
                let maximum_elapsed = self.green_timer >= MAX_GREEN_TICKS;
                let should_yield = other_demand
                    && minimum_elapsed
                    && (maximum_elapsed || queues[self.green_dir] == 0 || !active_is_critical);

                if should_yield {
                    self.phase = Phase::Clearing;
                    self.clear_timer = 0;
                }
            }
            Phase::Clearing => {
                self.clear_timer = self.clear_timer.saturating_add(1);
                if self.clear_timer >= MIN_CLEAR_TICKS && !conflict_occupied {
                    self.green_dir = self.choose_next(queues).unwrap_or(self.green_dir);
                    self.wait_ticks[self.green_dir] = 0;
                    self.round_robin_cursor = (self.green_dir + 1) % 4;
                    self.phase = Phase::Green;
                    self.green_timer = 0;
                }
            }
        }
    }

    fn critical_threshold() -> usize {
        ((capacity() as f64) * 0.85).ceil() as usize
    }

    fn update_wait_times(&mut self, queues: &[usize; 4]) {
        for (dir, &queue) in queues.iter().enumerate() {
            if queue == 0 {
                self.wait_ticks[dir] = 0;
            } else if !self.is_green(dir) {
                self.wait_ticks[dir] = self.wait_ticks[dir].saturating_add(1);
            }
        }
    }

    fn choose_next(&self, queues: &[usize; 4]) -> Option<usize> {
        let mut candidates: Vec<usize> = (0..4)
            .map(|offset| (self.round_robin_cursor + offset) % 4)
            .filter(|&dir| queues[dir] > 0)
            .collect();

        if candidates.iter().any(|&dir| dir != self.green_dir) {
            candidates.retain(|&dir| dir != self.green_dir);
        }

        let has_overdue = candidates
            .iter()
            .any(|&dir| self.wait_ticks[dir] >= MAX_WAIT_TICKS);
        if has_overdue {
            candidates.retain(|&dir| self.wait_ticks[dir] >= MAX_WAIT_TICKS);
        } else {
            let has_critical = candidates
                .iter()
                .any(|&dir| queues[dir] >= Self::critical_threshold());
            if has_critical {
                candidates.retain(|&dir| queues[dir] >= Self::critical_threshold());
            }
        }

        let mut best = None;
        let mut best_wait = 0;
        for dir in candidates {
            if best.is_none() || self.wait_ticks[dir] > best_wait {
                best = Some(dir);
                best_wait = self.wait_ticks[dir];
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clearing_waits_for_the_conflict_zone() {
        let mut lights = Lights::new();
        lights.phase = Phase::Clearing;
        let queues = [0, 1, 0, 0];
        for _ in 0..(MIN_CLEAR_TICKS + 20) {
            lights.update(&queues, true);
        }
        assert_eq!(lights.phase, Phase::Clearing);

        lights.update(&queues, false);
        assert_eq!(lights.phase, Phase::Green);
        assert_eq!(lights.green_dir, 1);
    }

    #[test]
    fn every_loaded_direction_eventually_gets_green() {
        let mut lights = Lights::new();
        let queues = [1, 1, 1, 1];
        let mut served = [false; 4];

        for _ in 0..(4 * (MAX_GREEN_TICKS + MIN_CLEAR_TICKS + 10)) {
            lights.update(&queues, false);
            if matches!(lights.phase, Phase::Green) {
                served[lights.green_dir] = true;
            }
            if served.iter().all(|served| *served) {
                break;
            }
        }

        assert!(served.iter().all(|served| *served), "{served:?}");
    }

    #[test]
    fn critical_lane_cannot_starve_noncritical_lane() {
        let mut lights = Lights::new();
        let queues = [capacity(), 1, 0, 0];
        let mut entered_clearing = false;

        for _ in 0..(MAX_GREEN_TICKS + MIN_CLEAR_TICKS + 10) {
            lights.update(&queues, false);
            entered_clearing |= matches!(lights.phase, Phase::Clearing);
            if entered_clearing && matches!(lights.phase, Phase::Green) {
                assert_eq!(lights.green_dir, 1);
                return;
            }
        }

        panic!("controller never completed the green-to-clearing transition");
    }

    #[test]
    fn current_lane_can_repeat_when_alone() {
        let mut lights = Lights::new();
        let queues = [capacity(), 0, 0, 0];

        for _ in 0..(2 * (MAX_GREEN_TICKS + MIN_CLEAR_TICKS)) {
            lights.update(&queues, false);
        }

        assert_eq!(lights.phase, Phase::Green);
        assert_eq!(lights.green_dir, 0);
    }

    #[test]
    fn current_lane_can_repeat_from_clearing_when_alone() {
        let mut lights = Lights::new();
        lights.phase = Phase::Clearing;
        lights.clear_timer = MIN_CLEAR_TICKS - 1;
        let queues = [capacity(), 0, 0, 0];

        lights.update(&queues, false);

        assert_eq!(lights.phase, Phase::Green);
        assert_eq!(lights.green_dir, 0);
    }

    #[test]
    fn overdue_lane_beats_critical_lane() {
        let mut lights = Lights::new();
        lights.phase = Phase::Clearing;
        lights.green_dir = 2;
        lights.clear_timer = MIN_CLEAR_TICKS - 1;
        lights.wait_ticks = [0, MAX_WAIT_TICKS, 0, 0];
        let queues = [capacity(), 1, 0, 0];

        lights.update(&queues, false);

        assert_eq!(lights.phase, Phase::Green);
        assert_eq!(lights.green_dir, 1);
    }

    #[test]
    fn oldest_overdue_lane_wins() {
        let mut lights = Lights::new();
        lights.phase = Phase::Clearing;
        lights.green_dir = 3;
        lights.clear_timer = MIN_CLEAR_TICKS - 1;
        lights.wait_ticks = [
            MAX_WAIT_TICKS + 20,
            MAX_WAIT_TICKS + 80,
            MAX_WAIT_TICKS + 40,
            0,
        ];
        let queues = [1, 1, 1, 0];

        lights.update(&queues, false);

        assert_eq!(lights.phase, Phase::Green);
        assert_eq!(lights.green_dir, 1);
    }
}
