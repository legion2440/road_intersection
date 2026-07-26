//! SDL2 road-intersection simulation.
//!
//! Arrow keys spawn from that side, `r` chooses a random side and `Esc` quits.

mod drawing;
mod geometry;
mod lights;
mod render;
mod sprites;
mod vehicle;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::{Duration, Instant};
use vehicle::Sim;

fn main() -> Result<(), String> {
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let window = video
        .window("road_intersection", geometry::W, geometry::H)
        .position_centered()
        .build()
        .map_err(|error| error.to_string())?;
    let mut canvas = window
        .into_canvas()
        .build()
        .map_err(|error| error.to_string())?;
    let texture_creator = canvas.texture_creator();
    let mut events = sdl.event_pump()?;

    let sprites = sprites::SpriteSheets::load(&texture_creator)
        .map_err(|error| format!("unable to initialize sprites: {error}"))?;
    let font = render::UiFont::load("assets/font.ttf").ok();
    if font.is_none() {
        eprintln!("note: assets/font.ttf is unavailable; HUD disabled");
    }

    let mut sim = Sim::new();
    let update_interval = Duration::from_nanos(1_000_000_000 / geometry::FIXED_HZ as u64);
    let mut previous_time = Instant::now();
    let mut accumulator = Duration::ZERO;
    let mut visual_tick = 0u64;

    'running: loop {
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown {
                    keycode: Some(key),
                    repeat,
                    ..
                } if !repeat || key == Keycode::Escape => {
                    let should_exit = handle_key(&mut sim, key);
                    if should_exit {
                        break 'running;
                    }
                }
                _ => {}
            }
        }

        let now = Instant::now();
        accumulator += now
            .saturating_duration_since(previous_time)
            .min(Duration::from_millis(250));
        previous_time = now;
        while accumulator >= update_interval {
            sim.step();
            visual_tick = visual_tick.wrapping_add(1);
            accumulator -= update_interval;
        }

        render::draw(
            &mut canvas,
            &texture_creator,
            &sim,
            &sprites,
            font.as_ref(),
            visual_tick,
        )?;
        std::thread::sleep(Duration::from_millis(1));
    }
    Ok(())
}

fn handle_key(sim: &mut Sim, key: Keycode) -> bool {
    match key {
        Keycode::Escape => return true,
        // Up = south, Down = north, Left = east, Right = west.
        Keycode::Up => {
            sim.spawn(2);
        }
        Keycode::Down => {
            sim.spawn(0);
        }
        Keycode::Left => {
            sim.spawn(1);
        }
        Keycode::Right => {
            sim.spawn(3);
        }
        Keycode::R => {
            sim.spawn_random();
        }
        _ => {}
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escape_requests_exit() {
        assert!(handle_key(&mut Sim::new(), Keycode::Escape));
    }

    #[test]
    fn arrow_keys_map_to_the_required_origins() {
        for (key, expected_origin) in [
            (Keycode::Up, 2),
            (Keycode::Down, 0),
            (Keycode::Left, 1),
            (Keycode::Right, 3),
        ] {
            let mut sim = Sim::new();
            assert!(!handle_key(&mut sim, key));
            assert_eq!(sim.vehicles[0].origin, expected_origin);
        }
    }

    #[test]
    fn r_spawns_from_a_random_origin() {
        let mut sim = Sim::new();
        assert!(!handle_key(&mut sim, Keycode::R));
        assert_eq!(sim.vehicles.len(), 1);
        assert!(sim.vehicles[0].origin < 4);
    }
}
