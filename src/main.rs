//! SDL2 road-intersection simulation.
//!
//! Arrow keys spawn from that side, `r` chooses a random side, `Space` pauses,
//! `Backspace` resets, and `Esc` quits. The side-panel controls are clickable.

mod drawing;
mod geometry;
mod lights;
mod render;
mod sprites;
mod vehicle;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::mouse::MouseButton;
use sdl2::render::Canvas;
use sdl2::video::Window;
use std::time::{Duration, Instant};
use vehicle::Sim;

const FALLBACK_RENDER_HZ: u64 = 60;
const FORCE_FALLBACK_ENV: &str = "ROAD_INTERSECTION_FORCE_RENDER_FALLBACK";

fn main() -> Result<(), String> {
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let prefer_vsync = std::env::var_os(FORCE_FALLBACK_ENV).is_none();
    let (mut canvas, vsync_active) = create_canvas(&video, prefer_vsync)?;
    if !vsync_active {
        eprintln!("note: VSync unavailable or disabled; rendering is limited to 60 FPS");
    }
    let texture_creator = canvas.texture_creator();
    let mut events = sdl.event_pump()?;

    let sprites = sprites::SpriteSheets::load(&mut canvas, &texture_creator)
        .map_err(|error| format!("unable to initialize sprites: {error}"))?;
    let font = render::UiFont::load("assets/font.ttf").ok();
    if font.is_none() {
        eprintln!("note: assets/font.ttf is unavailable; side panel disabled");
    }

    let mut sim = Sim::new();
    let update_interval = Duration::from_nanos(1_000_000_000 / geometry::FIXED_HZ as u64);
    let fallback_render_interval = Duration::from_nanos(1_000_000_000 / FALLBACK_RENDER_HZ);
    let mut previous_time = Instant::now();
    let mut accumulator = Duration::ZERO;
    let mut visual_tick = 0u64;
    let mut paused = false;

    'running: loop {
        let frame_started = Instant::now();
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown {
                    keycode: Some(key),
                    repeat,
                    ..
                } if !repeat || key == Keycode::Escape => {
                    let should_exit = handle_key(&mut sim, &mut paused, key);
                    if should_exit {
                        break 'running;
                    }
                }
                Event::MouseButtonDown {
                    x,
                    y,
                    mouse_btn: MouseButton::Left,
                    ..
                } => {
                    if let Some(action) = render::panel_action_at(x, y) {
                        apply_panel_action(&mut sim, &mut paused, action);
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
            if !paused {
                sim.step();
                visual_tick = visual_tick.wrapping_add(1);
            }
            accumulator -= update_interval;
        }

        render::draw(
            &mut canvas,
            &texture_creator,
            &sim,
            &sprites,
            font.as_ref(),
            visual_tick,
            paused,
        )?;

        if !vsync_active {
            let delay = fallback_render_interval.saturating_sub(frame_started.elapsed());
            if !delay.is_zero() {
                std::thread::sleep(delay);
            }
        }
    }
    Ok(())
}

fn create_window(video: &sdl2::VideoSubsystem) -> Result<Window, String> {
    video
        .window("road_intersection", geometry::WIN_W, geometry::H)
        .position_centered()
        .build()
        .map_err(|error| format!("unable to create SDL window: {error}"))
}

fn create_canvas(
    video: &sdl2::VideoSubsystem,
    prefer_vsync: bool,
) -> Result<(Canvas<Window>, bool), String> {
    if prefer_vsync {
        let window = create_window(video)?;
        match window.into_canvas().present_vsync().build() {
            Ok(canvas) => return Ok((canvas, true)),
            Err(vsync_error) => {
                let fallback_window = create_window(video).map_err(|fallback_window_error| {
                    format!(
                        "unable to create renderer: VSync failed ({vsync_error}); \
                             fallback window failed ({fallback_window_error})"
                    )
                })?;
                let fallback = fallback_window
                    .into_canvas()
                    .build()
                    .map_err(|fallback_error| {
                        format!(
                            "unable to create renderer: VSync failed ({vsync_error}); \
                                 fallback failed ({fallback_error})"
                        )
                    })?;
                return Ok((fallback, false));
            }
        }
    }

    create_window(video)?
        .into_canvas()
        .build()
        .map(|canvas| (canvas, false))
        .map_err(|error| format!("unable to create fallback renderer: {error}"))
}

fn apply_panel_action(sim: &mut Sim, paused: &mut bool, action: render::PanelAction) {
    match action {
        render::PanelAction::Spawn(origin) => {
            sim.spawn(origin);
        }
        render::PanelAction::SpawnRandom => {
            sim.spawn_random();
        }
        render::PanelAction::TogglePause => *paused = !*paused,
        render::PanelAction::Reset => {
            *sim = Sim::new();
            *paused = false;
        }
    }
}

fn handle_key(sim: &mut Sim, paused: &mut bool, key: Keycode) -> bool {
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
        Keycode::Space => *paused = !*paused,
        Keycode::Backspace => {
            *sim = Sim::new();
            *paused = false;
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
        assert!(handle_key(&mut Sim::new(), &mut false, Keycode::Escape));
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
            assert!(!handle_key(&mut sim, &mut false, key));
            assert_eq!(sim.vehicles[0].origin, expected_origin);
        }
    }

    #[test]
    fn r_spawns_from_a_random_origin() {
        let mut sim = Sim::new();
        assert!(!handle_key(&mut sim, &mut false, Keycode::R));
        assert_eq!(sim.vehicles.len(), 1);
        assert!(sim.vehicles[0].origin < 4);
    }

    #[test]
    fn space_toggles_pause() {
        let mut sim = Sim::new();
        let mut paused = false;
        assert!(!handle_key(&mut sim, &mut paused, Keycode::Space));
        assert!(paused);
        assert!(!handle_key(&mut sim, &mut paused, Keycode::Space));
        assert!(!paused);
    }

    #[test]
    fn backspace_resets_simulation_and_pause() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(0, 0));
        let mut paused = true;

        assert!(!handle_key(&mut sim, &mut paused, Keycode::Backspace));

        assert!(sim.vehicles.is_empty());
        assert_eq!(sim.spawned, 0);
        assert_eq!(sim.passed, 0);
        assert_eq!(sim.rejected, 0);
        assert!(!paused);
    }

    #[test]
    fn panel_actions_use_the_same_simulation_commands() {
        let mut sim = Sim::new();
        let mut paused = false;

        apply_panel_action(&mut sim, &mut paused, render::PanelAction::Spawn(3));
        assert_eq!(sim.vehicles[0].origin, 3);

        apply_panel_action(&mut sim, &mut paused, render::PanelAction::TogglePause);
        assert!(paused);

        apply_panel_action(&mut sim, &mut paused, render::PanelAction::Reset);
        assert!(sim.vehicles.is_empty());
        assert!(!paused);
    }
}
