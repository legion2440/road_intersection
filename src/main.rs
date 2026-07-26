//! SDL2 road-intersection simulation.
//!
//! Arrow keys spawn from that side, `r` chooses a random side, `Space` pauses,
//! `Backspace` resets, and `Esc` quits. The side-panel controls are clickable.

mod collision;
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
const REJECT_FEEDBACK_DURATION: Duration = Duration::from_millis(200);

#[derive(Clone, Copy)]
struct RejectedFeedback {
    origin_mask: u8,
    visible_until: Instant,
}

fn main() -> Result<(), String> {
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let prefer_vsync = std::env::var_os(FORCE_FALLBACK_ENV).is_none();
    let (mut canvas, vsync_active) = create_canvas(&video, prefer_vsync)?;
    canvas
        .set_logical_size(geometry::WIN_W, geometry::H)
        .map_err(|error| format!("unable to configure logical canvas size: {error}"))?;
    if !vsync_active {
        eprintln!("note: VSync unavailable or disabled; rendering is limited to 60 FPS");
    }
    let texture_creator = canvas.texture_creator();
    let mut events = sdl.event_pump()?;

    let sprites = sprites::SpriteSheets::load(&mut canvas, &texture_creator)
        .map_err(|error| format!("unable to initialize sprites: {error}"))?;
    let font = render::UiFont::embedded()
        .map_err(|error| format!("unable to initialize embedded UI font: {error}"))?;

    let mut sim = Sim::new();
    let update_interval = Duration::from_nanos(1_000_000_000 / geometry::FIXED_HZ as u64);
    let fallback_render_interval = Duration::from_nanos(1_000_000_000 / FALLBACK_RENDER_HZ);
    let mut previous_time = Instant::now();
    let mut accumulator = Duration::ZERO;
    let mut visual_tick = 0u64;
    let mut paused = false;
    let mut rejected_feedback = None;

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
                    let should_exit =
                        handle_key(&mut sim, &mut paused, &mut rejected_feedback, key);
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
                        apply_panel_action(&mut sim, &mut paused, &mut rejected_feedback, action);
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
        let rejected_origin_mask = match rejected_feedback {
            Some(feedback) if now < feedback.visible_until => feedback.origin_mask,
            _ => {
                rejected_feedback = None;
                0
            }
        };
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
            &font,
            render::ViewState {
                visual_tick,
                paused,
                rejected_origin_mask,
            },
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
    let (width, height) = fitted_window_size(
        video
            .display_usable_bounds(0)
            .map(|bounds| (bounds.width(), bounds.height()))
            .unwrap_or((geometry::WIN_W, geometry::H)),
    );
    video
        .window("road_intersection", width, height)
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

fn fitted_window_size(usable: (u32, u32)) -> (u32, u32) {
    let scale = (usable.0 as f64 / geometry::WIN_W as f64)
        .min(usable.1 as f64 / geometry::H as f64)
        .min(1.0);
    (
        (geometry::WIN_W as f64 * scale).floor().max(1.0) as u32,
        (geometry::H as f64 * scale).floor().max(1.0) as u32,
    )
}

fn apply_panel_action(
    sim: &mut Sim,
    paused: &mut bool,
    rejected_feedback: &mut Option<RejectedFeedback>,
    action: render::PanelAction,
) {
    match action {
        render::PanelAction::Spawn(origin) => {
            spawn_from(sim, origin, rejected_feedback);
        }
        render::PanelAction::SpawnRandom => {
            spawn_random(sim, rejected_feedback);
        }
        render::PanelAction::TogglePause => *paused = !*paused,
        render::PanelAction::Reset => {
            *sim = Sim::new();
            *paused = false;
            *rejected_feedback = None;
        }
    }
}

fn handle_key(
    sim: &mut Sim,
    paused: &mut bool,
    rejected_feedback: &mut Option<RejectedFeedback>,
    key: Keycode,
) -> bool {
    match key {
        Keycode::Escape => return true,
        // Up = south, Down = north, Left = east, Right = west.
        Keycode::Up => {
            spawn_from(sim, 2, rejected_feedback);
        }
        Keycode::Down => {
            spawn_from(sim, 0, rejected_feedback);
        }
        Keycode::Left => {
            spawn_from(sim, 1, rejected_feedback);
        }
        Keycode::Right => {
            spawn_from(sim, 3, rejected_feedback);
        }
        Keycode::R => {
            spawn_random(sim, rejected_feedback);
        }
        Keycode::Space => *paused = !*paused,
        Keycode::Backspace => {
            *sim = Sim::new();
            *paused = false;
            *rejected_feedback = None;
        }
        _ => {}
    }
    false
}

fn spawn_from(sim: &mut Sim, origin: usize, rejected_feedback: &mut Option<RejectedFeedback>) {
    record_spawn_result(sim.spawn(origin), 1 << origin, rejected_feedback);
}

fn spawn_random(sim: &mut Sim, rejected_feedback: &mut Option<RejectedFeedback>) {
    record_spawn_result(sim.spawn_random(), 0b1111, rejected_feedback);
}

fn record_spawn_result(
    spawned: bool,
    rejected_origin_mask: u8,
    rejected_feedback: &mut Option<RejectedFeedback>,
) {
    *rejected_feedback = (!spawned).then(|| RejectedFeedback {
        origin_mask: rejected_origin_mask,
        visible_until: Instant::now() + REJECT_FEEDBACK_DURATION,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escape_requests_exit() {
        assert!(handle_key(
            &mut Sim::new(),
            &mut false,
            &mut None,
            Keycode::Escape
        ));
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
            assert!(!handle_key(&mut sim, &mut false, &mut None, key));
            assert_eq!(sim.vehicles[0].origin, expected_origin);
        }
    }

    #[test]
    fn r_spawns_from_a_random_origin() {
        let mut sim = Sim::new();
        assert!(!handle_key(&mut sim, &mut false, &mut None, Keycode::R));
        assert_eq!(sim.vehicles.len(), 1);
        assert!(sim.vehicles[0].origin < 4);
    }

    #[test]
    fn space_toggles_pause() {
        let mut sim = Sim::new();
        let mut paused = false;
        let mut feedback = None;
        assert!(!handle_key(
            &mut sim,
            &mut paused,
            &mut feedback,
            Keycode::Space
        ));
        assert!(paused);
        assert!(!handle_key(
            &mut sim,
            &mut paused,
            &mut feedback,
            Keycode::Space
        ));
        assert!(!paused);
    }

    #[test]
    fn backspace_resets_simulation_and_pause() {
        let mut sim = Sim::new();
        assert!(sim.spawn_with_route(0, 0));
        let mut paused = true;
        let mut feedback = Some(RejectedFeedback {
            origin_mask: 1,
            visible_until: Instant::now() + REJECT_FEEDBACK_DURATION,
        });

        assert!(!handle_key(
            &mut sim,
            &mut paused,
            &mut feedback,
            Keycode::Backspace
        ));

        assert!(sim.vehicles.is_empty());
        assert_eq!(sim.spawned, 0);
        assert_eq!(sim.passed, 0);
        assert_eq!(sim.rejected, 0);
        assert!(!paused);
        assert!(feedback.is_none());
    }

    #[test]
    fn panel_actions_use_the_same_simulation_commands() {
        let mut sim = Sim::new();
        let mut paused = false;
        let mut feedback = None;

        apply_panel_action(
            &mut sim,
            &mut paused,
            &mut feedback,
            render::PanelAction::Spawn(3),
        );
        assert_eq!(sim.vehicles[0].origin, 3);

        apply_panel_action(
            &mut sim,
            &mut paused,
            &mut feedback,
            render::PanelAction::TogglePause,
        );
        assert!(paused);

        apply_panel_action(
            &mut sim,
            &mut paused,
            &mut feedback,
            render::PanelAction::Reset,
        );
        assert!(sim.vehicles.is_empty());
        assert!(!paused);
    }

    #[test]
    fn rejected_direction_and_random_requests_report_the_correct_origins() {
        let mut sim = Sim::new();
        let mut feedback = None;
        spawn_from(&mut sim, 2, &mut feedback);
        spawn_from(&mut sim, 2, &mut feedback);
        assert_eq!(feedback.unwrap().origin_mask, 1 << 2);

        let mut sim = Sim::new();
        for origin in 0..4 {
            assert!(sim.spawn(origin));
        }
        spawn_random(&mut sim, &mut feedback);
        assert_eq!(feedback.unwrap().origin_mask, 0b1111);
    }

    #[test]
    fn window_size_fits_small_displays_without_changing_aspect_ratio() {
        let fitted = fitted_window_size((1366, 728));
        assert!(fitted.0 <= 1366);
        assert!(fitted.1 <= 728);
        let fitted_ratio = fitted.0 as f64 / fitted.1 as f64;
        let logical_ratio = geometry::WIN_W as f64 / geometry::H as f64;
        assert!((fitted_ratio - logical_ratio).abs() < 0.01);
        assert_eq!(
            fitted_window_size((1920, 1080)),
            (geometry::WIN_W, geometry::H)
        );
    }
}
