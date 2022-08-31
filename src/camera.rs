use bevy::input::ButtonState;
use bevy::input::keyboard::KeyboardInput;
use bevy::prelude::*;
use bevy::input::mouse::{MouseWheel,MouseMotion};
use bevy::render::camera::Projection;
use bevy_inspector_egui::bevy_egui::EguiContext;
use crate::{Settings, FocusedEnity};
use crate::physics::PointMass;



#[derive(Component)]
pub struct MainCamera;

// https://bevy-cheatbook.github.io/cookbook/pan-orbit-camera.html
/// Tags an entity as capable of panning and orbiting.
#[derive(Component)]
pub struct PanOrbitCamera {
    /// The "focus point" to orbit around. It is automatically updated when panning the camera
    pub focus: Vec3,
    pub radius: f32,
    pub upside_down: bool,
}

impl Default for PanOrbitCamera {
    fn default() -> Self {
        PanOrbitCamera {
            focus: Vec3::ZERO,
            radius: 5.0,
            upside_down: false,
        }
    }
}

pub fn set_focus_camera(
    mut camera_query: Query<&mut PanOrbitCamera, With<MainCamera>>,
    stars: Query<&Transform, (With<PointMass>, Without<MainCamera>)>,
    focused_entity: Res<FocusedEnity>,
) {
    // let star = star.single();
    if let Some(star) = focused_entity.0 {
        let star_pos = stars.get(star.into()).unwrap();
        let mut camera = camera_query.get_single_mut().unwrap();
        camera.focus = star_pos.translation;
    }
    
    
        // let new_rotation = camera
        //     .looking_at(star_pos.translation, Vec3::Y)
        //     .rotation
        //     .lerp(camera.rotation, 0.1);
        // camera.rotation = new_rotation;

}
/// Pan the camera with middle mouse click, zoom with scroll wheel, orbit with right mouse click.
pub fn pan_orbit_camera(
    windows: Res<Windows>,
    mut ev_motion: EventReader<MouseMotion>,
    mut ev_scroll: EventReader<MouseWheel>,
    input_mouse: Res<Input<MouseButton>>,
    mut query: Query<(&mut PanOrbitCamera, &mut Transform, &Projection)>,
    input_keyboard: Res<Input<KeyCode>>,
    egui_context_opt: Option<ResMut<EguiContext>>, // egui context added by bevy_egui
) {

    let hovering_over_egui = match egui_context_opt {
        Some(mut egui_context) => egui_context.ctx_mut().is_pointer_over_area(),
        None => false,
    };
    if hovering_over_egui {
        return;
    }
    // change input mapping for orbit and panning here
    let orbit_button = MouseButton::Right;
    let pan_button = MouseButton::Middle;

    let mut pan = Vec2::ZERO;
    let mut rotation_move = Vec2::ZERO;
    let mut scroll = 0.0;
    let mut orbit_button_changed = false;

    if input_mouse.pressed(orbit_button) {
        for ev in ev_motion.iter() {
            rotation_move += ev.delta;
        }
    } else if input_mouse.pressed(pan_button) {
        // Pan only if we're not rotating at the moment
        for ev in ev_motion.iter() {
            pan += ev.delta;
        }
    }
    for ev in ev_scroll.iter() {
        scroll += ev.y;
    }
    if input_mouse.just_released(orbit_button) || input_mouse.just_pressed(orbit_button) {
        orbit_button_changed = true;
    }

    for (mut pan_orbit, mut transform, projection) in query.iter_mut() {
        if orbit_button_changed {
            // only check for upside down when orbiting started or ended this frame
            // if the camera is "upside" down, panning horizontally would be inverted, so invert the input to make it correct
            let up = transform.rotation * Vec3::Y;
            pan_orbit.upside_down = up.y <= 0.0;
        }

        let mut any = false;

        if pan_orbit.is_changed() {
            any = true;
        }
        if rotation_move.length_squared() > 0.0 {
            any = true;
            let window = get_primary_window_size(&windows);
            let delta_x = {
                let delta = rotation_move.x / window.x * std::f32::consts::PI * 2.0;
                if pan_orbit.upside_down { -delta } else { delta }
            };
            let delta_y = rotation_move.y / window.y * std::f32::consts::PI;
            let yaw = Quat::from_rotation_y(-delta_x);
            let pitch = Quat::from_rotation_x(-delta_y);
            transform.rotation = yaw * transform.rotation; // rotate around global y axis
            transform.rotation = transform.rotation * pitch; // rotate around local x axis
        } else if pan.length_squared() > 0.0 {
            any = true;
            // make panning distance independent of resolution and FOV,
            let window = get_primary_window_size(&windows);
            if let Projection::Perspective(projection) = projection {
                pan *= Vec2::new(projection.fov * projection.aspect_ratio, projection.fov) / window;
            }
            // translate by local axes
            let right = transform.rotation * Vec3::X * -pan.x;
            let up = transform.rotation * Vec3::Y * pan.y;
            // make panning proportional to distance away from focus point
            let translation = (right + up) * pan_orbit.radius;
            pan_orbit.focus += translation;
        } else if scroll.abs() > 0.0 {
            any = true;
            pan_orbit.radius -= scroll * pan_orbit.radius * 0.2;
            // dont allow zoom to reach zero or you get stuck
            pan_orbit.radius = f32::max(pan_orbit.radius, 0.05);
        }

        if any {
            // emulating parent/child to make the yaw/y-axis rotation behave like a turntable
            // parent = x and y rotation
            // child = z-offset
            let rot_matrix = Mat3::from_quat(transform.rotation);
            transform.translation = pan_orbit.focus + rot_matrix.mul_vec3(Vec3::new(0.0, 0.0, pan_orbit.radius));
        } else {
            let translation_amount = 0.1;
            // for event in keyboard_events.iter() {
            // if event.state != ButtonState::Pressed || event.key_code.is_none() {
            //     continue;
            // }
            let mut translation = Vec3::ZERO;
            
            if input_keyboard.any_pressed([KeyCode::W, KeyCode::Up]) {
                translation += transform.forward() * translation_amount;
            }

            if input_keyboard.any_pressed([KeyCode::S, KeyCode::Back]) {
                translation += transform.back() * translation_amount;
            }

            if input_keyboard.any_pressed([KeyCode::A, KeyCode::Left]) {
                translation += transform.left() * translation_amount;
            }

            if input_keyboard.any_pressed([KeyCode::D, KeyCode::Right]) {
                translation += transform.right() * translation_amount;
            }

            if input_keyboard.any_pressed([KeyCode::Space]) {
                translation += transform.up() * translation_amount;
            }

            if input_keyboard.any_pressed([KeyCode::LShift]) {
                translation += transform.down() * translation_amount;
            }
            transform.translation += translation;
            pan_orbit.focus += translation; 
        }
    }
}

pub fn get_primary_window_size(windows: &Res<Windows>) -> Vec2 {
    let window = windows.get_primary().unwrap();
    let window = Vec2::new(window.width() as f32, window.height() as f32);
    window
}

/// Spawn a camera like this
pub fn spawn_camera(mut commands: Commands) {
    let translation = Vec3::new(0.0, 10.5, -30.0);
    let radius = translation.length();

    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_translation(translation)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    }).insert(PanOrbitCamera {
        radius,
        ..Default::default()
    })
        .insert_bundle(bevy_mod_picking::PickingCameraBundle::default())
        .insert(bevy_transform_gizmo::GizmoPickSource::default())
        .insert(MainCamera);
}
