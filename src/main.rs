use std::fs;

use bevy::prelude::*;
use bevy::{pbr::AmbientLight, time::FixedTimestep};
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use serde::Deserialize;
use bevy_egui::{egui, EguiContext};
use bevy_prototype_debug_lines::*;


#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

fn main() {
    App::new()
        .add_startup_system(setup)
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::with_depth_test(true))
        // .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(EguiPlugin)
        .add_plugin(InspectorPlugin::<StarConfig>::new())
        .add_plugin(InspectorPlugin::<Settings>::new())
        .insert_resource(ClearColor(Color::BLACK))
        // transform gizmo
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugin(bevy_transform_gizmo::TransformGizmoPlugin::default())
        .insert_resource(AmbientLight {
            brightness: 0.03,
            ..default()
        })
        .add_system(ui_system)
        .add_startup_system(update_setup)
        .add_system(estimate_paths)
        .add_system(draw_paths)
        .add_stage_after(
            CoreStage::Update,
            FixedUpdateStage,
            SystemStage::parallel()
                .with_run_criteria(FixedTimestep::step(DELTA_TIME))
                .with_system(interact_bodies)
                .with_system(integrate),
        )
        .run();

    // let theta_0 = 1200.0; // kelvin
    // let h = 240.0; // seconds
    // let f = |_t: f32, theta: f32| -2.2067e-12 * (theta.powf(4.0) - 81e8);
    // let mut theta = theta_0;
    // let mut t = 0.0;
    // // println!("{}", f(theta_0, 0.0));
    // println!("it 0, theta: {}, t: {}", theta, t);
    // for it in 0..=2 {
    //     theta = runge_kutta_4(f, h, t, theta); 
    //     println!("it {}, theta: {}, t: {}", it, theta, t);
    //     t += h;
    // }
}
const GRAVITY_CONSTANT: f32 = 0.003;
const DELTA_TIME: f64 = 0.01;

#[derive(Component, Default, Deserialize, Debug, Copy, Clone)]
struct Mass(f32);
#[derive(Reflect, Component, Default, Debug, Clone)]
struct Acceleration(Vec3);
#[derive(Reflect, Component, Default, Debug, Clone)]
struct PrevAcceleration(Vec3);
#[derive(Reflect, Component, Default, Debug, Clone)]
struct Velocity(Vec3);
#[derive(Reflect, Component)]
struct Star;
#[derive(Component, Default)]
struct CelestialBody;
#[derive(Component, Default, Clone)]
struct PredictedPath {
    pos_vec: Vec<Vec3>
}

#[derive(Deserialize, Debug, Inspectable, Default)]
struct StarConfig {
    #[inspectable(min=100.0, max=10.0e5)]
    mass: f32,
    #[inspectable(min=0.001, max=100.0)]
    radius: f32,
    velocity: Vec3,
    pos: Vec3,
    color: [u8; 3],
    star: Option<bool>,
}


impl StarConfig {
    fn spawn(
        &self, 
        mesh: Handle<Mesh>, 
        materials: &mut ResMut<Assets<StandardMaterial>>,
        commands: &mut Commands,
    ) {
        let [r, g, b]: [u8; 3] = self.color.clone().try_into().unwrap();
        let color = Color::rgb_u8(r, g, b);
        let bundle = BodyBundle { 
            pbr: PbrBundle {
                transform: Transform {
                    translation: self.pos.into(),
                    scale: Vec3::splat(self.radius),
                    ..default()
                },
                mesh,
                material: materials.add(color.into()),
                ..default()
            },
            mass: Mass(self.mass),
            acceleration: Acceleration(Vec3::ZERO),
            prev_acceleration: PrevAcceleration(Vec3::ZERO),
            velocity: Velocity(self.velocity),
            ..default()
        }; 

        let mut command = commands.spawn_bundle(bundle);

        command
            .insert_bundle(bevy_mod_picking::PickableBundle::default())
            .insert(bevy_transform_gizmo::GizmoTransformable);

        if self.star.unwrap_or(false) {

            let material = materials.add(StandardMaterial {
                base_color: color,
                emissive: (color * 2.),
                ..default()
            });

            command.insert(material);

            command
                .insert(Star)
                .with_children(|p| {
                    p.spawn_bundle(PointLightBundle {
                        point_light: PointLight {
                            color: Color::WHITE,
                            intensity: 400.0,
                            range: 100.0,
                            radius: self.radius,
                            ..default()
                        },
                        ..default()
                    });
                });
        }
    }
}



#[derive(Inspectable)]
struct Settings {
    play: bool,
    #[inspectable(min = 1)]
    trail_length: u64,
    #[inspectable(min = 1)]
    trail_interval: u64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            play: false,
            trail_length: 500,
            trail_interval: 10,
        }
    }
}


fn ui_system(
    mut egui_context: ResMut<EguiContext>, 
    new_planet: ResMut<StarConfig>,
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {

    let mesh = meshes.add(Mesh::from(shape::Icosphere {
        radius: 1.0,
        subdivisions: 3,
    }));

    egui::Window::new(" ").show(egui_context.ctx_mut(), |ui| {
        if ui.button("Spawn new planet").clicked() {
            new_planet.spawn(mesh, &mut materials, &mut commands);
            // *new_planet = StarConfig::default();
            // commands.spawn_bundle(new_planet.) 
        };
    });
}


#[derive(Bundle, Default)]
struct BodyBundle {
    #[bundle]
    pbr: PbrBundle,
    mass: Mass,
    velocity: Velocity,
    acceleration: Acceleration,
    prev_acceleration: PrevAcceleration,
    celestial_body: CelestialBody,
    predicted_path: PredictedPath,
}

#[derive(Default)]
struct LastConfig(String);

fn update_setup(
    mut commands: Commands,
    planets: Query<Entity, With<CelestialBody>>,
    mut last_config: Local<LastConfig>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let config = fs::read_to_string("setup.json") // TODO configurable path
        .expect("Could not read file");


    if last_config.0 == config {
        return;
    }

    last_config.0 = config.clone();

    let mesh = meshes.add(Mesh::from(shape::Icosphere {
        radius: 1.0,
        subdivisions: 3,
    }));

    planets.for_each(|entity| commands.entity(entity).despawn());

    let result = serde_json::from_str::<Vec<StarConfig>>(&config);
    if let Ok(celestial_bodies) = &result {
        for celestial_body in celestial_bodies {
            celestial_body.spawn(mesh.clone(), &mut materials, &mut commands);
        }
    } if let Err(err) = result {
        println!("{}", err);
    }

}

fn setup(
    mut commands: Commands,
) {

    // camera
    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 10.5, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    })
        .insert_bundle(bevy_mod_picking::PickingCameraBundle::default())
        .insert(bevy_transform_gizmo::GizmoPickSource::default());

}

fn interact_bodies(
    mut query: Query<(&Mass, &GlobalTransform, &mut Acceleration)>,
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }

    let mut iter = query.iter_combinations_mut();
    while let Some(
        [
            (m1, transform1, mut acc1), 
            (m2, transform2, mut acc2)
        ]) = iter.fetch_next() {
        newtonian_gravity(
            &m1, &transform1.translation(), &mut acc1, 
            &m2, &transform2.translation(), &mut acc2
        );
    }
}

fn newtonian_gravity(
    Mass(m1): &Mass, &pos1: &Vec3, acc1: &mut Acceleration,
    Mass(m2): &Mass, &pos2: &Vec3, acc2: &mut Acceleration,
) {
    let delta = pos2 - pos1;
    let distance_sq: f32 = delta.length_squared();

    let f = GRAVITY_CONSTANT / distance_sq;
    let force_unit_mass = delta * f;

    acc1.0 += force_unit_mass * (*m2);
    acc2.0 -= force_unit_mass * (*m1);

}

fn integrate(
    mut query: Query<(&mut Acceleration, &mut PrevAcceleration, &mut Transform, &mut Velocity)>, 
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }
    let dt_sq = (DELTA_TIME * DELTA_TIME) as f32;
    let dt = DELTA_TIME as f32;
    for(mut acceleration, mut prev_acceleration, mut transform, mut velocity) in &mut query {
        // let new_pos = transform.translation + velocity.0 * dt + acceleration.0 * (dt_sq * 0.5);
        // let new_acc = Vec3::ZERO;
        // let new_vel = velocity.0 + (acceleration.0 + prev_acceleration.0) * (dt * 0.5);

        prev_acceleration.0 = acceleration.0;
        let (new_pos, new_vel, new_acc) = velocity_verlet(
            dt, 
            dt_sq, 
            acceleration.0, 
            prev_acceleration.0, 
            transform.translation, 
            velocity.0
        );
        transform.translation = new_pos;
        acceleration.0 = new_acc;
        velocity.0 = new_vel;
    }
}

struct System {
    entity: Entity,
    mass: Mass,
    acc: Acceleration, 
    prev_acc: PrevAcceleration,
    pos: Vec3, 
    vel: Velocity, 
    path: PredictedPath,
}

fn estimate_paths(
    mut query: Query<(Entity, &Mass, &Acceleration, &PrevAcceleration, &Transform, &Velocity, &mut PredictedPath)>, 
    settings: Res<Settings>,
) {
    if settings.play {
        return;
    }
    let mut system: Vec<System> = vec![];
    for(entity, mass, acceleration, prev_acceleration, transform, velocity, mut path) in &mut query {
        path.pos_vec.clear();

        system.push(System {
            entity,
            mass: mass.clone(),
            pos: transform.translation.clone(),
            vel: velocity.clone(),
            acc: acceleration.clone(),
            prev_acc: prev_acceleration.clone(),
            path: PredictedPath{ pos_vec: vec![] },
        });
    }

    for i in 0..(settings.trail_length * settings.trail_interval) {
        // update accelration
        for index1 in 0..system.len() {
            for index2 in 0..system.len() {
                if index1 == index2 {
                    continue;
                }
                let s1 = &system[index1]; 
                let s2 = &system[index2]; 
                let mut acc1 = s1.acc.clone();
                let mut acc2 = s2.acc.clone();
                newtonian_gravity(
                    &s1.mass, &s1.pos, &mut acc1, 
                    &s2.mass, &s2.pos, &mut acc2
                );
                system.get_mut(index1).unwrap().acc = acc1;
                system.get_mut(index2).unwrap().acc = acc2;
            }
        }

        // update pos, vel

        let dt = DELTA_TIME as f32;
        let dt_sq = (DELTA_TIME * DELTA_TIME) as f32;
        for s in &mut system {
            let (new_pos, new_vel, new_acc) = velocity_verlet(dt, dt_sq, s.acc.0, s.prev_acc.0, s.pos, s.vel.0);
            s.pos = new_pos;
            s.prev_acc = PrevAcceleration(s.acc.0);
            s.acc = Acceleration(new_acc);
            s.vel = Velocity(new_vel);


            if (i % settings.trail_interval) == 0 {
                s.path.pos_vec.push(new_pos);
            } 
        }
    }

    for s in &system {
        let (_, _, _, _, _, _, mut path) = query.get_mut(s.entity).unwrap();
        path.pos_vec.clone_from(&s.path.pos_vec);
    }

}

fn draw_paths(
    mut lines: ResMut<DebugLines>,
    query: Query<&PredictedPath>,
) {
    for path in &query {
        for i in 1..path.pos_vec.len() {
            lines.line(
                path.pos_vec[i-1],
                path.pos_vec[i],
                0.
            );
        } 
    }

}

fn velocity_verlet(
    dt: f32,
    dt_sq: f32,
    acceleration: Vec3,
    prev_acceleration: Vec3,
    pos: Vec3,
    velocity: Vec3,
) -> (Vec3, Vec3, Vec3) {
    let new_pos = pos + velocity * dt + acceleration * (dt_sq * 0.5);
    let new_acc = Vec3::ZERO;
    let new_vel = velocity + (acceleration + prev_acceleration) * (dt * 0.5);

    return (new_pos, new_vel, new_acc);
}

// fn runge_kutta_4<F>(f: F, h: f32, x0: f32, y0: f32) -> f32
// where F: Fn(f32, f32) -> f32 
// {
//     let k1 = f(x0, y0);
//     let k2 = f(x0 + 0.5 * h, y0 + 0.5 * h * k1);
//     let k3 = f(x0 + 0.5 * h, y0 + 0.5 * h * k2);
//     let k4 = f(x0 + h, y0 + h * k3);
//     y0 + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * h / 6.0
// }
