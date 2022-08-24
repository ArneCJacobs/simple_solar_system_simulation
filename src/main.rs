use std::fs;

use bevy::prelude::*;
use bevy::{pbr::AmbientLight, time::FixedTimestep};
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use serde::Deserialize;
use bevy_egui::{egui, EguiContext};


#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(EguiPlugin)
        .add_plugin(InspectorPlugin::<StarConfig>::new())
        .add_system(ui_system)
        .insert_resource(ClearColor(Color::BLACK))
        // transform gizmo
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugin(bevy_transform_gizmo::TransformGizmoPlugin::default())
        .insert_resource(AmbientLight {
            brightness: 0.03,
            ..default()
        })
        .add_startup_system(setup)
        .add_system(update_setup)
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
#[derive(Reflect, Component, Default, Debug)]
struct Acceleration(Vec3);
#[derive(Reflect, Component, Default, Debug)]
struct LastPos(Vec3);
#[derive(Reflect, Component)]
struct Star;
#[derive(Component, Default)]
struct CelestialBody;

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
    fn get_last_pos(&self) -> LastPos {
        LastPos(self.pos - (DELTA_TIME as f32) * self.velocity)
    }

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
            last_pos: self.get_last_pos(),
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
    last_pos: LastPos,
    acceleration: Acceleration,
    celestial_body: CelestialBody,
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

fn interact_bodies(mut query: Query<(&Mass, &GlobalTransform, &mut Acceleration)>) {
    let mut iter = query.iter_combinations_mut();
    while let Some([(Mass(m1), transform1, mut acc1), (Mass(m2), transform2, mut acc2)]) = iter.fetch_next() {
        let delta = transform2.translation() - transform1.translation();
        let distance_sq: f32 = delta.length_squared();

        let f = GRAVITY_CONSTANT / distance_sq;
        let force_unit_mass = delta * f;
        acc1.0 += force_unit_mass * (*m2);
        acc2.0 -= force_unit_mass * (*m1);
    }
}

fn integrate(mut query: Query<(&mut Acceleration, &mut Transform, &mut LastPos)>) {
    let dt_sq = (DELTA_TIME * DELTA_TIME) as f32;
    for(mut acceleration, mut transform, mut last_pos) in &mut query {
        let new_pos = 2.0 * transform.translation - last_pos.0 + acceleration.0 * dt_sq;
        acceleration.0 = Vec3::ZERO;
        last_pos.0 = transform.translation;
        transform.translation = new_pos;
    }

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
