#![feature(trait_alias)]

use std::{env, fs};

use bevy::prelude::*;
use bevy::{pbr::AmbientLight, time::FixedTimestep};
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use bevy_egui::{egui, EguiContext};
use bevy_prototype_debug_lines::*;
use physics::{DELTA_TIME, PredictedPath, CelestialBody};
use star_config::StarConfig;

mod star_config;
mod physics;


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
        // .add_system(physics::estimate_paths)
        .add_system(draw_paths)
        .add_stage_after(
            CoreStage::Update,
            FixedUpdateStage,
            SystemStage::single_threaded()
                .with_run_criteria(FixedTimestep::step(DELTA_TIME))
                .with_system(physics::interact_bodies)
                .with_system(physics::integrate.after(physics::interact_bodies)),
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



#[derive(Inspectable)]
pub struct Settings {
    pub play: bool,
    #[inspectable(min = 1)]
    pub trail_length: u64,
    #[inspectable(min = 1)]
    pub trail_interval: u64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            play: false,
            trail_length: 10_00,
            trail_interval: 20,
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
    let args: Vec<String> = env::args().collect();
    let config = fs::read_to_string(args.get(1).expect("Please give a file with the initial conditions")) // TODO configurable path
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
    } else if let Err(err) = result {
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
