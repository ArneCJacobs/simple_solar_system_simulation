#![feature(trait_alias)]

use std::collections::VecDeque;
use std::{env, fs};

use bevy::prelude::*;
use bevy::{pbr::AmbientLight, time::FixedTimestep};
use bevy_egui::EguiPlugin;
use bevy_egui::egui::plot::PlotPoints;
use bevy_inspector_egui::egui::plot::{Line, Legend, Plot, Values};
use bevy_inspector_egui::widgets::InspectorQuery;
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use bevy_inspector_egui::bevy_egui::{egui, EguiContext};
use bevy_prototype_debug_lines::*;
use itertools::izip;
use physics::{DELTA_TIME, PredictedPath, CelestialBody, Vector, calculate_circular_orbit_velocity, GRAVITY_CONSTANT};
use star_config::StarConfig;

mod star_config;
mod physics;


#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

#[derive(Inspectable, Default)]
struct Inspector {
    root_elements: InspectorQuery<&'static mut CelestialBody, &'static mut Transform>,
}

const LABEL: &str = "my_fixed_timestep";

fn main() {
    App::new()
        .add_startup_system(setup)
        .add_startup_system(update_setup)
        .add_plugins(DefaultPlugins)
        .add_system(look_at_star)
        .add_plugin(DebugLinesPlugin::with_depth_test(true))
        // .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(EguiPlugin)
        .add_plugin(InspectorPlugin::<StarConfig>::new())
        .add_plugin(InspectorPlugin::<Settings>::new())
        .register_type::<CelestialBody>()
        .add_plugin(InspectorPlugin::<Inspector>::new())
        .insert_resource(ClearColor(Color::BLACK))
        // transform gizmo
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugin(bevy_transform_gizmo::TransformGizmoPlugin::default())
        .insert_resource(AmbientLight {
            brightness: 0.03,
            ..default()
        })
        .add_system(ui_system)
        .add_system(physics::estimate_paths)
        .add_system(draw_paths)
        .add_stage_after(
            CoreStage::Update,
            FixedUpdateStage,
            SystemStage::single_threaded()
                .with_run_criteria(FixedTimestep::step(DELTA_TIME).with_label(LABEL))
                .with_system(physics::step_system)
        )
        .run();
}



#[derive(Inspectable)]
pub struct Settings {
    pub play: bool,
    #[inspectable(min = 1)]
    pub trail_length: u64,
    #[inspectable(min = 1)]
    pub trail_interval: u64,
    pub center_planet: Option<Entity>,
    pub energy_history_length: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            play: false,
            trail_length: 10_00,
            trail_interval: 10,
            center_planet: None,
            energy_history_length: 100,
        }
    }
}

fn look_at_star(
    mut camera: Query<&mut Transform, With<MainCamera>>,
    settings: Res<Settings>,
    stars: Query<&Transform, (With<CelestialBody>, Without<MainCamera>)>,
) {
    let mut camera = camera.single_mut();
    // let star = star.single();
    if let Some(star) = settings.center_planet {
        let star_pos = stars.get(star).unwrap();
        let new_rotation = camera
            .looking_at(star_pos.translation, Vec3::Y)
            .rotation
            .lerp(camera.rotation, 0.1);
        camera.rotation = new_rotation;

    }
}


fn ui_system(
    mut egui_context: ResMut<EguiContext>, 
    new_planet: ResMut<StarConfig>,
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    planets: Query<(Entity, &CelestialBody, &Transform)>,
    mut settings: ResMut<Settings>,
    mut energy_history: Local<VecDeque<(f32, f32, f32)>>,
) {

    let mesh = meshes.add(Mesh::from(shape::Icosphere {
        radius: 1.0,
        subdivisions: 3,
    }));

    egui::Window::new(" ").show(egui_context.ctx_mut(), |ui| {
        if ui.button("Spawn new planet").clicked() {
            new_planet.spawn(mesh.clone(), &mut materials, &mut commands);
            // *new_planet = StarConfig::default();
            // commands.spawn_bundle(new_planet.) 
        };
    });

    egui::Window::new("Select center planet").show(egui_context.ctx_mut(), |ui| {
        for (planet, _cb, _tr) in &planets {
            if ui.button(format!("{:?}", planet)).clicked() {
                settings.center_planet = Some(planet);
            }
        }
    });

    egui::Window::new("stats").show(egui_context.ctx_mut(), |ui| {
        let mut ke = 0.0;
        for (_, planet, _) in &planets {
            ke += 0.5 * planet.mass * planet.vel.length() * planet.vel.length();
        }

        let mut pe = 0.0;
        for [(_, cb1, pos1), (_, cb2, pos2)] in planets.iter_combinations() {
            let diff = pos2.translation - pos1.translation;
            pe -= GRAVITY_CONSTANT * cb1.mass * cb2.mass / diff.length(); 
        }
        if settings.play {
            energy_history.push_back((ke, pe, ke+pe));
            if energy_history.len() > settings.energy_history_length {
                energy_history.pop_front();
            } 
        }
        let plot = Plot::new("Energy history").legend(Legend::default());
        plot.show(ui, |plot_ui| {
            plot_ui.line(
                get_line_from_data(
                    energy_history.iter().map(|p| p.0).collect::<Vec<f32>>().as_slice(), 
                    "KE"
                )
            );

            plot_ui.line(
                get_line_from_data(
                    energy_history.iter().map(|p| p.1).collect::<Vec<f32>>().as_slice(), 
                    "PE"
                )
            );

            plot_ui.line(
                get_line_from_data(
                    energy_history.iter().map(|p| p.2).collect::<Vec<f32>>().as_slice(), 
                    "TE"
                )
            );
        });
        // ui.label(format!("KE: {:.2}, PE: {:.2}, TE: {:.2}", ke, pe, ke-pe));
    });
}

fn get_line_from_data(data: &[f32], name: &'_ str) -> Line {
    let points: Values = Values::from_ys_f32(data);
    Line::new(points).name(name)
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

    let mut result = serde_json::from_str::<Vec<StarConfig>>(&config);
    if let Err(err) = result {
        println!("{}", err);
    } else if let Ok(celestial_bodies) = &mut result {

        // let poss = Vector::from_iter(celestial_bodies.iter().map(|body| body.pos)); 
        // let masses = Vector::from_iter(celestial_bodies.iter().map(|body| body.mass)); 
        // let vels = calculate_circular_orbit_velocity(&poss, &masses); 
        // for (mut body, vel) in izip!(&mut celestial_bodies.into_iter(), vels) {
        //     body.velocity = vel;
        // }

        for celestial_body in celestial_bodies {
            celestial_body.spawn(mesh.clone(), &mut materials, &mut commands);
        }
    }

}

#[derive(Component)]
struct MainCamera;

fn setup(
    mut commands: Commands,
) {
    // camera
    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 10.5, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    })
        .insert_bundle(bevy_mod_picking::PickingCameraBundle::default())
        .insert(bevy_transform_gizmo::GizmoPickSource::default())
        .insert(MainCamera);
}


fn draw_paths(
    mut lines: ResMut<DebugLines>,
    query: Query<(Entity, &PredictedPath, &Transform)>,
    settings: Res<Settings>,
) {
    if let Some(centered_star_entity) = settings.center_planet {
        let (_, center_path, center_transform) = query.get(centered_star_entity).unwrap();

        for (entity, path, _) in &query {
            if entity == centered_star_entity {
                continue;
            }
            for i in 1..path.pos_vec.len() {
                lines.line(
                    path.pos_vec[i-1] - center_path.pos_vec[i-1] + center_transform.translation,
                    path.pos_vec[i] - center_path.pos_vec[i] + center_transform.translation,
                    0.0
                );
            } 
        }

    } else {
        for (_, path, _) in &query {
            for i in 1..path.pos_vec.len() {
                lines.line(
                    path.pos_vec[i-1],
                    path.pos_vec[i],
                    0.
                );
            } 
        }
    }
}
