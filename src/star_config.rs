use bevy_inspector_egui::Inspectable;
use crate::{Acceleration, Assets, BodyBundle, BuildChildren, Color, Commands, default, Handle, Mass, Mesh, PbrBundle, PointLight, PointLightBundle, PrevAcceleration, ResMut, StandardMaterial, Star, Transform, Vec3, Velocity};
use serde::Deserialize;

#[derive(Deserialize, Debug, Inspectable, Default)]
pub struct StarConfig {
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
    pub fn spawn(
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

