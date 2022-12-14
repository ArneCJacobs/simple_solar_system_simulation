use bevy_inspector_egui::Inspectable;
use bevy::prelude::*;
use serde::Deserialize;
use crate::physics::{Star, PointMass};
use crate::BodyBundle;

#[derive(Deserialize, Debug, Inspectable, Copy, Clone)]
pub struct StarConfig {
    #[inspectable(min=100.0)]
    pub mass: f32,
    #[inspectable(min=0.1)]
    radius: f32,
    pub velocity: Vec3,
    pub pos: Vec3,
    color: [u8; 3],
    star: Option<bool>,
}

impl Default for StarConfig {
    fn default() -> Self {
        StarConfig { 
            mass: default(), 
            radius: default(), 
            velocity: default(),
            pos: default(), 
            color: default(), 
            star: Some(default()), 
        }
    }
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
        let pbr = PbrBundle {
            transform: Transform {
                translation: self.pos.into(),
                scale: Vec3::splat(self.radius),
                ..default()
            },
            mesh,
            material: materials.add(color.into()),
            ..default()
        };
        let bundle = BodyBundle {
            pbr,
            celestial_body: PointMass {
                mass: self.mass,
                vel: self.velocity,
                ..default()
            },
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
                            color,
                            intensity: 4000.0,
                            range: 1000.0,
                            radius: self.radius+0.001,
                            ..default()
                        },
                        ..default()
                    });
                });
        }
    }
}

