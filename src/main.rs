use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_4},
    ops::Range,
};

use bevy::{input::mouse::AccumulatedMouseScroll, prelude::*};

#[derive(Debug, Resource)]
struct CameraSettings {
    pub orbit_distance: f32,
    pub pitch_speed: f32,
    // Clamp pitch to this range
    pub pitch_range: Range<f32>,
    pub roll_speed: f32,
    pub yaw_speed: f32,
}

impl Default for CameraSettings {
    fn default() -> Self {
        // Limiting pitch stops some unexpected rotation past 90° up or down.
        let pitch_limit = FRAC_PI_2 - 0.01;
        Self {
            // These values are completely arbitrary, chosen because they seem to produce
            // "sensible" results for this example. Adjust as required.
            orbit_distance: 5.0,
            pitch_speed: -0.03,
            pitch_range: -pitch_limit..pitch_limit,
            roll_speed: 1.0,
            yaw_speed: -0.040,
        }
    }
}

// Loading in the python simulation results
#[derive(Resource)]
struct MultiNpyData {
    datasets: Vec<NpyDataset>,
    last_accessed_index: usize,
}
struct NpyDataset {
    values: Vec<f64>,
}
fn load_npy_data(mut commands: Commands) {
    let r_b_x_bytes = std::fs::read("data/r_b_x.npy").expect("unable to read npy file");
    let r_b_x_values = npyz::NpyFile::new(&r_b_x_bytes[..])
        .expect("unable to parse npy file")
        .into_vec::<f64>()
        .expect("unable to read f64");
    let r_b_x = NpyDataset {
        values: (r_b_x_values),
    };

    let r_b_y_bytes = std::fs::read("data/r_b_y.npy").expect("unable to read npy file");
    let r_b_y_values = npyz::NpyFile::new(&r_b_y_bytes[..])
        .expect("unable to parse npy file")
        .into_vec::<f64>()
        .expect("unable to read f64");
    let r_b_y = NpyDataset {
        values: (r_b_y_values),
    };

    let r_b_z_bytes = std::fs::read("data/r_b_z.npy").expect("unable to read npy file");
    let r_b_z_values = npyz::NpyFile::new(&r_b_z_bytes[..])
        .expect("unable to parse npy file")
        .into_vec::<f64>()
        .expect("unable to read f64");
    let r_b_z = NpyDataset {
        values: (r_b_z_values),
    };

    let omega_b_x_bytes = std::fs::read("data/omega_b_x.npy").expect("unable to read npy file");
    let omega_b_x_values = npyz::NpyFile::new(&omega_b_x_bytes[..])
        .expect("unable to parse npy file")
        .into_vec::<f64>()
        .expect("unable to read f64");
    let omega_b_x = NpyDataset {
        values: (omega_b_x_values),
    };

    let omega_b_y_bytes = std::fs::read("data/omega_b_y.npy").expect("unable to read npy file");
    let omega_b_y_values = npyz::NpyFile::new(&omega_b_y_bytes[..])
        .expect("unable to parse npy file")
        .into_vec::<f64>()
        .expect("unable to read f64");
    let omega_b_y = NpyDataset {
        values: (omega_b_y_values),
    };

    let omega_b_z_bytes = std::fs::read("data/omega_b_z.npy").expect("unable to read npy file");
    let omega_b_z_values = npyz::NpyFile::new(&omega_b_z_bytes[..])
        .expect("unable to parse npy file")
        .into_vec::<f64>()
        .expect("unable to read f64");
    let omega_b_z = NpyDataset {
        values: (omega_b_z_values),
    };

    commands.insert_resource(MultiNpyData {
        datasets: vec![r_b_x, r_b_y, r_b_z, omega_b_x, omega_b_y, omega_b_z],
        last_accessed_index: 0,
    });
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.75, 0.75, 0.75)))
        .add_plugins(DefaultPlugins)
        .insert_resource(MultiNpyData {
            datasets: vec![],
            last_accessed_index: 0,
        })
        .init_resource::<CameraSettings>()
        .add_systems(
            Startup,
            (
                spawn_lights_and_camera,
                spawn_coordinate_axes,
                instructions,
                spawn_space_robot,
                load_npy_data,
            ),
        )
        .add_systems(
            FixedUpdate,
            (
                camera_orbit_movement,
                arm_1_rotator_system,
                arm_2_rotator_system,
                body_rotator_system,
            ),
        )
        .run();
}

fn spawn_coordinate_axes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let arrow_stem_scale: Mat4 = Mat4::from_scale(Vec3 {
        x: (0.02),
        y: (3.0),
        z: (0.02),
    });
    let arrow_point_scale: Mat4 = Mat4::from_scale(Vec3 {
        x: (0.05),
        y: (0.05),
        z: (0.05),
    });
    let cone_translate: Mat4 = Mat4::from_translation(Vec3 {
        x: (0.0),
        y: (1.5),
        z: (0.0),
    });
    let arrow_rot_to_point_at_z: Mat4 = Mat4::from_rotation_x(FRAC_PI_2);
    let arrow_rot_to_point_at_x: Mat4 = Mat4::from_rotation_z(-FRAC_PI_2);

    // Spawn the X-Coordinate Axis
    let all_transforms_stem_x = arrow_rot_to_point_at_x.mul_mat4(&arrow_stem_scale);
    commands.spawn((
        Name::new("X Arrow Stem"),
        Mesh3d(meshes.add(Cylinder::default())),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
        Transform::from_matrix(all_transforms_stem_x),
    ));
    let all_transforms_point_x =
        arrow_rot_to_point_at_x.mul_mat4(&cone_translate.mul_mat4(&arrow_point_scale));
    commands.spawn((
        Name::new("X Arrow Point"),
        Mesh3d(meshes.add(Cone::default())),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
        Transform::from_matrix(all_transforms_point_x),
    ));

    // Spawn the Y-Coordinate Axis
    commands.spawn((
        Name::new("Y Arrow Stem"),
        Mesh3d(meshes.add(Cylinder::default())),
        MeshMaterial3d(materials.add(Color::srgb(0.0, 0.7, 0.0))),
        Transform::from_matrix(arrow_stem_scale),
    ));
    let all_transforms_point_y = cone_translate.mul_mat4(&arrow_point_scale);
    commands.spawn((
        Name::new("Y Arrow Point"),
        Mesh3d(meshes.add(Cone::default())),
        MeshMaterial3d(materials.add(Color::srgb(0.0, 0., 0.0))),
        Transform::from_matrix(all_transforms_point_y),
    ));

    // Spawn the Z-Coordinate Axis
    let all_transforms_stem_z = arrow_rot_to_point_at_z.mul_mat4(&arrow_stem_scale);
    commands.spawn((
        Name::new("Z Arrow Stem"),
        Mesh3d(meshes.add(Cylinder::default())),
        MeshMaterial3d(materials.add(Color::srgb(0.0, 0.0, 1.0))),
        Transform::from_matrix(all_transforms_stem_z),
    ));
    let all_transforms_point_z =
        arrow_rot_to_point_at_z.mul_mat4(&cone_translate.mul_mat4(&arrow_point_scale));
    commands.spawn((
        Name::new("Z Arrow Point"),
        Mesh3d(meshes.add(Cone::default())),
        MeshMaterial3d(materials.add(Color::srgb(0.0, 0.0, 1.0))),
        Transform::from_matrix(all_transforms_point_z),
    ));
}

fn instructions(mut commands: Commands) {
    commands.spawn((
        Name::new("Instructions"),
        Text::new(
            "Middle Mouse / Two Finger Scroll\n\
            To Orbit and Tilt View",
        ),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.),
            left: Val::Px(12.),
            ..default()
        },
    ));
}

fn camera_orbit_movement(
    mut camera: Single<&mut Transform, With<Camera>>,
    camera_settings: Res<CameraSettings>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    time: Res<Time>,
) {
    let delta = mouse_scroll.delta;
    let mut delta_roll = 0.0;

    if mouse_buttons.pressed(MouseButton::Left) {
        delta_roll -= 1.0;
    }
    if mouse_buttons.pressed(MouseButton::Right) {
        delta_roll += 1.0;
    }

    // Mouse motion is one of the few inputs that should not be multiplied by delta time,
    // as we are already receiving the full movement since the last frame was rendered. Multiplying
    // by delta time here would make the movement slower that it should be.
    let delta_pitch = delta.y * camera_settings.pitch_speed;
    let delta_yaw = delta.x * camera_settings.yaw_speed;

    // Conversely, we DO need to factor in delta time for mouse button inputs.
    delta_roll *= camera_settings.roll_speed * time.delta_secs();

    // Obtain the existing pitch, yaw, and roll values from the transform.
    let (yaw, pitch, roll) = camera.rotation.to_euler(EulerRot::YXZ);

    // Establish the new yaw and pitch, preventing the pitch value from exceeding our limits.
    let pitch = (pitch + delta_pitch).clamp(
        camera_settings.pitch_range.start,
        camera_settings.pitch_range.end,
    );
    let roll = roll + delta_roll;
    let yaw = yaw + delta_yaw;
    camera.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);

    // Adjust the translation to maintain the correct orientation toward the orbit target.
    // In our example it's a static target, but this could easily be customized.
    let target = Vec3::ZERO;
    camera.translation = target - camera.forward() * camera_settings.orbit_distance;
}

fn spawn_lights_and_camera(mut commands: Commands) {
    commands.spawn((
        Name::new("Camera"),
        Camera3d::default(),
        Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Name::new("Light"),
        PointLight::default(),
        Transform::from_xyz(3.0, 8.0, 5.0),
    ));
}

#[derive(Component)]
struct Arm1Pivot {
    theta_0: f32,
    omega: f32,
}

#[derive(Component)]
struct Arm2Pivot {
    theta_0: f32,
    omega: f32,
}

#[derive(Component)]
struct BodyRotator;

fn spawn_space_robot(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Define initial position of cuboid body
    let position_cuboid_body = Mat4::from_translation(Vec3 {
        x: (0.5),
        y: (0.0),
        z: (0.0),
    });

    // Define shape and colors of the arms
    let arm_cylinder_mesh_handle = meshes.add(Cylinder::new(0.05, 1.));
    let arm_1_green_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.0, 0.8, 0.0),
        ..default()
    });
    let arm_2_yellow_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.8, 0.0),
        ..default()
    });

    // Transformation that changes the cylinder's pivot to an end instead of center
    let arm_origin_change: Mat4 = Mat4::from_translation(Vec3 {
        x: (0.),
        y: (0.5),
        z: (0.),
    });

    let rotate_arm_2: Mat4 = Mat4::from_rotation_z(-FRAC_PI_2);

    let total_transform_arm_2: Mat4 = rotate_arm_2.mul_mat4(&arm_origin_change);

    commands
        .spawn((
            Name::new("Robot_Body"),
            Mesh3d(meshes.add(Cuboid::new(1.0, 0.5, 0.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.0, 0.0, 0.8))),
            Transform::from_matrix(position_cuboid_body),
            BodyRotator,
        ))
        .with_children(|parent| {
            // Arm 1 Pivot
            parent
                .spawn((
                    Name::new("Arm1Pivot"),
                    Transform::from_xyz(-0.5, 0.25, 0.0), // Locating the Green Arm's Pivot
                    Arm1Pivot {
                        theta_0: FRAC_PI_4,
                        omega: 3.,
                    }, // Attach rotation behavior to the pivot
                ))
                .with_children(|pivot| {
                    // Attach the arm mesh to the pivot
                    pivot.spawn((
                        Name::new("Arm_1"),
                        Mesh3d(arm_cylinder_mesh_handle.clone()),
                        MeshMaterial3d(arm_1_green_material),
                        Transform::from_matrix(arm_origin_change), // Cylinder axis unchanged. Center shifted up on Y-Axis.
                    ));
                });

            // Arm 2 Pivot
            parent
                .spawn((
                    Name::new("Arm2Pivot"),
                    Transform::from_xyz(0.5, -0.25, 0.0), // Locating the Yellow Arm's Pivot
                    Arm2Pivot {
                        theta_0: FRAC_PI_4,
                        omega: 3.,
                    }, // Attach rotation behavior to the pivot
                ))
                .with_children(|pivot| {
                    // Attach the arm mesh to the pivot
                    pivot.spawn((
                        Name::new("Arm_2"),
                        Mesh3d(arm_cylinder_mesh_handle.clone()),
                        MeshMaterial3d(arm_2_yellow_material),
                        Transform::from_matrix(total_transform_arm_2), // Center shifted up on Y-Axis. Then Cylinder axis rotated to be positive X-Axis.
                    ));
                });
        });
}

fn arm_1_rotator_system(time: Res<Time>, mut query: Query<(&mut Transform, &Arm1Pivot)>) {
    for (mut transform, arm) in &mut query {
        let t = time.elapsed_secs();
        let current_angle = arm.theta_0 * (1.0 - (arm.omega * t).cos());
        transform.rotation = Quat::from_rotation_z(current_angle);
    }
}

fn arm_2_rotator_system(time: Res<Time>, mut query: Query<(&mut Transform, &Arm2Pivot)>) {
    for (mut transform, arm) in &mut query {
        let t = time.elapsed_secs();
        let current_angle = arm.theta_0 * (arm.omega * t).sin();
        transform.rotation = Quat::from_rotation_y(current_angle);
    }
}

fn body_rotator_system(
    mut query: Query<&mut Transform, With<BodyRotator>>,
    mut npy_data: ResMut<MultiNpyData>,
) {
    for mut transform in &mut query {
        let current_pos_x = npy_data.datasets[0].values[npy_data.last_accessed_index] as f32;
        let current_pos_y = npy_data.datasets[1].values[npy_data.last_accessed_index] as f32;
        let current_pos_z = npy_data.datasets[2].values[npy_data.last_accessed_index] as f32;
        let delta_rot_x = npy_data.datasets[3].values[npy_data.last_accessed_index] as f32;
        let delta_rot_y = npy_data.datasets[4].values[npy_data.last_accessed_index] as f32;
        let delta_rot_z = npy_data.datasets[5].values[npy_data.last_accessed_index] as f32;
        npy_data.last_accessed_index += 1;
        transform.translation = Vec3::new(0.5 + current_pos_x, current_pos_y, current_pos_z);
        transform.rotation = Quat::from_euler(EulerRot::XYZ, delta_rot_x, delta_rot_y, delta_rot_z);
    }
}
