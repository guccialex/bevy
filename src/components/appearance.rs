use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};








#[derive(Bundle)]
pub struct Appearance{

    #[bundle]
    pbrbundle: PbrBundle,
}

impl Appearance{

    pub fn new_blank( mut meshes: &mut ResMut<Assets<Mesh>>, mut materials: &mut ResMut<Assets<StandardMaterial>>) -> Appearance{

        Appearance{

            pbrbundle: PbrBundle {
                //mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1  })),
                //material: materials.add( Color::rgb(rand::random(), rand::random(), rand::random()).into() ),
                
                transform: Transform {
    
                    translation: Vec3::new(
                        0.,
                        0.,
                        0.,
                    ),
                    rotation: Quat::from_rotation_x( rand::random::<f32>() * 1.0 ),
                    scale: Vec3::new(1.0, 1.0, 1.0),
                    ..Default::default()
                },
    
                ..Default::default()
            },
        }

    }

    pub fn new_cube( mut meshes: &mut ResMut<Assets<Mesh>>, mut materials: &mut ResMut<Assets<StandardMaterial>>) -> Appearance{

        Appearance{

            pbrbundle: PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0  })),
                material: materials.add( Color::rgb(rand::random(), rand::random(), rand::random()).into() ),
                
                transform: Transform {
    
                    translation: Vec3::new(
                        0.,
                        0.,
                        0.,
                    ),
                    rotation: Quat::from_rotation_x( rand::random::<f32>() * 1.0 ),
                    ..Default::default()
                },
    
                ..Default::default()
            },
        }

    }




    pub fn new_healthbar( mut meshes: &mut ResMut<Assets<Mesh>>, mut materials: &mut ResMut<Assets<StandardMaterial>>) -> Appearance{


        Appearance{

            pbrbundle: PbrBundle {
                mesh: meshes.add(Mesh::from( shape::Box {min_x: -0.1,  max_x: 0.1, min_y: -0.1, max_y: 0.1, min_z: -0.0, max_z: 0.0  } )),

                material: materials.add( Color::rgb(1.0, 0.0, 0.0).into() ),
                
                transform: Transform {
    
                    translation: Vec3::new(
                        0.,
                        0.,
                        0.,
                    ),
                    rotation: Quat::from_rotation_x( rand::random::<f32>() * 1.0 ),
                    ..Default::default()
                },
    
                ..Default::default()
            },
        }

    }




    pub fn new_manabar( mut meshes: &mut ResMut<Assets<Mesh>>, mut materials: &mut ResMut<Assets<StandardMaterial>>) -> Appearance{


        Appearance{

            pbrbundle: PbrBundle {
                mesh: meshes.add(Mesh::from( shape::Box {min_x: -0.1,  max_x: 0.1, min_y: -0.1, max_y: 0.1, min_z: -0.0, max_z: 0.0  } )),

                material: materials.add( Color::rgb(0.0, 0.0, 1.0).into() ),
                
                transform: Transform {
    
                    translation: Vec3::new(
                        0.,
                        0.,
                        0.,
                    ),
                    rotation: Quat::from_rotation_x( rand::random::<f32>() * 1.0 ),
                    ..Default::default()
                },
    
                ..Default::default()
            },
        }

    }


    //influencemap
    //this will be a child to that object
    pub fn new_influenceshape( mut meshes: &mut ResMut<Assets<Mesh>>, mut materials: &mut ResMut<Assets<StandardMaterial>>) -> Appearance{

        Appearance{

            pbrbundle: PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Box {min_x: -0.5,  max_x: 0.5, min_y: -0.1, max_y: 0.1, min_z: -0.5, max_z: 0.5  })),
                //mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1  })),
                material: materials.add( Color::rgb(rand::random(), rand::random(), rand::random()).into() ),
                transform: Transform {
                    translation: Vec3::new(
                        0.,
                        0.,
                        0.,
                    ),
                    //rotation: Quat::from_rotation_x( rand::random::<f32>() * 1.0 ),
                    ..Default::default()
                },
                ..Default::default()
            },
        }
    }


}
