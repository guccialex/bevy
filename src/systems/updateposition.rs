use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};
use crate::Physical;
use crate::Physics;


pub fn update_position(mut commands: Commands,  mut physics: ResMut<Physics>, mut query: Query<( &mut Transform, &mut Physical)>) {


    for (  mut transform, mut physical) in query.iter_mut(){

        let id = physical.id();

        if physics.does_object_with_id_exist(  &id  ){

            let xyz = physics.get_translation(&id);
            *transform = Transform::from_xyz( xyz.0, xyz.1, xyz.2 );


            let rot = physics.get_isometry(&id);
            let rot = rot.rotation.as_vector();
            let rot = Quat::from_xyzw(rot[0],rot[1],rot[2],rot[3] );

            transform.rotate( rot );
        }
        
    }
    

}





use crate::PlayerControlled;



use bevy::math::Vec3;


pub fn update_camera(  mut query: Query<( &mut Transform, Option<&mut Camera>, Option<&PlayerControlled> )> ){


    let mut sumpos = Vec3::new(0. , 0., 0.);
    let mut numberofunits = 0;

    //the average position of the player controlled units
    
    for (transform, camera, playercontrol) in query.iter_mut(){

        if let Some(playercontrol) = playercontrol{

            sumpos += transform.translation;
            numberofunits += 1;

        }

    }

    let averagepos = sumpos / numberofunits as f32;




    //get th

    for (mut transform, camera, playercontrol) in query.iter_mut(){

        if let Some( camera ) = camera{

            transform.look_at(averagepos, Vec3::Y);


        }


    }



}
