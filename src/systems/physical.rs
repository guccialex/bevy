


//ticks the game

//queries




use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};


use crate::{FeetOnGround, Physics};




//influence objects
//with a shape and a 


//get the objects with 






pub fn physics_tick(mut physics: ResMut<Physics>,){

    physics.tick();


}


use crate::MotorControl;
use crate::Physical;
use crate::OnGround;
use crate::InfluenceTag;


use crate::Appearance;


/*
//the influence maps, the parent with a physical
pub fn update_influencemap(influences: Query<( & Transform , With<InfluenceTag> ) >,){


    for ( mut transform) in influences.iter_mut(){

        meshes.remove(mesh.clone() );

        let newhandle = meshes.add( Mesh::from(shape::Cube { size: 0.4  }) );

        *mesh = newhandle;

        if let Ok( (parenttransform) ) = parents.get(*parent.clone()  ){
            
            //the new transform 

            transform.rotation = parenttransform.rotation.conjugate() ;

            //let relativefromzero = Vec3::new(parenttransform.translation.x, -1.0,parenttransform.translation.z)* parenttransform.translation;

            let newtranslation = parenttransform.rotation * parenttransform.translation;
            
            transform.translation = -(newtranslation / 1.0);


            //add the parents x and z translation
            //transform.translation += relativefromzero ;


        }


    }


    
}

*/



//set the labels for each object, if its feet are on the ground
//and if its body is on the ground
pub fn set_on_ground_conditions(mut commands: Commands, mut physics: ResMut<Physics>, mut query: Query< (Entity, &mut MotorControl, &mut Physical) >  ){


    //if either the feet or the body are on hte ground, set on ground
    //if only the feet are on the ground, set "feet on ground"

    for (entity, motorcontrol, physical) in query.iter_mut(){


        //if the body is on the ground, the body is, and the feet arent
        if physics.is_on_an_object( &physical.id() ) {

            commands.entity(entity).insert( OnGround );

            //commands.entity(entity).insert( FeetOnGround );
            commands.entity(entity).remove::<FeetOnGround>();
        }
        //if the feet are on the ground, both the body and the feet are on the ground
        else if physics.are_feet_on_object(&physical.id() ){

            commands.entity(entity).insert( FeetOnGround );
            commands.entity(entity).insert( OnGround );
        }
        else{

            commands.entity(entity).remove::<FeetOnGround>();
            commands.entity(entity).remove::<OnGround>();
        }

    }

    



}




//its children
pub fn apply_motor_controls( mut physics: ResMut<Physics>, mut query: Query<(&mut MotorControl, &mut Physical,  Option<&OnGround>, Option<&FeetOnGround>   ), >  ){

    for (mut motorcontrol, physical, onground, feetonground) in query.iter_mut(){

        use rapier3d::math::Rotation;    
        use rapier3d::na as nalgebra;
        use rapier3d::prelude::vector;


        if onground.is_some(){
            physics.apply_desired_direction(&physical.id(), Rotation::from_euler_angles(0.0, 1.0, 0.0) );

        }

        if feetonground.is_some(){

            let temp = motorcontrol.to_force();
            let desiredvel = vector![temp.0, temp.1, temp.2];
            let desiredvel = desiredvel * 200.0;
            
            physics.apply_force_for_intended_velocity( &physical.id(), Some(desiredvel.x) , None, Some(desiredvel.z) , 2.0  );

            if motorcontrol.pop_jump(){
                physics.apply_delta_impulse_pub( &physical.id(),  (0.0, 20.0, 0.0) );
            }
        }

    }



}





//"affect" structs deal their damage to what they contact
pub fn deal_effects(){



}