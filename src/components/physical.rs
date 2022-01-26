use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};

use crate::Physics;




//which one is the source of truth?
//which updates in response to the others change
//i'd prefer having one source of truth, but it seems like i'd need two


//physical every tick updates the physics engine

//transformation updates at every tick according to the physics engine





//to create the physical engine
pub struct Physical{

    objectid: u16,

}


impl Physical{

    pub fn new(physics: &mut ResMut<Physics> ) -> Physical{

        let id = physics.get_new_object_id();

        physics.add_object(id, false);

        Physical{
            objectid: id,
        }
    }

    pub fn id(&self) -> u16{

        return self.objectid;
    }


    pub fn pos_shuffle(&self, physics: &mut ResMut<Physics>){


        physics.apply_delta_position( &self.id() , (rand::random::<f32>() * 0.01,rand::random::<f32>() * 0.01, rand::random::<f32>() * 0.01) );

    }

    pub fn set_pos(&self, physics: &mut ResMut<Physics>, pos: (f32,f32,f32)){


        physics.set_translation(&self.id(), pos);

    }




}