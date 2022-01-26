

use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};


use rapier3d::na::Quaternion;



struct MovementType{
}

impl MovementType{
    //given a speed, get the friction and 
}


//each object has its physical state
//but if it has motor control
//then it tries to set its movement
//by influincing itself in the direction it wants to 

//the intended motor actions
pub struct MotorControl{

    //the direction in radians
    intendeddirection: Option<f32>,

    intendedrotation: Option<Quaternion<f32>>,

    intendedjump: bool,
}

impl MotorControl{

    pub fn new() -> MotorControl{

        MotorControl{
            intendeddirection: None,
            //the direction it wants its face aligned to
            intendedrotation: None,

            intendedjump: false,
        }
    }

    //you jump as soon as you can
    //but less high 
    //if you hold shift, that changes how much energy you expend
    //so jump higher and sprint
    pub fn set_jump(&mut self){
        self.intendedjump = true;
    }

    pub fn pop_jump(&mut self) -> bool{
        let toreturn = self.intendedjump;
        self.intendedjump = false;

        return toreturn;
    }

    pub fn set_direction(&mut self, angle: Option<f32> ) {

        self.intendeddirection = angle;
    }
    

    pub fn to_force(&self) -> (f32,f32,f32){

        if let Some(dir) = &self.intendeddirection{

            let scalar = 1.0;

            let x = scalar * dir.cos();
            let y = 0.0;
            let z = scalar * dir.sin();


            return (x,y,z);
        }

        return (0., 0., 0.);
    }
}




