use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};

mod appearance;



mod physical;
mod motorcontrol;
mod onground;

pub use onground::*;
pub use physical::*;
pub use motorcontrol::*;
pub use appearance::*;
//pub use appearance::InfluenceComponent;




pub struct InfluenceTag;

pub struct PlayerControlled;