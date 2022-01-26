use bevy::{core::FixedTimestep, ecs::schedule::SystemSet, pbr::AmbientLight, prelude::*, render::{camera::Camera, render_graph::base::camera::CAMERA_3D}};

use rand::Rng;

mod physics;
pub use physics::Physics;

mod systems;

pub use components::Appearance;
mod components;
use components::*;

use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};

use parry2d::shape::ConvexPolygon;
use std::collections::HashMap;



pub struct FeetOnGround;

//components
/*
causes an influence

has a physical

has an appearance

has a position


is an image of an influence
(the ID of what causes it and the list of influences in it)
(just shows what it looks like on the ground)
*/



use bevy::math::Quat;
use bevy::math::f32::Vec3;


fn main() {

    // label for the "globaltransformediting" debug stage
    static GBTE: &str = "globaltransformediting";


    App::new()
    
        .add_plugins(DefaultPlugins)

        .insert_resource( Physics::new() )

        .add_startup_system( startup.system()  )

        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())

        .add_system( systems::physics_tick.system() )
        .add_system( set_player_intentions.system() )
        .add_system( systems::apply_motor_controls.system() )
        .add_system( systems::update_position.system() )
        .add_system( systems::update_camera.system() )
        .add_system( systems::set_on_ground_conditions.system() )
        //.add_system( systems::update_influencemap.system() )
        //.add_system( update_influence_appearance.system() )
        //.add_system( health_bars_face_forwards.system())

        .add_stage_after(CoreStage::PostUpdate, GBTE, SystemStage::single_threaded())
        .add_system_to_stage(GBTE, update_health_bar_positions.system())
        .add_system_to_stage(GBTE, update_mana_bar_positions.system())

        .add_system_to_stage(GBTE, update_health_bar_size.system())

        .run();

}



fn update_health_bar_positions(

    mut healthbars: Query< ( &mut GlobalTransform, &HealthBar)>,
) {

    for (mut globaltransform, _) in healthbars.iter_mut(){

        globaltransform.rotation = Quat::from_rotation_y( 0.0 );
        globaltransform.translation.y = globaltransform.translation.y + 2.5;
    }
}



fn update_mana_bar_positions(

    mut manabars: Query< ( &mut GlobalTransform, &ManaBar)>,

) {

    for (mut globaltransform, _) in manabars.iter_mut(){

        globaltransform.rotation = Quat::from_rotation_y( 0.0 );
        globaltransform.translation.y = globaltransform.translation.y + 2.0;
    }
}



fn update_health_bar_size(
    mut healthbars: Query< (&Parent, &mut GlobalTransform), With<HealthBar>>,
    parents: Query< (& Ethereal) >
){


    for (parent, mut transform) in healthbars.iter_mut(){

        let ethereal = parents.get( parent.0 ).unwrap();

        transform.scale.x = ethereal.health ;

    }


}


//apply the damage of the objects in teh scne to you




use core::f32::consts::PI;

// control the game character
fn set_player_intentions(
    mut commands: Commands,
    keyboard_input: Res<Input<KeyCode>>,

    mut query: Query<&mut MotorControl, With<PlayerControlled> >
) {

    let mut direction = None;
    let mut jump = false;


    if keyboard_input.pressed(KeyCode::Right) {
        direction = Some( 0.0 );
    }
    if keyboard_input.pressed(KeyCode::Down) {
        direction = Some( PI/2.0 );
    }
    if keyboard_input.pressed(KeyCode::Left) {
        direction = Some( PI );
    }
    if keyboard_input.pressed(KeyCode::Up) {
        direction = Some( PI*6./4.0 );
    }
    else{
    }
    

    if keyboard_input.just_pressed(KeyCode::Space) {
        jump = true;
    }


    for mut motorcontrol in query.iter_mut(){

        if jump == true{
            motorcontrol.set_jump();
        }

        motorcontrol.set_direction( direction );

    }

}






//is this a health bar
struct HealthBar;

//a bundle with children
struct ManaBar;


//to create ethereal, i also have to create the healthbar and manabar appearances



//ethereal is can be damaged
struct Ethereal{

    health: f32,
    defense: f32,
    mana: f32,
}
impl Ethereal{

    fn new() -> Ethereal{

        Ethereal{

            health: 100.0,
            defense: 10.0,
            mana: 50.0,
        }

    }
    
}


//actionable is can take actions
struct Actionable{

    strength: f32,
    intelligence: f32,
    allowedactions: Vec<u32>,
}

use bevy::ecs::system::EntityCommands;


//system that updates the health bar for each entity with health


fn startup(  mut physics: ResMut<Physics>,  asset_server: Res<AssetServer>, mut ambient_light: ResMut<AmbientLight>, mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {

    ambient_light.color = Color::WHITE;
    ambient_light.brightness = 10.0;


    // camera
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(0.0, 10.0, -15.0).looking_at(Vec3::new( 0., 0., 0.), Vec3::Y),
        ..Default::default()
    });


    // let mut entity = commands.spawn();
    // entity.insert_bundle( Appearance::InfluenceMap(&mut meshes, &mut materials) );
    // entity.insert( InfluenceShape{} );

    let mut previousid: Option<u16> = None;

    
    for x in 1..7{

        let mut entity = commands.spawn();

        entity.insert_bundle( Appearance::new_cube(&mut meshes, &mut materials) );


        let pos = (rand::random::<f32>() * 20.0 - 2.,6.0 + rand::random::<f32>() * 50.0, rand::random::<f32>() * 5.0 - 2.);
        let mut physical = Physical::new( &mut physics ) ;
        physical.set_pos( &mut physics, pos);




        if let Some(previd) = previousid{

            //physics.join_shapes( physical.id(), previd );

        
        }
        
        previousid = Some( physical.id() );



        entity.insert( physical );

        
        if x == 1{
            entity.insert( PlayerControlled );
        }

        entity.insert( MotorControl::new() );

        entity.insert( OnGround );

        //this means that this object contributes its influence to the map
        entity.insert(  InfluenceTag );


        entity.insert( Ethereal::new() );


        let parent = entity.id();



        
        entity.with_children(|cell| {
            cell.spawn_scene(asset_server.load("models/AlienCake/alien.glb#Scene0"));
        });


        //entity.get::<Transform>();


        /*
        let mut x = commands.spawn();
        let child = x.with_children(|cell| {
            cell.spawn_scene(asset_server.load("models/AlienCake/alien.glb#Scene0"));
        });
        let childid = child.id();
        commands.entity( parent ).push_children(&[childid]);
        */


    
        
        let mut child = commands.spawn_bundle( Appearance::new_healthbar(&mut meshes, &mut materials) );
        child.insert(  HealthBar );
        let childid = child.id();
        commands.entity( parent ).push_children(&[childid]);
    

        let mut child = commands.spawn_bundle( Appearance::new_manabar(&mut meshes, &mut materials) );
        child.insert(  ManaBar );
        let childid = child.id();
        commands.entity( parent ).push_children(&[childid]);
    
    }

}