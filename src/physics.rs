
//the physical state

use rapier3d::prelude::*;

use std::collections::HashSet;

use rapier3d::dynamics::{CCDSolver, JointSet, RigidBodySet, IntegrationParameters};
use rapier3d::geometry::{BroadPhase, NarrowPhase, ColliderSet};
use rapier3d::pipeline::PhysicsPipeline;

use rapier3d::dynamics::{ RigidBodyBuilder};

use rapier3d::math::Rotation;


use std::collections::HashMap;


use rapier3d::na as nalgebra;
use nalgebra::{Vector3, Isometry3};

//use ncollide3d::shape::ConvexHull;
use rapier3d::geometry::{ColliderBuilder, Shape, Ball};


use rapier3d::dynamics::RigidBody;
use rapier3d::dynamics::RigidBodyHandle;
use rapier3d::geometry::Collider;
use rapier3d::geometry::ColliderHandle;
use rapier3d::geometry::SharedShape;


use rapier3d::geometry::Cuboid;
use rapier3d::geometry::Cylinder;



pub struct Physics{

    physics_pipeline: PhysicsPipeline,
    
    
    //this should be stored but not serialized. hmm
    //pipeline: PhysicsPipeline,
    gravity: Vector3<f32>,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    joints: JointSet,
    ccdsolver: CCDSolver,
    
    //objectid to its rigidbody handle
    bodyhandles: HashMap<u16, RigidBodyHandle>,
    //the main shape/collider associated with each body
    shapehandles: HashMap<u16, ColliderHandle>,
    

    feethandles: HashMap<u16, (RigidBodyHandle, ColliderHandle) >,
    
    
    
    //the misions
    //the map of piece to mission
    missions: HashMap<u16, Mission>,
    
    //how long until this mission, the object its applied to, the mission
    futuremissions: Vec<(i32, u16, Mission)>,
    
}




use rapier3d::na::Point3;
use rapier3d::geometry::Ray;
use rapier3d::geometry::InteractionGroups;

use rapier3d::pipeline::QueryPipeline;
//a getter and setter for the state of the physics engine
//its the 
impl Physics{

    //get a unique object id
    pub fn get_new_object_id(&self) -> u16{


        loop{

            let randomid = rand::random::<u16>();

            if self.bodyhandles.contains_key( &randomid ){
                continue;
            }
            if self.shapehandles.contains_key( &randomid ){
                continue;
            }
            
            return randomid;
        }

    }

    pub fn new() -> Physics{
        


        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
      
      
      


        let gravity = vector![0.0, -29.81, 0.0];
        let mut integration_parameters = IntegrationParameters::default();
        let mut physics_pipeline = PhysicsPipeline::new();
        let mut island_manager = IslandManager::new();
        let mut broad_phase = BroadPhase::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = JointSet::new();
        let mut ccd_solver = CCDSolver::new();
        let physics_hooks = ();
        let event_handler = ();
        



        /* Create the ground. */
        let mut collider = ColliderBuilder::cuboid(700.0, 4.0, 700.0).build();
        collider.set_position( Isometry3::translation( 0., -5., 0. )  );
        collider.set_rotation(   vector![0.0, 0.0, 0.0] );
        collider.set_friction(0.7);

        colliders.insert(collider);



        /*
        let mut collider = ColliderBuilder::cuboid(100.0, 5000.0, 3.0).build();
        collider.set_translation(  vector![0.0, 4.0, -5.0] );
        colliders.insert(collider);
        
        
        let mut collider = ColliderBuilder::cuboid(100.0, 5000.0, 3.0).build();
        collider.set_translation(  vector![0.0, 4.0, 5.0] );
        colliders.insert(collider);


        let mut collider = ColliderBuilder::cuboid(3.0, 5000.0, 100.0).build();
        collider.set_translation(  vector![-5.0, 4.0, 0.0] );
        colliders.insert(collider);
        
        
        let mut collider = ColliderBuilder::cuboid(3.0, 5000.0, 100.0).build();
        collider.set_translation(  vector![5.0, 4.0, 0.0] );
        colliders.insert(collider);
        */
        
        
        
        /*
        integration_parameters.warmstart_coeff = 1.0;
        integration_parameters.max_ccd_substeps = 10;
        //integration_parameters.multiple_ccd_substep_sensor_events_enabled = true;
        integration_parameters.set_inv_dt(30.0);
        */
        
        
        
        // Run the simulation in the game loop.
        Physics{

            physics_pipeline,

            gravity,
            integration_parameters,
            island_manager,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            joints,
            ccdsolver: CCDSolver::new(),

            
            bodyhandles: HashMap::new(),
            shapehandles: HashMap::new(),
            feethandles: HashMap::new(),


            
            missions: HashMap::new(),
            futuremissions: Vec::new(),
        }
        
    }
    
    
    
    //pass in a position, and the direction of looking
    pub fn get_object_intersection(& self, ray: (Point3<f32>, Vector3<f32>) ) -> Option<u16>{

        let ray = Ray::new( ray.0, ray.1 );
        let max_toi = 1000.0;
        let solid = true;

        let groups = InteractionGroups::all();



        let filter = None;


        let mut query_pipeline = QueryPipeline::new();

        query_pipeline.update(&self.island_manager, &self.bodies, &self.colliders);




        if let Some((handle, toi)) = query_pipeline.cast_ray(
            &self.colliders, &ray, max_toi, solid, groups, filter
        ) {
            
            //get the id of the shape
            for (id, curhandle) in &self.shapehandles.clone(){

                if &handle == curhandle{

                    return Some(*id);
                }
            }
            
            panic!("ray intersects with a shape that doesnt have an id?");
        }else{

            //panic!(" no intersection");

            return None;
        }

    }
    
    
    
    pub fn join_shapes(&mut self, objectid1: u16, objectid2: u16){

        
        let joint = FixedJoint::new(Isometry::identity(), Isometry::translation(0.0, -1.2, 0.0));


        let body1 = self.bodyhandles.get(&objectid1).unwrap();

        let body2 = self.bodyhandles.get(&objectid2).unwrap();

        self.joints.insert( *body1, *body2, joint);


    }
    
    
    //add an object to this physics world
    //return the ID for the object
    pub fn add_object(&mut self, objectid: u16, isstatic: bool) -> u16{
        
        if self.bodyhandles.contains_key(&objectid){

            panic!("object with that ID already exists");
        }



        /* Create the bounding ball. */
        let rigid_body = RigidBodyBuilder::new_dynamic()
                .translation(vector![0.0, 0.0, 0.0])
                //.linear_damping(0.5)
                //.angular_damping( 0.0 )
                .gravity_scale(1. )
                .build();
        
        
        
        let collider = ColliderBuilder::cuboid(0.5 ,0.5 ,0.5)
        .restitution(0.1)
        .density( 1.0 )
        .friction( 0.7)
        .build();
        let rbhandle = self.bodies.insert(rigid_body);
        let chandle = self.colliders.insert_with_parent(collider, rbhandle, &mut self.bodies );



        self.bodyhandles.insert(objectid, rbhandle);        
        self.shapehandles.insert(objectid, chandle);
        



        use  nalgebra::base::Unit;


        


        let feet_rb = RigidBodyBuilder::new_dynamic()
                .translation(vector![0.0, 10.0, 0.0])
                //.linear_damping(0.5)
                //.angular_damping( 0.0 )
                .gravity_scale(1. )
                .build();
        
        
        
        let feet_c = ColliderBuilder::cuboid(0.5 ,0.1 ,0.5)
        .restitution(0.1)
        .density( 1.0 )
        .friction( 0.7)
        .build();
        let feet_rbhandle = self.bodies.insert(feet_rb);
        let feet_chandle = self.colliders.insert_with_parent(feet_c, feet_rbhandle, &mut self.bodies );

        let mut joint = FixedJoint::new(Isometry::identity(), Isometry::translation(0.0, 0.7, 0.0));

        //the rotation impulses
        joint.impulse[3] = 0.5;
        joint.impulse[4] = 0.5;
        joint.impulse[5] = 0.5;


        self.joints.insert( rbhandle, feet_rbhandle, joint);

        self.feethandles.insert( objectid, (feet_rbhandle, feet_chandle));
        

        





        /* 
        
        let mut rigid_body = RigidBodyBuilder::new(RigidBodyType::Dynamic)
        .angular_damping(3.5)
        .linear_damping(0.0)
        .can_sleep(false)
        //.restrict_rotations(false, true, false)
        
        .build();
        */
        
        
        /*
        let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0)
        .translation( vector![0.0, 0.0, 0.0] )
        .density(1.3)
        .friction(0.8)
        .build();
        */
        
        if isstatic{
            self.make_static(&objectid);
        }
        
        return objectid;
    }


    
    
    fn make_static(&mut self, ID: &u16){  
        self.get_mut_rigidbody(ID).set_body_type(RigidBodyType::Static) ;
    }
    

    
    
    pub fn remove_object(&mut self, id:&u16){
        
        
        //wake every rigidbody
        for (x, _) in self.bodyhandles.clone().iter_mut(){
            
            let x = self.get_mut_rigidbody(x);
            x.wake_up( true );
        }

        
        if let Some(rhandle) = self.bodyhandles.remove(id){
            
            self.shapehandles.remove(id);
            
            
            self.bodies.remove(rhandle, &mut self.island_manager, &mut self.colliders, &mut self.joints);
        }
        
    }
    

    //apply a force to try to match that velocity
    
    
    
    pub fn get_mut_rigidbody(&mut self, ID: &u16) -> &mut RigidBody{
        let rbhandle = self.bodyhandles.get(ID).unwrap();
        self.bodies.get_mut(*rbhandle).unwrap()
    }

    pub fn get_rigidbody(& self, ID: &u16) -> & RigidBody{

        let rbhandle = self.bodyhandles.get(ID).unwrap();
        self.bodies.get(*rbhandle).unwrap()
    }

    
    //apply a change of position to an object
    pub fn apply_delta_position(&mut self, ID: &u16, deltapos: (f32,f32,f32) ){
        
        use nalgebra::geometry::Translation3;
        
        let rigidbody = self.get_mut_rigidbody(ID);
        
        let mut pos = *rigidbody.position();
        
        let tra = vector![deltapos.0, deltapos.1, deltapos.2];

        let tra = Translation::from( tra );
        
        pos.append_translation_mut(&tra);
        
        rigidbody.set_position(pos, false);
    }
    
    //apply a impulse force to an object
    fn apply_delta_impulse(&mut self, ID: &u16, impulse: Vector3<f32>){
        self.get_mut_rigidbody(ID).apply_impulse(impulse, false);
    }

    pub fn apply_random_rotation(&mut self, ID: &u16, force: f32){
        
        let vec = vector![rand::random(), rand::random(), rand::random() ];
        let vec = vec * force;

        //self.get_mut_rigidbody(ID).apply_torque(vec, true);

        let vec = vector![rand::random(), rand::random(), rand::random() ];
        let vec = vec * 100.;
        
        use std::f32::consts::PI;

        let vec = vector![PI*4. ,PI*4., PI*4.];

        self.get_mut_rigidbody(ID).set_rotation( vec, true );

    }


    //is this object on another object
    pub fn is_on_an_object(&mut self, ID: &u16) -> bool{

        //get the objects collision box

        let colliderhandle = self.shapehandles.get(ID).unwrap();
        
        for contactpair in self.narrow_phase.contacts_with( colliderhandle.clone() ) {

            if contactpair.has_any_active_contact{
                return true;
            }
        }

        return false;
    }


    //return true if it has feet and they're on an object
    pub fn are_feet_on_object(&mut self, ID: &u16) -> bool{

        if let Some( (bodyhandle, colliderhandle) ) = self.feethandles.get(ID){

            for contactpair in self.narrow_phase.contacts_with( colliderhandle.clone() ) {

                if contactpair.has_any_active_contact{
                    return true;
                }
            }
    
        }
        

        return false;



    }


    //apply this amount of force
    //to get this object to rotate to this direction
    pub fn apply_desired_direction(&mut self, ID: &u16, desireddirection: Rotation<f32>){

        let rb = self.get_mut_rigidbody(ID);


        let curdirection = rb.rotation();
        let futuredirection = curdirection * Rotation::from_scaled_axis(*rb.angvel() ).powf( 0.1 )   ;
        

        //the amount I want this object to rotate 
        let desiredrotation = futuredirection.rotation_to( &desireddirection );
        


        let desiredangvel = desiredrotation.scaled_axis() ;
        let desiredangvel = desiredangvel / (desiredangvel.magnitude() + 0.5);




        rb.set_angvel(  desiredangvel * 10.0 , true);
        /*
        if desiredrotation.angle() < 0.1{

            rb.set_rotation(  desireddirection.scaled_axis() , true);
            let desiredangvel = desiredangvel * 0.01;
            rb.set_angvel(  desiredangvel , true);

        }
        else if desiredrotation.angle() < 0.2{

            rb.set_angvel(  desiredangvel * 5.0 , true);
        }
        else{


            rb.set_angvel(  desiredangvel * 10.0 , true);
        }*/


        
        /*
        //let desiredangvel = desiredrotation.scaled_axis() ;
        //let desiredangvel = desiredangvel / (desiredangvel.magnitude() + 1.0);
        //let desiredangvel = desiredangvel * 10.;
        
        //the desired angular velocity is the norm of the angle and the difference in rotation magnitude
        //let desiredangvel = desiredangvel * desiredrotation.magnitude()* desiredrotation.magnitude()* desiredrotation.magnitude()  * 1.;

        //let curangvel: Vector3<f32> = rb.angvel().clone();
        //let curangvel = curangvel / (curangvel.magnitude() + 0.0001);
        
        //get the change in angvel i want
        
        //let angvelchange = (desiredangvel - curangvel);

        //the force is in the direction of the angvel change
        //up to the 6

        //how to get the angle


        //let force = direction * 0.1;
        //println!("desired rot{:?}", desiredrotation.angle() );
        
        
        */

    }



    pub fn get_shadow(&self, ID: &u16)  -> parry2d::shape::ConvexPolygon{

        //get a list of point2s for each point
        //let handle = self.shapehandles.get(ID).unwrap();

        //let collider = self.colliders.get(&handle).unwrap();

        let collider = self.get_collider(ID);
        let rb = self.get_rigidbody(ID);
        let isometry = rb.position();

        let aabb = collider.compute_aabb(  );



        let mut toreturn = Vec::new();

        //its verticies projected onto the flat ground (Y)is the same as its (x cordinate, z coordinate)

        for point3 in aabb.vertices(){

            let point2 = point![point3.x, point3.z];
            
            if !toreturn.contains(&point2){
                toreturn.push(point2);
            }
            
        }

        use parry2d::shape::ConvexPolygon;


        return ConvexPolygon::from_convex_hull( &toreturn ).unwrap();


    }



    
    //for each base, if they care about the direction
    //get the speed I want
    //get the direction of the force

    pub fn apply_force_for_intended_velocity(&mut self, ID: &u16, x: Option<f32>, y:Option<f32>, z: Option<f32>, force: f32 ){

        let curvel = self.get_mut_rigidbody(ID);
        let curvel = curvel.linvel();

        let mut desiredvel = curvel.clone();

        if let Some(x) = x{
            desiredvel.x = x;
        }
        
        if let Some(y) = y{
            desiredvel.y = y;
        }

        if let Some(z) = z{
            desiredvel.z = z;
        }

        //want to maximize movement in some direction
        //and put as much force into that as possible

        //if its close to the intended force, 

        let mut influencedirection = desiredvel - curvel;

        let x = 3.0;

        if influencedirection.magnitude() > x{
            influencedirection = influencedirection.normalize();
        }
        else{
            let scale = influencedirection.magnitude() / x;

            influencedirection = influencedirection * scale.powf(1.5);
        }

        let influenceforce = influencedirection * force;

        self.apply_delta_impulse( ID, influenceforce );

    }


    pub fn apply_delta_impulse_pub(&mut self, ID: &u16, impulse: (f32,f32,f32)){

        let vec = vector![impulse.0, impulse.1, impulse.2];

        self.apply_delta_impulse(ID, vec);
        
    }


    //get the translation of the position of an object
    pub fn get_translation(&self, ID: &u16) -> (f32,f32,f32){
        
        let translation = self.get_rigidbody(ID).position().translation;
        
        (translation.x, translation.y, translation.z)
    }
    
    pub fn set_translation(&mut self, ID: &u16, position: (f32,f32,f32)  ) {
        
        let pos = Isometry3::translation(position.0, position.1, position.2);
        self.get_mut_rigidbody(ID).set_position(pos, false);
    }

    
    pub fn get_isometry(&self, ID: &u16) -> Isometry3<f32>{
        self.get_rigidbody(ID).position().clone()
    }

    pub fn set_isometry(&mut self, ID: &u16, isometry: Isometry3<Real>){
        self.get_mut_rigidbody(ID).set_position(isometry, false);
    }

    
    //get the translation of an object
    pub fn get_rotation(&self, ID: &u16) -> (f32,f32,f32){
        
        let rotation = self.get_rigidbody(ID).position().rotation.euler_angles();
        
        (rotation.0, rotation.1, rotation.2)
    }
    
    pub fn set_rotation(&mut self, ID: &u16, rotation:(f32,f32,f32) ){

        let oldisometry = self.get_mut_rigidbody(ID).position();
        let oldtranslation = oldisometry.translation;
        
        use nalgebra::geometry::UnitQuaternion;
        
        let newrotation = UnitQuaternion::from_euler_angles( rotation.0, rotation.1, rotation.2);
        
        let newisometry = Isometry3::from_parts(oldtranslation, newrotation);
        
        self.get_mut_rigidbody(ID).set_position(newisometry, false);
    }

    

    pub fn get_mut_collider(&mut self, ID: &u16) -> &mut Collider{

        let chandle = self.shapehandles.get(ID).unwrap();

        self.colliders.get_mut( *chandle ).unwrap()
    }

    pub fn get_collider(& self, ID: &u16) -> & Collider{

        let chandle = self.shapehandles.get(ID).unwrap();

        self.colliders.get( *chandle ).unwrap()
    }


    pub fn get_shape(& self, ID: &u16) -> Box<dyn Shape>{

        self.get_collider(ID).shape().clone_box()
    }

    
    pub fn set_shape_sphere(&mut self, ID: &u16, diameter: f32){
        
        let radius = diameter /2.0;

        self.get_mut_collider(ID).set_shape( SharedShape::new( Ball::new(radius) ) );
    }
    
    pub fn set_shape_cuboid(&mut self, ID: &u16, dimensions: (f32,f32,f32)){
        
        let dimensions = (dimensions.0 / 2.0, dimensions.1/2.0, dimensions.2 / 2.0);
        
        self.get_mut_collider(ID).set_shape( SharedShape::new( Cuboid::new( vector![dimensions.0, dimensions.1, dimensions.2] ) ) );
    }
    
    pub fn set_shape_cylinder(&mut self, ID: &u16, height: f32, diameter: f32){
        
        let halfheight = height /2.0;
        let radius = diameter /2.0;
        
        self.get_mut_collider(ID).set_shape( SharedShape::new( Cylinder::new( halfheight, radius) ) ) ;   
    }
    
    
    
    pub fn set_materials(&mut self, ID: &u16, elasticity: f32, friction: f32){
        
        let collider = self.get_mut_collider(ID);

        collider.set_friction( friction);
        collider.set_restitution(elasticity );
    }
    
    
    
    
    pub fn tick(&mut self){
        

        self.tick_missions();

        
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joints,
            &mut self.ccdsolver,
            &(),
            &(),
        );

        
        
    }
    
    
    
    pub fn does_object_with_id_exist(&self, id: &u16) -> bool{
        
        if let Some(rbhandle) = self.bodyhandles.get(id){

            if let Some(_) = self.bodies.get(*rbhandle){
            
                return true;
            }
        }
            
        return false;

    } 
    
    
    
    
    
    
    
    
    
    //get all the active missions and the object they're for
    pub fn get_active_missions(&self ) -> Vec<(u16, Mission)>{
        
        let mut toreturn = Vec::new();
        
        //for each active mission
        for (objectid, mission) in &self.missions{
            
            toreturn.push( (*objectid, mission.clone()) );
        }
        
        toreturn
    }
    
    
    
    pub fn is_object_on_mission(&self, objectid: &u16) -> bool{
        
        self.missions.contains_key(objectid)
    }
    
    pub fn set_mission(&mut self, pieceid: u16, mission: Mission){
        
        //if that piece already has a mission, end its
        self.set_future_mission(0, pieceid, mission);
    }
    
    pub fn set_future_mission(&mut self, ticks: u32, pieceid: u16, mission: Mission){
        
        self.futuremissions.push( (ticks as i32, pieceid, mission) );        
    }
    
    
    pub fn end_mission(&mut self, ID: &u16){

        //if this object is on a mission
        if let Some(mission) = self.missions.get(ID){
            
            //if this object has a default isometry
            if let Some(isometry) = mission.get_default_isometry(){
                
                self.set_isometry(ID, isometry);
            }
        }
        
        self.missions.remove(ID);
    }
    
    
    fn tick_missions(&mut self){
        
        
        //the future missions
        {   
            
            //tick the future missions down and start it if the tick is 0
            for thing in self.futuremissions.iter_mut(){
                
                let (tick, objectid, mission) = thing;
                
                *tick = *tick - 1;
                
                //if its time to start the mission, just start it by putting it in the list of missions 
                //if its less than zero
                if *tick <= 0 { 
                    
                    //if there is already a mission for this object
                    if self.missions.contains_key(&objectid){
                    }
                    else{
                        //make the mission started
                        mission.started = true;
                        
                        //set the mission and return true
                        self.missions.insert(*objectid, mission.clone());
                    }
                }
            };
            
            
            
            //remove the future mission if the tick is 0
            self.futuremissions.retain(|(tick, objectid, mission)|{            
                
                //if the tick is 0 or less
                if *tick <= 0 {
                    
                    //remove it
                    return false;
                }
                else{
                    //keep it
                    return true;
                }
            });
        }
        
        
        //the ids of the missions that are expired
        let mut finishedmissions: Vec<u16> = Vec::new();
        
        //for each mission
        for (physicalid, mission) in self.missions.clone().iter(){
            
            //if an object with that physical ID exists in the engine
            if self.does_object_with_id_exist(physicalid){
                
                //if there is an impulse
                if mission.is_current_impulse(){
                    let currentimpulsevector = mission.get_current_impulse();
                    
                    self.apply_delta_impulse(physicalid, currentimpulsevector);
                }
                
                if mission.is_current_position_change(){          
                    
                    let poscvector = mission.get_current_delta_position();
                    
                    self.apply_delta_position(physicalid, (poscvector.x, poscvector.y, poscvector.z) );
                    
                }
                
            }
            
            
            
            
        }
        
        
        for (physicalid, mission) in self.missions.iter_mut(){
            
            //then tick the mission
            //end and remove it if it needs to be ended and removed
            //and remove the sensor that the piece had on that mission
            mission.tick();
            
            if mission.is_finished() {
                finishedmissions.push(*physicalid);
            }
        }
        
        //remove each finished mission
        for objectid in &finishedmissions{
            
            self.end_mission(objectid);
        }
    }
    
    
}





use rapier3d::math::Real;


#[derive(Clone)]
pub struct Mission{
    
    //the current tick the mission is currently on
    currenttick: u32,
    
    //START is inclusive
    //END is exclusive
    
    //the force (impulse) to apply when the current tick is in range
    //a vector with the scalar of the force
    impulses: Vec< (u32, u32, Vector3<Real>) >,
    
    //the change in position to apply when the current tick is in range
    //(call the "disable gravity for a tick" when this is being called)
    positionchanges: Vec< (u32, u32, Vector3<Real>) >,
    
    //the position and velocity that this object should be in when this mission is over
    defaultpos: Option< Isometry3<Real> >,
    
    
    //if the mission has started, it cant have its position, impulse or default position change
    started: bool,
    
    
    //user data
    data: u16,
    
}

impl Mission{
    
    
    pub fn set_mission_data(&mut self, data: u16){
        
        self.data = data;
    }
    
    pub fn get_mission_data(&self) -> u16{
        self.data
    }
    
    
    pub fn add_position_change(&mut self, starttick: u32, endtick: u32, stepchange: (f32,f32,f32)){
        
        self.positionchanges.push( (starttick, endtick, vector![stepchange.0 , stepchange.1 , stepchange.2]  ) );
    }
    
    pub fn add_impulse_change(&mut self, starttick: u32, endtick: u32, stepchange: (f32,f32,f32)){
        
        self.impulses.push( (starttick, endtick, vector![stepchange.0 , stepchange.1 , stepchange.2]  ) );
    }
    

    pub fn default_mission() -> Mission{
        
        let data = 1;

        Mission{
            currenttick: 0,
            
            impulses: Vec::new(),
            
            positionchanges: Vec::new(),
            
            defaultpos: None,
            
            started: false,
            
            data: data,
        }
    }
    
    
    //tick the mission
    //the tick should be done after performing the effects of the mission
    //so tick 0 is run
    fn tick(&mut self){
        self.currenttick += 1;
    }
    
    //if this mission is finished
    fn is_finished(&self) -> bool{
        
        let mut isfinished = true;
        
        //see if theres any position change currently or in the future
        for (starttick, endtick, vector) in &self.positionchanges{
            
            if endtick >= &self.currenttick{
                
                isfinished = false;   
            }
        }
        
        //see if theres any impulses currently or in the future
        for (starttick, endtick, vector) in &self.impulses{
            if endtick >= &self.currenttick{
                isfinished = false;   
            }   
        }
        
        isfinished   
    }
    
    
    fn get_current_delta_position(&self) -> Vector3<f32>{
        
        let mut totalpositionchange = vector![0.0,0.0,0.0];
        
        for (starttick, endtick, vector) in &self.positionchanges{
            if  self.currenttick >= *starttick {
                if  self.currenttick < *endtick {   
                    totalpositionchange += vector;
                }
            }        
        }
        
        totalpositionchange
    } 
    
    fn get_current_impulse(&self) -> Vector3<f32>{
        
        let mut totalimpulse = vector![0.0,0.0,0.0];
        
        for (starttick, endtick, vector) in & self.impulses{
            if  self.currenttick >= *starttick {
                if  self.currenttick < *endtick {
                    totalimpulse += vector;   
                }   
            }
        }
        
        totalimpulse
    } 
    
    
    
    //is there a position change currently going on?
    //this should be plural but i just never seem to have plurals in method titles as a rule
    fn is_current_position_change(&self) -> bool{
        
        //for every one of the position changes in the list
        for (starttick, endtick, vector) in &self.positionchanges{
            if  self.currenttick >= *starttick {
                if  self.currenttick < *endtick {
                    return(true);                    
                }   
            }    
        }
        
        return(false);
    }
    
    fn is_current_impulse(&self) -> bool{
        
        for (starttick, endtick, vector) in &self.impulses{
            if self.currenttick >= *starttick {
                if self.currenttick < *endtick {
                    return(true);
                }
            }
        }
        
        return(false);   
    }
    
    
    pub fn set_default_isometry(&mut self, isometry: Isometry3<f32> ){
        self.defaultpos = Some( isometry );
    }
    
    fn get_default_isometry(&self) -> Option<Isometry3<Real>>{
        self.defaultpos
    }
    
}