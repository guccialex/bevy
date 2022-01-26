

/*
everything into an influence

and a way to turn the state of an agent into a status


a "medium attack range for unit X"


the influences

damage ranges for unit X






*/








//neural network
//do a cnn on objects


//If the entity is an agent
//if the entity is an influence


//path planning
//hmm

//express the state of the game as some 

//the neural network, gives me the state of the game


//know what the environment will look like in the next state


//either the AI can get a prediction about the value of the state for an agent 

//how do I know the value of the state? (in the near future, does the score increase or decrease)
//or (in the near future, does the highest possible score of the neural network increase)

//train the success and failure 








struct Influence{

    //the list of components that comprise this entity

    //the "null" for each component being used when creating a tensor



}


//create this from the entity
struct State{

    //the list of components that comprise the state

    //the "null" for each component being used when creating a tensor



}



//how to most concisely describe exactly what moves are allowed
//you know? for the sake of NARROW, I need a way to describe the state and effects in their entirety
//for broad, I just need to way to classify each different state an entity can be in all cases examined, not all possible cases
//state really doesnt need to be anything but a reference to the components

//and then how states deal with influences



/*

for each action, do I need to describe how it on average affects the thing its cast on?


make "tick" slim? so that agents can look at the future




agents look into the future


they are trained by a genetic history of past experiences in the environment
to reciognize patterns

by past experiences, to estimate their posiition in their environment


agents know

What their state is

what their environment is "relative to them"
    "they don't think , there's a bridge at 1,1, a tunnel at 0,1 , im at 1,1"
    they think "im at 0,0  a bridge under me, a tunnel to my left"

how the state changes with time






a network that

predicts the future

predicts the value an agent has with a certain surrounding environment




"score"
is basically
"how happy you are about the things you did + current state"



agents also have a "amount of enemies hurt"






*/





//
fn perform_action(){



}