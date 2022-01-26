


//tie the AI with the game system
/*
does the game system have some method

the game has a method that takes a "agent state"
and returns it after some "state effect" is applied

a state effect can also be in an influence
and the effects are the same as the 





the "mind" component
that has a "desires"

then a series of components that together compose a "state"

and a bunch of methods that together compose a "state effect"

and 



a state is a list of game components
(just needs to implement a way to serialize self into a list of vectors)




state + influence -> new state


desires + old state + new state -> compulsion score





the map is just a list of 

there's a list of "actions"

that get their "status effect"
by their environment around them

what options do they have in taking an action?

maybe its 

get the 



the solver




The environment is the map of influences

including "within x range of influences on the ground"

"effects"

"combine effects"

influences that can be added



the "influence"
is a list of "single effects"



"a list of influences"
is "a series of influneces"



a series of influences on each state

"damage done to enemy"
is an influnce


"damaged"
is an influnece
but "dead" is a state





the map of decisions all agents can take

its similarity between ticks captuerd with the nodes updating their paths with td between ticks


all possible decision

and what the path in these tds tha t



TD pathfinding

go from each node to each other node
with the goal of knowing





train on the everything 


a way to represent decisions?

I guess I have to have every method as a decision? right?
they're all unique

either represent actions as a choice, or represent


either represent:

ALL DECISIONS AS A DIFFERENT ACTION TO TAKE AT EACH NODE

or

A DECISION TO DO AN ACTION IS A SINGLE NODE
and then 
and then the actual decision is made by choosing the best move in the narrow case
and train by like, the best possible if it chooses the best possible moves and actions
and assume the action taken at the point is the best one


or like put the different moves they can do into classes, that represent the types of moves there are
"how good is doing a "class1" move here. How good is doing a "class2" move here"


or see if the result of the state increases depending on what move is allowed
and then deciding the correct action by what action it could do has the highest score


thats a different question though
"Whats the action that this agent that can only do one type of action will take"


maybe the state can have 3 actions

then its, if these three actions have to be picked, what is the best actions and order of actions


it IS "1 action"
and then i try to ask the broad phase "what an agent with no actions allowed" would try to perform
then when going through with the narrow phase, 


the agent is passed in with all actions that are available,
so that the game can learn "if these are the 4 choices for actions at each node,
how good is this agent going to be at solving the problem"
(I don't know what X is, where X is the amount of actions a character can have available,
and if its very big, it feels like the vector representing the state is going to get very big)


and then i can either deep search with this agent, and check what the best action is



and maybe
"the agents arent given teh information abotu waht actions it has available.
And so the agents still perform the best with their skills available."


"No. For each action, there is a X dimensional vector for each action, and a 0 or 1 if available or not
And then that is the state that the game searches through

And I can either set an action as "some action" and assume the best action
or it can give a rating for each score, and returns the state where the list of actions is listed by the highest
score on the action that is the best for that agent


and then pick the index of the action that gives the highest score
(what about multiple actions again?)
(two vectors, one for each )




WAYS TO DEAL WITH MANY AND VARIED ACTIONS TO PERFORM AT DIFFERENT NODES



1.

A state that doesn't include what actions this agent is allowed to perform
then when doing the search, i get reccomendations for a state that could do SOME actions based on the subset given
and try to predict the score
so you can still get whether you should move or pick an action, but you wont know what the action is, or if its for a 
distribution of skillsets similar to yours
and therefore have to use the narrow pathfinder to get the WHAT action it is



2.

if STATE could define "what actions allowed" would this be an issue?
then it knows what the best influences would be for this state with these available actions

I use a narrow pathfinder to find what the best actions are (this is the same as 1, but with the state getting info about moves, however it wants)



3.

The state has a list of x "moves"

and then tries to predict its highest score future state

PLUS return a queue of moves it would perform (every time the highest score path of action from it suggests to make a certain move)



4.


the state into highest state froma  path of these
PLUS, return an RNN that would output a vector of the order of moves to use (or just three blank spots for the first 3 actions to use (then repeat))
and how high the score would be for this agent if it used "that move"
while taking its actions
then i can get what the highest score action is, that this agent is allowed to perform, and perform that
and I guess, don't predict whatever the next action is going to be after this one, because I can assume its fine
and figure it out when I get to that point

(
train the results layer by what action would give the highest score if used in the action space where the queue returned

(extending 4)

maybe have 2 returns, so i can get, train it to predict how much WHICH first move in place of "any action" 
and WHICH SECOND move in place of the second "any action" would increase the score

train it to predict how each move, if it could do any move, would raise or lower the score relative to other moves

(I think this might be the best and most complicated option, its the most thorough at least, but "pass state representing actions allowed,
use narrow" also seems OK)
)







"state into neural layer and then neural layer back to state"
'but neural layer taken through a map of influences also stores the data about
what the effect the best influence map this map of infleunces has on this influence, on this influence"

copy and use the reversing of it to train it, the state to neural layer to state again










*/







struct Action{


}


struct Environment{

    //the aabb of the 


    /*
    relative the position of each object


    An "In Melee range, in medium range, in long range" parts on the floor for each object

    but then the influences of that character associated



    like the neural layer figures out how different stats for the character affect the score when going through it
    and therefore what the highest value state that this agent can take with this path is


    
    you cant really combine scores of multiple agents

    like infleunces of "killed when thsi number is exceeded
    
    


    depending on position is doing what the "Td" pathfinding thing does

    a neural layer for each allowed action on top, for each position

    then... how combine?


    i want to find a series of actions

    I want a way to describe actions taken and how the world changes because of that


    each agent takes in a cnn
    where each pixel is a representation of the state there
    "the difficulty of terrain, how much it can damage an enemy"
    and it returns the list of actions
    
    
    "







    
    go down the path
    */


}



struct Desires{



}

//take in the nodemap


fn solver(environment: &Environment, desires: &Desires ) -> Vec<Action>{



}



struct Influence{

    //the time that it takes to have this influence exerted
    time: f32,




}

impl Influence{


    //add another influence to self
    //how do I do something liKe "if one of things dies"
    //maybe influence is a list of "effects"
    //then to add, you just concat the list of effects
    fn add(&mut self, other: &Influence){



    }

}




struct State{

    health: f32,

    //how many seconds in the future is this state
    timeinfuture: f32,


    enemydamage: f32,

}


/*

a map of data on hte ground
and a movable agent state

the agent has "ranges" it can do things



different choices for an agent
form a map of choices

but different agents have the same choices in 


the influence + its position provides all the information the entity needs


i could have nodes

and then actions that 


some nodes have "influence otehr nodes"
that apply an influence to nodes around it
I want to follow a node to get to the spots that 



the influences are the environment


and then actions change depending on the state and the position


a map of "effects"
things that can be applied to a status
walking through


the influence has a "points, if a certain amount of damage is dealt here"


the influences

passing over them


a map of the shapes

the cost of passing over them


an influence has 




what does the neural engine need?

A graph (tree really) of things 

that stays consistent between agents and timesteps



a cnn that takes the scene around me
and returns a score

an influence at each of the squares around me


"the results model knows that this status, allows this influencemap to have its influence applied to it at this distance"



the input is the state of the game

the output should 100% be "a queue of events for this agent to perform"


there should probably be a way to define actions
like pass in a way to define actions and the consequences of them


"left right up down" and then the list of actions
how does that change the environment

thats the thing, I can't simulate the environment changing in response to my actions




changing the environment:


get a list of agents

they simulate the things that all others might choose to do



is this good enough?
What I currently imagine
I think I should stick with that
because I really can get "bigger" about it


the thing is that the user has to define the "base truth"
the way to get teh correct answer
the neural network just makes it more efficient


how to specify those nodes of decisions?
can other agents use them?



the "map" isnt anything about state, just about the decisions that are allowed


the decisions that 

the decisions units have in common





the state:
desires, directly related to its state:

its preferences are set to be as they currently are

the change in score


//4 teams
//what teams you hate

influence

"enemies killed (sum of enemy value scores)"
"damage done to enemies"



"step into the future and see "


"the influences"






how would I make this ai?


the units have boxes
boxes that when certain entities are inside the border of
affect the state
the agent also has a state

that state then can be used to calculate the actions



like
"this thing is in my forward box. Is it an enemy. If it is, apply "enemy in this box" function to affect my state"


get the state relative to the objects around me





so influence has "team" for the enemy
then state lets me know 


it has to support multiple agents
the influences have "team"

and the agents have "team"

and then I can program into the "influence affecting state"
to, when the teams dont intersect, and damage is done
that "damage done to enemy" increases which increases the score





influences:




//the "to_score" takes in an old state and a new state and returns the relative score

//the "results" COULD take an old state and then return a new state that represents the state taken through the highest path
//OR it could take an old state and return the SCORE when the state is taken through the highest path


//the damage done to enemies
//and the "priorities on damage done"
//"goals"
//and "achievements"
/*
If i could predict how it influences other entities
I could

a square on the ground when he wants to attack the ground



*/




state:


team
damage done to enemies



//the value of the new state
value (current state, new state )-> f32:


let joyinservice = damage to enemies + enemeies killed * loyaltytocause;

let stillalive = newstate.alive - currentstate.alive;

return new_state.health - currentstate.health;








*/

//turning the 

//at each position, the actions the agent can take
//what that results in


//the influences


//