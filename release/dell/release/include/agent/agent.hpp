#ifndef AGENT_HPP
#define AGENT_HPP

#include "sim/steppable.hpp"
#include "movementstrategies/randomwalk.hpp"
#include "movementstrategies/informationGain.hpp"

#include "eigen3/Eigen/Dense"
#include "sim/cell.hpp"
#include <map>


/**
 * Generic agent implementing the stappble method.
 * Implement this class.
 *
 * @author Dario Albani
 */

class Agent : public Steppable {

protected:
  unsigned id; /** < unique id for the agent */
  unsigned timeStep; /** < last execution time */

  std::array<float,3> position; /** < position 3d of agent */
  std::array<float,3> target; /** < target 3d of agent */
  int targetId;
  float linearVelocity; /** < Linear Velocity of UAV */
  std::array<float,3> velocity; /** < actual Agent velocity */
  float theta = 0; /* Agent orientation */

  RandomWalkStrategy rw; /** < random walk strategy */
  InformationGainStrategy ig; /** < information gain strategy */

  unsigned targetLifeTime; /** < Time until agent junk target */

  float agentRadius = 0.3; /** < Real space occupated by the agent */
  unsigned collisions = 0; /** < Total collisions */
  
  /*
  * What is the next action to be performed?
  * If do not have a target -> pick one
  * If no available cell -> standby
  * If in position -> scan cell, pick another target
  * If not in position -> move to toward the target
  */ 
  enum Action{NONE=0, PICK, MOVE, SCAN};

  Action nextAction();
  /*
  * Check if Agent has reached the target
  * @return True if is reached, otherwise false
  */
  bool checkTargetReached(){
    
    if(this->getTargetZ() == 0 ){
      if(std::abs(this->getX() - this->getTargetX()) < 0.05f && std::abs(this->getY() - this->getTargetY()) < 0.05f ){
        return true;
      }
      else return false;
    }
    else
      if(std::abs(this->getX() - this->getTargetX()) < 0.5f && std::abs(this->getY() - this->getTargetY()) < 0.5f && std::abs(this->getZ() - this->getTargetZ()) < 1.5f){
        return true;
      }
      return false;
  }

  unsigned scanCurrentLocation(Cell* c);
  
  /**
   * Get Next Position
   * @return array<float,3> representing the position
   */
  std::array<float,3> getNextPosition();

  /*
  * Get Next Target
  * @return void but set protected variable target.
  */
  void getNextTarget();

  /*
  * Utility method used for debug collision
  * Unused anymore
  */
  bool checkCollision(std::array<float,3> nextPose,std::vector<Agent*> others);


public:
  bool sceltaRandom = 0;

  std::map<unsigned, Cell*> cells;             /** < in my agent knowledge there are cells */


  Agent(unsigned id, float x, float y, float z);
  ~Agent();

  /*
  * Compute Linear Distance to Target
  *  @return unsigned int representing linear distance
  */
  float calculateLinearDistanceToTarget(){
    return (sqrt(pow(this->getX()-this->getTargetX(),2)+pow(this->getY()-this->getTargetY(),2)+pow(this->getZ()-this->getTargetZ(),2)));
  }
  float calculateLinearDistanceToTarget(std::array<float,3> t){
    return (sqrt(pow(this->getX()-t.at(0),2)+pow(this->getY()-t.at(1),2)+pow(this->getZ()-t.at(2),2)));
  }
  float calculateLinearDistanceToTarget(std::array<unsigned,3> t){
    return (sqrt(pow(this->getX()-((float)t.at(0)+0.5),2)+pow(this->getY()-((float)t.at(1)+0.5),2)));
  }

  /**
  * @return true, if x and y are within width and height
  */
  bool isInBounds(unsigned x, unsigned y, unsigned size_x, unsigned size_y){     
	  return x >= 0 && y >= 0 && x < size_x && y < size_y;
  }
  
  /* Set Agent Position */
  inline void setPosition(float x, float y, float z){
    this->position.at(0) = x;
    this->position.at(1) = y;
    this->position.at(2) = z;
  }
  /* Set Velocity */
  inline void setVelocity(float linearVelocity){this->linearVelocity = linearVelocity;}
  /* Set Agent's Target manually */
  inline void setTarget(std::array<float,3> target){
    this->target = target;
    this->targetLifeTime = 2 * this->calculateLinearDistanceToTarget()/this->linearVelocity;
  }
  /* set Agent Radius ( used for Orca - Collision avoidance ) */
  inline void setAgentRadius(float radius){this->agentRadius = radius;}
  /* Set the targetId, targetId is a unsigned int that represent the id of the node where the target position live.
  * Remember that the Node is a discretization of the world.
  */
  inline void setTargetId(int id){this->targetId = id;}

  /* Getters */

  inline unsigned getId() { return id; }
  inline unsigned get_time_step() { return timeStep; }
  inline std::array<float,3> getPosition(){return this->position;}
  inline std::array<float,3> getTarget(){return this->target;}
  inline float getX(){return this->position.at(0);}
  inline float getY(){return this->position.at(1);}
  inline float getZ(){return this->position.at(2);}
  inline float getVX(){return this->velocity.at(0);}
  inline float getVY(){return this->velocity.at(1);}
  inline float getVZ(){return this->velocity.at(2);}
  inline float getTargetX(){return this->target.at(0);}
  inline float getTargetY(){return this->target.at(1);}
  inline float getTargetZ(){return this->target.at(2);}
  inline float getLinearVelocity(){return this->linearVelocity;} 
  inline float getCollisions(){return this->collisions;}  
  inline float getAgentRadius(){return this->agentRadius;}  
  inline float getTheta(){return this->theta;} 
  inline int getTargetId(){return this->targetId;} 
  /**
   * Do one simulation step
   */
  bool doStep(unsigned timeStep);

  /**
   * Retrieve information about this agent last simulation step
   */
  bool getInfo(std::stringstream& ss);


  /**
   * Used deleting target from and agent
   */
  void forgetTarget();


  //proposta nuova funzione: calcolare mio knowledge vector
  //per ogni cella utilizzando le conoscenze degli agenti entro un certo raggio
};

#endif /* AGENT_HPP */
