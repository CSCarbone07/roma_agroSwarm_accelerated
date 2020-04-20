#pragma once

#ifndef AGENT_HPP
#define AGENT_HPP

#include "graphics/Mesh.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "sim/steppable.hpp"
#include "sim/WObject.h"
#include "movementstrategies/randomwalk.hpp"
#include "movementstrategies/informationGain.hpp"

#include "eigen3/Eigen/Dense"
#include "sim/cell.hpp"
#include <map>  

//#include "sim/world.hpp"

/**
 * Generic agent implementing the stappble method.
 * Implement this class.
 *
 * @author Dario Albani
 */

class Agent : public Steppable, public WObject {

protected:
  
 // World* myWorld;   //world of current agent

  unsigned id; /** < unique id for the agent */
  unsigned timeStep; /** < last execution time */
  Mesh* mesh = nullptr;
  glm::vec4 currentColor = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
  glm::vec4 movingColor = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
  glm::vec4 sendingMessageColor = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
  glm::vec4 receivingMessageColor = glm::vec4(0.0f, 0.0f, 0.5f, 1.0f);
  glm::vec4 scanningColor = glm::vec4(0.0f, 1.0f, 1.0f, 1.0f);



  float communicationsRange=-1;
  std::string knowledgeBaseLocation = "World";
  std::vector<Agent*> agentsBroadcasting;


  std::array<float,3> position; /** < position 3d of agent */
  std::array<float,3> target; /** < target 3d of agent */
  int targetId;
  float linearVelocity; /** < Linear Velocity of UAV */
  std::array<float,3> velocity; /** < actual Agent velocity */
  float theta = 0; /* Agent orientation */

  std::string currentInspectionStrategy = "rw";

  RandomWalkStrategy* rw; /** < random walk strategy */
  InformationGainStrategy* ig; /** < information gain strategy */

  float knowledgeClusterRadius = 0;
  std::string targetSelectionStrategy = "greedy";
  float softmaxLambda = 1;
  bool useSocialInfo = false;
  bool useDistanceForIG = false;

  unsigned targetLifeTime; /** < Time until agent junk target */

  float agentRadius = 0.3; /** < Real space occupated by the agent */
  unsigned collisions = 0; /** < Total collisions */
 
  void BroadcastCell(Agent* agent, Cell* cellToSend); //sending cell with local data
  void ReceiveCell(Cell* recievedCell); //recieving cell with new data

  float max_range_5x5 = 3*sqrt(2);
  float min_range_5x5 = 2*sqrt(2);
  float max_range_3x3 = 2*sqrt(2);
  float min_range_3x3 = sqrt(2);

 
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
  
  bool isInBound(unsigned x, unsigned y);
  /*
  bool isInBound(unsigned x, unsigned y, unsigned size_x, unsigned size_y);
  {     
	  return x >= 0 && y >= 0 && x < size_x && y < size_y;
  }
  */

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
  inline Mesh* getMesh() { return mesh; }
  inline void setMesh(Mesh* inMesh) { mesh = inMesh; }
  inline glm::vec4 getCurrentColor(){return currentColor;}
  inline unsigned getId() { return id; }
  inline unsigned get_time_step() { return timeStep; }
  inline float GetCommunicationsRange() { return communicationsRange; }
  inline std::string GetTargetSelectionStrategy() {return targetSelectionStrategy; }
  inline float GetSoftmaxLambda() {return softmaxLambda; }
  inline bool GetUseSocialInfo() { return useSocialInfo; }
  inline bool GetUseDistanceForIG() { return useDistanceForIG; }
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
