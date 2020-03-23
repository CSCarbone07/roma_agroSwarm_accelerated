#include "agent/agent.hpp"
#include "util/randomgenerator.hpp"
#include "sim/engine.hpp"
#include "eigen3/Eigen/Dense"
#include "collisionavoidance/orca.hpp"
#include "movementstrategies/randomwalk.hpp"
#include "movementstrategies/informationGain.hpp"

//#define ONLY_COVERAGE
#define IG
//#define PERFECT_COMMUNICATION      //according to the same define in informationGain.cpp or randomwalk.cpp


Agent::Agent(unsigned id, float x, float y, float z) {
  this->id = id;
  this->position = {float(x),float(y),float(z)};
  this->target = {-1,-1,-1};
  this->targetId = -2;
  this->velocity = {0,0,0};
  this->timeStep = 0;

  this->communicationsRange = Engine::getInstance().getCommunicationsRange();  

  std::array<unsigned,3> size = Engine::getInstance().getWorld()->getSize();

  unsigned cellId = 0;
  unsigned cellSize = 1;
  for(unsigned y=0; y<size.at(1); y++){
    for(unsigned x=0; x<size.at(0); x++){
      Cell* c = new Cell(cellId,x,y,0,cellSize, 0);
      c->setUtility(Engine::getInstance().getWorld()->getCells().at(cellId)->getUtility());
      this->cells.insert(std::make_pair<>(cellId, c));
      cellId++;
    }
  }
}

Agent::Action Agent::nextAction(){
  if(this->getTargetX() != -1){
    if(checkTargetReached()) {
      if(this->getTargetZ() == 0){
        return SCAN;
      }
      else
        return NONE;
    }
    else{
      return MOVE;
    }
  }
  else{
    return PICK;
  }
}

void Agent::forgetTarget(){
  this->target = {-1,-1,-1};
  this->targetId = -1;
}

std::array<float,3> Agent::getNextPosition(){
  // Variables for storing optimal velocity
  float optimalVx = 0.0;
  float optimalVy = 0.0;
  float optimalVz = 0.0;
  // Move

  float newX = this->getX();
  float newY = this->getY();
  float newZ = this->getZ();
  std::vector<Agent*> others;
  // Difference between target and my position
  float diff_x = this->getTargetX() - this->getX();
  float diff_y = this->getTargetY() - this->getY();
  float diff_z = this->getTargetZ() - this->getZ();

  // Angle to target
  float alpha = atan2(diff_y,diff_x);
  
  // float beta = atan2(diff_z,diff_x);

// //Compute optimal velocity
	if(diff_y < 0)
	    optimalVy = -float(fmin(std::abs(diff_y), this->getLinearVelocity()));
	else
	    optimalVy = float(fmin(std::abs(diff_y), this->getLinearVelocity()));

	if(diff_x < 0)
	    optimalVx = -float(fmin(std::abs(diff_x), this->getLinearVelocity()));
	else
	    optimalVx = float(fmin(std::abs(diff_x), this->getLinearVelocity()));
  
  if(diff_z < 0)
	    optimalVz = -float(fmin(std::abs(diff_z),this->getLinearVelocity()));
	else
	    optimalVz = float(fmin(std::abs(diff_z), this->getLinearVelocity()));

  // optimalVx = cos(alpha) * this->getLinearVelocity();
  // optimalVy = sin(alpha) * this->getLinearVelocity();
  // optimalVz = sin(beta) * this->getLinearVelocity();
  // optimalVx = float(fmin(diff_x, this->getLinearVelocity()));
  // optimalVy = float(fmin(diff_y, this->getLinearVelocity()));
  // optimalVz = float(fmin(diff_z, this->getLinearVelocity()));

  // Orca
  float dt = 1;
  // Ideal orca velocity
  std::array<float,2> orca_velocity;
  orca_velocity={optimalVx, optimalVy};
  
  // float distance = calculateLinearDistanceToTarget();
  // if(distance < sqrt(optimalVx*optimalVx + optimalVy*optimalVy + optimalVz*optimalVz) || distance <= 0.1){
  //   orca_velocity = { diff_x, diff_y};
  //   optimalVx = diff_x;
  //   optimalVy = diff_y;
  //   optimalVz = diff_z;
  // }

  if(this->getTargetX() > -1){
    // If Collision Avoidance is enabled
    if(Engine::getInstance().getCollisionAvoidance()){
      if(Engine::getInstance().getWorld()->getAgents().size() > 1){
            Eigen::Vector2f orca_vel;
            orca_vel << orca_velocity.at(0), orca_velocity.at(1);
            Eigen::Vector2f new_vel = Orca::compute_orca(this, orca_vel, Engine::getInstance().getWorld()->committedAgents, Engine::getInstance().getTau(), dt);
            optimalVx = new_vel(0);
            optimalVy = new_vel(1);
            //assert(optimalVx==0);
      }
    }
  }
  this->velocity.at(0) = optimalVx;
  this->velocity.at(1) = optimalVy;
  this->velocity.at(2) = optimalVz;
  this->theta = alpha;
  std::array<float,3> nextPose;
  nextPose = { newX + optimalVx, newY + optimalVy, newZ+optimalVz };
  if(nextPose.at(2) < 0.5)
    this->checkCollision(nextPose,Engine::getInstance().getWorld()->getAgents());
  return nextPose;
}

bool Agent::checkCollision(std::array<float,3> nextPose,std::vector<Agent*> others){
  for(Agent* a : others){
    if(a->getId() == this->getId())
      continue;
    if(sqrt(pow((nextPose.at(0)-a->getX()),2)+pow((nextPose.at(1) - a->getY()),2)+pow((nextPose.at(2)-a->getZ()),2)) < (2*this->agentRadius)){
      this->collisions++;
      return true;
    }
  }
  return false;
}

void Agent::getNextTarget(){
  
  std::array<float,3> newTarget;

//choose the strategy IG or RW
#ifdef IG
  newTarget = ig.pickNextTarget(this);
#else
  newTarget = rw.pickNextTarget(this);
#endif
 
  if(newTarget.at(0) == -1){
    //the agent has no task! increase in altitude so as not to interfere with ORCA
    newTarget = {this->getX(), this->getY(), 4};
  }

  this->target = {float(newTarget.at(0)),float(newTarget.at(1)),float(newTarget.at(2))};
}


void Agent::broadcastCellInfo(Cell* cellToSend)
{



}


void Agent::RecieveCell(Cell* recievedCell) //recieving broadcasted latest observation of cell by agent to updating knowledge 
{

//    Engine::getInstance()

    
    Cell* updatingCell = this->cells.at(recievedCell->getId());
    updatingCell->isTargetOf = recievedCell->isTargetOf;
    updatingCell->observationVectors.insert(recievedCell->observationVectors.begin(), recievedCell->observationVectors.end());
    updatingCell->knowledgeVectors.insert(recievedCell->knowledgeVectors.begin(), recievedCell->knowledgeVectors.end());
    

/*
#ifndef PERFECT_COMMUNICATION
      // non-ideal case, communication range limited
      for(auto t : Engine::getInstance().getWorld()->getAgents()){
        if(this->getId() != t->getId()){ 
          float distance_t = t->calculateLinearDistanceToTarget(this->getPosition());
          if( distance_t != 0 && distance_t < Engine::getInstance().getWorld()->communication_range)
          {
            for (std::map<unsigned, Cell*>::iterator it=this->cells.begin(); it!=this->cells.end(); ++it)
            {  
              int ID = cells->first;
              this->cells.at(ID)->knowledgeVectors.insert(t->cells.at(ID)->knowledgeVectors.begin(), t->cells.at(ID)->knowledgeVectors.end());
              this->cells.at(ID)->observationVectors.insert(t->cells.at(ID)->observationVectors.begin(), t->cells.at(ID)->observationVectors.end());
            }
          }
            
        }
      } 
#else 
      for(auto c : Engine::getInstance().getWorld()->getCells())
      {
         this->cells.at(c->getId())->knowledgeVectors.insert(c->knowledgeVectors.begin(), c->knowledgeVectors.end());
         this->cells.at(c->getId())->observationVectors.insert(c->observationVectors.begin(), c->observationVectors.end());
      }
#endif
*/


}


bool Agent::doStep(unsigned timeStep){
  this->timeStep = timeStep;  

  switch(nextAction()){
    case PICK:
    {
	    this->velocity = {0.0,0.0,0.0};
      getNextTarget();
      if(this->getTargetZ() == 0){
        this->setTargetId(Engine::getInstance().getWorld()->getCellId(this->getTargetX(), this->getTargetY(), this->getTargetZ()));
      }

      break;
    }
    case MOVE:
    {
      std::array<float,3> nextPose = getNextPosition();
      if(!Engine::getInstance().moveAgentTo(nextPose.at(0),nextPose.at(1),nextPose.at(2),this->id)){
        std::cout << "I'M NOT MOVING" << std::endl;
        this->velocity.at(0) = 0;
        this->velocity.at(1) = 0;
      }
      break;
    }
    case SCAN:
	  {

      Cell* c = Engine::getInstance().getWorld()->getCell(this->position.at(0),this->position.at(1),this->position.at(2));
      c->numOfVisits++;
      
      this->velocity = {0.0,0.0,0.0};
      Engine::getInstance().getWorld()->remainingTasksToVisit.erase(c->getId());

      
#ifndef PERFECT_COMMUNICATION
      unsigned targetId = c->getId();
      currentCell = this->cells.at(targetId);
      currentCell->lastTimeVisit = timeStep;    
      Cell* scanningCell = currentCell;
#else 
      Cell* scanningCell = c;
#endif
       

        if(!scanningCell->isMapped()){
        float currentObservation = scanCurrentLocation(scanningCell);
        if(scanningCell->getResidual() < 0.27 ){ 
          scanningCell->setMapped();
          Engine::getInstance().getWorld()->remainingTasksToMap.erase(c->getId());
          Engine::getInstance().getWorld()->remainingTasksIntoClusters.erase(c->getId());
      #ifndef IG
          //IG does not use beacons
          if(scanningCell->getBeacon() != 0){
            Engine::getInstance().getWorld()->beacons.erase(scanningCell->getId());
            scanningCell->setBeacon(0);
          }
        }
        else{    // the cell is not yet mapped
          float beacon = currentObservation/12;
          scanningCell->setBeacon(beacon);
          Engine::getInstance().getWorld()->beacons.insert(std::make_pair<>(scanningCell->getId(), scanningCell));
        }
      #else
        }
      #endif        
      }
      else{
        ; //cell was already mapped. This situation may occur using a fixed size valid set.
      }

    c->isTargetOf.pop_back();
    this->cells.at(c->getId())->isTargetOf.pop_back();
    this->targetId = -1;
    this->target = {-1,-1,-1};
    break;
  }
    default:
      break;
  }
  
  return true;
};


/**
 * Perform a scan at the current location (that is supposed to be the targetLocation)
 */
unsigned Agent::scanCurrentLocation(Cell* currentCell){ 
	assert(getTargetId() != -1);

  // scan at current location and return the perceived cell
    
    //clear observationVector
    currentCell->observationVector.fill(0);
    //compute observationVector using the current knowledge and the constant sensorTable
    for(unsigned k = 0; k < 13; k++ ){
        for(unsigned l = 0; l < 13; l++ ){
            //std::cout<<"  TABLE   =  indice: "<<l<<" - "<<k<<"  "<<Engine::getInstance().getWorld()->getSensorTable()[l][k]<<std::endl;
            currentCell->observationVector[k] += currentCell->knowledgeVector[l]*Engine::getInstance().getWorld()->getSensorTable()[k][l];
        }
    }    
    //get current observation
    unsigned currentObservation;
    float random = RandomGenerator::getInstance().nextFloat(1);
    for (unsigned i = 0; i < 13; i++){  
      random -= Engine::getInstance().getWorld()->getSensorTable()[i][currentCell->getUtility()];
      if(random <= 0){
        currentObservation = i;
        break;
      }
    }                        
     
    currentCell->observationVectors.insert(std::pair<float, std::array<float, 13>>(timeStep, currentCell->observationVector));
    
    currentCell->residual_uncertainty = 0.0;
    float entr = 0;

    //update knowledgeVector given the currentObervation
    for(unsigned i = 0; i < 13; i++){
      currentCell->knowledgeVector[i] = currentCell->knowledgeVector[i] * Engine::getInstance().getWorld()->getSensorTable()[currentObservation][i] 
                                        / currentCell->observationVector[currentObservation];
      //if i-th element is != 0  ---> calculate H(c)
      if(currentCell->knowledgeVector[i] != 0)
        entr +=  currentCell->knowledgeVector[i]*(std::log(currentCell->knowledgeVector[i]));
    }
   
    currentCell->knowledgeVectors.insert(std::pair<float, std::array<float, 13>>(timeStep, currentCell->knowledgeVector));
    
    
 
    //store H(c) for the current cell
    currentCell->residual_uncertainty = -entr;

    return currentObservation;
  
};

bool Agent::getInfo(std::stringstream& ss){
  ss << this->getId() << ':' << ' ' << this->getX() << ' ' << this->getY() << ' ' << this->getZ()+2 << '\n';
  return true;
};
