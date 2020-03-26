#include "agent/agent.hpp"
#include "util/randomgenerator.hpp"
#include "sim/engine.hpp"
#include "eigen3/Eigen/Dense"
#include "collisionavoidance/orca.hpp"
#include "movementstrategies/randomwalk.hpp"
#include "movementstrategies/informationGain.hpp"

//#define ONLY_COVERAGE
//#define IG
//#define PERFECT_COMMUNICATION      //according to the same define in informationGain.cpp or randomwalk.cpp


Agent::Agent(unsigned id, float x, float y, float z) {
  this->id = id;
  this->position = {float(x),float(y),float(z)};
  this->target = {-1,-1,-1};
  this->targetId = -2;
  this->velocity = {0,0,0};
  this->timeStep = 0;

  this->communicationsRange = Engine::getInstance().getCommunicationsRange();  
  if(communicationsRange > 0)
  {
  knowledgeBaseLocation = "local";     
  }
  if(communicationsRange == -1)
  {
  knowledgeBaseLocation = "world";
  }
    
  this->currentInspectionStrategy = Engine::getInstance().getInspectionStrategy();  
   


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

  rw = new RandomWalkStrategy(this);
  
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
if(currentInspectionStrategy == "ig")
{newTarget = ig.pickNextTarget(this);}
else
{newTarget = rw->pickNextTarget(this);}


        //debug lines (can delete)       
        //int tempTargetID=Engine::getInstance().getWorld()->getCellId(newTarget.at(0), newTarget.at(1), newTarget.at(2));   
        //std::cout << "Debug ID cell " << tempTargetID << " has " << cells.at(tempTargetID)->isTargetOf.size() << " agents that had it as target." << std::endl;
        //end of debug lines 

  if(newTarget.at(0) == -1){
    //the agent has no task! increase in altitude so as not to interfere with ORCA
    newTarget = {this->getX(), this->getY(), 4};
  }

  this->target = {float(newTarget.at(0)),float(newTarget.at(1)),float(newTarget.at(2))};
}

           
void Agent::BroadcastCell(Cell* cellToSend)
{
    
    std::cout << "Agent: " << this->getId() << ". Broadcasting cell: " << cellToSend->getId() << " with " 
    << cellToSend->observationVectors.size()  << " observations. Mapping: " << cellToSend->isMapped() << std::endl;
    
    if(communicationsRange == -1)
    {
    Cell* worldCell_REF = Engine::getInstance().getWorld()->getCells().at(cellToSend->getId());
    worldCell_REF->isTargetOf = cellToSend->isTargetOf;
    worldCell_REF->observationVectors.insert(cellToSend->observationVectors.begin(), cellToSend->observationVectors.end());
    worldCell_REF->knowledgeVectors.insert(cellToSend->knowledgeVectors.begin(), cellToSend->knowledgeVectors.end());
    std::cout << "World knowledge base recieving cell " << worldCell_REF->getId()<< " observations and isTargetOf data" << std::endl;
    }
    if(communicationsRange > 0)
    {
      for(auto t : Engine::getInstance().getWorld()->getAgents()){
        if(this->getId() != t->getId())
        { 
          float distance_t = t->calculateLinearDistanceToTarget(this->getPosition());
          if( distance_t != 0 && distance_t <= this->communicationsRange)
          {
            std::cout << "Agent " << this->getId() << " at " << this->getX() << "x + " << this->getY() << "y + " << this->getZ() << "z"
            <<" sending cell " << cellToSend->getId() << " to agent " << t->getId() << " withing " << distance_t << "m of distance" << std::endl;
            t->ReceiveCell(cellToSend); 
          }   
        }
      } 
    }
    

}


void Agent::ReceiveCell(Cell* receivedCell) //recieving broadcasted latest observation of cell by agent to updating knowledge 
{

//    Engine::getInstance()

    std::cout << "Agent " << this->getId() << " at " << this->getX() << "x + " << this->getY() << "y + " << this->getZ() << "z"
    << " Recieving cell: " << receivedCell->getId() << ". Its current target cell is: " << targetId << std::endl;
    
    Cell* updatingCell = this->cells.at(receivedCell->getId());
    updatingCell->isTargetOf = receivedCell->isTargetOf;
    updatingCell->observationVectors.insert(receivedCell->observationVectors.begin(), receivedCell->observationVectors.end());
    updatingCell->knowledgeVectors.insert(receivedCell->knowledgeVectors.begin(), receivedCell->knowledgeVectors.end());
    if(receivedCell->isMapped())
    {updatingCell->setMapped();} 

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
        std::cout << "Agent " << this->getId() << " currently at: " << this->getX() << "x + " << this->getY() << "y + " << this->getZ() << "z"
        << " is picking target cell using: " << this->currentInspectionStrategy << " strategy" << std::endl;

        //std::cout << Engine::getInstance().getWorld()->getCellId(this->getX(), this->getY(), this->getZ()) << std::endl;   
        //std::cout << cells.at(Engine::getInstance().getWorld()->getCellId(this->getX(), this->getY(), this->getZ()))->isTargetOf.size() << std::endl;   



        this->velocity = {0.0,0.0,0.0};
        getNextTarget();
        if(this->getTargetZ() == 0){
        this->setTargetId(Engine::getInstance().getWorld()->getCellId(this->getTargetX(), this->getTargetY(), this->getTargetZ()));   

        Cell* chosenCell;        

        if(communicationsRange == -1)
        {
            chosenCell = Engine::getInstance().getWorld()->getCells().at(this->getTargetId());
        }
        if(communicationsRange>0)
        {
            chosenCell = cells.at(this->getTargetId());
        }
	    
        std::cout << "Agent " << this->getId() << " has picked "<< this->getTargetX() << "x + " << this->getTargetY() << "y in Cell " 
        << chosenCell->getId() << " from " << knowledgeBaseLocation << " knowledge base as Target, which previously had " << cells.at(this->getTargetId())->isTargetOf.size() << " agents that had it as target." << std::endl;
        
        if(communicationsRange == -1)
        {
        std::cout << "Agent " << this->getId() << " pushing isTargetOf to cell " << chosenCell->getId() << std::endl;
        BroadcastCell(chosenCell);
        }
        if(communicationsRange>0)
        {BroadcastCell(chosenCell);} 
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

      Cell* scanningCell;
      
      if(communicationsRange == -1)
      {scanningCell = c;}
      if(communicationsRange > 0)
      {
      unsigned targetId = c->getId();
      scanningCell = this->cells.at(targetId);
      scanningCell->lastTimeVisit = timeStep;    
      }
     
      
        std::cout << "Agent " << this->getId() << " scanning cell " << scanningCell->getId() << std::endl;    
        
        if(!scanningCell->isMapped())
        {
        
          float currentObservation = scanCurrentLocation(scanningCell);       
          if(scanningCell->getResidual() < 0.27 )
          { 
            scanningCell->setMapped();
            Engine::getInstance().getWorld()->remainingTasksToMap.erase(c->getId());
            Engine::getInstance().getWorld()->remainingTasksIntoClusters.erase(c->getId());

            c->isTargetOf.clear();
            this->cells.at(c->getId())->isTargetOf.clear();
            this->targetId = -1;
            this->target = {-1,-1,-1};
       
            if(scanningCell->getBeacon() != 0 && currentInspectionStrategy == "rw")
            {
              Engine::getInstance().getWorld()->beacons.erase(scanningCell->getId());
              scanningCell->setBeacon(0);
            }

          }
          else
          {    // the cell is not yet mapped
              if(currentInspectionStrategy == "rw")
               {
                  float beacon = currentObservation/12;
                  scanningCell->setBeacon(beacon);
                  Engine::getInstance().getWorld()->beacons.insert(std::make_pair<>(scanningCell->getId(), scanningCell));
               }
          }      
          
          std::cout << "Agent " << this->getId() << " starting to broadcast scan results of cell " << scanningCell->getId() << std::endl;
          if(communicationsRange == -1)
          {
            BroadcastCell(scanningCell);
          }
          if(communicationsRange>0)
          {BroadcastCell(scanningCell);} 
        
        }
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
    }
  
    currentCell->knowledgeVectors.insert(std::pair<float, std::array<float, 13>>(timeStep, currentCell->knowledgeVector));

    for (std::map<float, std::array<float,13>>::iterator it=currentCell->knowledgeVectors.begin(); it!=currentCell->knowledgeVectors.end(); ++it)
    {
      for(unsigned i = 0; i<13; i++)
      {
      //if i-th element is != 0  ---> calculate H(c)
      if(it->second[i] != 0)
        entr +=  it->second[i]*(std::log(it->second[i]));
      } 
    }  

    //if i-th element is != 0  ---> calculate H(c)
     // if(currentCell->knowledgeVector[i] != 0)
       // entr +=  currentCell->knowledgeVector[i]*(std::log(currentCell->knowledgeVector[i]));
 
    
    
 
    //store H(c) for the current cell
    currentCell->residual_uncertainty = -entr;

    return currentObservation;
  
};

bool Agent::getInfo(std::stringstream& ss){
  ss << this->getId() << ':' << ' ' << this->getX() << ' ' << this->getY() << ' ' << this->getZ()+2 << '\n';
  return true;
};
