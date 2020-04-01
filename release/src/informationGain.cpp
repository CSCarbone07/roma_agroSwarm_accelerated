#include "sim/engine.hpp"
#include "movementstrategies/informationGain.hpp"
#include "sim/world.hpp"


//#define INCREMENTAL_SET
#define FIXEDSIZE3x3    //ifndef  --> default 5x5
//#define PERFECT_COMMUNICATION  //ifndef the communication range is limited according to the parameter stored in World.hpp, change also in agent.cpp
//#define RELAXED_VERSION


InformationGainStrategy::InformationGainStrategy(Agent* ag)
{
    ownerAgent = ag;

}




std::array<float,3> InformationGainStrategy::pickNextTarget(Agent* ag){
  unsigned id = ag->getId();
  unsigned sizex = Engine::getInstance().getWorld()->getSize().at(0)-1;
  unsigned sizey = Engine::getInstance().getWorld()->getSize().at(1)-1;
  std::array<float,3> agentPos = Engine::getInstance().getAgent(id)->getPosition();// Agent position
  
  /*  
  * Where to store elegibles. 
  * Vector of a pair: <Cell*, float> 
  * Cell* -> reference to the cell
  * float -> distance between cell and the agent
  */
  std::vector<std::pair<Cell*, float>> elegibles;

  std::array<unsigned,3> agentDiscretePos = {unsigned(fmax(0,fmin(agentPos.at(0),sizex))),unsigned(fmax(0,fmin(agentPos.at(1),sizey))),0}; // Discretization of agent position

  #ifdef INCREMENTAL
    max_range = 50*sqrt(2);
    min_range = 2*sqrt(2);  
  #else
    #ifndef FIXEDSIZE3x3
      max_range = 3*sqrt(2);
      min_range = 2*sqrt(2);
    #else
      max_range = 2*sqrt(2);
      min_range = sqrt(2);
    #endif
  #endif

  elegibles = getElegibles(ag, agentDiscretePos);

  std::vector<float> myProbabilities; // where to store probabilities for this agent
  myProbabilities.reserve(elegibles.size());
  std::vector<float> nextNearAgentProbabilities; // where to store probabilities for one near agent
  nextNearAgentProbabilities.reserve(elegibles.size());
  std::vector<float> allNearAgentsProbabilities; // where to store total probabilities for all near agents
  allNearAgentsProbabilities.reserve(elegibles.size());
  std::vector<float> finalProbabilitiesVector; // where to store final probabilities
  finalProbabilitiesVector.reserve(elegibles.size());

  //if there is only one elegible
  if(elegibles.size() == 1){
    Cell* c = elegibles.at(0).first;
    c->isTargetOf.push_back(id);
    ag->cells.at(c->getId())->isTargetOf.push_back(id);
    std::array<unsigned,3> cellPos = c->getPosition();
    return {cellPos.at(0)+0.5f,cellPos.at(1)+0.5f,float(cellPos.at(2))};
  }

float ig=0, sum=0, ig2=0, sum2=0, tot=0;

  //compute the probabilities for the elegibles
  for(unsigned i = 0; i<elegibles.size(); i++){
    ig = computeInformationGain(ag, elegibles.at(i).first);  //agent ag respect to elegible cell i
    myProbabilities.push_back((1/elegibles.at(i).second)*ig); // compute my probability for elegible i considering also the distance
    sum += myProbabilities.at(i);
    //for all agents in swarm  
    for(auto t : Engine::getInstance().getWorld()->getAgents()){
      //if agent t is near 
      float distance_t = t->calculateLinearDistanceToTarget(elegibles.at(i).first->getPosition());
      if(t->getId()!= id && distance_t != 0 && distance_t < max_range){
        for(unsigned j = 0; j<elegibles.size(); j++){
          //compute IG of agent t w.r.t. elegibles j
          ig2 = computeInformationGain(t, elegibles.at(j).first);
          float distance_t2 = t->calculateLinearDistanceToTarget(elegibles.at(j).first->getPosition());
          nextNearAgentProbabilities.push_back((1/distance_t2)*ig2); // probability of t to choose j 
          sum2 += nextNearAgentProbabilities.at(j);
        }
        tot *= 1 - (nextNearAgentProbabilities.at(i)/sum2);   

        sum2 = 0;
        nextNearAgentProbabilities.clear();
      }
    }
    //if there isn't any near agent
    if( tot == 0)
      tot = 1;
  
    allNearAgentsProbabilities.push_back(tot);
    tot = 0;
  }

  if(ownerAgent->GetUseSocialInfo())
  {
  //merge probabilities of this agent with the sum of each other
  tot = 0; sum2=0;
  for(unsigned i = 0; i<elegibles.size(); i++){
    finalProbabilitiesVector.push_back(myProbabilities.at(i)/sum * allNearAgentsProbabilities.at(i));
    sum2+=finalProbabilitiesVector.at(i);
  }
  }
  else
  {
  finalProbabilitiesVector = myProbabilities;
  }

//#ifndef RELAXED_VERSION     //complete version  
if(ownerAgent->GetTargetSelectionStrategy() == "random")
{


  //extract cell
  if(sum != 0){
    float random = RandomGenerator::getInstance().nextFloat(1);
    float cumulative = .0;
    for(unsigned i=0; i<elegibles.size(); i++){
      cumulative += finalProbabilitiesVector.at(i);
      if(random <= cumulative){
                Cell* c = elegibles.at(i).first;
                c->isTargetOf.push_back(id);
                ag->cells.at(c->getId())->isTargetOf.push_back(id);
                std::array<unsigned,3> cellPos = c->getPosition();
                return {cellPos.at(0)+0.5f,cellPos.at(1)+0.5f,float(cellPos.at(2))};
      }
    }
  }
}

if(ownerAgent->GetTargetSelectionStrategy() == "greedy")
{
  
  float max = 0;
  int index = -1;
//#else  //relaxed version: only the contribution of the individual agent is considered. The cell with the highest probability is chosen.
/*
  for(unsigned i = 0; i<elegibles.size(); i++){
    float ig = computeInformationGain(ag, elegibles.at(i).first);
    myProbabilities.push_back((1/elegibles.at(i).second)*ig);
    sum += myProbabilities.at(i);
  }
*/

  for(unsigned i = 0; i<finalProbabilitiesVector.size(); i++){
    if(finalProbabilitiesVector.at(i) > max){
      max = finalProbabilitiesVector.at(i);
      index = i;
    }
  }
  if(index != -1){
    Cell* c = elegibles.at(index).first;
    c->isTargetOf.push_back(id);
    ag->cells.at(c->getId())->isTargetOf.push_back(id);
    std::array<unsigned,3> cellPos = c->getPosition();
    return {cellPos.at(0)+0.5f,cellPos.at(1)+0.5f,float(cellPos.at(2))};
  }
}
//#endif

  // if there are no elegibles
  Engine::getInstance().getWorld()->unCommittedAgents ++;
  return {-1,-1,-1};
}


bool InformationGainStrategy::isElegible(Cell* c, Agent* ag)
{
 
   if(ownerAgent->GetCommunicationsRange() == -1)
   {
     Cell* worldCell_REF = Engine::getInstance().getWorld()->getCells().at(c->getId());
     return (!(worldCell_REF->isMapped()) && (worldCell_REF->isTargetOf.size())==0);
   }
   else
   {
     return ((!ownerAgent->cells.at(c->getId())->isMapped()) && (!(ownerAgent->cells.at(c->getId())->isTargetOf.size())==0));
     //return ((!ag->cells.at(c->getId())->isMapped()) && (ag->cells.at(c->getId())->isTargetOf.size())<=0);
   }
}


/**
* @return true, if x and y are within width and height
*/
bool isInBound(unsigned x, unsigned y){     
	return x >= 0 && y >= 0 && x < Engine::getInstance().getWorld()->getSize().at(0) && y < Engine::getInstance().getWorld()->getSize().at(1);;
}

//update the knowledge base of agent a in case of  limited communication range
void updateKB(Cell* cell, Agent* a){
  unsigned targetId = cell->getId();
  int agentId = -1;
  unsigned cellTimeStep = a->cells.at(targetId)->lastTimeVisit;
  int isTarget=-1;
  for(auto t : Engine::getInstance().getWorld()->getAgents()){
    if(a->getId() != t->getId()){ 
      float distance_t = t->calculateLinearDistanceToTarget(a->getPosition());
        if( distance_t != 0 && distance_t < Engine::getInstance().getWorld()->communication_range){     
          if(t->cells.at(targetId)->lastTimeVisit >cellTimeStep){
            cellTimeStep = t->cells.at(targetId)->lastTimeVisit;
            agentId = t->getId();
          }
          if (!t->cells.at(targetId)->isTargetOf.empty()){
            isTarget=t->getId();
          }
        }
      }
    } 
    if (agentId != -1){
      a->cells.erase(targetId);
      a->cells.insert(std::make_pair<>(targetId, Engine::getInstance().getWorld()->getAgent(agentId)->cells.at(targetId)));
    }
    if(isTarget != -1 && a->cells.at(targetId)->isTargetOf.empty()){
      a->cells.at(targetId)->isTargetOf.push_back(isTarget);
    }  
  
}

std::vector<std::pair<Cell*, float>> InformationGainStrategy::getElegibles(Agent* ag, std::array<unsigned,3> agentDiscretePos){
  std::vector<std::pair<Cell*, float>> ret;

  Cell* currentOccupiedCell; // cell where agent is located
  if(ownerAgent->GetCommunicationsRange()==-1)
  {
    currentOccupiedCell = Engine::getInstance().getWorld()->getCell(agentDiscretePos);
  }
  if(ownerAgent->GetCommunicationsRange()>0)
  {
    currentOccupiedCell = ownerAgent->cells.at(Engine::getInstance().getWorld()->getCell(agentDiscretePos)->getId());
  }

  //check first current cell
  if((Engine::getInstance().getWorld()->unCommittedAgents == Engine::getInstance().getWorld()->getAgents().size()-1 || Engine::getInstance().getWorld()->getAgent(ag->getId())->getTargetId() == -2) && isElegible(currentOccupiedCell, ag)){
    ret.push_back(std::make_pair<>(currentOccupiedCell, 0));
    #ifndef PERFECT_COMMUNICATION        
      //updateKB(Engine::getInstance().getWorld()->getCell(agentDiscretePos), ag);
    #endif
      return ret;
  }

  #ifdef INCREMENTAL_SET
  bool found = false;
  //incremental valid-set 
  for (std::map<float, std::vector<std::pair<int, int>>>::iterator it=Engine::getInstance().getWorld()->distanceVectors.begin(); it!=Engine::getInstance().getWorld()->distanceVectors.end(); ++it){
    for(std::vector<std::pair<int,int>>::iterator it2=it->second.begin(); it2!=it->second.end(); ++it2){
      int newX = it2->first + int (agentDiscretePos.at(0));
      int newY = it2->second + int (agentDiscretePos.at(1));
      if(isInBound(newX, newY)){
        
        //Cell* cell = Engine::getInstance().getWorld()->getCell(it2->first + agentDiscretePos.at(0), it2->second + agentDiscretePos.at(1), 0);
         Cell* cell;

         Cell* worldCell_REF = Engine::getInstance().getWorld()->getCell(it2->first + agentDiscretePos.at(0), it2->second + agentDiscretePos.at(1), 0);
         if(ownerAgent->GetCommunicationsRange() == -1)
         {cell = worldCell_REF;}
         if(ownerAgent->GetCommunicationsRange() > 0)
         {cell = ownerAgent->cells.at(worldCell_REF->getId());}  

        if(isElegible(cell, ag)){
          found = true;
          ret.push_back(std::make_pair<>(cell, it->first));
        }
      }
    }
    if(found){
      break;
    }
  }
  #else     //fixed size valid-set 5x5
  
  std::vector<std::pair<Cell*, float>> ret2;   //elegible cells on the boundary
  std::vector<std::pair<Cell*, float>> ret3;   //mapped but not targeted cells on the boundary
  std::vector<std::pair<Cell*, float>> ret4;   //all boundary cells

  float max_range_5x5 = 3*sqrt(2);
  float min_range_5x5 = 2*sqrt(2);
  float max_range_3x3 = 2*sqrt(2);
  float min_range_3x3 = sqrt(2);

  Cell* cella;


  for (std::map<float, std::vector<std::pair<int, int>>>::iterator it=Engine::getInstance().getWorld()->distanceVectors.begin(); it!=Engine::getInstance().getWorld()->distanceVectors.end(); ++it){
    for(std::vector<std::pair<int,int>>::iterator it2=it->second.begin(); it2!=it->second.end(); ++it2){
      int newX = it2->first + int (agentDiscretePos.at(0));
      int newY = it2->second + int (agentDiscretePos.at(1));
      if(it->first > max_range){
        break;
      }
      if(isInBound(newX, newY)){
        //Cell* cell = Engine::getInstance().getWorld()->getCell(it2->first + agentDiscretePos.at(0), it2->second + agentDiscretePos.at(1), 0);
        Cell* worldCell_REF = Engine::getInstance().getWorld()->getCell(it2->first + agentDiscretePos.at(0), it2->second + agentDiscretePos.at(1), 0);
        if(ownerAgent->GetCommunicationsRange() == -1)
        {cella = worldCell_REF;}
        if(ownerAgent->GetCommunicationsRange() > 0)
        {cella = ownerAgent->cells.at(worldCell_REF->getId());}  
       
        if(it->first > min_range_3x3 && it->first < max_range_3x3){
           if(isElegible(cella, ownerAgent)){
             ret2.push_back(std::make_pair<>(cella, it->first));
           }
         }
         if(it->first > min_range_5x5 && it->first < max_range_5x5 && cella->isTargetOf.empty())
         {
             if(isElegible(cella, ownerAgent))
             ret3.push_back(std::make_pair<>(cella, it->first));
         }
         if(it->first > min_range_3x3 && it->first < max_range_5x5 && cella->isTargetOf.empty())
             ret4.push_back(std::make_pair<>(cella, it->first));
         if(it->first <= min_range_3x3 && isElegible(cella, ownerAgent)){
             ret.push_back(std::make_pair<>(cella, it->first));
         }
       }
     }
   }


  //if there are no eligible cells
  if(ret.size()== 0){
    //check for not mapped cells on the boundary
    if(ret2.size()!=0){
      return ret2;   
    }
    //else check for mapped but not targeted cells on the boundary
    else if(ret3.size()!=0){
      Engine::getInstance().getWorld()->getAgent(ag->getId())->sceltaRandom = true;
      return ret3;
    }
    // if every cells is mapped and already targeted
    else{
      Engine::getInstance().getWorld()->getAgent(ag->getId())->sceltaRandom = true;
      return ret4;
    }
  }
#endif
  return ret;
}

//compute H(c|o)
float InformationGainStrategy::computeInformationGain(Agent* a, Cell* cell){

  Cell* targetCell = cell;
/*
#ifdef PERFECT_COMMUNICATION
  Cell* targetCell = cell;
#else
  Cell* targetCell = a->cells.at(cell->getId());
#endif
  */

    
  targetCell->observationVector.fill(0);

  for(unsigned k = 0; k < 13; k++ ){
    for(unsigned l = 0; l < 13; l++ ){
      targetCell->observationVector[k] += targetCell->knowledgeVector[l]*Engine::getInstance().getWorld()->getSensorTable()[k][l];
    }
  }
  
  float informationGain = 0;
  for(unsigned o = 0; o < 13; o++){
    for(unsigned c = 0; c < 13; c++){
      float logg = 0;
      if(targetCell->knowledgeVector[c] != 0)
        logg+=log(targetCell->knowledgeVector[c]);
      if(Engine::getInstance().getWorld()->getSensorTable()[o][c] != 0)
        logg+=log(Engine::getInstance().getWorld()->getSensorTable()[o][c]);
      if(targetCell->observationVector[o] != 0)
        logg-=log(targetCell->observationVector[o]);
      informationGain -= targetCell->knowledgeVector[c]*Engine::getInstance().getWorld()->getSensorTable()[o][c]*logg;
    }
  }

  return targetCell->residual_uncertainty - informationGain;
}

