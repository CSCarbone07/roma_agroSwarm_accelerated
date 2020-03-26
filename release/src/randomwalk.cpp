#include "sim/engine.hpp"
#include "movementstrategies/randomwalk.hpp"
#include "sim/world.hpp"

//#define INCREMENTAL_SET
#define FIXEDSIZE3x3    //ifndef  --> defalut 5x5
#define PERFECT_COMMUNICATION        //change also in agent.cpp


RandomWalkStrategy::RandomWalkStrategy(Agent* ag)
{
 
   ownerAgent = ag;
   
}

std::array<float,3> RandomWalkStrategy::pickNextTarget(Agent* ag){
  unsigned id = ag->getId();
  unsigned sizex = Engine::getInstance().getWorld()->getSize().at(0)-1;
  unsigned sizey = Engine::getInstance().getWorld()->getSize().at(1)-1;
  std::array<float,3> agentPos = Engine::getInstance().getAgent(id)->getPosition();// Agent position
  std::vector<std::pair<Cell*, float>> elegibles; // where to store elegibles
  std::array<unsigned,3> agentDiscretePos = {unsigned(fmax(0,fmin(agentPos.at(0),sizex))),unsigned(fmax(0,fmin(agentPos.at(1),sizey))),0};//Engine::getInstance().getCommittedLevel()}; // Discretization of agent position  

  elegibles = getElegibles(ag, agentDiscretePos, id);

  std::vector<float> probabilities; // where to store probabilities
  probabilities.reserve(elegibles.size());

  if(elegibles.size() == 1){
    Cell* c = elegibles.at(0).first;
    //c->isTargetOf.push_back(id);
    std::array<unsigned,3> cellPos = c->getPosition();
    return {cellPos.at(0)+0.5f,cellPos.at(1)+0.5f,float(cellPos.at(2))};
  }

  //compute the probabilities for the elegibles
  float df=0, sum=0;
  Eigen::Vector2f directionVector(.0, .0);
  Eigen::Vector2f momentumVector(.0, .0);
  Eigen::Vector2f repulsionVector(.0, .0);
  Eigen::Vector2f attractionVector(.0, .0);
  
  if(Engine::getInstance().getRepulsion() > 0){
    repulsionVector = computeRepulsion(ag, agentPos);
  }
  
  if(Engine::getInstance().getAttraction() > 0){
    attractionVector = computeAttraction(ag, agentPos);
  }
  
  directionVector = momentumVector + repulsionVector + attractionVector;
  
  for(unsigned i = 0; i<elegibles.size(); i++){  
    Cell* cell = elegibles.at(i).first;
#ifndef INCREMENTAL_SET
      df = computeDirectionFactor(agentPos, cell, directionVector)/elegibles.at(i).second;
#else
      df = computeDirectionFactor(agentPos, cell, directionVector);
#endif
      probabilities.push_back(df);
      sum += df;
  }
  
  //extract cell
  if(elegibles.size() != 0){
    float random = RandomGenerator::getInstance().nextFloat(1);
    float cumulative = .0;
    for(unsigned i=0; i<elegibles.size(); i++){
      if(sum != 0)
        cumulative += probabilities.at(i)/sum;
      else
        cumulative += 1/elegibles.size();
      if(random <= cumulative){
        Cell* c = elegibles.at(i).first;
        //c->isTargetOf.push_back(id);
        std::array<unsigned,3> cellPos = c->getPosition();
        return {cellPos.at(0)+0.5f,cellPos.at(1)+0.5f,float(cellPos.at(2))};
      }
    }
  }

  Engine::getInstance().getWorld()->unCommittedAgents ++;
  return {-1,-1,-1};
}




bool RandomWalkStrategy::isElegible(Cell* c, Agent* ag)
{
  if(ownerAgent->GetCommunicationsRange() == -1)
  {
    return ((!ag->cells.at(c->getId())->isMapped()) && (c->isTargetOf.size())==0);
    //return ((!ag->cells.at(c->getId())->isMapped()) && (ag->cells.at(c->getId())->isTargetOf.size())<=0);
  }
  else
  {
    return ((!c->isMapped()) && (c->isTargetOf.size())==0);
  }
}
bool inRadius(unsigned x, unsigned y, Cell* c, unsigned radius){
//return (abs(x - c->getX()) < radius && abs(y - c->getY()) <= radius) || (abs(x - c->getX()) <= radius && abs(y - c->getY()) < radius);
}

/**
* @return true, if x and y are within width and height
*/
bool isInBounds(unsigned x, unsigned y){     
	return x >= 0 && y >= 0 && x < Engine::getInstance().getWorld()->getSize().at(0) && y < Engine::getInstance().getWorld()->getSize().at(1);;
}

void updateKBrw(Cell* cell, Agent* a){
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

std::vector<std::pair<Cell*, float>> RandomWalkStrategy::getElegibles(Agent* ag, std::array<unsigned,3> agentDiscretePos, unsigned id){
  Cell* c;
  std::vector<std::pair<Cell*, float>> ret;
  bool found = false;
  
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
  if((Engine::getInstance().getWorld()->unCommittedAgents == Engine::getInstance().getWorld()->getAgents().size()-1 || Engine::getInstance().getWorld()->getAgent(id)->getTargetId() == -2) && isElegible(currentOccupiedCell, ag))
  {
    ret.push_back(std::make_pair<>(currentOccupiedCell, 0));
#ifndef PERFECT_COMMUNICATION        
  //updateKBrw(Engine::getInstance().getWorld()->getCell(agentDiscretePos), ag);
#endif

    //std::cout << "Agent: " << ownerAgent->getId() << ", Found as elegibles cells: " ; 

    return ret;
  }
  
   


#ifdef INCREMENTAL_SET
  for (std::map<float, std::vector<std::pair<int, int>>>::iterator it=Engine::getInstance().getWorld()->distanceVectors.begin(); it!=Engine::getInstance().getWorld()->distanceVectors.end(); ++it){
    for(std::vector<std::pair<int,int>>::iterator it2=it->second.begin(); it2!=it->second.end(); ++it2){
      int newX = it2->first + int (agentDiscretePos.at(0));
      int newY = it2->second + int (agentDiscretePos.at(1));
      if(isInBounds(newX, newY)){
        Cell* cella = Engine::getInstance().getWorld()->getCell(it2->first + agentDiscretePos.at(0), it2->second + agentDiscretePos.at(1), 0);
        if(isElegible(cella, ag)){
          found = true;
          ret.push_back(std::make_pair<>(cella, 0));
        }
      }
    }
    if(found){
      break;
    }
  }
#else
  
  std::vector<std::pair<Cell*, float>> ret2;
  std::vector<std::pair<Cell*, float>> ret3;
  std::vector<std::pair<Cell*, float>> ret4;

#ifndef FIXEDSIZE3x3
    max_range = 3*sqrt(2);
    min_range = 2*sqrt(2);
#else
    max_range = 2*sqrt(2);
    min_range = sqrt(2);
#endif

  
  Cell* cella; 


  for (std::map<float, std::vector<std::pair<int, int>>>::iterator it=Engine::getInstance().getWorld()->distanceVectors.begin(); it!=Engine::getInstance().getWorld()->distanceVectors.end(); ++it){
    for(std::vector<std::pair<int,int>>::iterator it2=it->second.begin(); it2!=it->second.end(); ++it2){
      int newX = it2->first + int (agentDiscretePos.at(0));
      int newY = it2->second + int (agentDiscretePos.at(1));      
      if(it->first > max_range){
        break;
      }
      if(isInBounds(newX, newY)){

        Cell* worldCell_REF = Engine::getInstance().getWorld()->getCell(it2->first + agentDiscretePos.at(0), it2->second + agentDiscretePos.at(1), 0);
        if(ownerAgent->GetCommunicationsRange() == -1)
        {cella = worldCell_REF;}
        if(ownerAgent->GetCommunicationsRange() > 0)
        {cella = ownerAgent->cells.at(worldCell_REF->getId());}  

      if(it->first > min_range){
          if(isElegible(cella, ownerAgent)){
            ret2.push_back(std::make_pair<>(cella, it->first));
          }
          else if(cella->isTargetOf.size() == 0)
            ret3.push_back(std::make_pair<>(cella, it->first));
          
          ret4.push_back(std::make_pair<>(cella, it->first));    
        }
        if(it->first <= min_range && isElegible(cella, ownerAgent)){
          ret.push_back(std::make_pair<>(cella, it->first));
        }
      }
    }
  }

  if(ret.size()== 0){
    if(ret2.size()!=0){
      return ret2;   
    }
    else if(ret3.size()!=0){
      Engine::getInstance().getWorld()->getAgent(id)->sceltaRandom = true;
      return ret3;
    }
    else{
      Engine::getInstance().getWorld()->getAgent(id)->sceltaRandom = true;
      return ret4;
    }
  }
#endif
  return ret;
}

float RandomWalkStrategy::computeDirectionFactor(std::array<float,3> agentPos, Cell* c, Eigen::Vector2f directionVector){
  Eigen::Vector2f pos((c->getX()+0.5)-agentPos.at(0), (c->getY()+0.5)-agentPos.at(1));

 if(std::abs(pos[0]) <0.01 )
    pos[0] = 0;
  if(std::abs(pos[1]) <0.01 )
    pos[1] = 0;

  // compute the angle between the new cell vector and df
  float angle = (pos[0]*directionVector[0]+pos[1]*directionVector[1])/(pos.norm()*directionVector.norm());
  float arc_cosine = acosf(angle);

  if(isnanf(arc_cosine)){
    if(angle - 1 > -0.5){
      arc_cosine = 0;
    }
    else{
      arc_cosine = M_PI;
    }
  }
  float cauchyParameter = 1-exp(-(directionVector.norm()/2));
  // Cauchy PDF
  // c++ 1 only offers the random number generator
  return cauchyPDF(std::abs(arc_cosine),cauchyParameter);
}

Eigen::Vector2f RandomWalkStrategy::computeMomentum(float theta)
{
  Eigen::Vector2f momentum(0,0);
  // Angle to target
  momentum(0) = cos(theta);
  momentum(1) = sin(theta);
  
  //momentum.normalized();
  if(std::abs(momentum[0]) <0.01 )
    momentum[0] = 0;
  if(std::abs(momentum[1]) <0.01 )
    momentum[1] = 0;
  return momentum;
}

Eigen::Vector2f RandomWalkStrategy::computeRepulsion(Agent* ag, std::array<float,3> agentPos){
  float newX = 0;
  float newY = 0;
  for(auto t : Engine::getInstance().getWorld()->getAgents()){
    if(t->getId() != ag->getId()){
#ifndef PERFECT_COMMUNICATION
      float distance_t = t->calculateLinearDistanceToTarget(ag->getPosition());
      if( distance_t != 0 && distance_t < Engine::getInstance().getWorld()->communication_range){
#endif
        std::array<float,3> otherPos = t->getPosition();
        Eigen::Vector2f tv(agentPos.at(0) - otherPos.at(0), agentPos.at(1) - otherPos.at(1));

        float weight = 2*Engine::getInstance().gaussianPDF(tv.norm(), 0, Engine::getInstance().getRepulsion());  //(tc.length(), 0, Mav.potentialSpread, 2);
        newX += weight*tv[0];
        newY += weight*tv[1];
#ifndef PERFECT_COMMUNICATION
      }
#endif
    }
  }
  Eigen::Vector2f repulsion(newX,newY);
  //avoid normalizing a zero vector
  if(!repulsion.isZero())
    return repulsion.normalized();

  return Eigen::Vector2f(0,0);
}

Eigen::Vector2f RandomWalkStrategy::computeAttraction(Agent* ag, std::array<float,3> agentPos){
  float newX = 0;
  float newY = 0;
  std::map<unsigned, Cell*> beaconsCell = Engine::getInstance().getWorld()->beacons;
  for (std::map<unsigned, Cell*>::iterator i = beaconsCell.begin(); i != beaconsCell.end(); ++i) { 
#ifndef PERFECT_COMMUNICATION    
    //updateKBrw((*i).second, ag);
    if(ag->cells.at((*i).second->getId())->getBeacon() != 0){
#endif 
      Eigen::Vector2f tv(((*i).second->getX()+0.5) - agentPos.at(0), ((*i).second->getY()+0.5) - agentPos.at(1));
      float weight = (*i).second->getBeacon()* 2*Engine::getInstance().gaussianPDF(tv.norm(), 0, Engine::getInstance().getAttraction());  //(tc.length(), 0, Mav.potentialSpread, 2);
      newX += weight*tv[0];
      newY += weight*tv[1];
#ifndef PERFECT_COMMUNICATION
    }
#endif
  }
  Eigen::Vector2f attraction(newX,newY);
  //avoid normalizing a zero vector
  if(!attraction.isZero())
    return attraction.normalized();

  return Eigen::Vector2f(0,0);  
}
