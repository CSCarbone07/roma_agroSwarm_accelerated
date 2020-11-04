#include "sim/engine.hpp"
#include "movementstrategies/randomwalk.hpp"
#include "sim/world.hpp"
#include <iterator>
#include <algorithm>

//#define INCREMENTAL_SET
#define FIXEDSIZE3x3    //ifndef  --> defalut 5x5
//#define PERFECT_COMMUNICATION        //change also in agent.cpp


RandomWalkStrategy::RandomWalkStrategy(Agent* ag)
{
   ownerAgent = ag;
   testingId = ownerAgent->getTestingId();
   worldCellSize.at(0) = Engine::getInstance().getWorld()->getSize().at(0);
   worldCellSize.at(1) = Engine::getInstance().getWorld()->getSize().at(1);
}

std::array<float,3> RandomWalkStrategy::pickNextTarget(Agent* ag){
  unsigned id = ag->getId();
  unsigned sizex = Engine::getInstance().getWorld()->getSize().at(0)-1;
  unsigned sizey = Engine::getInstance().getWorld()->getSize().at(1)-1;
  std::array<float,3> agentPos = Engine::getInstance().getAgent(id)->getPosition();// Agent position
  std::vector<std::pair<Cell*, float>> elegibles; // where to store elegibles
  std::array<unsigned,3> agentDiscretePos = {unsigned(fmax(0,fmin(agentPos.at(0)+0.5,sizex))),unsigned(fmax(0,fmin(agentPos.at(1)+0.5,sizey))),0};//Engine::getInstance().getCommittedLevel()}; // Discretization of agent position  

  elegibles = getElegibles(ag, agentDiscretePos, id);
  //std::cout << elegibles.size() << std::endl;

  bool DEBUG_THIS = false;
  if(ownerAgent->getId() == testingId && DEBUG_THIS)
  {
    std::cout << "Agent " << ownerAgent->getId() << " Discrete Position: " 
    << agentDiscretePos.at(0) << "x + " << agentDiscretePos.at(1) << "y" << std::endl;
      std::cout << "Agent " << ownerAgent->getId() << " non Discrete Position: " 
    << ownerAgent->getX() << "x + " << ownerAgent->getY() << "y" << std::endl;

    std::cout << "Elegible cells found: " << elegibles.size() << std::endl;
    for(auto ele : elegibles)
    {
      std::cout << "Elegible cell " << ele.first->getId() << " at: " << ele.first->getX() << "x " << ele.first->getY() << "y" 
      //<< " test id: " << Engine::getInstance().getWorld()->getCell(std::array<unsigned,3>{ele.first->getX(),ele.first->getY(),0})->getId() << std::endl;
      << " test id: " << Engine::getInstance().getWorld()->getCell(ele.first->getX(),ele.first->getY(),0)->getId() << std::endl;
      //if(ele.first->isMapped())
      //{std::cout << "ERROR! Cell: " << ele.first->getId() << " was found as elegible although it was mapped" << std::endl;} 
    }
  }
  
  


  std::vector<float> probabilities; // where to store probabilities
  probabilities.reserve(elegibles.size());

  if(elegibles.size() == 1){
    Cell* c = elegibles.at(0).first;
    //c->isTargetOf.push_back(id);
    std::array<unsigned,3> cellPos = c->getPosition();
    //std::cout << "returning position of current picked cell " << c->getId() << ". Mapped:" << Engine::getInstance().getWorld()->getCells().at(c->getId()) << std::endl;
    return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
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

  bool DEBUG_VECTORS = false;
  
  if(DEBUG_VECTORS && ownerAgent->getId() == testingId)
  {
    std::cout << "attraction vector: " << attractionVector << std::endl;
    std::cout << "repulsion vector: " << repulsionVector << std::endl;

  }
  

  directionVector = momentumVector + repulsionVector + attractionVector;
  
  for(unsigned i = 0; i<elegibles.size(); i++)
  {  
    Cell* cell = elegibles.at(i).first;
#ifndef INCREMENTAL_SET
      df = computeDirectionFactor(agentPos, cell, directionVector)/elegibles.at(i).second;
#else
      df = computeDirectionFactor(agentPos, cell, directionVector);
#endif
      probabilities.push_back(df);
      sum += df;

      if(ownerAgent->getId() == testingId && DEBUG_THIS)
      {
        std::cout << "df: " << df << " with sum: " << sum << " and elegibles: " << elegibles.at(i).second << std::endl;
      }

  }
  
  //extract cell
  if(elegibles.size() != 0)
  {
    float random = RandomGenerator::getInstance().nextFloat(1);
    float cumulative = .0;
    for(unsigned i=0; i<elegibles.size(); i++)
    {
      if(sum != 0)
        {cumulative += probabilities.at(i)/sum;}
      else
        {cumulative += 1.0/elegibles.size();}
      if(ownerAgent->getId() == testingId && DEBUG_THIS)
      {
        std::cout << "Cumulative " << cumulative << " with sum: " << sum << std::endl;
      }
      if(random <= cumulative)
      {
        Cell* c = elegibles.at(i).first;
        c->isTargetOf.push_back(id);
        std::array<unsigned,3> cellPos = c->getPosition();

        if(ownerAgent->getId() == testingId && DEBUG_THIS)
        {
          std::cout << "Agent " << ownerAgent->getId() << " choose cell: " << c->getId() << " at: " 
          << cellPos.at(0) << "x + " << cellPos.at(1) << "y + " << cellPos.at(2) << "z" << std::endl;
        }

        return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
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
    Cell* worldCell_REF = Engine::getInstance().getWorld()->getCells().at(c->getId());
    return (!(worldCell_REF->isMapped()) && (worldCell_REF->isTargetOf.empty()));
  }
  else
  {

    return (!(ownerAgent->cells.at(c->getId())->isMapped()) && ((ownerAgent->cells.at(c->getId())->isTargetOf.size())==0));

    /*
    bool isElegible = false;
    float distanceToTargetingAgent = 0;
    Agent* targetingAgent = nullptr;
    if((ownerAgent->cells.at(c->getId())->isTargetOf.size())>0)
    {
      int targetingAgentID = ownerAgent->cells.at(c->getId())->isTargetOf.at(0);
      targetingAgent = Engine::getInstance().getWorld()->getAgents().at(targetingAgentID);
      distanceToTargetingAgent = ownerAgent->calculateLinearDistanceToTarget(targetingAgent->getPosition());
    }

    if(!(ownerAgent->cells.at(c->getId())->isMapped()) && ((ownerAgent->cells.at(c->getId())->isTargetOf.size())==0))
    {
      isElegible = true;
    }
    if(!(ownerAgent->cells.at(c->getId())->isMapped()) && distanceToTargetingAgent<=ownerAgent->GetCommunicationsRange()
    && targetingAgent!=nullptr && targetingAgent->getTargetId() != c->getId())
    {
      isElegible = true;
    }
    if(!(ownerAgent->cells.at(c->getId())->isMapped()) && distanceToTargetingAgent>ownerAgent->GetCommunicationsRange())
    {
      isElegible = true;
    }
    return isElegible;
    */
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
  if((Engine::getInstance().getWorld()->unCommittedAgents == Engine::getInstance().getWorld()->getAgents().size()-1 
  || Engine::getInstance().getWorld()->getAgent(id)->getTargetId() == -2) && isElegible(currentOccupiedCell, ag))
  {
    ret.push_back(std::make_pair<>(currentOccupiedCell, 0));  //returning cell, distance
#ifndef PERFECT_COMMUNICATION        
  //updateKBrw(Engine::getInstance().getWorld()->getCell(agentDiscretePos), ag);
#endif

    //std::cout << "Agent: " << ownerAgent->getId() << " found current cell " << currentOccupiedCell->getId() << " as elegible" << std::endl; 

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
  //std::vector<std::pair<Cell*, float>> ret5;

#ifndef FIXEDSIZE3x3
    max_range = 3*sqrt(2);
    min_range = 2*sqrt(2);
#else
    max_range = 2*sqrt(2);
    min_range = sqrt(2);
#endif

    float max_range_5x5 = 3*sqrt(2);
    float min_range_5x5 = 2*sqrt(2);

    float max_range_3x3 = 2*sqrt(2);
    float min_range_3x3 = sqrt(2);
  
  // min range = 1
  // max range = root square 2

  // do check with ids using the rows and columns
  // 


  Cell* cella = currentOccupiedCell; 


  std::vector<Cell*> cells_3x3 = cella->get3x3();
  std::vector<Cell*> cells_5x5 = cella->get5x5();

  float distanceToCell = 0;
  for(Cell* c : cells_3x3)
  {
    distanceToCell = sqrt(pow((float) c->getX() - ownerAgent->getX(), 2) + pow((float) c->getY() - ownerAgent->getY(), 2));
    if(isElegible(c, ownerAgent))
      {ret2.push_back(std::make_pair<>(c, distanceToCell));}
    //if(c->isTargetOf.empty())
      //{ret4.push_back(std::make_pair<>(c, distanceToCell));}
  }
  for(Cell* c : cells_5x5)
  {
    distanceToCell = sqrt(pow((float) c->getX() - ownerAgent->getX(), 2) + pow((float) c->getY() - ownerAgent->getY(), 2));
    //std::cout << distanceToCell << std::endl;
    if(isElegible(c, ownerAgent))
      {ret3.push_back(std::make_pair<>(c, distanceToCell));}
    if(c->isTargetOf.empty())
      {ret4.push_back(std::make_pair<>(c, distanceToCell));}
  }
  distanceToCell = sqrt(pow((float) currentOccupiedCell->getX() - ownerAgent->getX(), 2) + pow((float) currentOccupiedCell->getY() - ownerAgent->getY(), 2));
  ret.push_back(std::make_pair<>(currentOccupiedCell, distanceToCell));




    if(ret2.size()!=0){
      //std::cout << "returning ret2" << std::endl;
      return ret2;   
    }
    else if(ret3.size()!=0){
      //std::cout << "returning ret3" << std::endl;
      //Engine::getInstance().getWorld()->getAgent(id)->sceltaRandom = true;
      return ret3;
    }
    else{
      //std::cout << "returning ret4" << std::endl;
      //Engine::getInstance().getWorld()->getAgent(id)->sceltaRandom = true;
      return ret4;
    }
  
#endif
  return ret;
}

float RandomWalkStrategy::computeDirectionFactor(std::array<float,3> agentPos, Cell* c, Eigen::Vector2f directionVector){
  
  bool DEBUG_FUNCTION = false;

  Eigen::Vector2f pos((c->getX())-agentPos.at(0), (c->getY())-agentPos.at(1));

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
  float beta = 5;
  double sigmoid = (1-exp(-beta * directionVector.norm()))/(1+exp(-beta * directionVector.norm()));
  sigmoid *= 0.99;
  
  // Cauchy PDF
  // c++ 1 only offers the random number generator

   if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
   {
      std::cout << "Arc cosine: " << std::abs(arc_cosine) << std::endl;
      std::cout << "Sigmoid 1: " << -beta * directionVector.norm() << std::endl;
      std::cout << "Sigmoid 2: " << (exp(-beta * directionVector.norm())) << std::endl;
      std::cout << "Sigmoid: " << sigmoid << std::endl;
      std::cout << "Direction vector: " << directionVector.norm() << std::endl;
      std::cout << "Cauchy: " << cauchyPDF(std::abs(arc_cosine),sigmoid) << std::endl;
   }
  return cauchyPDF(std::abs(arc_cosine),sigmoid); //cauchy parameter replaced by sigmoid
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
  bool DEBUG_FUNCTION = false;
  float newX = 0;
  float newY = 0;
  int agentsInRange = 0;
  for(auto t : Engine::getInstance().getWorld()->getAgents())
  {
    if(t->getId() != ag->getId())
    {
      float distance_t = t->calculateLinearDistanceToTarget(ag->getPosition());
      if(distance_t <= distance_9x9 &&
      (( distance_t != 0 && distance_t < Engine::getInstance().getWorld()->communication_range) ||  Engine::getInstance().getWorld()->communication_range == -1))
      {

        std::array<float,3> otherPos = t->getPosition();
        Eigen::Vector2f tv(otherPos.at(0) - agentPos.at(0), otherPos.at(1) - agentPos.at(1));
        if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
        {
          std::cout << "Agent " << ag->getId() 
          << " at " << ownerAgent->getX() << " x, " << ownerAgent->getY() << " y"
          << " repulsing agent " << t->getId() 
          << " at distance: " << tv[0] << " x, " << tv[1] << " y ";
        }
        float weight = 2*Engine::getInstance().gaussianPDF(tv.norm(), 0, Engine::getInstance().getRepulsion());  //(tc.length(), 0, Mav.potentialSpread, 2);
        tv.normalize();
        newX += -weight*tv[0];
        newY += -weight*tv[1];
        agentsInRange ++;

        if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
        {
          std::cout << " weight: " << weight
          << " with repulsion: " << -weight*tv[0] << " x, " << -weight*tv[1] << " y" << std::endl;
        }

      }
    }
  }
  
  //if(DEBUG_FUNCTION)
  //{std::cout << "Weights found: " << newX << "x + " << newY << "y" << std::endl;}
  Eigen::Vector2f repulsion(newX,newY);
  //avoid normalizing a zero vector

  if(!repulsion.isZero())
  {
    if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
    {
      std::cout << "Repulsion found: " << newX << " x, " << newY << " y. With: " << agentsInRange << " agents" << std::endl;
    }
    repulsion[0];///=agentsInRange;
    repulsion[1];///=agentsInRange;

    if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
    {
      std::cout << "Repulsion set: " << repulsion[0] << " x, " << repulsion[1] << " y. With: " << agentsInRange << " agents" << std::endl;
    }

    return repulsion;//.normalized();

  } 
  if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
  {
     std::cout << "Zero found" << std::endl;
  }
  return Eigen::Vector2f(0,0);
}

Eigen::Vector2f RandomWalkStrategy::computeAttraction(Agent* ag, std::array<float,3> agentPos)
{
  bool DEBUG_FUNCTION = false;
  
  float newX = 0;
  float newY = 0;
  int beaconsCount = 0;
  std::map<unsigned, Cell*> beaconsCells;
  Cell* currentOccupiedCell;

  unsigned sizex = Engine::getInstance().getWorld()->getSize().at(0)-1;
  unsigned sizey = Engine::getInstance().getWorld()->getSize().at(1)-1;
  std::array<unsigned,3> agentDiscretePos = {unsigned(fmax(0,fmin(agentPos.at(0)+0.5,sizex))),unsigned(fmax(0,fmin(agentPos.at(1)+0.5,sizey))),0};//Engine::getInstance().getCommittedLevel()}; // Discretization of agent position  

  currentOccupiedCell = Engine::getInstance().getWorld()->getCell(agentDiscretePos);

  if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
  {std::cout << "communication range: " << Engine::getInstance().getWorld()->communication_range << std::endl;}

  if(Engine::getInstance().getWorld()->communication_range == -1)
  {
    //beaconsCells = Engine::getInstance().getWorld()->beacons;
    
    beaconsCells.insert(std::make_pair<>(currentOccupiedCell->getId(),currentOccupiedCell));   
    for (Cell* c : currentOccupiedCell->get3x3())
    {
      beaconsCells.insert(std::make_pair<>(c->getId(),c));
    }
    for (Cell* c : currentOccupiedCell->get5x5())
    {
      beaconsCells.insert(std::make_pair<>(c->getId(),c));
    }
    
    
    
    if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
    {std::cout << "beacons set for unlimited range, found " << beaconsCells.size() << " beacons" << std::endl;}
  }
  if(Engine::getInstance().getWorld()->communication_range > 0)
  {
    beaconsCells = ownerAgent->beacons;
      if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
      {std::cout << "beacons set for limited range, found " << beaconsCells.size() << " beacons" << std::endl;}
  }

  
  //if(DEBUG_FUNCTION)
  //{std::cout << "checking" << beaconsCells.size() << std::endl;}
  //std::cout << "Beacons found" << beaconsCells.size() << std::endl;

  //for (std::map<unsigned, Cell*>::iterator i = beaconsCells.begin(); i != beaconsCells.end(); ++i) 
  for (std::map<unsigned, Cell*>::iterator i = beaconsCells.begin(); i != beaconsCells.end(); ++i) 
  { 
    float distance_t = ownerAgent->calculatePlaneLinearDistanceToObject((*i).second->getPosition());

    //if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
    //{std::cout << "Agent " << ag->getId() << " seeing cell at distance " << distance_t << std::endl;}

    if(//distance_t <= (distance_9x9+0.25) &&
    (ag->cells.at((*i).second->getId())->getBeacon() != 0 || 
    Engine::getInstance().getWorld()->communication_range == -1 && (*i).second->getBeacon() != 0))
    {
      /*
      if(abs((*i).second->getPosition().at(0)- ownerAgent->getPosition().at(0)) < (4+0.1) 
      && abs((*i).second->getPosition().at(1)- ownerAgent->getPosition().at(1)) < (4+0.1))
      */
      if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
      {
        std::cout << "Agent " << ag->getId() << " found beacon in range " << distance_t 
        << " distance difference " << (*i).second->getPosition().at(0)- ownerAgent->getPosition().at(0) << "x" 
        << " + " << (*i).second->getPosition().at(1)- ownerAgent->getPosition().at(1) << "y"
        << " with magnitude " << (*i).second->getBeacon() << std::endl;
      }
      Eigen::Vector2f tv(((*i).second->getX()) - agentPos.at(0), ((*i).second->getY()) - agentPos.at(1));
      float weight = (*i).second->getBeacon()* 2*Engine::getInstance().gaussianPDF(tv.norm(), 0, Engine::getInstance().getAttraction());  //(tc.length(), 0, Mav.potentialSpread, 2);
      tv.normalize();
      newX += weight*tv[0];
      newY += weight*tv[1];
      beaconsCount++;

    }
  }
  Eigen::Vector2f attraction(newX,newY);
  //avoid normalizing a zero vector
  if(!attraction.isZero())
  {
    if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
    {
      std::cout << "Attraction found: " << newX << " x, " << newY << " y. With: " << beaconsCount << " beacons" << std::endl;
    }

    attraction[0];///=beaconsCount;
    attraction[1];///=beaconsCount;

    if(DEBUG_FUNCTION && ownerAgent->getId() == ownerAgent->getTestingId())
    {
      std::cout << "Attraction set: " << attraction[0] << " x, " << attraction[1] << " y. With: " << beaconsCount << " beacons" << std::endl;
    }

    return attraction;
  }//.normalized();
  
  return Eigen::Vector2f(0,0);
}
