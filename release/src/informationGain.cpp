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
  testingId = ownerAgent->getTestingId();

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

  std::array<unsigned,3> agentDiscretePos = {unsigned(fmax(0,fmin(agentPos.at(0)+0.5,sizex))),unsigned(fmax(0,fmin(agentPos.at(1)+0.5,sizey))),0}; // Discretization of agent position

  
  elegibles = getElegibles(ag, agentDiscretePos);

  
  bool DEBUG_THIS = false;
  if(ownerAgent->getId() == testingId && DEBUG_THIS)
  {
  /*
    std::cout << "Agent " << ownerAgent->getId() << " Discrete Position: " 
    << agentDiscretePos.at(0) << "x + " << agentDiscretePos.at(1) << "y" << std::endl;
      std::cout << "Agent " << ownerAgent->getId() << " non Discrete Position: " 
    << ownerAgent->getX() << "x + " << ownerAgent->getY() << "y" << std::endl;
  */
    std::cout << "Using selection strategy: " << ownerAgent->GetTargetSelectionStrategy() << std::endl;

    std::cout << "Elegible cells found: " << elegibles.size() << std::endl;
    for(auto ele : elegibles)
    {
     Cell* worldCell_REF = Engine::getInstance().getWorld()->getCells().at(ele.first->getId());


      std::cout << "Elegible cell " << worldCell_REF->getId() << " at: " << worldCell_REF->getX() << "x " << worldCell_REF->getY() << "y" 
      //<< " test id: " << Engine::getInstance().getWorld()->getCell(std::array<unsigned,3>{ele.first->getX(),ele.first->getY(),0})->getId() << std::endl;
      << " cell id: " << Engine::getInstance().getWorld()->getCell(ele.first->getX(),ele.first->getY(),0)->getId() 
      << " within distance of " << ele.second << " with previously " << worldCell_REF->isTargetOf.size() << " agents that had it as target"
      << std::endl;
      
      //if(ele.first->isMapped())
      //{std::cout << "ERROR! Cell: " << ele.first->getId() << " was found as elegible although it was mapped" << std::endl;}
      
    }
  }
  


  std::vector<float> myProbabilities; // where to store probabilities for this agent
  myProbabilities.reserve(elegibles.size());
  std::vector<float> nextNearAgentProbabilities; // where to store probabilities for one near agent
  nextNearAgentProbabilities.reserve(elegibles.size());
  std::vector<float> allNearAgentsProbabilities; // where to store total probabilities for all near agents
  allNearAgentsProbabilities.reserve(elegibles.size());
  std::vector<float> finalProbabilitiesVector; // where to store final probabilities
  finalProbabilitiesVector.reserve(elegibles.size());


  

  //if there is only one elegible
  if(elegibles.size() == 1)
  {
    Cell* c = elegibles.at(0).first;
    //c->isTargetOf.push_back(id);
    //ag->cells.at(c->getId())->isTargetOf.push_back(id);
    std::array<unsigned,3> cellPos = c->getPosition();
    return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
  }





  float ig=0, sum=0, ig2=0, sum2=0, tot=1;


  bool DEBUG_IG = false;
  //compute the probabilities for the elegibles
  for(unsigned i = 0; i<elegibles.size(); i++)
  {

    ig = computeInformationGain(ag, elegibles.at(i).first);  //agent ag respect to elegible cell i
    
    if(DEBUG_IG && ownerAgent->getId() == testingId)
    {
      std::cout << "Cell " << elegibles.at(i).first->getId() << " got a base ig of: " << ig << std::endl;
    }

    if(ownerAgent->GetUseDistanceForIG()) // compute my probability for elegible i considering also the distance
    {
      myProbabilities.push_back((1/elegibles.at(i).second)*ig);
    } 
    else
    {myProbabilities.push_back(ig);}


    if(DEBUG_IG && ownerAgent->getId() == testingId)
    {
      std::cout << "Cell " << elegibles.at(i).first->getId() << " got a base ig of: " << myProbabilities[i] << " after distance consideration" << std::endl;
    }

    sum += myProbabilities.at(i);

    allNearAgentsProbabilities.push_back(1);

    //for all agents in swarm  
    if(ownerAgent->GetUseSocialInfo())
    {
      tot = 1;
      for(auto t : Engine::getInstance().getWorld()->getAgents())
      {
        //if agent t is near 
        float distance_t = t->calculateLinearDistanceToTarget(elegibles.at(i).first->getPosition());
        if(t->getId()!= id && distance_t != 0 && (distance_t <= ownerAgent->GetCommunicationsRange() 
        || ownerAgent->GetCommunicationsRange() == -1))
        {
          if(DEBUG_IG && ownerAgent->getId() == testingId)
          {
          std::cout << "Starting if for agent " << t->getId() << " from agent " << ownerAgent->getId() << std::endl;
          }

          nextNearAgentProbabilities.clear();
          sum2 = 0;

          Cell* OtherAgentCell = Engine::getInstance().getWorld()->getCell(t->getX(),t->getY(),t->getZ());
          std::vector<Cell*> cells_5x5 = OtherAgentCell->get5x5();
          

          bool isCellInRange = false;
          for(Cell* c : cells_5x5)
          {
            if (c->getId() == elegibles.at(i).first->getId())
            {
              isCellInRange = true;
              if(DEBUG_IG && ownerAgent->getId() == testingId)
              {
              std::cout << "Elegible in range for agent " << t->getId() << " from agent " << ownerAgent->getId() << std::endl;
              }
            }
          }
          if(isCellInRange==false)
          {
            if(DEBUG_IG && ownerAgent->getId() == testingId)
            {
            std::cout << "Elegible not in range for agent " << t->getId() << " from agent " << ownerAgent->getId() << std::endl;
            }
            continue;
          }

          if(isCellInRange==true)
          {
            for(unsigned j = 0; j<elegibles.size(); j++)
            {
              ig2 = 0;
              
              //compute IG of agent t w.r.t. elegibles j
              if(ownerAgent->GetCommunicationsRange() == -1)
              ig2 = computeInformationGain(t, elegibles.at(j).first);
              if(ownerAgent->GetCommunicationsRange() > 0)
              {
                ig2 = computeInformationGain(t, t->cells[elegibles.at(j).first->getId()]);
              }
              float distance_t2 = 1;
              if(ownerAgent->GetUseDistanceForIG()) 
              {
                distance_t2 = t->calculateLinearDistanceToTarget(elegibles.at(j).first->getPosition());
              }
              
              nextNearAgentProbabilities.push_back((1/distance_t2)*ig2); // probability of t to choose j 
              sum2 += nextNearAgentProbabilities.at(j);

              if(DEBUG_IG && ownerAgent->getId() == testingId)
              {
                std::cout << "Cell " << elegibles.at(j).first->getId() << " for Agent " << t->getId() << " got a base ig of: " << ig2 << " after social consideration" << std::endl;
              }

              if(DEBUG_IG && ownerAgent->getId() == testingId)
              {
                std::cout << "Cell " << elegibles.at(j).first->getId() << " for Agent " << t->getId() << " got a base ig of: " << nextNearAgentProbabilities.at(j) << " after distance in social consideration" << std::endl;
              }

            } //end of "other agent" cells
          }

          float socialIG = 1;
          if(sum2 != 0)
          {socialIG = 1 - (nextNearAgentProbabilities.at(i)/sum2);} // i = cell whose probability is being calculated using IG of agent t
          tot *= socialIG;   

          if(DEBUG_IG && ownerAgent->getId() == testingId)
          {
            std::cout << "Cell " << elegibles.at(i).first->getId() << " for Agent " << t->getId() << " got a total ig contribution of: " << socialIG << " after social consideration" << std::endl;
          }

        } // end of if in range
      } //end of "other agents" loop
      
      if(DEBUG_IG && ownerAgent->getId() == testingId)
      {
        std::cout << "Cell " << elegibles.at(i).first->getId() << " got a total social ig of: " << tot << std::endl;
      }

      allNearAgentsProbabilities[i] = tot;
    } //end of if social


  } //end of elegible cells loop


  bool DEBUG_PROBABILITIES = false;

  //merge probabilities of this agent with the sum of each other
  tot = 0; sum2=0;
  for(unsigned i = 0; i<elegibles.size(); i++)
  {
    if(ownerAgent->GetUseSocialInfo())
    {
      finalProbabilitiesVector.push_back(myProbabilities.at(i)/sum * allNearAgentsProbabilities.at(i));
    }
    else
    {
      finalProbabilitiesVector.push_back(myProbabilities.at(i)/sum);
    }
    sum2+=finalProbabilitiesVector.at(i);


    if((DEBUG_IG || DEBUG_PROBABILITIES) && ownerAgent->getId() == testingId)
    {
      std::cout << "Cell " << elegibles.at(i).first->getId() << " got an ig of: " << finalProbabilitiesVector[i] << " after merging" << std::endl;
    }
  }

  for(unsigned i = 0; i<elegibles.size(); i++)
  {
    finalProbabilitiesVector.at(i) /= sum2;
    if((DEBUG_IG || DEBUG_PROBABILITIES) && ownerAgent->getId() == testingId)
    {
      std::cout << "Cell " << elegibles.at(i).first->getId() << " got a normalized ig of: " << finalProbabilitiesVector[i] << " after merging" << std::endl;
    }
  }
  


  if(DEBUG_PROBABILITIES && ownerAgent->getId() == testingId)
  {
    std::cout << "final probabilities size: " << finalProbabilitiesVector.size() << std::endl;
  }


//#ifndef RELAXED_VERSION     //complete version  
if(ownerAgent->GetTargetSelectionStrategy() == "random")
{
  //extract cell
  float random = RandomGenerator::getInstance().nextFloat(1);
  float cumulative = .0;
  if(DEBUG_PROBABILITIES && ownerAgent->getId() == testingId)
  {
    std::cout << "Using Random strategy with " << random << " as random generated float" << std::endl;
  }
  for(unsigned i=0; i<elegibles.size(); i++)
  {
    cumulative += finalProbabilitiesVector.at(i);
    if(DEBUG_PROBABILITIES && ownerAgent->getId() == testingId)
    {
      std::cout << "Cumulative " << cumulative << " for probability of " << finalProbabilitiesVector.at(i) << std::endl;
    }
    if(random <= cumulative)
    {
              Cell* c = elegibles.at(i).first;
              //c->isTargetOf.push_back(id);
              //ag->cells.at(c->getId())->isTargetOf.push_back(id);
              std::array<unsigned,3> cellPos = c->getPosition();
              return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
    }
  }

}

if(ownerAgent->GetTargetSelectionStrategy() == "softmax")
{
    bool DEBUG_THIS = false;

    if(DEBUG_THIS && ownerAgent->getId() == testingId)
    {
      std::cout << "Using Softmax" << std::endl;
    }

    float expSum = 0;
    for(float i : finalProbabilitiesVector)
    {
    expSum += exp(ownerAgent->GetSoftmaxLambda()*i);
    }
    for(unsigned i =0; i<finalProbabilitiesVector.size(); i++)
    {
      if(DEBUG_THIS && ownerAgent->getId() == testingId)
      {
        std::cout << "Final probability before softmax " << finalProbabilitiesVector[i] << std::endl;
      }
      finalProbabilitiesVector[i]=(exp(ownerAgent->GetSoftmaxLambda()*finalProbabilitiesVector[i]))/expSum;
      if(DEBUG_THIS && ownerAgent->getId() == testingId)
      {
        std::cout << "Final probability after softmax " << finalProbabilitiesVector[i] << std::endl;
      }
    }


  float random = RandomGenerator::getInstance().nextFloat(1);
  float cumulative = .0;
  for(unsigned i=0; i<elegibles.size(); i++)
  {
    cumulative += finalProbabilitiesVector.at(i);
    if(random <= cumulative)
    {
              Cell* c = elegibles.at(i).first;
              //c->isTargetOf.push_back(id);
              //ag->cells.at(c->getId())->isTargetOf.push_back(id);
              std::array<unsigned,3> cellPos = c->getPosition();

              if(ownerAgent->getId() == testingId && DEBUG_THIS)
              {
                std::cout << "Index Selected: " << i << std::endl;
              }

              return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
    }
  }
  

}

if(ownerAgent->GetTargetSelectionStrategy() == "greedy")
{
  bool DEBUG_THIS = false;
  float max = 0;
  int index = -1;


  std::vector<int> repeatedMaxIDs;
  repeatedMaxIDs.reserve(elegibles.size());
  std::vector<float> repeatedMaxProbs;
  repeatedMaxProbs.reserve(elegibles.size());

  for(unsigned i = 0; i<finalProbabilitiesVector.size(); i++){
    if(finalProbabilitiesVector.at(i) == max)
    {
      repeatedMaxIDs.push_back(i);
      repeatedMaxProbs.push_back(finalProbabilitiesVector.at(i)); 
    }
    
    if(finalProbabilitiesVector.at(i) > max )
    {
      repeatedMaxIDs.clear();
      repeatedMaxProbs.clear();
      
      index = i;
      max = finalProbabilitiesVector.at(i);

      repeatedMaxIDs.push_back(index);
      repeatedMaxProbs.push_back(max); 
    }

    if(ownerAgent->getId() == testingId && DEBUG_THIS)
    {
        std::cout << "Probability " << finalProbabilitiesVector.at(i) << " in index " << i << std::endl;
    }
  }
  
  if(ownerAgent->getId() == testingId && DEBUG_THIS)
  {
    std::cout << "Repeated indexes: " << repeatedMaxIDs.size() << std::endl;
  }
  
  int mappedCells = 0;
  if(repeatedMaxIDs.size()==1) //in case all cells are already mapped, still make selection at random
  {
    for(unsigned i = 0; i<elegibles.size(); i++)
    {
      if(elegibles.at(0).first->isMapped())
      {
        mappedCells ++;
      }
    }
  }

  if (mappedCells == finalProbabilitiesVector.size())
  {

    repeatedMaxProbs.clear();
    repeatedMaxIDs.clear();
    for(unsigned i = 0; i<finalProbabilitiesVector.size(); i++)
    {
      //repeatedMaxProbs.push_back(1.0f/finalProbabilitiesVector.size());
      repeatedMaxProbs.push_back(finalProbabilitiesVector[i]);
      repeatedMaxIDs.push_back(i);
      if(ownerAgent->getId() == testingId && DEBUG_THIS)
      {
        std::cout << "All cells were mapped, new probability: " << repeatedMaxProbs[i] << std::endl;
      }
    }    
  }
  

  if(repeatedMaxIDs.size()>=2)
  {
    float maxSum = 0;
    for(unsigned i=0; i<repeatedMaxProbs.size(); i++)
    {
      maxSum += repeatedMaxProbs[i];
      if(ownerAgent->getId() == testingId && DEBUG_THIS)
      {
        std::cout << "Repeated max prob : " << repeatedMaxProbs[i] << std::endl;
      }
    }
    for(unsigned i=0; i<repeatedMaxProbs.size(); i++)
    {
      repeatedMaxProbs[i] /= maxSum;
      
      if(ownerAgent->getId() == testingId && DEBUG_THIS)
      {
        std::cout << "Repeated index prob: " << repeatedMaxProbs[i] << " with id " << repeatedMaxIDs[i] << " max sum " << maxSum << std::endl;
      }
    }

    float random = RandomGenerator::getInstance().nextFloat(1);
    float cumulative = .0;
    if(DEBUG_THIS && ownerAgent->getId() == testingId)
    {
      std::cout << "Using Greedy strategy with " << random << " as random generated float for multiple max probs" << std::endl;
    }
    for(unsigned i=0; i<repeatedMaxProbs.size(); i++)
    {
      cumulative += repeatedMaxProbs.at(i);
      if(random <= cumulative)
      {
                Cell* c = elegibles.at(repeatedMaxIDs[i]).first;
                //c->isTargetOf.push_back(id);
                //ag->cells.at(c->getId())->isTargetOf.push_back(id);
                std::array<unsigned,3> cellPos = c->getPosition();
                return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
      }
    }
  }

  
  if(ownerAgent->getId() == testingId && DEBUG_THIS)
  {
    std::cout << "Index Selected: " << index << std::endl;
  }

  if(index != -1)
  {
    Cell* c = elegibles.at(index).first;
    //c->isTargetOf.push_back(id);
    //ag->cells.at(c->getId())->isTargetOf.push_back(id);
    std::array<unsigned,3> cellPos = c->getPosition();
    return {cellPos.at(0),cellPos.at(1),float(cellPos.at(2))};
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
     return (!(worldCell_REF->isMapped()) && (worldCell_REF->isTargetOf.empty()));
   }
   else
   {
    return (!(ownerAgent->cells.at(c->getId())->isMapped()) && ((ownerAgent->cells.at(c->getId())->isTargetOf.size())==0));
     //return ((!ag->cells.at(c->getId())->isMapped()) && (ag->cells.at(c->getId())->isTargetOf.size())<=0);
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
  if((Engine::getInstance().getWorld()->unCommittedAgents == Engine::getInstance().getWorld()->getAgents().size()-1 
  || ownerAgent->getTargetId() == -2) && isElegible(currentOccupiedCell, ag)){
    ret.push_back(std::make_pair<>(currentOccupiedCell, 0));
    #ifndef PERFECT_COMMUNICATION        
      //updateKB(Engine::getInstance().getWorld()->getCell(agentDiscretePos), ag);
    #endif
      return ret;
  }
/*
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
 */
  std::vector<std::pair<Cell*, float>> ret2;   //elegible cells on the boundary
  std::vector<std::pair<Cell*, float>> ret3;   //mapped but not targeted cells on the boundary
  std::vector<std::pair<Cell*, float>> ret4;   //all boundary cells


  Cell* cella = currentOccupiedCell; 


  std::vector<Cell*> cells_3x3 = cella->get3x3();
  std::vector<Cell*> cells_5x5 = cella->get5x5();

  float distanceToCell = 0;
  for(Cell* c : cells_3x3)
  {
    distanceToCell = sqrt(pow((float) c->getX() - ownerAgent->getX(), 2) + pow((float) c->getY() - ownerAgent->getY(), 2));
    if(isElegible(c, ownerAgent))
      {
        ret2.push_back(std::make_pair<>(c, distanceToCell));
      }
    if(c->isTargetOf.empty())
      {ret4.push_back(std::make_pair<>(c, distanceToCell));}
  }
  for(Cell* c : cells_5x5)
  {
    distanceToCell = sqrt(pow((float) c->getX() - ownerAgent->getX(), 2) + pow((float) c->getY() - ownerAgent->getY(), 2));
    //std::cout << distanceToCell << std::endl;
    if(isElegible(c, ownerAgent))
      {
        ret3.push_back(std::make_pair<>(c, distanceToCell));
      }
    if(c->isTargetOf.empty())
      {ret4.push_back(std::make_pair<>(c, distanceToCell));}
  }
  distanceToCell = sqrt(pow((float) currentOccupiedCell->getX() - ownerAgent->getX(), 2) + pow((float) currentOccupiedCell->getY() - ownerAgent->getY(), 2));
  ret.push_back(std::make_pair<>(currentOccupiedCell, distanceToCell));



  //if there are no eligible cells
    //check for not mapped cells on the boundary
  if(ret2.size()!=0)
  {
    return ret2;   
  }  //else check for mapped but not targeted cells on the boundary
  else if(ret3.size()!=0)
  {
    //Engine::getInstance().getWorld()->getAgent(ag->getId())->sceltaRandom = true;
    ownerAgent->sceltaRandom = true; //this is to collect data in the engine cpp, how many times this was the selection in the simulation
    return ret3;
  }  // if every cells is mapped and already targeted
  else
  {
    //Engine::getInstance().getWorld()->getAgent(ag->getId())->sceltaRandom = true;
    ownerAgent->sceltaRandom = true;
    return ret4;
  }

//#endif
  return ret;
   
}

//compute H(c|o)
float InformationGainStrategy::computeInformationGain(Agent* a, Cell* cell)
{
  bool DEBUG_THIS = false;

  Cell* targetCell = cell;

  float IG_value = 0;
  std::array<float, 13> entropyVector;
  entropyVector.fill(0);

  if(DEBUG_THIS && ownerAgent->getId() == ownerAgent->getTestingId())
  {
    //std::cout << "Entropy vector: ";
  }

  for(unsigned l = 0; l < 13; l++ )
  {
    for(unsigned k = 0; k < 13; k++ )
    {
      if(Engine::getInstance().getWorld()->getSensorTable()[k][l]!=0)
      {
        entropyVector[l] += -Engine::getInstance().getWorld()->getSensorTable()[k][l]*log(Engine::getInstance().getWorld()->getSensorTable()[k][l]);        
        //if(DEBUG_THIS || ownerAgent->getId() == ownerAgent->getTestingId())
        //{
          //std::cout << entropyVector[k] << " ";
        //}
      }
    }
    if(DEBUG_THIS && ownerAgent->getId() == ownerAgent->getTestingId())
    {
      //std::cout << entropyVector[k] << " ";
    }
    //cell->observationVector[k] += -cell->observationVector[k]*log(cell->observationVector[k]);
  }
  
  if(DEBUG_THIS && ownerAgent->getId() == ownerAgent->getTestingId())
  {
    std::cout << std::endl;
  }
  

  //std::cout << "Using dimitri's pseudocode" << std::endl;
  float informationGain = 0;
  float term1 = 0;
  float term2 = 0;
  float term3 = 0;
  for(unsigned c = 0; c < 13; c++)
  {
    if(cell->knowledgeVector[c]>0)
    {term1 = -cell->knowledgeVector[c] * log(cell->knowledgeVector[c]);}
  }
  
  if(DEBUG_THIS && ownerAgent->getId() == ownerAgent->getTestingId())
  {
    std::cout << "Agent " << ownerAgent->getId() << " evaluating IG for cell " << cell->getId() << ". Term 1 = " << term1 << " ";
  }

  for(unsigned c = 0; c < 13; c++)
  {
    term2 += cell->knowledgeVector[c] * entropyVector[c];
  }

  if(DEBUG_THIS && ownerAgent->getId() == ownerAgent->getTestingId())
  {
    std::cout << "Term 2 = " << term2 << " ";
  }

  float term3_int = 0;
  for(unsigned o = 0; o < 13; o++)
  {
    term3_int = 0;
    for(unsigned c = 0; c < 13; c++)
    {
      term3_int += cell->knowledgeVector[c] * Engine::getInstance().getWorld()->getSensorTable()[o][c];
    }
    if(term3_int>0)
    {term3 += term3_int * log(term3_int);}
  }

  IG_value = - term2 - term3;


  if(DEBUG_THIS && ownerAgent->getId() == ownerAgent->getTestingId())
  {
    std::cout << "Term 3 = " << term3 << " ";
    std::cout << "IG = " << IG_value << " ";
    std::cout << std::endl;
  }


  return IG_value;
}

