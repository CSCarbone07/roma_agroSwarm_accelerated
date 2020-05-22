#ifndef INFORMATIONGAIN_HPP
#define INFORMATIONGAIN_HPP

#include <vector>
#include <math.h>
#include "agent/agent.hpp"
#include "eigen3/Eigen/Dense"
class Agent;

class InformationGainStrategy {
    public:
        /*
        *  Get next target for this WalkStrategy
        *  @param Agent* a, is the reference to Agent a
        *  @return  std::array<float,3> with the real target
        */
        std::array<float,3> pickNextTarget(Agent* a);

        InformationGainStrategy(Agent* ag);

    private:
        int testingId = 1;

        Agent* ownerAgent;            
 
        //float max_range;
        //float min_range;

        float max_range_5x5 = 3*sqrt(2);
        float min_range_5x5 = 2*sqrt(2);
        float max_range_3x3 = 2*sqrt(2);
        float min_range_3x3 = sqrt(2);


        /*
        * Get cells that can be choosen by the agent
        * @param Agent* a, is the reference to Agent a
        * @param std::array<unsigned,3> agentDiscretePos, that represent the agentDiscretepos
        * @return std::vector<Cell*> with all Cell that can be choosen
        */
        std::vector<std::pair<Cell*, float>> getElegibles( Agent* a, std::array<unsigned,3> agentDiscretePos);
        
        
        float computeInformationGain(Agent* a, Cell* cell);
        
        /*
        *  Check if a Cell is Elegible or not
        *  @param Cell* c, is the reference to the Cell
        *  @param Agent* ag, is the reference to the Agent
        *  @return bool
        */
        bool isElegible(Cell* c, Agent* ag); // check if a cell is elegibles or not


};


#endif
