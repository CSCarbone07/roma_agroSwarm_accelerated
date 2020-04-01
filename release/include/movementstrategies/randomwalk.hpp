#ifndef RANDOMWALK_HPP
#define RANDOMWALK_HPP

#include <vector>
#include <math.h>
#include "sim/cell.hpp"
#include "eigen3/Eigen/Dense"
class Agent;

class RandomWalkStrategy {
    public:
        /*
        *  Get next target for this WalkStrategy
        *  @param Agent* a, is the reference to Agent a
        *  @return  std::array<float,3> with the real target
        */     
        std::array<float,3> pickNextTarget(Agent* ag);
        
        RandomWalkStrategy(Agent* ag);


    private:

        Agent* ownerAgent;  //Agent using this strategy class
        float max_range;
        float min_range;
        /**
         * Compute the value of the cauchy PDF at a given x, with a given mean (location parameter) and cauchy variance
         * for 0 <= x <= 2*pi, 0 < c < 1.
         * see: https://en.wikipedia.org/wiki/Cauchy_distribution for more details
         */
        double cauchyPDF(float x, float c){
            return (1-pow(c,2)) / (2*M_PI*(1+pow(c,2)-2*c*cos(x)));
        }

        /*
        * Get cells that can be choosen by the agent
        * @param Agent* a, is the reference to Agent a
        * @param std::array<unsigned,3> agentDiscretePos, that represent the agentDiscretepos
        * @return std::vector<Cell*> with all Cell that can be choosen
        */
        std::vector<std::pair<Cell*, float>> getElegibles( Agent* ag, std::array<unsigned,3> agentDiscretePos, unsigned id);
        

        /*
        *   Get the direction factor of the Cell
        *  @param std::array<unsigned,3> agentPosition, represent the discreteposition of the robot
        *  @param Cell* c, is the reference to cell c
        *  @param Eigen::Vector2f directionVector, represent the direction vector
        *  @return float 
        */
        float computeDirectionFactor(std::array<float,3> agentPosition, Cell* c, Eigen::Vector2f directionVector);

        /*
        *  Check if a Cell is Elegible or not
        *  @param Cell* c, is the reference to the Cell
        *  @param Agent* ag, is the reference to the Agent
        *  @return bool
        */
        bool isElegible(Cell* c, Agent* ag); // check if a node is elegibles or not

        Eigen::Vector2f computeMomentum(float theta); // compute momentum
        Eigen::Vector2f computeRepulsion(Agent* ag, std::array<float,3> agentPos); // compute repulsion
        Eigen::Vector2f computeAttraction(Agent* ag, std::array<float,3> agentPos); // compute attraction
};

#endif
