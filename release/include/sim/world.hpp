#ifndef WORLD_HPP
#define WORLD_HPP

#include "sim/cell.hpp"
#include "agent/agent.hpp"
#include <vector>
#include <array>
#include <map>
#include <math.h>
#include <assert.h>
#include "sim/weed.hpp"

class World {

private:
  std::array<unsigned, 3> size; /** < the world is a line, square, cube */
  std::vector<Cell*> cells; /** < in my world there are cells */
  std::map<unsigned, Weed*> weeds; /** < map structure for weeds in the world **/

  std::vector<Agent*> agents; /** < list of all agents in the world */
  std::map<unsigned, unsigned> grid; /** < map structure matching cells to the number of agents **/
    
  unsigned posToCellId(unsigned x, unsigned y, unsigned z) const; /** < Convert Cell Position to cell Id */
  std::array<unsigned, 3> idToCellPos(unsigned id) const; /** < Convert Cell id to Position */
  unsigned int maxWeed4Cell = 12; /** <A cell can contain max 12 weeds> */

  std::array<std::array<float,13>,13> sensorTable;   /** < The table represents the probability of
                                                              having the observation o given that the true value is c. */
  
public:
  std::map<unsigned, Cell*> remainingTasksToVisit; /** < all cells in the world */
  std::map<unsigned, Cell*> remainingTasksToMap; /** < all cells with weeds in the world */
  std::map<unsigned, Cell*> remainingTasksIntoClusters; /** < all cells that belongs to clusters in the world */

  unsigned communication_range = 50;   /** <Communication range limit, in non-ideal case> */


  std::map<unsigned, Cell*> beacons; /** < in my world there are beacons */

  std::map<float, std::vector<std::pair<int, int>>> distanceVectors;   /**precompute distance */


  std::vector<Agent*> committedAgents; /** < list of all agents in the world */
  unsigned unCommittedAgents = 0; 

World(std::array<unsigned,3> size);
~World();

// Getters

inline std::array<std::array<float,13>,13> getSensorTable(){return this->sensorTable;}

inline std::array<unsigned,3> getSize() const{ return this->size;} /** < Get the size of the world, return array<unsigned,3> */

inline std::vector<Cell*> getCells() const{return this->cells;}

inline  bool getCell(Cell& cell, unsigned id) const { /** < Get Cell, return array<unsigned,3> */
  for (Cell* c : cells) {
    if (c->getId() == id) {
      cell = *c;
      return true;
    }
  }
  return false;
}
inline  Cell* getCell(unsigned x, unsigned y, unsigned z) const {
    for (Cell* c : cells) {
      if (c->getX() == x && c->getY()==y && c->getZ()==z){
        return c;
      }
    }
    return nullptr;
}
/*
inline  Cell* getCell(float x, float y, float z) const {

    for (Cell* c : cells) {
      if (c->getX() == x && c->getY()==y && c->getZ()==z){
        return c;
      }
    }
    return nullptr;
}
*/
inline  Cell* getCell(std::array<unsigned,3> agentDiscretePos) const {
    for (Cell* c : cells) {
      if (c->getX() == agentDiscretePos.at(0) && c->getY()==agentDiscretePos.at(1) && c->getZ()==agentDiscretePos.at(2)){
        return c;
      }
    }
    return nullptr;
}

inline bool getCell(Cell& cell, unsigned x, unsigned y, unsigned z) const {
  if(x < this->size.at(0) && y < this->size.at(1) && z < this->size.at(2)){
    return getCell(cell, posToCellId(x,y,z));
  }
  return false;
}

inline std::array<unsigned, 3> getCellPosition(unsigned id){
  return idToCellPos(id);
}
inline unsigned getCellId(unsigned x, unsigned y, unsigned z){
  return posToCellId(x,y,z);
}
inline unsigned getCellId(std::array<unsigned,3> agentDiscretePos){
  return posToCellId(agentDiscretePos.at(0),agentDiscretePos.at(1),agentDiscretePos.at(2));
}
std::vector<Agent*> getAgents(unsigned cellId) const{
  assert(cellId > (this->size.at(0)*this->size.at(1)*this->size.at(2)));
  std::vector<Agent*> toReturn;
  std::array<unsigned, 3> cellPos = idToCellPos(cellId);
  for (Agent* a : this->agents){
    if(floor(a->getX()) == cellPos.at(0) &&
        floor(a->getY()) == cellPos.at(1) &&
        floor(a->getZ()) == cellPos.at(2))
      toReturn.push_back(a);
  }
  assert(toReturn.size() == this->grid.at(cellId));
  return toReturn;
}
inline std::vector<Agent*> getAgents(unsigned x, unsigned y, unsigned z) const{
  return getAgents(posToCellId(x,y,z));
}
inline std::vector<Agent*> getAgents(){return this->agents;}

inline Agent* getAgent(unsigned id) const{
  return this->agents[id];
}
inline unsigned getAgentsIn(unsigned cellId) const{ return this->grid.at(cellId);}

inline unsigned getAgentsIn(unsigned x, unsigned y, unsigned z) const {return getAgentsIn(posToCellId(x,y,z));}
/* get info */
bool getPopulationInfo(std::stringstream& ss);

// Adders
bool addAgent(Agent* agent, unsigned x, unsigned y, unsigned z);

bool addCell(Cell* cell, unsigned x, unsigned y, unsigned z);

bool moveAgentTo(float x, float y, float z, unsigned agentId);

bool populateAndInitialize(unsigned clusters, unsigned maxweeds, unsigned isolated);

inline bool addWeed(Weed* weed){
  unsigned id = getCellId(weed->x,weed->y, 0);
  if(weeds.find(id) != weeds.end()){
    return false;
  }
  this->cells.at(id)->addWeed(weed);
  this->weeds[id] = weed;
  return true;
}

inline bool isPositionFree(unsigned x, unsigned y, unsigned z){
  if(grid[posToCellId(x,y,z)] != 0){
    return false;
  }
  return true;
}

inline bool isMapped(){
  return this->remainingTasksToMap.size()==0;
}

inline bool isCovered(){
  return this->remainingTasksToVisit.size()==0;
}

inline bool isMappedOnlyClusters(){
  return this->remainingTasksIntoClusters.size()==0;
}

inline bool isInWorld(unsigned x, unsigned y, unsigned z){
  if(x < this->size.at(0) && y < this->size.at(1) && z < this->size.at(2))
    return true;
  return false;
}
};
#endif /* WORLD_HPP */
