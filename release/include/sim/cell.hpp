#pragma once

#ifndef CELL_HPP
#define CELL_HPP

#include "sim/WObject.h"
#include "graphics/Mesh.h"
#include "sim/weed.hpp"
#include <vector>
#include <array>
#include <map>


class Agent;

class Cell : public WObject {

private:
  unsigned id; /** < Id associated to this cell (must be unique) */

  float x; /** < x coordinate of the bottom left corner */
  float y; /** < y coordinate of the bottom left corner */
  float z; /** < z coordinate of the bottom left corner */

  float size; /** < size of the cell */
  float utility; /** < utility associated to this cell */

  std::vector<Cell*> cells;                   //all cells in world
  std::array<unsigned, 3> worldCellSize;      //world size in amount of cells
  std::vector<Cell*> cells_3x3;
  std::vector<Cell*> cells_5x5;
  std::vector<Cell*> cells_7x7;
  std::vector<Cell*> cells_9x9;

  unsigned numAgents; /** < number of agents committed to this cell */
  bool mapped = false;
  float beacon = 0;  
  Weed* weed;

  Agent* ownerAgent = nullptr;  

  float timerToForgetTarget = 0;
  float timerToForgetBeacon = 0;


  int lastWeedsSeen = -1;

  Mesh* mesh = nullptr; 
  glm::vec4 nonInspectedColor = glm::vec4(0.309f, 0.176f, 0.152f, 1.0f);
  glm::vec4 inspectedColor = glm::vec4(0.725f, 0.564f, 0.533f, 1.0f);
  glm::vec4 beaconColor = glm::vec4(0.525f, 0.364f, 0.333f, 1.0f);
  
  int testingId = -1;//1275;
  glm::vec4 testColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);


public:

  float residual_uncertainty = 1;
  int firstTimeVisit = -1;
  unsigned lastTimeVisit = 0;
  unsigned numOfVisits = 0;
  std::vector<unsigned> isTargetOf;       /** < store the id of agent with this target (size should be always one) */

  std::array<float, 13> knowledgeVector;  /** Knowledge vector for each cell, where each entry corresponds to
                                          the probablty that the cell has the corresponding number of balls**/
  std::array<float, 14> observationVector;  /** p(o i,j ) Is the marginal probability of having a given 
                                            observation given the current knowledge about the state of cell c i,j*/
  
  std::map<float, std::array<float,13>> knowledgeVectors;
  std::map<float, std::array<float,13>> observationVectors;

  Cell(int id, float x, float y, float z, float size, bool mapped, float utility=0);
  

  inline unsigned getId() const { return id;}
  inline float getX() const { return x; }
  inline float getY() const { return y; }
  inline float getZ() const { return z; }
  inline std::array<unsigned,3> getPosition() const{ 
    return {x,y,z}; 
  }

  void restartTimer(float maxTime);
  void restartBeaconTimer(float maxTime);
  void decreaseTimer(float time);
  void forgetTargetOf();
  void forgetBeacon();

  inline unsigned getVisits() const { return numOfVisits; }
  
  inline float getSize() const { return size; }
  inline float getUtility() const { return utility; }
  inline float getResidual() const {return residual_uncertainty;}
  inline bool isMapped() const { return mapped; }
  inline float getBeacon() const { return beacon; }
  inline int getLastWeedsSeen() const {return lastWeedsSeen; }


  void setBeacon(float beacon);
  void setLastWeedsSeen(int weeds);


  inline std::vector<Cell*> get3x3() { return cells_3x3; }
  inline std::vector<Cell*> get5x5() { return cells_5x5; }
  inline std::vector<Cell*> get7x7() { return cells_7x7; }
  inline std::vector<Cell*> get9x9() { return cells_9x9; }

  void setUtility(float inUtility);

  inline void setResidual(float residual_uncertainty){this->residual_uncertainty = residual_uncertainty;}

  void addWeed(Weed* inWeed);

  void SetNeighbors(std::vector<Cell*> inCells);

  void setOwnerAgent(Agent* ag);

  void setMapped();
  void resetCell();

  void ChangeColor(glm::vec4 inColor);

  void ResetColor();

};




#endif /* CELL_HPP */
