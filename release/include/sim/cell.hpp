#pragma once

#ifndef CELL_HPP
#define CELL_HPP

#include "sim/WObject.h"
#include "graphics/Mesh.h"
#include "sim/weed.hpp"
#include <vector>
#include <array>
#include <map>

class Cell : public WObject {

private:
  unsigned id; /** < Id associated to this cell (must be unique) */

  unsigned x; /** < x coordinate of the bottom left corner */
  unsigned y; /** < y coordinate of the bottom left corner */
  unsigned z; /** < z coordinate of the bottom left corner */

  unsigned size; /** < size of the cell */
  float utility; /** < utility associated to this cell */

  std::vector<Cell*> cells;                   //all cells in world
  std::array<unsigned, 3> worldCellSize;      //world size in amount of cells
  std::vector<Cell*> cells_3x3;
  std::vector<Cell*> cells_5x5;

  unsigned numAgents; /** < number of agents committed to this cell */
  bool mapped = false;
  float beacon = 0;  
  Weed* weed;

  Mesh* mesh = nullptr; 
  glm::vec4 nonInspectedColor = glm::vec4(0.309f, 0.176f, 0.152f, 1.0f);
  glm::vec4 inspectedColor = glm::vec4(0.725f, 0.564f, 0.533f, 1.0f);
  


public:
  float residual_uncertainty = 1;
  unsigned lastTimeVisit = 0;
  unsigned numOfVisits = 0;
  std::vector<unsigned> isTargetOf;       /** < store the id of agent with this target (size should be always one) */

  std::array<float, 13> knowledgeVector;  /** Knowledge vector for each cell, where each entry corresponds to
                                          the probablty that the cell has the corresponding number of balls**/
  std::array<float, 13> observationVector;  /** p(o i,j ) Is the marginal probability of having a given 
                                            observation given the current knowledge about the state of cell c i,j*/
  
  std::map<float, std::array<float,13>> knowledgeVectors;
  std::map<float, std::array<float,13>> observationVectors;

  Cell(unsigned id, unsigned x, unsigned y, unsigned z, unsigned size, bool mapped, float utility=0);
  

  inline unsigned getId() const { return id;}
  inline unsigned getX() const { return x; }
  inline unsigned getY() const { return y; }
  inline unsigned getZ() const { return z; }
  inline std::array<unsigned,3> getPosition() const{ 
    return {x,y,z}; 
  }

  inline unsigned getVisits() const { return numOfVisits; }
  
  inline unsigned getSize() const { return size; }
  inline float getUtility() const { return utility; }
  inline float getResidual() const {return residual_uncertainty;}
  inline bool isMapped() const { return mapped; }
  inline float getBeacon() const { return beacon; }
  inline void setBeacon(float beacon) { this->beacon = beacon; }

  inline std::vector<Cell*> get3x3() { return cells_3x3; }
  inline std::vector<Cell*> get5x5() { return cells_5x5; }

  inline void setUtility(float utility){this->utility = utility;}
  inline void setResidual(float residual_uncertainty){this->residual_uncertainty = residual_uncertainty;}

  inline void addWeed(Weed* weed){this->weed = weed;}

  void SetNeighbors(std::vector<Cell*> inCells);

  void setMapped();

};


#endif /* CELL_HPP */
