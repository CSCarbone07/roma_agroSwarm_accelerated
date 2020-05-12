#pragma once

#ifndef WEED_HPP
#define WEED_HPP

#include "sim/WObject.h"
#include "graphics/Mesh.h"
//#include "sim/cell.hpp"

class Cell;



class Weed: public WObject {

private:

    Cell* cell;

public:
    float x;
    float y;
    float z;
    float density;
    unsigned population;

    Weed(float x, float y, float z, float density);

    Mesh* mesh;
    glm::vec4 currentColor = glm::vec4(0.0f, 0.5f, 0.0f, 1.0f);

    void SetCell(Cell* inCell);

    void SetPopulation(unsigned inPopulation); //population = utility

};

#endif