#include "sim/weed.hpp"
#include "libraries/WMath.h"
#include "sim/engine.hpp"

#include "sim/cell.hpp"

Weed::Weed(float x, float y, float z, float density){

    if(Engine::getInstance().getDisplaySimulation())
    { 
        mesh = new Mesh();
        mesh->SetParent(this);
        mesh->SetPlane();
        mesh->SetCurrentColor(currentColor);

        std::vector<float> lowWeedColor{0.4,0.4,0};
        std::vector<float> HighWeedColor{0.4,0.7,0};
        
        float finalWeedColor_Alpha = MapRangeClamped(density,0,12,0,12);
        std::vector<float> weedColor = VectorLerp(finalWeedColor_Alpha, lowWeedColor, HighWeedColor);

        mesh->SetCurrentColor(glm::vec4(weedColor.at(0),weedColor.at(1),weedColor.at(2),1));
    }

    this->x = x;
    this->y = y;
    this->z = z;
    this->density = density;

    SetPopulation(unsigned (density*13));

    SetWorldLocation(std::vector<float>{x,y,z});
    SetWorldScale(std::vector<float> {0.5f, 0.5f, 0.5f});

}


void Weed::SetCell(Cell* inCell)
{
    cell = inCell;
}

void Weed::SetPopulation(unsigned inPopulation)
{
    population = inPopulation;
}





