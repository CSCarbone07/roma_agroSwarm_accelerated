#include "sim/cell.hpp"
//#include "sim/WObject.h"
//#include "graphics/Mesh.h"
#include "sim/engine.hpp"


Cell::Cell(unsigned id, unsigned x, unsigned y, unsigned z, unsigned size, bool mapped, float utility){

    if(Engine::getInstance().getDisplaySimulation())
    { 
        mesh = new Mesh();
        mesh->SetParent(this);
        mesh->SetPlane();
        mesh->SetCurrentColor(nonInspectedColor);
    }
       
    this->id = id;
    this->x = x; 
    this->y = y; 
    this->z = z; 
    this->size = size;
    this->utility = utility;
    this->knowledgeVector.fill(1.0/13.0);
    this->observationVector.fill(0.0);
    this->mapped = mapped;
    
    SetWorldLocation(std::vector<float>{x,y,-0.5f});
    SetWorldScale(std::vector<float> {0.9f, 0.9f, 0.9f});


}


    
void Cell::setMapped()
{
    mapped = true; 
    if(mesh != nullptr)
    {mesh->SetCurrentColor(inspectedColor);}
}


