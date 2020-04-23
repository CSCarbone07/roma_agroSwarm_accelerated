#include "sim/cell.hpp"
//#include "sim/WObject.h"
//#include "graphics/Mesh.h"
#include "sim/world.hpp"
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

    //worldCellSize.at(0) = Engine::getInstance().getWorld()->getSize().at(0);
    //worldCellSize.at(1) = Engine::getInstance().getWorld()->getSize().at(1);
    
    //std::cout<< "Cell " << this->getId() << " in position " << x << " x " << y << " y " << z << " z" << std::endl;


    SetWorldLocation(std::vector<float>{x,y,-0.5f});
    SetWorldScale(std::vector<float> {0.9f, 0.9f, 0.9f});

    

}


    
void Cell::setMapped()
{
    mapped = true; 
    if(mesh != nullptr)
    {mesh->SetCurrentColor(inspectedColor);}
}

void Cell::SetNeighbors(std::vector<Cell*> inCells)
{
    
    cells = inCells;
    float loopCellSize = (float) size;
    for(float i = -1*loopCellSize; (i <= 1*loopCellSize); i=i+loopCellSize)
    {
        for(float j = -1*loopCellSize; (j <= 1*loopCellSize); j=j+loopCellSize)
        {   
            if(!(i == 0 && j == 0) && (x+i) >= 0 && (y+j) >= 0)
            {
                for(Cell* c : cells)
                {
                    if (c->getX() == x + i && c->getY()== y + j && c->getZ()== z)
                    {
                    cells_3x3.push_back(c);
                    //std::cout << "Cell " << id << " added cell " << c->getId() << " as 3x3 neighbor" << std::endl; 
                    break;
                    }
                } 
            }        
        }
    }
    for(float i = -2*loopCellSize; (i <= 2*loopCellSize); i=i+loopCellSize)
    {
        for(float j = -2*loopCellSize; (j <= 2*loopCellSize); j=j+loopCellSize)
        {   
            if(!(i == 0 && j == 0) && (x+i) >= 0 && (y+j) >= 0)
            {
                for(Cell* c : cells)
                {
                    if (c->getX() == x + i && c->getY()== y + j && c->getZ()== z)
                    {
                    if (std::find(cells_3x3.begin(), cells_3x3.end(), c) != cells_3x3.end())
                        {break;}
                    cells_5x5.push_back(c);
                    //std::cout << "Cell " << id << " added cell " << c->getId() << " as 5x5 neighbor" << std::endl; 
                    break;
                    }
                } 
            }        
        }
    }
}


