#include "sim/cell.hpp"
//#include "sim/WObject.h"
//#include "graphics/Mesh.h"
#include "agent/agent.hpp" 
#include "sim/world.hpp"
#include "sim/engine.hpp"


Cell::Cell(int id, float x, float y, float z, float size, bool mapped, float utility){
      
    this->id = id;
    this->x = x; 
    this->y = y; 
    this->z = z; 
    this->size = size;
    this->utility = utility;
    this->knowledgeVector.fill(1.0/13.0);
    this->observationVector.fill(0.0);
    this->mapped = mapped;
    this->ownerAgent = nullptr;

    //std::cout << "receiving ID " << id << std::endl;

    if(testingId == id)
    {std::cout << "testing cell: " << x << "x +" << y << "y" << std::endl;}

    if(Engine::getInstance().getDisplaySimulation())
    {      
        mesh = new Mesh();
        mesh->SetParent(this);
        mesh->SetPlane();
        ChangeColor(nonInspectedColor);
    }

    //worldCellSize.at(0) = Engine::getInstance().getWorld()->getSize().at(0);
    //worldCellSize.at(1) = Engine::getInstance().getWorld()->getSize().at(1);
    
    //std::cout<< "Cell " << this->getId() << " spawned in position " << x << " x " << y << " y " << z << " z" << std::endl;


    SetWorldLocation(std::vector<float>{x,y,-0.5f});
    SetWorldScale(std::vector<float> {0.9f, 0.9f, 0.9f});


}

void Cell::addWeed(Weed* inWeed)
{
    this->weed = inWeed;
    if(weed!=nullptr)
    {weed->SetCell(this);}
}

void Cell::setUtility(float inUtility)
{
    this->utility = inUtility;
    //std::cout << "Cell " << getId() << " has now a utility of: " << utility << std::endl;
    /*
    if(utility>0)
    {
        ChangeColor(testColor);
    }
    */
}

void Cell::setOwnerAgent(Agent* ag)
{
    this->ownerAgent = ag;
}

void Cell::restartTimer(float maxTime)
{
    //std::cout << "Restarting time by " << maxTime << " for cell " << id << std::endl;

    timerToForgetTarget = maxTime;
}

void Cell::restartBeaconTimer(float maxTime)
{
    //std::cout << "Restarting time by " << maxTime << " for cell " << id << std::endl;

    timerToForgetBeacon = maxTime;
}

void Cell::decreaseTimer(float time)
{         
    if(timerToForgetTarget>0)
    {
        //std::cout << "Decreasing timer " << timerToForgetTarget << " by " << time << " for cell " << id << std::endl;

        timerToForgetTarget-=time;

        //std::cout << "Remaining timer " << timerToForgetTarget << " reduced by " << time << " for cell " << id << std::endl;

        if(timerToForgetTarget <= 0)
        {
            //std::cout << "Timer's up, forgeting cell " << id << std::endl;
            forgetTargetOf();
        }
    }

    if(timerToForgetBeacon>0)
    {
        timerToForgetBeacon -=time;
        if(timerToForgetBeacon <= 0)
        {
            forgetBeacon();
        }
    }
}

void Cell::forgetBeacon()
{
    timerToForgetBeacon = 0;
    ownerAgent->removeBeacon(this);
}

void Cell::forgetTargetOf()
{
    //std::cout << "Forgeting targets of " << id << std::endl;
    timerToForgetTarget = 0;
    isTargetOf.clear();
}

void Cell::setBeacon(float beacon) 
{
    this->beacon = beacon;    
    if(beacon = 0)
    {
        timerToForgetBeacon=0;
    } 
    if(mesh!=nullptr && beacon>0)
    {ChangeColor(beaconColor);}

}

void Cell::setLastWeedsSeen(int weeds)
{
    lastWeedsSeen = weeds;
}

void Cell::setMapped()
{
    mapped = true; 
    if(mesh!=nullptr)
    {ChangeColor(inspectedColor);}
}

void Cell::resetCell()
{
    mapped = false; 
    knowledgeVector.fill(1.0/13.0);
    observationVector.fill(0.0);
    lastWeedsSeen=-1;
    if(mesh!=nullptr)
    {ChangeColor(nonInspectedColor);}
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
    
    if(false)
    {
        std::cout << "5x5 has " << cells_5x5.size() << " cells" << std::endl; 
    }
    /*
    for(float i = -3*loopCellSize; (i <= 3*loopCellSize); i=i+loopCellSize)
    {
        for(float j = -3*loopCellSize; (j <= 3*loopCellSize); j=j+loopCellSize)
        {   
            if(!(i == 0 && j == 0) && (x+i) >= 0 && (y+j) >= 0)
            {
                for(Cell* c : cells)
                {
                    if (c->getX() == x + i && c->getY()== y + j && c->getZ()== z)
                    {
                    if (std::find(cells_3x3.begin(), cells_3x3.end(), c) != cells_3x3.end())
                        {break;}
                    if (std::find(cells_5x5.begin(), cells_5x5.end(), c) != cells_5x5.end())
                        {break;}
                    cells_7x7.push_back(c);
                    //std::cout << "Cell " << id << " added cell " << c->getId() << " as 5x5 neighbor" << std::endl; 
                    break;
                    }
                } 
            }        
        }
    }
    
    if(false)
    {
        std::cout << "7x7 has " << cells_7x7.size() << " cells" << std::endl; 
    }
    
    for(float i = -4*loopCellSize; (i <= 4*loopCellSize); i=i+loopCellSize)
    {
        for(float j = -4*loopCellSize; (j <= 4*loopCellSize); j=j+loopCellSize)
        {   
            if(!(i == 0 && j == 0) && (x+i) >= 0 && (y+j) >= 0)
            {
                for(Cell* c : cells)
                {
                    if (c->getX() == x + i && c->getY()== y + j && c->getZ()== z)
                    {
                    if (std::find(cells_3x3.begin(), cells_3x3.end(), c) != cells_3x3.end())
                        {break;}
                    if (std::find(cells_5x5.begin(), cells_5x5.end(), c) != cells_5x5.end())
                        {break;}
                    if (std::find(cells_7x7.begin(), cells_7x7.end(), c) != cells_7x7.end())
                        {break;}
                    cells_9x9.push_back(c);
                    //std::cout << "Cell " << id << " added cell " << c->getId() << " as 5x5 neighbor" << std::endl; 
                    break;
                    }
                } 
            }        
        }
    }
    if(false)
    {
        std::cout << "9x9 has " << cells_9x9.size() << " cells" << std::endl; 
    }
    */
}

void Cell::ChangeColor(glm::vec4 inColor)
{   
    int ownId = (int)(this->getId());
    if(mesh!=nullptr)
    {
      if(ownId != testingId)
      {
        mesh->SetCurrentColor(inColor);
      }
      else
      {
        mesh->SetCurrentColor(testColor);
      }
    }

}

void Cell::ResetColor()
{
    if(mapped == true)
    {
    ChangeColor(inspectedColor);
    }
    else
    {
        if(beacon>0)
        {
            ChangeColor(beaconColor);
        }
        else
        {
            ChangeColor(nonInspectedColor);
        }
        
        
    }
    
}


