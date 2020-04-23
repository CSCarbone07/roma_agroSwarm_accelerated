#include "sim/weed.hpp"
#include "sim/engine.hpp"


Weed::Weed(float x, float y, float density){

    if(Engine::getInstance().getDisplaySimulation())
    { 
        mesh = new Mesh();
        mesh->SetParent(this);
        mesh->SetPlane();
        mesh->SetCurrentColor(currentColor);
    }

    this->x = x;
    this->y = y;
    this->density = density;

    SetWorldLocation(std::vector<float>{x,y,-0.25f});
    SetWorldScale(std::vector<float> {0.5f, 0.5f, 0.5f});


}


    

