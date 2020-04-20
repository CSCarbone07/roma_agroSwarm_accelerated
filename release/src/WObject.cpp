#include "sim/WObject.h"


WObject::WObject(){

    SetWorldLocation(std::vector<float> {0,0,0});
    SetWorldRotation(std::vector<float> {0,0,0});
    SetWorldScale(std::vector<float> {1,1,1});
}

std::vector<float> WObject::GetWorldLocation(){return wTransform.Location;}
std::vector<float> WObject::GetWorldRotation(){return wTransform.Rotation;}
std::vector<float> WObject::GetWorldScale(){return wTransform.Scale;}

WObject* WObject::GetParent(){return parent;}

void WObject::SetWorldLocation(std::vector<float> inLocation)
{

    for(WObject* c : childs)
    {
        std::vector<float> newLocation {0,0,0};
        for(int i = 0; i < 3; i++)
        {newLocation[i] = c->GetWorldLocation()[i] + inLocation[i] - GetWorldLocation()[i];}
        c->SetWorldLocation(newLocation);
    }

    wTransform.Location = inLocation;
}
void WObject::SetWorldRotation(std::vector<float> inRotation){wTransform.Rotation = inRotation;}
void WObject::SetWorldScale(std::vector<float> inScale)
{
    for(WObject* c : childs)
    {
        std::vector<float> newScale {0,0,0};
        for(int i = 0; i < 3; i++)
        {newScale[i] = c->GetWorldScale()[i] * (inScale[i]);}
        c->SetWorldScale(newScale);
    }
wTransform.Scale = inScale;
}



void WObject::SetParent(WObject* inParent)
{
if(inParent != nullptr)// && GetParent != inParent->GetParent())
    {
    parent = inParent;
    parent->AddChild(this);
    }
}
void WObject::AddChild(WObject* inWObject)
{
if(inWObject != nullptr)
{
if(!(std::find(childs.begin(), childs.begin(), inWObject) != childs.end()))
    childs.push_back(inWObject);
}
}


