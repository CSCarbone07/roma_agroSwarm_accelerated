#pragma once

#ifndef W_OBJECT_H
#define W_OBJECT_H


#include <vector>
#include <algorithm>

struct Transform
{
std::vector<float> Location = {0, 0, 0};
std::vector<float> Rotation = {0, 0, 0};
std::vector<float> Scale = {1, 1, 1};
};


class WObject
{

private:
    WObject* parent = nullptr;

    Transform wTransform;   //world transform
    Transform rTransform;   //relative transform

    std::vector<WObject*> childs;

public:
    WObject();

    std::vector<float> GetWorldLocation();
    std::vector<float> GetWorldRotation();
    std::vector<float> GetWorldScale();

    WObject* GetParent();

    void SetWorldLocation(std::vector<float> inLocation = {0, 0, 0});
    void SetWorldRotation(std::vector<float> inRotation = {0, 0, 0});
    void SetWorldScale(std::vector<float> inScale = {1, 1, 1});

    void SetParent(WObject* inParent = nullptr);
    void AddChild(WObject* inWObject = nullptr);

};

#endif
