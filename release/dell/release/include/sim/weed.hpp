#ifndef WEED_HPP
#define WEED_HPP

class Weed{

public:
    float x;
    float y;
    float density;

    Weed(float x, float y, float density){
        this->x = x;
        this->y = y;
        this->density = density;
    }
};

#endif