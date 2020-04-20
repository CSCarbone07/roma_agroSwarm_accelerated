#ifndef WEED_HPP
#define WEED_HPP

#include "sim/WObject.h"
#include "graphics/Mesh.h"

class Weed: public WObject {

private:
    Mesh* mesh;
    glm::vec4 currentColor = glm::vec4(0.0f, 0.5f, 0.0f, 1.0f);

public:
    float x;
    float y;
    float density;

    Weed(float x, float y, float density);

};

#endif