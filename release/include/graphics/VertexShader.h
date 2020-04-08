#ifndef VERTEX_SHADER_H
#define VERTEX_SHADER_H

#include<string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

class VertexShader
{

private:

    //gl_Position = vec4(pos.x, pos.y, pos.z, 1.0);

public:

    VertexShader();
    //~VertexShader();

    //CompileShaders();

    void CreateTriangle();
    //static const char* vShader = "version 330";
    //static const char* fShader = "version 330";
/*
    static const char* vShader = "                \n\
    #version 330                                  \n\
    layout (location = 0) in vec3 pos;            \n\
    
    gl_Position = vec4(pos.x, pos.y, pos.z, 1.0);   \n\
                                                   \n\
                                                   \n\
   ";

   */
};    








#endif
