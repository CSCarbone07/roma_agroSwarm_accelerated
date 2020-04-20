#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "sim/WObject.h"



class Mesh : public WObject
{

private:
  glm::vec4 currentColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);

public:
	Mesh();

	void CreateMesh(GLfloat *vertices, unsigned int *indices, unsigned int numOfVertices, unsigned int numOfIndices);
	void RenderMesh();
	void ClearMesh();

    void SetPlane();
    void SetInvertedPyramid();


    inline glm::vec4 getCurrentColor(){return currentColor;}
    void SetCurrentColor(glm::vec4 inColor = glm::vec4 (1.0f, 1.0f, 1.0f, 1.0f));

	~Mesh();

private:
	GLuint VAO, VBO, IBO;
	GLsizei indexCount;
};

