#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "sim/WObject.h"
#include <map>


class Mesh : public WObject
{

private:
  glm::vec4 currentColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);

  FT_Library ft;
  FT_Face face;

  //std::map<GLchar, Character> Characters;


public:
	Mesh();

	void CreateMesh(GLfloat *vertices, unsigned int *indices, unsigned int numOfVertices, unsigned int numOfIndices);
	void RenderMesh();
  void RenderText(std::string text, GLfloat x, GLfloat y, GLfloat scale, glm::vec3 color);
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

