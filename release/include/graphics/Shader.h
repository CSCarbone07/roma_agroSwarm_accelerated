#ifndef VERTEX_SHADER_H
#define VERTEX_SHADER_H


#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

class Shader
{
public:
	Shader(GLuint inShaderID);
	~Shader();
    void ClearShader();

    void CreateFromString(const char* vertexCode, const char* fragmentCode);
	void CreateFromFiles(const char* vertexLocation, const char* fragmentLocation);

	std::string ReadFile(const char* fileLocation);
    GLuint GetShaderID();

	GLuint GetProjectionLocation();
	GLuint GetViewLocation();
	GLuint GetModelLocation();
	GLuint GetXmoveLocation();
    GLuint GetInColor();

	void UseShader();




private:
	GLuint shaderID, uniformProjection, uniformView, uniformModel, uniformInColor, uniformXmove;

	void CompileShader(const char* vertexCode, const char* fragmentCode);
	void AddShader(GLuint theProgram, const char* shaderCode, GLenum shaderType);
};









#endif
