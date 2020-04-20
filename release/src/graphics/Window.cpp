#include "graphics/Window.h"

Window::Window()
{
	width = 800;
	height = 600;
    xChange = 0.0f;
    yChange = 0.0f;

    for(size_t i = 0; i < 1024; i++)
    {
        keys[i] = 0;
    }
}

Window::Window(GLint windowWidth, GLint windowHeight)
{
	width = windowWidth;
	height = windowHeight;
    xChange = 0.0f;
    yChange = 0.0f;

    for(size_t i = 0; i < 1024; i++)
    {
        keys[i] = 0;
    }

}

int Window::Initialise()
{
    // Initialize GLFW
	if (!glfwInit())
	{
		printf("Error Initialising GLFW");
		glfwTerminate();
		return 1;
	}

	// Setup GLFW Windows Properties
	// OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// Core Profile
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	// Allow forward compatiblity
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	// Create the window
	mainWindow = glfwCreateWindow(width, height, "Test Window", NULL, NULL);
	if (!mainWindow)
	{
		printf("Error creating GLFW window!");
		glfwTerminate();
		return 1;
	}

	// Get buffer size information
	glfwGetFramebufferSize(mainWindow, &bufferWidth, &bufferHeight);

	// Set the current context
	glfwMakeContextCurrent(mainWindow);

    // Hnalde Key + Mouse Input
    createCallbacks();
    //glfwSetInputMode(mainWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED); //Use to lock mouse invisibly in the created window

	// Allow modern extension access
	glewExperimental = GL_TRUE;

	GLenum error = glewInit();
	if (error != GLEW_OK)
	{
		printf("Error: %s", glewGetErrorString(error));
		glfwDestroyWindow(mainWindow);
		glfwTerminate();
		return 1;
	}

	glEnable(GL_DEPTH_TEST);

	// Create Viewport
	glViewport(0, 0, bufferWidth, bufferHeight);

    glfwSetWindowUserPointer(mainWindow, this);


}

Window::~Window()
{
	glfwDestroyWindow(mainWindow);
	glfwTerminate();
}

void Window::createCallbacks()
{
    glfwSetKeyCallback(mainWindow, handleKeys);
    glfwSetCursorPosCallback(mainWindow, handleMouse);


}

GLfloat Window::getXChange()
{
    GLfloat theChange = xChange;
    xChange=0.0f;
    return theChange;

}

GLfloat Window::getYChange()
{
    GLfloat theChange = yChange;
    yChange=0.0f;
    return theChange;

}

void Window::handleKeys(GLFWwindow* window, int key, int code, int action, int mode)
{
    Window* theWindow = static_cast<Window*>(glfwGetWindowUserPointer(window));
    
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        //glfwSetWindowShouldClose(window, GL_TRUE);    //for future if we want to close window with scape key

    }

    if(key >= 0 && key < 1024)
    {
        if(action == GLFW_PRESS)
        {
            theWindow->keys[key] = true;
            //printf("Pressed: %d\n", key);
        }
        else if (action == GLFW_RELEASE)
        {
            theWindow->keys[key] = false;
            //printf("Released: %d\n", key);
        }
    }


}

void Window::handleMouse(GLFWwindow* window, double xPos, double yPos)    
{
    /*
    Window* theWindow = static_cast<Window*>(glfwGetWindowUserPointer(window));
    
    if(theWindow->mouseFirstMoved)
    {
        theWindow->lastX = xPos;
        theWindow->lastY = yPos;
        theWindow->mouseFirstMoved = false;
    }

    theWindow->xChange = xPos - theWindow->lastX;
    theWindow->yChange = theWindow->lastY - yPos;
    
    theWindow->lastX = xPos;
    theWindow->lastY = yPos;
  
    //printf("x:%.6f, y:%.6f\n", theWindow->xChange, theWindow->yChange);

    */
}












