#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "graphics/VertexShader.h"
#include "agent/agent.hpp"
#include "sim/world.hpp"
#include "sim/steppable.hpp"
#include "sim/weed.hpp"
#include "util/randomgenerator.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <yaml-cpp/yaml.h>
#include <fstream>


/**
 * Singleton
 *
 * The engine is responsible for:
 *   - storing/initializing/running the world
 *   - storing/initializing/running the agents
 *
 * @author Dario Albani
 * @email albani@diag.uniroma1.it
 */

#define PERFECT_COMMUNICATION      //according to the same define in informationGain.cpp or randomwalk.cpp

const GLint WIDTH = 800, HEIGHT = 600;
static GLuint VAO, VBO, shader;

static const char* vShader = "version 330";
/*
    static const char* vShader = "                \n\
    #version 330                                  \n\
    layout (location = 0) in vec3 pos;            \n\
    
    gl_Position = vec4(pos.x, pos.y, pos.z, 1.0);   \n\
                                                   \n\
                                                   \n\
   ";
*/
class Engine {

public:
  /* Singleton specific method */
  static Engine& getInstance() {
    static Engine instance;
    return instance;
  }
  unsigned rand = 0;

private:
  /* Simulation Settings */
  YAML::Node config; /** < configuration file as a map */
  bool displaySimulation = false;
  unsigned maxSteps; /** < maximum number of steps for the simulation */
  unsigned numOfAgents; /** < total number of agents in the simulation */
  std::array<unsigned, 3> size; /** < world size */

  float communicationsRange;

  std::map<unsigned, Weed*> weeds; /*mat that 

  /* Simulation Variables */
  World* world; /** < pointer to the world */
  std::vector<Agent*> agents; /** < pointers to the agents, this is not accessible from here  */

  //Orca Variables
  float orcaRadius;
  float tau;
  bool collisionAvoidance;

  // Visit Variables

  float repulsion;
  float attraction;

  bool stop = false;


  /* output file */
  std::ofstream knowledgeBasesFile;
  std::ofstream movesFile;
  std::ofstream statusFile;
  std::ofstream randomChoice;
  std::ofstream timing;
  std::ofstream visitedCells;

  /*Strategies features*/
  float knowledgeClusterRadius = 0;
  std::string inspectionStrategy;
  std::string targetSelectionStrategy;
  float softmaxLambda = 1;
  bool useSocialInfo = false;
  bool useDistanceForIG = false;


  Engine() {}


public:
  Engine(Engine const&) = delete;
  void operator=(Engine const&) = delete;

  float gaussianPDF(float x, float mean, float variance){
    return exp(-(9*(x - mean)*(x - mean))/(2*(variance*variance)));
  }
  /* Getters */
  inline unsigned getMaxSteps() const {
    return maxSteps;
  }


  inline float getCommunicationsRange() const {
    return communicationsRange;
  } 

  inline float getKnowledgeClusterRadius() const {
    return knowledgeClusterRadius;
  } 
  
  inline std::string getInspectionStrategy() const {
    return inspectionStrategy;
  }

  inline std::string getTargetSelectionStrategy() const {
    return targetSelectionStrategy;
  }

  inline float getSoftmaxLambda() const {
    return softmaxLambda;
  }

  inline bool getUseSocialInfo() const {
    return useSocialInfo;
  }

  inline bool getUseDistanceForIG() const {
    return useDistanceForIG;
  }
  
  inline unsigned getRepulsion() const {
    return repulsion;
  }

  inline unsigned getAttraction() const {
    return attraction;
  }

  inline unsigned getNumOfAgents() const {
    return numOfAgents;
  }

  inline World* getWorld() const {
    return world;
  }

  bool moveAgentTo(float x, float y, float z, unsigned id){
    return world->moveAgentTo(x,y,z,id);
  }

  inline Agent* getAgent(unsigned id) const{
    return this->world->getAgent(id);
  }
  inline float getOrcaRadius(){return this->orcaRadius;}
  inline float getTau(){return this->tau;}
  inline bool getCollisionAvoidance(){return this->collisionAvoidance;}

  inline void stopSimulation(){this->stop = true;}
  /**
   * Initialize the simulation.
   *
   * @param inputFile the yaml input file with all the simulation settings
   * @param movesFile (optional) the yaml output file where simulation results are stored
   */

  void CompileShaders()
  {
    shader = glCreateProgram(); 

    if(!shader)
    {
        printf("Error creating shader program! \n");
        return;

    }  

    AddShader(shader, vShader, GL_VERTEX_SHADER);

    GLint result = 0;
    GLchar eLog[1024] = {0};

    glLinkProgram(shader);
    glGetProgramiv(shader, GL_LINK_STATUS, &result);
    if(!result)
    {
        glGetProgramInfoLog(shader, sizeof(eLog), NULL, eLog);
        printf("Error linking program: '%s'\n", eLog);
        return;
    }

    glValidateProgram(shader);
    glGetProgramiv(shader, GL_VALIDATE_STATUS, &result);
    if(!result)
    {
        glGetProgramInfoLog(shader, sizeof(eLog), NULL, eLog);
        printf("Error validating program: '%s'\n", eLog);
        return;
    }


  }

  void AddShader(GLuint theProgram, const char* shaderCode, GLenum shaderType)
  {
    GLuint theShader = glCreateShader(shaderType);
    
    const GLchar* theCode[1];
    theCode[0] = shaderCode;
   
    GLint codeLenght[1];
    codeLenght[0] = strlen(shaderCode);
    
    glShaderSource(theShader, 1, theCode, codeLenght);
    glCompileShader(theShader);

    GLint result = 0;
    GLchar eLog[1024] = {0};

    glGetProgramiv(theShader, GL_COMPILE_STATUS, &result);
    if(!result)
    {
        glGetProgramInfoLog(theShader, sizeof(eLog), NULL, eLog);
        printf("Error compiling the %d shader: '%s'\n", shaderType, eLog);
        return;
    }

    glAttachShader(theProgram, theShader);
    

  }

  void CreateTriangle()
    {
        GLfloat vertices[] = {
            -1.0f, -1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            0.0f, 1.0f, 0.0f
        };

        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);

        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindVertexArray(0);


    }


  void init(YAML::Node config) {

    displaySimulation = config["Display_Simulation"].as<bool>();


    // Initilize agents and world
    maxSteps = config["max_steps"].as<unsigned>();
    numOfAgents = config["num_of_agents"].as<unsigned>();
    communicationsRange = config["communications_range"].as<float>();
    size = {config["world"]["x"].as<unsigned>(), config["world"]["y"].as<unsigned>(), config["world"]["z"].as<unsigned>()};
    //knowledgeBasesFile.open(config["knowledgeBasesFile"].as<std::string>());
    movesFile.open(config["movesFile"].as<std::string>());
    statusFile.open(config["statusFile"].as<std::string>(), std::ios_base::app);
    randomChoice.open(config["randomChoice"].as<std::string>(), std::ios_base::app);
    timing.open(config["timing"].as<std::string>(), std::ios_base::app);
    visitedCells.open(config["visitedCells"].as<std::string>(), std::ios_base::app);



    float linearVelocity =  config["linearVelocity"].as<float>();
    
    // Orca Variables
    tau = config["tau"].as<float>();
	  collisionAvoidance = config["collisionAvoidance"].as<bool>();
    orcaRadius = config["orcaRadius"].as<float>();

    //Weed Variables
    unsigned clusters = config["weeds"]["clusters"].as<unsigned>();
    unsigned maxweeds = config["weeds"]["maxweeds"].as<unsigned>();
    unsigned isolated = config["weeds"]["isolated"].as<unsigned>();

    //Visit Varibles

    repulsion = config["repulsion"].as<float>();
    attraction = config["attraction"].as<float>();

    knowledgeClusterRadius = config["KnowledgeClusterRadius"].as<float>();

    //Inspection Strategies
    inspectionStrategy = config["InspectionStrategy"].as<std::string>();
    if(inspectionStrategy != "ig")
    {inspectionStrategy = "rw"; }
    std::cout << "Inspection Strategy: " << inspectionStrategy << std::endl;

    if(inspectionStrategy == "ig")
    {
        targetSelectionStrategy = config["TargetSelectionStrategy"].as<std::string>();
        if(targetSelectionStrategy != "greedy" && targetSelectionStrategy != "softmax")
        {
        targetSelectionStrategy = "random";
        }
        if(targetSelectionStrategy == "softmax")
        {
        softmaxLambda = config["Softmax_Lambda"].as<float>();
        std::cout << "Lambda for softmax on target selection strategy: " << softmaxLambda << std::endl; 
        }
        std::cout << "Target selection strategy: " << targetSelectionStrategy << std::endl;
        useSocialInfo = config["UseSocialInfo"].as<bool>();
        std::cout << "Include social info: " << useSocialInfo << std::endl;
        useDistanceForIG = config["UseDistanceForIG"].as<bool>();
        std::cout << "Include distance for IG weights: " << useDistanceForIG << std::endl;        

    }

    std::cout << std::endl;    

    //print info
    std::cout << "World size: " << size[0] << "x" << size[1] << "x" << size[2] << std::endl;
    std::cout << "Num Of Agents: " << numOfAgents << std::endl;
    std::cout << "Communications Range";
    if(communicationsRange == -1)
    {
        std::cout << "Communications Range: " << "Unlimited";
    }
    else
    {
        std::cout << "Communications Range: " << communicationsRange;
    }  
    std::cout << std::endl << std::endl;

    std::cout << "Orca Variables:" << std::endl;
    std::cout << "---Tau:" << tau << std::endl;
    std::cout << "---collisionAvoidance enabled:" << collisionAvoidance << std::endl;
    std::cout << "---Agent radius:" << (orcaRadius/2)-0.05f << std::endl;
    std::cout << "---Orca radius:" << orcaRadius << std::endl;
    std::cout << "---Linear Velocity:" << linearVelocity << std::endl << std::endl;

    std::cout << "---Repulsion:" << repulsion << std::endl;
    std::cout << "---Attraction:" << attraction << std::endl << std::endl;

    // initialize the world
    world = new World(size);


    // tree->setParameters(config["delta"].as<float>(),config["xi"].as<float>());
    
    //---------- populate world
    std::cout << "populate world..." << std::endl;

    world->populateAndInitialize(clusters, maxweeds, isolated);


    // initiailize the agents
    agents.reserve(numOfAgents);
    unsigned x,y,z;
    for(unsigned id = 0; id < numOfAgents; ++id) {
      x = RandomGenerator::getInstance().nextInt(size.at(0));
      y = RandomGenerator::getInstance().nextInt(size.at(1));
      z = size.at(2)-1;//zPos.at(RandomGenerator::getInstance().nextInt(0,2));
      while(!world->isPositionFree(x,y,z)){
        x = RandomGenerator::getInstance().nextInt(size.at(0));
        y = RandomGenerator::getInstance().nextInt(size.at(1));
        //z = zPos.at(RandomGenerator::getInstance().nextInt(0,2));
      }
      Agent* agent = new Agent(id, x+0.5, y+0.5, z); // MARK --> change this according to your needs
      if(this->world->addAgent(agent, x+0.5, y+0.5, z)){
        printf("Agent %i in position %f %f %f \n", agent->getId(), agent->getX(), agent->getY(), agent->getZ());
        agent->setVelocity(linearVelocity);
        agent->setAgentRadius(((orcaRadius/2)-0.05f));
        agents.push_back(agent);
      }
    }
  }

  /**
   * Run the simulation.
   */
  void run() {
  

    // Initialize GLFW
    if(!glfwInit())
    {
        printf("GLFW init failed!");
        glfwTerminate();
        return;
    }
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    // Core profile = no backwards compability
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Allow forward compatiblity
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow *mainWindow = glfwCreateWindow(WIDTH, HEIGHT, "Swarm Simulator", NULL, NULL);
    if(!mainWindow)
    {
        printf("GLFW window creation failed!");
        glfwTerminate();
        return;
    }

    int bufferWidth, bufferHeight;
    glfwGetFramebufferSize(mainWindow, &bufferWidth, &bufferHeight);

    // Set context for GLEW to use
    glfwMakeContextCurrent(mainWindow);
    

    // Allow modern extension features
    glewExperimental = GL_TRUE;    

    if(glewInit() != GLEW_OK)
    {
        printf("GLEW initialisation failed!");
        glfwDestroyWindow(mainWindow);
        glfwTerminate();
        return;

    }

    // Setup viewport size
    glViewport(0, 0, bufferWidth, bufferHeight);
    
    
    /*
    while(!glfwWindowShouldClose())
    {
    }
    */
    
    CreateTriangle();
    CompileShaders();


    //knowledgeBasesFile << "Agents Knowledge Base:" << std::endl;
       
    #ifdef PERFECT_COMMUNICATION


    #endif    

 
    std::stringstream ss;
    this->world->getPopulationInfo(ss);
    movesFile << "population:\n"<< ss.rdbuf();
    unsigned timeStep = 0;

    // support vector for shuffled agents
    std::vector<Steppable*> shuffled(agents.begin(), agents.end());
    movesFile << "steps:\n";
    movesFile << ' ' << timeStep << ':' << '\n';
    //add first position
    for(auto a : agents) {
      std::stringstream ss;
      if (a->getInfo(ss))
        movesFile << ' ' << ' ' << ss.rdbuf();
    }
     ++timeStep;
    // do one step until max steps or for infinite time (need to terminate manually)
    unsigned timeToCoverage = 10000;
    unsigned timeToMapping = 10000;
    unsigned timeToMappingOnlyClusters = 10000;
    bool isCovered = false;
    bool isMapped = false;
    bool isMappedOnlyClusters = false;

    while(timeStep < maxSteps || maxSteps == 0) {
    std::cout << "Time step: " << timeStep << std::endl;


    // Get + handle user input events
    glfwPollEvents();

    // clear window
    glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shader);

    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);


    glUseProgram(0);

    glfwSwapBuffers(mainWindow);

    
    


      if(!isCovered && this->world->isCovered()){
        timeToCoverage = timeStep;
        isCovered = true;
      }
      if(!isMapped && this->world->isMapped()){
        timeToMapping = timeStep;
        isMapped = true;
      }
      if(!isMappedOnlyClusters && this->world->isMappedOnlyClusters()){
        timeToMappingOnlyClusters = timeStep;
        isMappedOnlyClusters = true;                
      }
      if((this->world->isCovered() && this->world->isMapped()) || stop){// || this->world->getTree()->getRoot()->getUtility() == 0.f){
        statusFile << ' '<< timeToCoverage <<' '<<timeToMapping<<' '<<timeToMappingOnlyClusters<<"\n";
        std::cout << "\n\n### world mapped ---- at Step:" << timeStep<<std::endl;
        break;
      }
      // limit the cout
      if(timeStep % 1000 == 0)
        std::cout << "#### time_step: " << timeStep << std::endl;
      
      // shuffle the agents at every iteration
      std::shuffle(shuffled.begin(), shuffled.end(), RandomGenerator::getInstance().getGenerator());
      movesFile << ' ' << timeStep << ':' << '\n';
      // do step for each agent
      for(auto a : shuffled) {
        a->doStep(timeStep);
        std::stringstream ss;
        if (a->getInfo(ss))
          movesFile << ' ' << ' ' << ss.rdbuf();
      }

      randomChoice << ' '<< timeStep <<':'<<' ';
      timing << ' '<< timeStep <<':'<<' ';
      //mappingTime << ' '<< timeStep <<':'<<' ';

      unsigned rand2 = 0;
      for(auto a : agents) {
        if(a->sceltaRandom == true){
          rand++;
          rand2++;
          a->sceltaRandom = false;
        }
      }
      randomChoice << rand<<' '<<rand2<<"\n";
      timing << this->world->remainingTasksToVisit.size()<<' '<<this->world->remainingTasksToMap.size()<<' '<<this->world->remainingTasksIntoClusters.size()<<"\n";
      
      ++timeStep;
    }
    
    std::cout<<"celle visitate  =  "<<this->world->getCells().size()-this->world->remainingTasksToVisit.size()<<std::endl;
    if(timeStep == maxSteps){
      statusFile << ' '<< timeToCoverage <<' '<<timeToMapping<<' '<<timeToMappingOnlyClusters<<"\n";
      for(std::map<unsigned, Cell*>::iterator it=this->world->remainingTasksToVisit.begin(); it!=this->world->remainingTasksToVisit.end(); ++it)
        std::cout<<"la cella  "<<it->second->getId()<<"  non è stata visitata"<<std::endl;
      for(std::map<unsigned, Cell*>::iterator it=this->world->remainingTasksToMap.begin(); it!=this->world->remainingTasksToMap.end(); ++it)
        std::cout<<"la cella  "<<it->second->getId()<<"  non è stata mappata"<<std::endl;
    }
    for(Cell * cc : this->world->getCells()){
      std::cout<<"la cella  "<<cc->getId()<<"  con palline  "<<cc->getUtility()<<"  è stata visitata  "<<cc->numOfVisits<<"  volte!"<<std::endl;
      visitedCells<<' '<<cc->getId()<<':'<<' '<<cc->numOfVisits<<"\n";
    }
    unsigned tot_collisions = 0;
    for(Agent* a : agents){
      tot_collisions += a->getCollisions();
      std::cout << "Agent " << a->getId() << " : " << a->getCollisions()  << std::endl;
    } 
    std::cout << "Collisions " << tot_collisions  << std::endl;
  }
};

#endif /* ENGINE_HPP*/
