#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>

#include "graphics/Window.h"
#include "graphics/Shader.h"
#include "graphics/Mesh.h"

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


  Window mainWindow;
  glm::mat4 projection;
  std::vector<Mesh*> meshList;
  std::vector<Shader*> shaderList;
  Shader* shader1;
  GLuint uniformProjection = 0, uniformModel = 0;
  

  // Vertex Shader
  const char* vShader = "../include/graphics/Shaders/shader.vert";
  //static const char* vShader = "version 330";
  // Fragment Shader
  const char* fShader = "../include/graphics/Shaders/shader.frag";

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



  void init(YAML::Node config);

  void CreateShaders();
  void CreateObjects();

  void run();

};

#endif /* ENGINE_HPP*/
