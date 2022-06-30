#pragma once

#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "graphics/Window.h"
#include "graphics/Camera.h"
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

struct Character {
    GLuint TextureID;   // ID handle of the glyph texture
    glm::ivec2 Size;    // Size of glyph
    glm::ivec2 Bearing;  // Offset from baseline to left/top of glyph
    GLuint Advance;    // Horizontal offset to advance to next glyph
};

class Engine {

public:
  /* Singleton specific method */
  static Engine& getInstance() {
    static Engine instance;
    return instance;
  }
  unsigned rand = 0;

private:
  void RenderScene();

  FT_Library ft;
  FT_Face face;

  /* Simulation Settings */
  YAML::Node config; /** < configuration file as a map */

  unsigned seed = 0;
  unsigned timeStep = 0;


  bool displaySimulation = false;
  unsigned maxSteps; /** < maximum number of steps for the simulation */
  unsigned numOfAgents; /** < total number of agents in the simulation */
  std::array<unsigned, 3> size; /** < world size */

  float communicationsRange;

  std::map<unsigned, Weed*> weeds; /*mat that 

  /* Simulation Variables */
  World* world = nullptr; /** < pointer to the world */
  std::vector<Agent*> agents; /** < pointers to the agents, this is not accessible from here  */

  float limitForTargetReselection = 50;

  //Orca Variables
  float orcaRadius;
  float tau;
  bool collisionAvoidance;

  // Visit Variables

  float repulsion;
  float attraction;

  bool stop = false;

  bool finishVisit = false;
  bool finishMapping = false;
  bool finishCluster = false;

  int nextRemainingVisitsStack = 500;
  int lastRemainingVisits = 0;
  int lastRemainingMapping = 0;
  int lastRemainingClusters = 0;

  GLfloat deltaTime = 0.0f;
  GLfloat lastTime = 0.0f;

  Window mainWindow;
  Camera camera;
  float FOV = 45.0f;
  glm::mat4 projection;
  std::vector<Mesh*> meshList;
  std::vector<Shader*> shaderList;
  Shader* shader1;
  Shader* shader2;
  GLuint uniformProjection = 0, uniformView = 0, uniformModel = 0, uniformInColor = 0;
  
  std::map<GLchar, Character> Characters;
  //GLuint VAO, VBO;

  // Vertex Shader
  const char* vShader = "../include/graphics/Shaders/shader.vert";
  //static const char* vShader = "version 330";
  // Fragment Shader
  const char* fShader = "../include/graphics/Shaders/shader.frag";

  const char* vTextShader = "../include/graphics/Shaders/text.vert";
  const char* fTextShader = "../include/graphics/Shaders/text.frag";


  /* output file */
  std::ofstream SensorErrorsFile;
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
  bool useCommsForSocialIG = false;

  /* Test variables */
  bool test_IG = false;
  float test_IG_value=0;
  int test_maxWeedsPerCell = 12;
  unsigned test_weeds_seen=0;
  //std::vector<std::vector<float>> test_sensorTable; 
  Cell* test_cell;
  std::array<std::array<float,13>,13> test_sensorTable; 
  std::array<float, 13> test_knowledgeVector;
  std::array<float, 13> test_observationVector;
  /*
  std::map<float, std::array<float,13>> test_knowledgeVectors;
  std::map<float, std::array<float,13>> test_observationVectors;
  */

  Engine() {}


public:
  Engine(Engine const&) = delete;
  void operator=(Engine const&) = delete;

  float gaussianPDF(float x, float mean, float variance){ //gaussian probability density function
    return exp(-(9*(x - mean)*(x - mean))/(2*(variance*variance)));
  }
  /* Getters */
  inline unsigned getMaxSteps() const {
    return maxSteps;
  }

  inline bool getDisplaySimulation() const {
    return displaySimulation;
  }

  inline float getCommunicationsRange() const {
    return communicationsRange;
  } 

inline float getlimitForTargetReselection() const {
    return limitForTargetReselection;
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
  
  inline bool getUseCommsForSocialIG() const {
    return useCommsForSocialIG;
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


  void WriteKnowledgeBasesFile(std::string inString);

  void init(YAML::Node config);

  void CreateShaders();
  void AddMesh(Mesh* inMesh = nullptr);
  void RenderText(std::string text, GLfloat x, GLfloat y, GLfloat scale, glm::vec3 color);

  bool doSensorError = false;
  void ResetWorld();
  void run();

  void MeanSquareError_World(std::vector<Cell*> cells); 
  void MeanSquareError_duringSimulation(); 

  std::vector<Cell*> GetAgentCells_LowestUncertainty();
  std::vector<Cell*> cells_lowestUncertainty;


  int errorRuns = 100;
  int currentErrorRun = 1;

  int errorMaxScansPerCell = 5;
  int errorScansPerCell = 1;

  double current_world_MSE_lastSeen = 0;
  double current_world_MSE_greedy = 0;
  double current_world_MSE_random = 0;

  double accumulated_world_MSE_lastSeen = 0;
  double accumulated_world_MSE_greedy = 0;
  double accumulated_world_MSE_random = 0;

  void save_visitedCells();

  void TestFunction_IG();
  void TestFunction_computeIG(bool printTable);
  void TestFunction_SetSensorTable(bool printTable);

  double TestFunction_logChoose(int n, int k);
  double TestFunction_PMFBinomial(double p, int n, int k) ;

  void TestFunction_Scan(bool printThis);

};

#endif /* ENGINE_HPP*/
