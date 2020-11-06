#include "sim/engine.hpp"


const GLint WIDTH = 800, HEIGHT = 600;
static GLuint VAO, VBO, IBO, shader, uniformXMove;

static bool direction = true;
static float triOffset = 0.0f;
static float triMaxoffset = 0.7f;
static float triIncrement = 0.005f;
static float tR = 3.14159265f / 180.0f; //conversion to radians
static float currentAngle = 0.0f;
static float currentScale = 0.4f;
static bool scaleDirection = true;
static float maxScale = 0.8f;
static float minScale = 0.1f;

//static const char* vShader = std::ifstream in("FileReadExample.cpp");
//std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
//static const char* vShader = "version 330";
/*
    static const char* vShader = "                                 \n\
    #version 330                                                   \n\
    layout (location = 0) in vec3 pos;                             \n\
    void main()                                                    \n\
    {                                                              \n\
    gl_Position = vec4(0.4 * pos.x, 0.4 * pos.y, pos.z, 1.0);      \n\
    }                                                              \n\
                                                                   \n\
   ";
*/

 void Engine::init(YAML::Node config) {

    // Initilize agents and world
    maxSteps = config["max_steps"].as<unsigned>();
    numOfAgents = config["num_of_agents"].as<unsigned>();
    communicationsRange = config["communications_range"].as<float>();
    size = {config["world"]["x"].as<unsigned>(), config["world"]["y"].as<unsigned>(), config["world"]["z"].as<unsigned>()};
    SensorErrorsFile.open(config["SensorErrorsFile"].as<std::string>(), std::ios_base::app);
    knowledgeBasesFile.open(config["knowledgeBasesFile"].as<std::string>(), std::ios_base::app);
    movesFile.open(config["movesFile"].as<std::string>(), std::ios_base::app);
    statusFile.open(config["statusFile"].as<std::string>(), std::ios_base::app);
    randomChoice.open(config["randomChoice"].as<std::string>(), std::ios_base::app);
    timing.open(config["timing"].as<std::string>(), std::ios_base::app);
    visitedCells.open(config["visitedCells"].as<std::string>(), std::ios_base::app);


    float linearVelocity =  config["linearVelocity"].as<float>();
    
    limitForTargetReselection = config["limitForTargetReselection"].as<float>();

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

        useCommsForSocialIG = config["UseCommRangeForSocial"].as<bool>();
        std::cout << "Use communication range as range limit for social info inclusion: " << useCommsForSocialIG << std::endl;  

    }

    std::cout << std::endl; 

    // Display simulation
    displaySimulation = config["Display_Simulation"].as<bool>();
    
    if(displaySimulation)
    {
    mainWindow = Window(1080, 1080);
	  mainWindow.Initialise();
    CreateShaders();

	  camera = Camera(glm::vec3(size.at(0)/2, size.at(1)/2, 75.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f, 20.0f, 0.5f);

	  projection = glm::perspective(glm::radians(FOV), (GLfloat)mainWindow.getBufferWidth() / mainWindow.getBufferHeight(), 0.1f, 1000.0f);
    
    //CreateObjects();
    //CompileShaders();


    
    if (FT_Init_FreeType(&ft))
    std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;

    if (FT_New_Face(ft, "../fonts/Arial/Arial.ttf", 0, &face))
      std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;  
    
    FT_Set_Pixel_Sizes(face, 0, 48); 

    if (FT_Load_Char(face, 'X', FT_LOAD_RENDER))
      std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;  
        
    // Disable byte-alignment restriction
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1); 

    for (GLubyte c = 0; c < 128; c++)
    {
      // Load character glyph 
      if (FT_Load_Char(face, c, FT_LOAD_RENDER))
      {
        std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
        continue;
      }
      // Generate texture
      GLuint texture;
      glGenTextures(1, &texture);
      glBindTexture(GL_TEXTURE_2D, texture);
      glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RED,
        face->glyph->bitmap.width,
        face->glyph->bitmap.rows,
        0,
        GL_RED,
        GL_UNSIGNED_BYTE,
        face->glyph->bitmap.buffer
      );
      // Set texture options
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      // Now store character for later use
      Character character = {
        texture, 
        glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
        glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
        face->glyph->advance.x
      };
      Characters.insert(std::pair<GLchar, Character>(c, character));
    }
    
    glBindTexture(GL_TEXTURE_2D, 0);
    // Destroy FreeType once we're finished
    FT_Done_Face(face);
    FT_Done_FreeType(ft);


        // Configure VAO/VBO for texture quads
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
      
    }
    // end of display init

    // testing variables
    //test_IG = config["Test_IG"].as<bool>();
    //test_maxWeedsPerCell = config["Test_Max_Weeds_Per_Cell"].as<unsigned>();


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
      Agent* agent = new Agent(id, x, y, z); // MARK --> change this according to your needs
      if(this->world->addAgent(agent, x, y, z)){
        printf("Agent %i in position %f %f %f \n", agent->getId(), agent->getX(), agent->getY(), agent->getZ());
        agent->setVelocity(linearVelocity);
        agent->setAgentRadius(((orcaRadius/2)-0.05f));
        agents.push_back(agent);
      }
    }
   
    //knowledgeBasesFile << "Knowledge Base (registration of each scan)" << std::endl;
    //knowledgeBasesFile << "cell_ID / agent_ID / timestep / current observation / knowledge vector (13) / observation vector (13)" << std::endl << std::endl;

  }



/**
* Run the simulation.
*/
void Engine::run() {
          
    #ifdef PERFECT_COMMUNICATION

    #endif    

    std::stringstream ss;
    std::cout << "running sim" << std::endl;
    this->world->getPopulationInfo(ss);
    //std::cout << "test" << std::endl;
    movesFile << "population:\n" << ss.rdbuf();
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

    bool weTesting = false;
    if(test_IG)
    {weTesting = true;}

    if(weTesting)
    {
      TestFunction_IG();

    }


    //---------------------SENSOR ERROR AREA-----------------------------------//
    if(doSensorError)
    {
      std::cout << "Calculating base errors" << std::endl;
      //SensorErrorsFile << "testing input" << '\n';
      //SensorErrorsFile << "testing input" << std::endl;
      //SensorErrorsFile << "testing input" << std::endl;

      float errorAltitude = 0.5;
      Agent* errorAgent = new Agent(1000, 0, 0, errorAltitude); // dummy agent, needs to be destroy later
      errorAgent->SetCommunicationsRange(-1);

      for (unsigned s = 1; s <= errorMaxScansPerCell; s++)
      {
        errorScansPerCell = s;
       
        for (unsigned r = 1; r <= errorRuns; r++)
        {
        currentErrorRun = r;

          for(Cell* c : getWorld()->getCells())
          {
            errorAgent->setPosition(c->getPosition().at(0),c->getPosition().at(1),errorAltitude);
            for (unsigned sc = 1; sc <= errorScansPerCell; sc++)
            {
              errorAgent->scanCurrentLocation(c, -1); 
            }
          }

          MeanSquareError_World(getWorld()->getCells());

          if(currentErrorRun == 1)
          {
            accumulated_world_MSE_lastSeen = current_world_MSE_lastSeen;
            accumulated_world_MSE_greedy = current_world_MSE_greedy;
            accumulated_world_MSE_random = current_world_MSE_random;
          }
          else
          {
            //adding using the moving average
            accumulated_world_MSE_lastSeen = accumulated_world_MSE_lastSeen + (current_world_MSE_lastSeen-accumulated_world_MSE_lastSeen)/currentErrorRun;
            accumulated_world_MSE_greedy = accumulated_world_MSE_greedy + (current_world_MSE_greedy-accumulated_world_MSE_greedy)/currentErrorRun;
            accumulated_world_MSE_random = accumulated_world_MSE_random + (current_world_MSE_random-accumulated_world_MSE_random)/currentErrorRun;
          }

        ResetWorld();
        }

        // this will insert into the file the error for 1-5 scans for the field with 100 scans per cell
        // 1 = each cell with 1 scan over 100 variations, 2 = each cell with 2 scans over 100 variations
        // this covers only the first step
        // 
        //SensorErrorsFile << "first input " << s << " ";
        SensorErrorsFile << accumulated_world_MSE_lastSeen << ' ';
        SensorErrorsFile << accumulated_world_MSE_greedy << ' ';
        SensorErrorsFile << accumulated_world_MSE_random << ' ';
        SensorErrorsFile << std::endl;
  
        accumulated_world_MSE_lastSeen = 0;
        accumulated_world_MSE_greedy = 0;
        accumulated_world_MSE_random = 0;
 
      }


      
      // reset world before starting simulation
      ResetWorld();
      errorAgent->setPosition(-100,-100,errorAltitude);
      //errorAgent->SetWorldLocation(-100,-100,errorAltitude); //need to move triangle mesh
      //delete errorAgent;

      std::cout << "Base errors calculated" << std::endl;
    }
    
    //---------------------SIMULATION WHILE LOOP------------------------------//
    while((timeStep < maxSteps || maxSteps == 0) && !weTesting) {
      if(false)
      {std::cout << "Time step: " << timeStep << std::endl;}

      if(displaySimulation)
      {
      GLfloat now = glfwGetTime(); // SDL_GetPerformanceCounter();
      deltaTime = now - lastTime; // (now - lastTime)*1000/SDL_GetPerformanceFrequency();
      lastTime = now;
      RenderScene();
      }

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
        std::cout << "\n\n### world finished ---- at Step:" << timeStep<<std::endl;
        break;
      }

      // limit the cout
      if(timeStep % 500 == 0)
      {
        std::cout << "---- time_step: " << timeStep << std::endl;
        MeanSquareError_duringSimulation();
      }  

      if(this->world->remainingTasksToVisit.size() == 0 && finishVisit == false)
      {
        finishVisit = true;
        std::cout << "XXXX visits finished: " << timeStep << std::endl;
      }

      if(this->world->remainingTasksToMap.size() == 0 && finishMapping == false)
      {
        finishMapping = true;
        std::cout << "XXXX Mapping finished: " << timeStep << std::endl;
      }

      if(this->world->remainingTasksIntoClusters.size() == 0 && finishCluster == false)
      {
        finishCluster = true;
        std::cout << "XXXX Clusters finished: " << timeStep << std::endl;
      }


      if(2500 - this->world->remainingTasksToVisit.size() >= nextRemainingVisitsStack)// && this->world->remainingTasksToVisit.size() != lastRemainingVisits)
      {
        std::cout << "#### remaining visits: " << this->world->remainingTasksToVisit.size() << std::endl;
        //lastRemainingVisits = this->world->remainingTasksToVisit.size();
        nextRemainingVisitsStack += 500;
      }  

      if(this->world->remainingTasksToMap.size() % 50 == 0 && this->world->remainingTasksToMap.size() != lastRemainingMapping)
      {
        std::cout << "#### remaining mapping: " << this->world->remainingTasksToMap.size() << std::endl;
        lastRemainingMapping = this->world->remainingTasksToMap.size();
      }

      if(this->world->remainingTasksIntoClusters.size() % 50 == 0 && this->world->remainingTasksIntoClusters.size() != lastRemainingClusters)
      {
        std::cout << "#### remaining clusters: " << this->world->remainingTasksIntoClusters.size() << std::endl;
        lastRemainingClusters = this->world->remainingTasksIntoClusters.size();
      }



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


    // Square error calculations
    int world_weedsSeen_greedy = 0;
    int world_weedsSeen_random = 0;
    float world_highestProbability = 0;

    double world_squaredCumulative_greedy = 0.0;
    double world_squaredCumulative_random = 0.0;
    double world_MSE_greedy = 0.0;
    double world_MSE_random = 0.0;
    int world_dataPoints = 0;

    MeanSquareError_duringSimulation();




/*
    if(communicationsRange==-1)
    {
      for(Cell* c : this->world->getCells())
      {
        world_weedsSeen_greedy = 0;
        world_weedsSeen_random = 0;
        world_highestProbability = 0;
        double random = RandomGenerator::getInstance().nextFloat(1);
        for (unsigned i = 0; i < (13+1); i++)
        {
          if(c->knowledgeVector[i] > world_highestProbability)
          {
            world_highestProbability = c->knowledgeVector[i];
            world_weedsSeen_greedy = i;
          }

          random -= c->knowledgeVector[i];
          if(random <= 0 && world_weedsSeen_random == 0)
          {
            world_weedsSeen_random = i;
          }
        }
        
        world_squaredCumulative_greedy += pow((world_weedsSeen_greedy - c->getUtility()),2);
        world_squaredCumulative_random += pow((world_weedsSeen_random - c->getUtility()),2);
        world_dataPoints++;
      }

      world_MSE_greedy = world_squaredCumulative_greedy/world_dataPoints;
      world_MSE_random = world_squaredCumulative_random/world_dataPoints;

      knowledgeBasesFile << world_MSE_greedy << " " << world_MSE_random << "\n";
    }
*/
/*
    if(communicationsRange>0) 
    {
      float lowestEntropy = 0.0;
      std::vector<int> cells_lowestEntropy;
      cells_lowestEntropy.resize(this->world->getCells().size());
        
      for (unsigned i = 0; i < cells_lowestEntropy.size(); i++)
      {
        lowestEntropy = 0.0;
        for(Agent* ag : this->world->getAgents())
        {
          if(ag->cells.at(i)->getResidual()<lowestEntropy || lowestEntropy == 0)
          {
            lowestEntropy=ag->cells.at(i)->getResidual();
            cells_lowestEntropy[i]=ag->getId();
          }        
        }
      }

      world_squaredCumulative_greedy = 0;
      world_squaredCumulative_random = 0;
      world_dataPoints = 0;

      for (unsigned c = 0; c < cells_lowestEntropy.size(); c++)
      {
        world_weedsSeen_greedy = 0;
        world_weedsSeen_random = 0;
        world_highestProbability = 0;
        double random = RandomGenerator::getInstance().nextFloat(1);
        for (unsigned i = 0; i < (13+1); i++)
        {
          Cell* cellOfInterest = this->world->getAgents().at(cells_lowestEntropy[c])->cells.at(c);
          if(cellOfInterest->knowledgeVector[i] > world_highestProbability)
          {
            world_highestProbability = cellOfInterest->knowledgeVector[i];
            world_weedsSeen_greedy = i;
          }

          random -= cellOfInterest->knowledgeVector[i];
          if(random <= 0 && world_weedsSeen_random == 0)
          {
            world_weedsSeen_random = i;
          }
        }
        
        world_squaredCumulative_greedy += pow((world_weedsSeen_greedy - this->world->getCells().at(c)->getUtility()),2);
        world_squaredCumulative_random += pow((world_weedsSeen_random - this->world->getCells().at(c)->getUtility()),2);
        world_dataPoints++;
      }
      world_MSE_greedy = world_squaredCumulative_greedy/world_dataPoints;
      world_MSE_random = world_squaredCumulative_random/world_dataPoints;

      knowledgeBasesFile << world_MSE_greedy << " " << world_MSE_random << " ";

      for(Agent* ag : this->world->getAgents())
      {
        world_squaredCumulative_greedy = 0;
        world_squaredCumulative_random = 0;
        world_dataPoints = 0;
        for(Cell* c : ag->cellsPointers)
        {
          world_weedsSeen_greedy = 0;
          world_weedsSeen_random = 0;
          world_highestProbability = 0;
          double random = RandomGenerator::getInstance().nextFloat(1);
          for (unsigned i = 0; i < (13+1); i++)
          {
            if(c->knowledgeVector[i] > world_highestProbability)
            {
              world_highestProbability = c->knowledgeVector[i];
              world_weedsSeen_greedy = i;
            }

            random -= c->knowledgeVector[i];
            if(random <= 0 && world_weedsSeen_random == 0)
            {
              world_weedsSeen_random = i;
            }
          }
          
          world_squaredCumulative_greedy += pow((world_weedsSeen_greedy - c->getUtility()),2);
          world_squaredCumulative_random += pow((world_weedsSeen_random - c->getUtility()),2);
          world_dataPoints++;
        }

        world_MSE_greedy = world_squaredCumulative_greedy/world_dataPoints;
        world_MSE_random = world_squaredCumulative_random/world_dataPoints;

        knowledgeBasesFile << world_MSE_greedy << " " << world_MSE_random << " ";

      }

      knowledgeBasesFile << "\n";
    }
*/
}


void Engine::RenderScene()
{
    // Get + handle user input events
    glfwPollEvents();

	camera.keyControl(mainWindow.getKeys(), deltaTime);
	camera.mouseControl(mainWindow.getXChange(), mainWindow.getYChange());

    // clear window
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    //glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //glUseProgram(shaderList[0].GetShaderID());
	shaderList[0]->UseShader();
	uniformModel = shaderList[0]->GetModelLocation();
	uniformProjection = shaderList[0]->GetProjectionLocation();
	uniformView = shaderList[0]->GetViewLocation();
  uniformInColor = shaderList[0]->GetInColor();

    glUniform1f(shaderList[0]->GetXmoveLocation(), triOffset);
    //glUseProgram(shader1->GetShaderID());
    //glUniform1f(shader1->GetXmoveLocation(), triOffset);
    //glUniform1f(0, triOffset);
    
      
    glm::mat4 model = glm::mat4(1.0f); 
  
  for(Mesh* m : meshList)
  {
  glm::vec4 agentColor = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
  model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(m->GetWorldLocation()[0], m->GetWorldLocation()[1], m->GetWorldLocation()[2]));
	model = glm::scale(model, glm::vec3(m->GetWorldScale()[0], m->GetWorldScale()[1], m->GetWorldScale()[2]));
  glUniform4fv(uniformInColor, 1, glm::value_ptr(m->getCurrentColor())); 
	glUniformMatrix4fv(uniformModel, 1, GL_FALSE, glm::value_ptr(model));
	glUniformMatrix4fv(uniformProjection, 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(uniformView, 1, GL_FALSE, glm::value_ptr(camera.calculateViewMatrix()));
	if(m != nullptr)
  {m->RenderMesh();}
  }

  //RenderText("This is sample text", 25.0f, 25.0f, 1.0f, glm::vec3(0.5, 0.8f, 0.2f));
  //RenderText("(C) LearnOpenGL.com", 540.0f, 570.0f, 0.5f, glm::vec3(0.3, 0.7f, 0.9f));

    glUseProgram(0);

    mainWindow.swapBuffers();


}

void Engine::CreateShaders()
{
	shader1 = new Shader(0);
	shader1->CreateFromFiles(vShader, fShader);
	shaderList.push_back(shader1);

	shader2 = new Shader(1);
	shader2->CreateFromFiles(vTextShader, fTextShader);
	shaderList.push_back(shader1);

    printf("Shaders created \n");
}


void Engine::AddMesh(Mesh* inMesh)
{
    meshList.push_back(inMesh);
}


void Engine::RenderText(std::string text, GLfloat x, GLfloat y, GLfloat scale, glm::vec3 color)
{
    glm::vec4 textColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);

    // Activate corresponding render state	
    shader2->UseShader();
    uniformModel = shader2->GetModelLocation();
    uniformProjection = shader2->GetProjectionLocation();
    uniformView = shader2->GetViewLocation();
    uniformInColor = shader2->GetInColor();

    glUniform4fv(uniformInColor, 1, glm::value_ptr(textColor)); 
    //glUniform3f(glGetUniformLocation(shader.get, "textColor"), color.x, color.y, color.z);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

    // Iterate through all characters
    std::string::const_iterator c;
    for (c = text.begin(); c != text.end(); c++) 
    {
        Character ch = Characters[*c];

        GLfloat xpos = x + ch.Bearing.x * scale;
        GLfloat ypos = y - (ch.Size.y - ch.Bearing.y) * scale;

        GLfloat w = ch.Size.x * scale;
        GLfloat h = ch.Size.y * scale;
        // Update VBO for each character
        GLfloat vertices[6][4] = {
            { xpos,     ypos + h,   0.0, 0.0 },            
            { xpos,     ypos,       0.0, 1.0 },
            { xpos + w, ypos,       1.0, 1.0 },

            { xpos,     ypos + h,   0.0, 0.0 },
            { xpos + w, ypos,       1.0, 1.0 },
            { xpos + w, ypos + h,   1.0, 0.0 }           
        };
        // Render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.TextureID);
        // Update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); // Be sure to use glBufferSubData and not glBufferData

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // Render quad
        glDrawArrays(GL_TRIANGLES, 0, 6);
        // Now advance cursors for next glyph (note that advance is number of 1/64 pixels)
        x += (ch.Advance >> 6) * scale; // Bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}



void Engine::WriteKnowledgeBasesFile(std::string inString)
{
knowledgeBasesFile << inString << std::endl;
}




void Engine::ResetWorld()
{
    for(Cell* c : getWorld()->getCells())
    {
      c->resetCell();
    }
}

void Engine::MeanSquareError_World(std::vector<Cell*> cells)
{
  current_world_MSE_lastSeen = 0;
  current_world_MSE_greedy = 0;
  current_world_MSE_random = 0;

  float cumulative_weedsSeen_lastSeen = 0;
  float cumulative_weedsSeen_greedy = 0;
  float cumulative_weedsSeen_random = 0;


  for(Cell* c : cells)
  {
    float highestWeedsProbability = 0;
    int weedsSeen_highestProbability = -1;
    int weedsSeen_randomProbability = -1;

    double random = RandomGenerator::getInstance().nextFloat(1);

    for (unsigned i = 0; i < (13+1); i++)
    {
      if(c->knowledgeVector[i]>highestWeedsProbability)
      {
        highestWeedsProbability = c->knowledgeVector[i];
        weedsSeen_highestProbability = i;
      }

      random -= c->knowledgeVector[i];
      if(random <= 0 && weedsSeen_randomProbability == -1)
      {
        weedsSeen_randomProbability = i;
      }
    }

    int lastWeedsSeen = 0;
    if(c->getLastWeedsSeen())
    {
      lastWeedsSeen=c->getLastWeedsSeen();
    }
    cumulative_weedsSeen_lastSeen += pow(lastWeedsSeen-c->getUtility(),2);
    cumulative_weedsSeen_greedy += pow(weedsSeen_highestProbability-c->getUtility(),2);
    cumulative_weedsSeen_random += pow(weedsSeen_randomProbability-c->getUtility(),2);

  }

  current_world_MSE_lastSeen = cumulative_weedsSeen_lastSeen / cells.size();
  current_world_MSE_greedy = cumulative_weedsSeen_greedy / cells.size();
  current_world_MSE_random = cumulative_weedsSeen_random / cells.size();

}


void Engine::MeanSquareError_Agents()
{
  
}

void Engine::MeanSquareError_duringSimulation()
{
    if(communicationsRange==-1)
    {
      MeanSquareError_World(getWorld()->getCells());
      knowledgeBasesFile << current_world_MSE_lastSeen << " " << current_world_MSE_greedy << " " << current_world_MSE_random << "\n";
    }
    if(communicationsRange>0)
    {
      MeanSquareError_World(getWorld()->getCells());
      knowledgeBasesFile << current_world_MSE_lastSeen << " " << current_world_MSE_greedy << " " << current_world_MSE_random << " ";
      for (Agent* a : agents)
      {
        MeanSquareError_World(a->cellsPointers);
        knowledgeBasesFile << current_world_MSE_lastSeen << " " << current_world_MSE_greedy << " " << current_world_MSE_random << " ";
      }
      knowledgeBasesFile << "\n";
    }
}

//-------------------- TEST FUNCTIONS AREA -----------------------------------//

double Engine::TestFunction_logChoose(int n, int k) 
{
  return std::lgamma(double(n+1)) - std::lgamma(double(k+1)) - std::lgamma(double(n-k+1));
}

double Engine::TestFunction_PMFBinomial(double p, int n, int k) 
{
  //double lgrTest = factorial(n) * pow(p,k)*pow((1-p), n-k);
  
  double lgr = TestFunction_logChoose(n, k) + double(k)*std::log(p) + double(n-k)*std::log(1-p);
  return std::exp(lgr);
}

void Engine::TestFunction_SetSensorTable(bool printTable)
{
  bool DEBUG_FUNCTION = true;
  //std::vector<std::vector<float,test_maxWeedsPerCell+1>,test_maxWeedsPerCell+1> newSensorVector;
  //test_sensorTable = newSensorVector;
  //test_sensorTable[0].resize(test_maxWeedsPerCell+1);
  //test_sensorTable.resize(test_maxWeedsPerCell+1);
  /*
  for (auto i : test_sensorTable)
  {
    i.resize(test_maxWeedsPerCell+1);
  }
  */
  
  if(printTable)
  {std::cout << std::endl << "Sensor Table: " << std::endl;}
  
  for (unsigned c = 0; c <= this->test_maxWeedsPerCell; c++ )
  {
    for (unsigned o = 0; o <= this->test_maxWeedsPerCell; o++ )
    {
      if(o>c)
        test_sensorTable[o][c] = 0;
      else
      {
        test_sensorTable[o][c] = TestFunction_PMFBinomial(0.95, c, o); // make 0.95 to parameter
      }
      if(printTable)
      {std::cout << test_sensorTable[o][c] << " ";}
    }
    if(printTable)
    {std::cout << std::endl;}
  }
  if(printTable)
  {std::cout << std::endl;}               

}

void Engine::TestFunction_Scan(bool printThis)
{
  bool DEBUG_THIS = false;

  // scan at current location and return the perceived cell
    //std::cout << std::endl;    
    //clear observationVector

    test_cell->observationVector.fill(0);
    //compute observationVector using the current knowledge and the constant sensorTable
    if(DEBUG_THIS)
    {std::cout << "Observation Vector: " << std::endl;}
    for(unsigned l = 0; l < test_maxWeedsPerCell+1; l++ ){
        for(unsigned k = 0; k < test_maxWeedsPerCell+1; k++ )
        {
            test_cell->observationVector[k] += test_cell->knowledgeVector[l]*test_sensorTable[k][l]; //equation 3
            
            if(DEBUG_THIS)
            {std::cout << test_cell->observationVector[k] << " ";}                    
        }
        if(DEBUG_THIS)
        {std::cout << std::endl;}
    }
    //std::cout << std::endl;    

    //get amount of weeds seen by sensor in current observation
    unsigned world_weedsSeen;
    float random = RandomGenerator::getInstance().nextFloat(1);
    //std::cout << "random observation: " << random << std::endl;
    for (unsigned i = 0; i < test_maxWeedsPerCell+1; i++)
    {  
      random -= test_sensorTable[i][test_cell->getUtility()];
      if(random <= 0)
      {
        world_weedsSeen = i;
        test_weeds_seen = world_weedsSeen;
        break;
      }
    } 

    test_cell->residual_uncertainty = 0.0;
    float entr = 0;

    if(DEBUG_THIS)
    {std::cout << "Knowledge Vector for cell with " << test_cell->getUtility() << " weeds and " << world_weedsSeen << " seen weeds:" << std::endl;}
    
    for(unsigned i = 0; i<test_maxWeedsPerCell+1; i++)
    {
      test_cell->knowledgeVector[i] = test_cell->knowledgeVector[i] * test_sensorTable[world_weedsSeen][i] 
                                  / test_cell->observationVector[world_weedsSeen]; //current cell, equation 5
      
      if(DEBUG_THIS)
      {std::cout << test_cell->knowledgeVector[i] << " ";}   
      //if i-th element is != 0  ---> calculate H(c)
     
      if(test_cell->knowledgeVector[i] != 0)
      {
        entr +=  test_cell->knowledgeVector[i]*(std::log(test_cell->knowledgeVector[i])); //equation 6
      }
    }
    
    if(DEBUG_THIS)
    {std::cout << std::endl;}
    
    
    //std::cout << "total residual uncertainty " << -entr << " of cell " << currentCell->getId() << std::endl; 
    //store H(c) for the current cell
    test_cell->residual_uncertainty = -entr;

    //std::cout << "scan of agent " << this->getId() << " result " << currentCell->residual_uncertainty << std::endl;
    std::stringstream scanReport;
    //scanReport << "Agent " << this->getId() << " at timestep " << timeStep << " scanned cell " << currentCell->getId() 
    //<< "" << std::endl; 
    if(DEBUG_THIS)
    {scanReport << test_cell->getId() << " " << world_weedsSeen << " ";}
    for(float f : test_cell->knowledgeVector)
    {
      scanReport << f << " ";
    }
    for(float f : test_cell->observationVector)
    {
      scanReport << f << " ";
    }
    //scanReport = "Agent " + toString(this->getId());
    //std::cout << scanReport << std::endl;
    std::string outString;
    outString = scanReport.str();

    //std::cout << world_weedsSeen << std::endl;

}

void Engine::TestFunction_computeIG(bool printTable)
{

  test_observationVector.fill(0);

  for(unsigned k = 0; k < test_maxWeedsPerCell+1; k++ )
  {
    for(unsigned l = 0; l < test_maxWeedsPerCell+1; l++ )
    {
      test_observationVector[k] += test_knowledgeVector[l]*test_sensorTable[k][l];
    }
  }
  
  float informationGain = 0; // part of equation
  
  for(unsigned o = 0; o < test_maxWeedsPerCell+1; o++)
  {
    for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
    {
      float logg = 0;
      if(test_knowledgeVector[c] != 0)
        logg+=log(test_knowledgeVector[c]);    //first term multiplication by table missing
      if(test_sensorTable[o][c] != 0)
        logg+=log(test_sensorTable[o][c]);  // second term (only a part is considered)
        //logg+=targetCell->knowledgeVector[c]*Engine::getInstance().getWorld()->getSensorTable()[o][c]*log(Engine::getInstance().getWorld()->getSensorTable()[o][c]);
      if(test_cell->observationVector[o] != 0)
        logg-=log(test_cell->observationVector[o]);    //third term, also it only considers part of the term
        //logg-=log(targetCell->observationVector[o]);
        informationGain -= test_cell->knowledgeVector[c]*test_sensorTable[o][c]*logg; //
        //inclusion of dimitri's last comment
    }
  }
  informationGain = test_cell->residual_uncertainty - informationGain; // why - residual uncertainty
  
 /*
  float sum = 0;
  float sum2 = 0;
  for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
  {
    for(unsigned o = 0; o < test_maxWeedsPerCell+1; o++)
    {
      sum = test_sensorTable[o][c];
      if(test_sensorTable[o][c] > 0)
      {sum2 += test_knowledgeVector[c] * test_sensorTable[o][c] * log(test_sensorTable[o][c]); }
    }
    informationGain += - test_knowledgeVector[c] * sum - test_knowledgeVector[c] * sum2;
    sum = 0;
    sum2 = 0;
  }
  
  sum = 0;
  sum2 = 0;
  for(unsigned o = 0; o < test_maxWeedsPerCell+1; o++)
  {
    for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
    {
      if(test_cell->observationVector[o] != 0)
      {sum += test_sensorTable[o][c] * test_knowledgeVector[c];}
      sum2 += test_knowledgeVector[c] + test_sensorTable[o][c];
    }
    informationGain += (log(sum))*sum2; 
    sum = 0;
    sum2 = 0;
  }
*/
  test_IG_value = -informationGain;

}



void Engine::TestFunction_IG()
{

  std::cout << std::endl;
  std::cout << std::endl;

  unsigned weedNumber = 12;
  std::cout << "Testing IG" << std::endl;
  std::cout << "Enter number of weeds in cell: ";
  std::cin >> weedNumber;
  if(weedNumber > test_maxWeedsPerCell)
  {
    std::cout << "Maximum possible weeds is 12, selecting 12 now" << std::endl;
    weedNumber = test_maxWeedsPerCell;
  }
  float mapping_threshold;
  std::cout << "Enter mapping threshold: " ;
  std::cin >> mapping_threshold;
 
  TestFunction_SetSensorTable(true);

  test_knowledgeVector.fill(1.0/(test_maxWeedsPerCell+1.0));
  test_observationVector.fill(0.0);

  test_cell = new Cell(1000, 1000, 1000, 1000, 1, false, weedNumber);
  bool isMapped = false;
  unsigned observations_count = 0;
  while(!isMapped)
  {    
    if(observations_count==0)
    {
      TestFunction_computeIG(false);
      std::cout << "Observation: " << observations_count << ", weeds seen: " << test_weeds_seen << " residual entropy: " 
      << test_cell->getResidual() << ", IG: " << test_IG_value << std::endl;
    }

    TestFunction_Scan(false);
    TestFunction_computeIG(false);

    observations_count++;
    std::cout << "Observation: " << observations_count << ", weeds seen: " << test_weeds_seen << " residual entropy: " 
    << test_cell->getResidual() << ", IG: " << test_IG_value << std::endl;
    
    if(test_cell->getResidual() < mapping_threshold)
    {
      std::cout << "cell mapped with " << observations_count << " observations" << std::endl;
      isMapped = true;
    }

  }

  std::cout << "Press any key to exit";

  std::cin.get();
  std::cin.get();

}

