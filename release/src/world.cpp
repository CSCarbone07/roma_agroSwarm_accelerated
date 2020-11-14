#include "sim/world.hpp"
#include "sim/engine.hpp"

#include <boost/math/distributions/poisson.hpp>



//#define NO_ERROR

unsigned World::posToCellId(unsigned x, unsigned y, unsigned z) const{
  return (x + (this->size.at(0) * y) + (z * (this->size.at(0) * this->size.at(1))));
}

std::array<unsigned, 3> World::idToCellPos(unsigned id) const{
  std::array<unsigned, 3> ret;
  for(auto c : this->cells){
    if( c->getId() == id){
      ret = {c->getX(),c->getY(),c->getZ()};
      break;
    }
  }
  return ret;
}

double logChoose(int n, int k) {
  return std::lgamma(double(n+1)) - std::lgamma(double(k+1)) - std::lgamma(double(n-k+1));
}

double PMFBinomial(double p, int n, int k) {
  //double lgrTest = factorial(n) * pow(p,k)*pow((1-p), n-k);
  
  double lgr = logChoose(n, k) + double(k)*std::log(p) + double(n-k)*std::log(1-p);
  return std::exp(lgr);
}

double MyPoissonFunction(double k, double lambda) 
{
  float factorial = 1;
  //factorial of k
  if(k>1)
  {
    for(unsigned c = 1; c <= k; c++)
    {
      factorial *=c;
    }
  }


  double poisson = pow(lambda,k)*pow(2.71828,-lambda)/factorial;
  return poisson;
}


World::World(std::array<unsigned,3> size) {
  this->cells = std::vector<Cell*>(size.at(0)*size.at(1)*size.at(2), NULL);
  this->agents = std::vector<Agent*>(Engine::getInstance().getNumOfAgents(), NULL);
  this->size = size;
  int id = 0;
  for(unsigned z=0; z<size.at(2); z++) {
    for(unsigned y=0; y<size.at(1); y++){
      for(unsigned x=0; x<size.at(0); x++){
        Cell* c = new Cell(id,x,y,z,cellSize, 0);
        this->cells.at(id) = c;
        this->grid[id] = 0;
        this->remainingTasksToVisit.insert(std::make_pair<>(id, this->cells.at(id)));
        //std::cout<<"Inserting cell "<<this->remainingTasksToVisit.at(id)->getId()<<std::endl;
        id++;
      }
    }
  }
  //std::cout << "init cell neighbors" << std::endl;
  for(Cell* c : cells)
  { 
    c->SetNeighbors(cells);
  }
  

  communication_range = Engine::getInstance().getCommunicationsRange();
  std::cout<<"numero celle   "<<id<<std::endl;
};




bool World::populateAndInitialize(const unsigned clusters, unsigned maxweeds, unsigned isolated){

  boost::math::poisson_distribution<> boostPoisson(0.5);

  //Old Table without false positives
  //sensorTable 
 /*
  for (unsigned c = 0; c <= this->maxWeed4Cell; c++ ){
    for (unsigned o = 0; o <= this->maxWeed4Cell; o++ ){
      if(o>c)
        this->sensorTable[o][c] = 0;
      else{
#ifndef NO_ERROR          
        this->sensorTable[o][c] = PMFBinomial(0.95, c, o); // make 0.95 to parameter
#else
        this->sensorTable[o][c] = 0;
#endif
      }
      std::cout << sensorTable[o][c] << " ";
    }
    std::cout << std::endl;
  }                
*/

  bool abstractSensor = false;
  if(abstractSensor)
  {
    double tableSum = 0;
    for (unsigned c = 0; c < this->maxWeed4Cell+1; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o < this->maxWeed4Cell+1; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        tableSum = 0;
        if(o<=c) //false negatives part
        {
          for (unsigned i = 0; i <= o; i++ )
          {
            tableSum += PMFBinomial(0.95, c, o-i) * MyPoissonFunction(i,0.5);   
          }
        }
        else //false positives part
        {
          for (unsigned i = 0; i <= c; i++ ) 
          {
            tableSum += PMFBinomial(0.95, c, c-i) * MyPoissonFunction(i+(o-c),0.5);
          }
        }
        sensorTable[o][c] = tableSum; 

        sensorTable[o][this->maxWeed4Cell+1] = 0;
      }

      sensorTable[this->maxWeed4Cell+1][c] = 0;
      tableSum = 0;
      
      for (unsigned i = 0; i <= c; i++ ) 
      {
        tableSum += PMFBinomial(0.95, c, c-i)*(1-cdf(boostPoisson,i+(12-c)));
      }
      
      sensorTable[this->maxWeed4Cell+1][c] = tableSum;
      //sensorTable[this->maxWeed4Cell+1][c] = 5;
      //std::cout << "o " << this->maxWeed4Cell+1 << " Table value " << sensorTable[13][c] << std::endl;
      //std::cout << "Table size " << sizeof(sensorTable) << std::endl;
    }
    sensorTable[this->maxWeed4Cell+1][this->maxWeed4Cell+1] = 0;
  }
  else
  {
    
    int fileRow = 0;
    int fileColumn = 0;
    std::ifstream inFile("/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/current_sensor.txt");
    if (inFile.is_open())
    {
        std::string line;
        while( std::getline(inFile,line) )
        {
            std::stringstream ss(line);

            std::string prob;

            while( std::getline(ss,prob,' ') )
            {
              //std::cout << " course " << course;

              sensorTable[fileRow][fileColumn] = std::stod(prob);
              //std::cout << " prob " << prob;
              fileColumn++;
              if(fileColumn==13)
              {
                fileColumn = 0;
                sensorTable[fileRow][13] = 0;
                fileRow ++;
                //std::cout << std::endl;
              }
            }
            std::cout<<"\n";
            
            
        }
    }
    sensorTable[13][13] = 0;

  }


  Print_SensorTable();


  for(int h = - int(this->size.at(0)); h < int (this->size.at(0)); h++){
    for(int w = - int(this->size.at(0)); w < int (this->size.at(0)); w++){
      // compute the distance
      // assumption: cell size is 1
      float d = sqrt(h*h + w*w);
      if(d != 0){  
        if(this->distanceVectors.count(d) != 1){
          std::vector<std::pair<int,int>> dv;
          dv.push_back(std::make_pair<>(h,w));
          this->distanceVectors.insert(std::make_pair<>(d, dv));
        }
        else{
          this->distanceVectors.at(d).push_back(std::make_pair<>(h,w));
        }
      }
    }
  }

  
   // show content:
  // for (std::map<float, std::vector<std::pair<int, int>>>::iterator it=distanceVectors.begin(); it!=distanceVectors.end(); ++it)
  //   for (std::vector<std::pair<int, int>>::iterator it2=it->second.begin(); it2!=it->second.end(); ++it2)
  //     std::cout << it->first << " => " <<it2->first << "  ||   "<<it2->second<<'\n';

  std::cout << "Weeds placement..." << std::endl;
    //isolated
    bool check = true;
    int tot = 0;
    float weedX,weedY;
    int counter=0;
    std::vector<std::pair<int, int>> clustersCenter;
    //clusters
    Weed* w;
    for(unsigned wc = 0; wc < clusters; wc++){
      float densities = 0;
      int clusterCenterX,clusterCenterY, nextLocationX,nextLocationY;
      //try to find a new cluster center
      do{
        //find a random location for the weed
        clusterCenterX = RandomGenerator::getInstance().nextInt(3, size.at(0)-3);
        clusterCenterY = RandomGenerator::getInstance().nextInt(3, size.at(1)-3);
        w = new Weed(clusterCenterX,clusterCenterY, weedCluster_Altitude,1); // Cell size is 1 (Just centered)
        if(clustersCenter.size() == 0){
          if(addWeed(w)){
            tot++;
            clustersCenter.push_back(std::make_pair<>(clusterCenterX, clusterCenterY));
            unsigned population = RandomGenerator::getInstance().nextInt(8,12);
            std::cout<<"population  "<<population<<std::endl;
            this->cells.at(getCellId(w->x,w->y, 0))->setUtility(population);
            this->remainingTasksToMap.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));            
            this->remainingTasksIntoClusters.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));            
            check = false;
            break;
          }
        }
        else{
            float distanza = 50;
          for(unsigned i=0; i<clustersCenter.size(); i++){
            float d = sqrt(pow((clusterCenterX-clustersCenter.at(i).first), 2) + pow((clusterCenterY-clustersCenter.at(i).second), 2));
            if (d<distanza)
              distanza = d;            
          }
          if( distanza > (6*sqrt(2))){
            if(addWeed(w)){
              tot++;
              clustersCenter.push_back(std::make_pair<>(clusterCenterX, clusterCenterY));
              unsigned population = RandomGenerator::getInstance().nextInt(8,12);
              std::cout<<"population  "<<population<<std::endl;
              this->cells.at(getCellId(w->x,w->y, 0))->setUtility(population);
              check = false;
              this->remainingTasksToMap.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));            
              this->remainingTasksIntoClusters.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));                          
              break;
            }
          }
        }   
      }while(check);
      int radius = 1;
      float density;
      //while(radius < size.at(0) && radius < size.at(1)){
      while(radius < 4){
        for(int i=-radius; i<=int(radius); i++){
          for(int j=-radius; j<=int(radius); j++){
            nextLocationX = clusterCenterX + i;
            nextLocationY = clusterCenterY + j;
            if(nextLocationX >= 0 && nextLocationY >= 0 && nextLocationX < size.at(0) && nextLocationY < size.at(1)){
              density = Engine::getInstance().gaussianPDF(sqrt(i*i+j*j),(RandomGenerator::getInstance().nextInt(2)-1)*RandomGenerator::getInstance().nextFloat(1), 8);            
              //std::cout << "spawning density" << density << std::endl;
              Weed* w = new Weed(nextLocationX, nextLocationY, weedCluster_Altitude, density);
              if(density < 0.1) {
                goto finish;
              }
              if(addWeed(w)){
                tot++;
                //avoid placing invisible weeds with too low density
                counter ++;
                this->cells.at(getCellId(w->x,w->y, 0))->setUtility(unsigned (w->density * 13));
                this->remainingTasksToMap.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));
                this->remainingTasksIntoClusters.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));
              }
            }
          }
        }
        radius++;
      }
      finish:
        check = true;
        continue;
    }
    std::cout << "...cluster done   " <<tot<<std::endl << std::endl;
    counter = isolated;
    while(counter > 0){
      std::cout << "...isolated " <<std::endl ;

      //find a random location for the weed
      weedX = RandomGenerator::getInstance().nextInt(size.at(0));
      weedY = RandomGenerator::getInstance().nextInt(size.at(1));
      
      if(getCell(weedX,weedY,0)->getUtility() > 0)
      {
        continue;
      }
      unsigned population = RandomGenerator::getInstance().nextInt(1,4);
      Weed* w = new Weed(weedX,weedY, isolatedWeed_Altitude,float(population/13));
      //w->mesh->SetCurrentColor(glm::vec4(0.4f, 0.7f, 0.0f, 1.0f));

      if(addWeed(w)){
        tot++;
        std::cout<<"   population    "<<population<<std::endl;
        this->cells.at(getCellId(w->x,w->y, 0))->setUtility(population);
        this->remainingTasksToMap.insert(std::make_pair<>(getCellId(w->x,w->y, 0), this->cells.at(getCellId(w->x,w->y, 0))));
        counter--;
      }else{
        continue;
      }

    }
    
    std::cout << "...isolated done   " <<tot<<std::endl;
}


bool World::addAgent(Agent* agent, unsigned x, unsigned y, unsigned z) {
    if(!(x < this->size.at(0) || y < this->size.at(1) || z < this->size.at(2)))
      return false;
    if(grid[posToCellId(x, y, z)] != 0){
      std::cout << "ERROR ADD AGENT " << std::endl;
      return false;
    }
    this->agents.at(agent->getId()) = agent;
    grid[posToCellId(x,y,z)]++;
    this->committedAgents.push_back(agent);
    return true;
  }

  bool World::moveAgentTo(float x, float y, float z, unsigned agentId) {
    // retrieve the agent, assumes non ordered vector
    Agent* agent;
    for (Agent* a : agents) {
      if (a->getId() == agentId) {
        agent = a;
        break;
      }
    }
    // update the grid with new agent position
    grid[posToCellId(floor(agent->getX()), floor(agent->getY()), floor(agent->getZ()))]--;  //TODO not affecting at the moment but it is wrong with cells being centered
    // update agent's position
    agent->setPosition(x,y,z);
    //update the grid with the new agent position
    grid[posToCellId(floor(agent->getX()), floor(agent->getY()), floor(agent->getZ()))]++;
    return true;
  }

  bool World::getPopulationInfo(std::stringstream& ss){
    std::vector<Cell*>::iterator it;
    for (it = this->cells.begin(); it != this->cells.end(); it++){

        ss << ' ' <<(*it)->getId() << ':'<< ' ' << (*it)->getX() << ' ' << (*it)->getY() << ' ' << (*it)->getUtility() << '\n';
    }
    //std::cout<< "test" << std::endl;
    return true;
  }


void World::Print_SensorTable()
{
  double sum = 0;
  for (unsigned c = 0; c <= this->maxWeed4Cell+1; c++ )
  {
    sum = 0;
    for (unsigned o = 0; o <= this->maxWeed4Cell+1; o++ )
    {
      sum += sensorTable[o][c];
      std::cout << sensorTable[o][c] << " ";
    }
    std::cout << " sum: " << sum << " ";
    std::cout << std::endl;
  }
  std::cout << std::endl;

  

}