#include "sim/world.hpp"
#include "sim/engine.hpp"

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
  double lgr = logChoose(n, k) + double(k)*std::log(p) + double(n-k)*std::log(1-p);
  return std::exp(lgr);
}


World::World(std::array<unsigned,3> size) {
  this->cells = std::vector<Cell*>(size.at(0)*size.at(1)*size.at(2), NULL);
  this->agents = std::vector<Agent*>(Engine::getInstance().getNumOfAgents(), NULL);
  this->size = size;
  unsigned id = 0;
  unsigned cellSize = 1;
  for(unsigned z=0; z<size.at(2); z++) {
    for(unsigned y=0; y<size.at(1); y++){
      for(unsigned x=0; x<size.at(0); x++){
        Cell* c = new Cell(id,x,y,z,cellSize, 0);
        this->cells.at(id) = c;
        this->grid[id] = 0;
        this->remainingTasksToVisit.insert(std::make_pair<>(id, this->cells.at(id)));
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

  //sensorTable
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
      //std::cout<<sensorTable[o][c]<<"  ";
    }
    //std::cout<<std::endl;
  }                


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
        w = new Weed(clusterCenterX,clusterCenterY,1); // Cell size is 1 (Just centered)
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
      unsigned radius = 1;
      float density;
      //while(radius < size.at(0) && radius < size.at(1)){
      while(radius < 4){
        for(int i=-radius; i<=int(radius); i++){
          for(int j=-radius; j<=int(radius); j++){
            nextLocationX = clusterCenterX + i;
            nextLocationY = clusterCenterY + j;
            if(nextLocationX >= 0 && nextLocationY >= 0 && nextLocationX < size.at(0) && nextLocationY < size.at(1)){
              density = Engine::getInstance().gaussianPDF(sqrt(i*i+j*j),(RandomGenerator::getInstance().nextInt(2)-1)*RandomGenerator::getInstance().nextFloat(1), 8);            
              Weed* w = new Weed(nextLocationX, nextLocationY, density);
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
      Weed* w = new Weed(weedX+0.5,weedY+0.5,1); // Cell size is 1 (Just centered)
      if(addWeed(w)){
        unsigned population = RandomGenerator::getInstance().nextInt(1,4);
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
    grid[posToCellId(floor(agent->getX()), floor(agent->getY()), floor(agent->getZ()))]--;
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
