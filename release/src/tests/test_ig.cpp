#include "sim/cell.hpp"
#include "util/randomgenerator.hpp"

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

  /* Test variables */
static bool test_IG = false;
static float test_IG_value=0;
static int test_maxWeedsPerCell = 12;
static unsigned test_weeds_seen=0;
  //std::vector<std::vector<float>> test_sensorTable; 
static Cell* test_cell;
static std::array<std::array<float,13>,13> test_sensorTable; 
static std::array<float, 13> test_knowledgeVector;
static std::array<float, 13> test_observationVector;
  /*
  std::map<float, std::array<float,13>> test_knowledgeVectors;
  std::map<float, std::array<float,13>> test_observationVectors;
  */

double TestFunction_logChoose(int n, int k) 
{
  return std::lgamma(double(n+1)) - std::lgamma(double(k+1)) - std::lgamma(double(n-k+1));
}

double TestFunction_PMFBinomial(double p, int n, int k) 
{
  //double lgrTest = factorial(n) * pow(p,k)*pow((1-p), n-k);
  
  double lgr = TestFunction_logChoose(n, k) + double(k)*std::log(p) + double(n-k)*std::log(1-p);
  return std::exp(lgr);
}

void TestFunction_SetSensorTable(bool printTable)
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
  
  for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )
  {
    for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ )
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

void TestFunction_Scan(bool printThis)
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
    unsigned weedsSeen;
    float random = RandomGenerator::getInstance().nextFloat(1);
    //std::cout << "random observation: " << random << std::endl;
    for (unsigned i = 0; i < test_maxWeedsPerCell+1; i++)
    {  
      random -= test_sensorTable[i][test_cell->getUtility()];
      if(random <= 0)
      {
        weedsSeen = i;
        test_weeds_seen = weedsSeen;
        break;
      }
    } 

    test_cell->residual_uncertainty = 0.0;
    float entr = 0;

    if(DEBUG_THIS)
    {std::cout << "Knowledge Vector for cell with " << test_cell->getUtility() << " weeds and " << weedsSeen << " seen weeds:" << std::endl;}
    
    for(unsigned i = 0; i<test_maxWeedsPerCell+1; i++)
    {
      test_cell->knowledgeVector[i] = test_cell->knowledgeVector[i] * test_sensorTable[weedsSeen][i] 
                                  / test_cell->observationVector[weedsSeen]; //current cell, equation 5
      
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
    {scanReport << test_cell->getId() << " " << weedsSeen << " ";}
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

    //std::cout << weedsSeen << std::endl;

}

void TestFunction_computeIG(bool printTable)
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

  test_IG_value = -informationGain;

}



void TestFunction_IG()
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


int main(int argc, char *argv[]) 
{
  TestFunction_IG();


  //std::cout << "## Saving Results..." << std::endl;
  //TODO save results
  std::cout << "## ...Exiting" << std::endl;
  return 0;
}
