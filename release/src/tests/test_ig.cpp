#include "sim/cell.hpp"
#include "util/randomgenerator.hpp"

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include <boost/math/distributions/poisson.hpp>

  /* Test variables */
static float test_IG_value=0;           // current IG for test cell
static unsigned test_maxWeedsPerCell = 12;   // current max test
static unsigned test_weeds_seen=0;      // weeds visible in each scan

static Cell* test_cell;                 // test cell with all the properties of the simulator
static std::array<std::array<float,14>,13> test_sensorTable;  // Test sensor table 
static std::array<std::array<float,14>,13> test_tempSensorTable;  // Test sensor table 
static std::array<std::array<float,14>,13> test_sensorTable_noNoise;  // No noise sensor table 
static std::array<std::array<float,14>,13> test_sensorTable_noise;  // Noise sensor table 

static boost::math::poisson_distribution<> testPoisson(0.5);


static unsigned seed = 10;

static int currentTable = 0;
static int currentMethod = 0;


double TestFunction_logChoose(int n, int k) 
{
  return std::lgamma(double(n+1)) - std::lgamma(double(k+1)) - std::lgamma(double(n-k+1));
}

double TestFunction_PMFBinomial(double p, int n, int k) 
{
  double lgr = TestFunction_logChoose(n, k) + double(k)*std::log(p) + double(n-k)*std::log(1-p);
  return std::exp(lgr);
}

double TestFunction_poisson(double k, double lambda) 
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

// function to create test sensor table
void TestFunction_SetSensorTable(bool printTable)
{
  bool DEBUG_FUNCTION = false; // to print debuging values

  // Show table to review math if needed, the rest of the table is printed as it is built (for this put true in function call in test function)
  if(printTable)    
  {std::cout << std::endl << "Sensor Table: " << currentTable << std::endl;}
  
  // Loop to build test sensor table. 
  // The purpose of this table is to precalculate the probabilities of having the right perception of weeds in a cell
  // The table is a test_maxWeedsPerCell+1xtest_maxWeedsPerCell+1 table (0 weeds to 12 weeds)
  if(currentTable == 0)
  {
    for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        if(o>c)                             // since there are not false positives yet, 0 is introduced for observations above the actual cells
          test_sensorTable[o][c] = 0;       
        else
        {
          // the values of the sensor table are obtained through probability mass function
          test_sensorTable[o][c] = TestFunction_PMFBinomial(0.95, c, o); 
        }
      }
    }  
  }

  if(currentTable == 5)
  {
    if(printTable)    
    {std::cout << std::endl << "Using PMF binomial and poison distribution to include false positives" << std::endl;}

    for (unsigned c = 0; c <= test_maxWeedsPerCell+1; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o <= test_maxWeedsPerCell+1; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        if(o>c)                             // since there are not false positives yet, 0 is introduced for observations above the actual cells
          test_tempSensorTable[o][c] = 0;       
        else
        {
          // the values of the sensor table are obtained through probability mass function
          test_tempSensorTable[o][c] = TestFunction_PMFBinomial(0.95, c, o); 
        }
      }
    } 
    double tableSum = 0;
    for (unsigned c = 0; c < test_maxWeedsPerCell+1; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o < test_maxWeedsPerCell+1; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        tableSum = 0;
        if(o<=c) //false negatives part
        {
          for (unsigned i = 0; i <= o; i++ )
          {
            tableSum += TestFunction_PMFBinomial(0.95, c, o-i) * TestFunction_poisson(i,0.5);   
          }
        }
        else //false positives part
        {
          for (unsigned i = 0; i <= c; i++ ) 
          {
            tableSum += TestFunction_PMFBinomial(0.95, c, c-i) * TestFunction_poisson(i+(o-c),0.5);
          }
        }
        test_sensorTable[o][c] = tableSum;    
      }

      test_sensorTable[test_maxWeedsPerCell+1][c] = 0;
      for (unsigned i = 0; i <= c; i++ ) 
      {
        if(DEBUG_FUNCTION)
        {
          std::cout << "Last column before: " << test_sensorTable[test_maxWeedsPerCell+1][c] << std::endl;
        }
        test_sensorTable[test_maxWeedsPerCell+1][c] += TestFunction_PMFBinomial(0.95, c, c-i)*(1-cdf(testPoisson,i+(12-c)));
        if(DEBUG_FUNCTION)
        {
          //std::cout << "Last column after: " << test_sensorTable[test_maxWeedsPerCell+1][c] << std::endl;
          //std::cout << "Last column after: " << TestFunction_PMFBinomial(0.95, c, c-i) << ", " << (1-TestFunction_poisson(i+(12-c),0.5)) << std::endl;
          //boost::math::poisson_distribution<> testPoisson(0.5);
          //std::cout << cdf(testPoisson, 5) << std::endl;
          std::cout << "Last column after: " << TestFunction_PMFBinomial(0.95, c, c-i) << ", " << (1-cdf(testPoisson,i+(12-c))) << std::endl;
        }

      }
    }  
  }


  if(currentTable == 1 || currentTable == 3) // no noise
  {
    for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )   // dimension for the real amount of weeds
      {
        for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ ) // dimension for the current observation the agent perceives from the cells
        {
          if(o == c)                             
            test_sensorTable_noNoise[o][c] = 1;       
          else
          {
            test_sensorTable_noNoise[o][c] = 0; 
          }
        }
      } 
  }

  if(currentTable == 1) // no noise
  {
    test_sensorTable = test_sensorTable_noNoise;
  }

  if(currentTable == 2 || currentTable == 3) // noise
  {
    float sum = 0;
    for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        test_sensorTable_noise[o][c] = RandomGenerator::getInstance().nextFloat(1);
        sum += test_sensorTable_noise[o][c];     
      }
      for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        test_sensorTable_noise[o][c] /= sum;     
      }
      sum = 0;
    }
  }

  if(currentTable == 2) // noise
  {
    test_sensorTable = test_sensorTable_noise;
  }
    
  if(currentTable == 3) // not so noisy
  {
  for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        test_sensorTable[o][c] = 0.8*test_sensorTable_noNoise[o][c] + 0.2*test_sensorTable_noise[o][c];       
      }
    }
  }

  if(currentTable == 4) // invisible man
  {
  for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o <= test_maxWeedsPerCell; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        test_sensorTable[o][c] = 1.0f/(test_maxWeedsPerCell+1);       
      }
    } 
  }
  double tableSum = 0;
  if(printTable)
  {
    for (unsigned c = 0; c <= test_maxWeedsPerCell; c++ )   // dimension for the real amount of weeds
    {
      for (unsigned o = 0; o <= test_maxWeedsPerCell+1; o++ ) // dimension for the current observation the agent perceives from the cells
      {
        tableSum += test_sensorTable[o][c];
        std::cout << test_sensorTable[o][c] << " ";
      }
      std::cout << " sum=" << tableSum << std::endl;
      tableSum = 0;
    }
    std::cout << std::endl;
  }

}

// function for the agent to scan current location (test cell in this case)
void TestFunction_Scan(bool printThis)
{
  bool DEBUG_THIS = false; // to print debuging values

  // the cell observationVector is filled with ceros to be refiled with the current perception of the agent
  test_cell->observationVector.fill(0);

  if(DEBUG_THIS)
  {std::cout << "Observation Vector: " << std::endl;}

  // compute the new observationVector using the current knowledge and the constant sensorTable
  int columns = test_maxWeedsPerCell;
  if(currentTable == 5)
  {columns = columns+1;}
  for(unsigned l = 0; l < columns; l++ )
  {
      for(unsigned k = 0; k < columns+1; k++ )
      {
          // the observation of each posibility (0-12 weeds) is filled with 
          test_cell->observationVector[k] += test_cell->knowledgeVector[l]*test_sensorTable[k][l]; 
          
          if(DEBUG_THIS)
          {std::cout << test_cell->observationVector[k] << " ";}                    
      }
      if(DEBUG_THIS)
      {std::cout << std::endl;}
  }



  bool DEBUG_WEEDSSEEN = false; // to print debuging values

  // Set amount of weeds "observed" by sensor in current scan
  // a random value is generated to selected a value in the observation dimensions
  unsigned weedsSeen;
  float initialRandom = RandomGenerator::getInstance().nextFloat(1);
  float random = initialRandom;

  if(DEBUG_WEEDSSEEN)
  {std::cout << "True negatives random: " << random << std::endl;}      

  for (unsigned i = 0; i < columns+1; i++)
  {  
    // sensor table has o x c dimensions
    // o = current amount of weeds observed by the agent
    // c = actual weed amount in cell (0-12). 
    // The actual value in the array table is the proability of the sensor seeing the real amount of weeds (c)
    random -= test_sensorTable[i][test_cell->getUtility()]; // Utility = amount of weeds
    if(DEBUG_WEEDSSEEN)
    {std::cout << "Random: " << random << ", Table value: " << test_sensorTable[i][test_cell->getUtility()] << std::endl;}    
    if(random <= 0)
    {

      /* 
      when the random generated value has a match with the probabilities in the sensor table along
      the o dimension in c = real amount of weeds in the current scanning cell
      This works since the term of the values in the table is equal to 1
      */
      weedsSeen = i;
      test_weeds_seen = weedsSeen;
      break;
    }
  } 
  
  if(DEBUG_WEEDSSEEN)
  {
    std::cout << "Weeds seen: " << weedsSeen << std::endl;
  }

  //random = RandomGenerator::getInstance().nextFloat(1);
  random = initialRandom;
  
/*   
  if(DEBUG_WEEDSSEEN)
  {std::cout << "False positives random: " << random << std::endl;}  

  for (unsigned i = 0; i < test_maxWeedsPerCell+1; i++)
  {  
    // sensor table has o x c dimensions
    // o = current amount of weeds observed by the agent
    // c = actual weed amount in cell (0-12). 
    // The actual value in the array table is the proability of the sensor seeing the real amount of weeds (c)
    random -= test_sensorTable[i][test_maxWeedsPerCell-test_cell->getUtility()]; // Utility = amount of weeds
    if(DEBUG_WEEDSSEEN)
    {std::cout << "Random: " << random << ", Table value: " << test_sensorTable[i][test_maxWeedsPerCell-test_cell->getUtility()] << std::endl;}     
    if(random <= 0)
    {
      
      when the random generated value has a match with the probabilities in the sensor table along
      the o dimension in c = real amount of weeds in the current scanning cell
      This works since the term of the values in the table is equal to 1
      
      weedsSeen += test_maxWeedsPerCell-i;
      test_weeds_seen = weedsSeen;
      break;
    }
  } 

  if(DEBUG_WEEDSSEEN)
  {
    std::cout << "Weeds seen: " << weedsSeen << std::endl;
  }
*/
    if(DEBUG_THIS)
    {std::cout << "Knowledge Vector for cell with " << test_cell->getUtility() << " weeds and " << weedsSeen << " seen weeds:" << std::endl;}
    

    // reseting the residual uncertainty to be reasigned after scan
    test_cell->residual_uncertainty = 0.0;
    float entr = 0;
    // filling entr with uncertainty equation terms
    float temp_z = 0;
    // The term is negated at the end to set the actual current cell uncertainty
    for(unsigned i = 0; i<columns+1; i++)
    {
      if(DEBUG_THIS)
      {
        std::cout << test_cell->knowledgeVector[i] << " "; 
        std::cout << " |" << test_sensorTable[weedsSeen][i] << "| "; 

      }

      test_cell->knowledgeVector[i] = test_cell->knowledgeVector[i] * test_sensorTable[weedsSeen][i]; 
      //test_cell->knowledgeVector[i] = test_cell->knowledgeVector[i] * test_sensorTable[weedsSeen][i] 
      //                      / test_cell->observationVector[weedsSeen]; //current cell, equation 5
     
      temp_z += test_cell->knowledgeVector[i];

      if(DEBUG_THIS)
      {
        //std::cout << test_cell->observationVector[i] << " ";
        //std::cout << '(' << test_cell->observationVector[i]*(std::log(test_cell->observationVector[i])) << ')' << " ";
        std::cout << " -" << test_cell->knowledgeVector[i] << "- ";
        //std::cout << '(' << test_cell->knowledgeVector[i]*(std::log(test_cell->knowledgeVector[i])) << ')' << " ";
      }
    }

    if(DEBUG_THIS)
    {
      std::cout << std::endl;
      std::cout << "Temp_z: " << temp_z << " ";
      std::cout << std::endl;
    }

    double test_sum = 0;
    for(unsigned i = 0; i<columns+1; i++)
    {
      test_cell->knowledgeVector[i] /= temp_z;
      if(true)
      {
        test_sum += test_cell->knowledgeVector[i];
        std::cout << '(' << test_cell->knowledgeVector[i] << ')' << " ";
      }

    }

    if(true)
    {
      std::cout << std::endl << " sum = " << test_sum << " " << std::endl;
    }


    for(unsigned i = 0; i<columns+1; i++)
    {
      if(test_cell->knowledgeVector[i] != 0)
      {
        //entr +=  test_cell->observationVector[i]*(std::log(test_cell->observationVector[i]));
        entr +=  -test_cell->knowledgeVector[i]*(std::log(test_cell->knowledgeVector[i]));
      }
    }
    
    if(DEBUG_THIS)
    {
      std::cout << std::endl;
      std::cout << "Residual entropy: " << entr << " ";
      std::cout << std::endl;
    }
    
    test_cell->residual_uncertainty = entr;
}

// calculate IG value for cell before current scan
void TestFunction_computeIG(bool printTable)
{
  bool DEBUG_THIS = true;

  test_cell->observationVector.fill(0);
  std::array<float, 13+1> entropyVector;
  entropyVector.fill(0);

  for(unsigned k = 0; k < test_maxWeedsPerCell+1; k++ )
  {
    for(unsigned l = 0; l < test_maxWeedsPerCell+1; l++ )
    {
      //test_cell->observationVector[k] += test_cell->knowledgeVector[l]*test_sensorTable[k][l];
      test_cell->observationVector[k] += -test_sensorTable[k][l]*log(test_sensorTable[k][l]);
    }
  }

 
  // current method implemented in the simulator (the result is negated at the end)
  if(currentMethod == 0)
  {
    float informationGain = 0;
    for(unsigned o = 0; o < test_maxWeedsPerCell+1; o++)
    {
      for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
      {
        float logg = 0;
        if(test_cell->knowledgeVector[c] != 0)
          logg+=log(test_cell->knowledgeVector[c]);    
        if(test_sensorTable[o][c] != 0)
          logg+=log(test_sensorTable[o][c]);  
        if(test_cell->observationVector[o] != 0)
          logg-=log(test_cell->observationVector[o]);  
          informationGain -= test_cell->knowledgeVector[c]*test_sensorTable[o][c]*logg;
      }
    }
    informationGain = test_cell->residual_uncertainty - informationGain; 
    test_IG_value = -informationGain;

  }

  // my understanding of the equations (the result is negated at the end)
  if(currentMethod == 1)
  {
    float informationGain = 0;
    float term = 0;
    float term2 = 0;
    for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
    {
      for(unsigned o = 0; o < test_maxWeedsPerCell+1; o++)
      {
        term = test_sensorTable[o][c];
        if(test_sensorTable[o][c] > 0)
        {term2 += test_cell->knowledgeVector[c] * test_sensorTable[o][c] * log(test_sensorTable[o][c]); }
      }
      informationGain += - test_cell->knowledgeVector[c] * term - test_cell->knowledgeVector[c] * term2;
      term = 0;
      term2 = 0;
    }
    
    term = 0;
    term2 = 0;
    for(unsigned o = 0; o < test_maxWeedsPerCell+1; o++)
    {
      for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
      {
        if(test_cell->observationVector[o] != 0)
        {term += test_sensorTable[o][c] * test_cell->knowledgeVector[c];}
        term2 += test_cell->knowledgeVector[c] * test_sensorTable[o][c];
      }
      informationGain += (log(term))*term2; 
      term = 0;
      term2 = 0;
    }

    test_IG_value = informationGain;

  }

  // Dimitri's code
  if(currentMethod == 2)
  {
    
    if(DEBUG_THIS)
    {
      std::cout << "Entropy vector: ";
    }

    for(unsigned l = 0; l < test_maxWeedsPerCell+1; l++ )
    {
      for(unsigned k = 0; k < test_maxWeedsPerCell+2; k++ )
      {
        if(test_sensorTable[k][l]!=0)
        {
          entropyVector[l] += -test_sensorTable[k][l]*log(test_sensorTable[k][l]);        
          //if(DEBUG_THIS)
          //{
            //std::cout << entropyVector[k] << " ";
          //}
        }
      }
      if(DEBUG_THIS)
      {
        //std::cout << entropyVector[k] << " ";
      }
      //test_cell->observationVector[k] += -test_cell->observationVector[k]*log(test_cell->observationVector[k]);
    }
    
    if(DEBUG_THIS)
    {
      std::cout << std::endl;
    }
    

    //std::cout << "Using dimitri's pseudocode" << std::endl;
    float informationGain = 0;
    float term1 = 0;
    float term2 = 0;
    float term3 = 0;
    for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
    {
      if(test_cell->knowledgeVector[c]>0)
      {term1 = -test_cell->knowledgeVector[c] * log(test_cell->knowledgeVector[c]);}
    }
    
    if(DEBUG_THIS)
    {
      std::cout << "Term 1 = " << term1 << " ";
    }

    for(unsigned c = 0; c < test_maxWeedsPerCell+1; c++)
    {
      term2 += test_cell->knowledgeVector[c] * entropyVector[c];
    }

    if(DEBUG_THIS)
    {
      std::cout << "Term 2 = " << term2 << " ";
    }

    float term3_int = 0;
    for(unsigned o = 0; o <= test_maxWeedsPerCell+1; o++)
    {
      term3_int = 0;
      for(unsigned c = 0; c <= test_maxWeedsPerCell; c++)
      {
        term3_int += test_cell->knowledgeVector[c] * test_sensorTable[o][c];
      }
      if(term3_int>0)
      {term3 += term3_int * log(term3_int);}
    }


    if(DEBUG_THIS)
    {
      std::cout << "Term 3 = " << term3 << " ";
    }

    std::cout << std::endl;

    test_IG_value = - term2 - term3;

  }




}



void TestFunction_IG()
{
  std::cout << std::endl;
  std::cout << std::endl;

  RandomGenerator::getInstance().init(seed);

  
  unsigned weedNumber = test_maxWeedsPerCell;                           // Variable to be fed the amount of desired weeds in test cell 
  std::cout << "Testing IG" << std::endl;
  std::cout << "Enter number of weeds in cell: ";
  std::cin >> weedNumber;                             // Enter amount of weeds in test cell 
  if(weedNumber > test_maxWeedsPerCell)               // Restraining the simulation to have the maximum possible weeds in case the request is higher
  {
    std::cout << "Maximum possible weeds is 12, selecting 12 now" << std::endl;
    weedNumber = test_maxWeedsPerCell;
  }
  float mapping_threshold;      // uncertainty threshold for the cell to be considered map
  std::cout << "Enter mapping threshold: " ;
  std::cin >> mapping_threshold;


  // Select sensor table model
  std::cout << std::endl;
  std::cout << "Select sensor table for test" << std::endl;
  std::cout << "0 = original PMF binomial table (no false positives)" << std::endl;
  std::cout << "1 = no noise" << std::endl;
  std::cout << "2 = noise" << std::endl;
  std::cout << "3 = not so noisy" << std::endl;
  std::cout << "4 = invisible man" << std::endl;
  std::cout << "5 = PMF binomial table (with false positives)" << std::endl;

  std::cin >> currentTable;
  
  if(currentTable < 0 || currentTable > 5)
  {
    std::cout << "Number selected not valid, selecting 0 as current table" << std::endl;
    currentTable = 0;
  }


  // Select IG computing method we are currently testing
  std::cout << std::endl;
  std::cout << "Select computing method to be tested" << std::endl;
  std::cout << "0 = original method currently in the simulator" << std::endl;
  std::cout << "1 = Carlos interpretation of equations shared by dimitri" << std::endl;
  std::cout << "2 = Method with Dimitri's pseudocode" << std::endl;

  std::cin >> currentMethod;
  
  if(currentMethod < 0 || currentMethod > 2)
  {
    std::cout << "Number selected not valid, selecting 0 as current method" << std::endl;
    currentMethod = 0;
  }






  TestFunction_SetSensorTable(true);      // Setting sensor test table (true for printing table)


  // create test cell for to do scans
  // the values inserted represent the location of the cell
  test_cell = new Cell(1, 1, 1, 1, 1, false, weedNumber); 
  bool isMapped = false;
  unsigned scans_count = 0;

  // Do first IG
  TestFunction_computeIG(false);          // Computing IG for each scan attempt (true for printing the process (not implemented yet))
  std::cout << "Scan: " << scans_count << ", weeds seen: " << test_weeds_seen << " residual entropy: " 
  << test_cell->getResidual() << ", IG: " << test_IG_value << std::endl;

  // main loop until the cell is "mapped"
  while(!isMapped)
  {    

    std::cout << std::endl;

    TestFunction_Scan(false);       // do scan of cell to increase the agent knowledge vector about it
    TestFunction_computeIG(false);  // Computing IG for each scan attempt (true for printing the process (not implemented yet))


    // Printing results after scan
    scans_count++;    
    std::cout << "Scan: " << scans_count << ", weeds seen: " << test_weeds_seen << " residual entropy: " 
    << test_cell->getResidual() << ", IG: " << test_IG_value << std::endl;
    
    if(test_cell->getResidual() < mapping_threshold)
    {
      std::cout << "cell mapped with " << scans_count << " scans" << std::endl;
      isMapped = true;
    }


  }

  std::cout << "Press any key to exit";

  std::cin.get();
  std::cin.get();

}


int main(int argc, char *argv[]) 
{

  // Main test function
  // The objective is to simulate an isolated cell and do scans over it asterming an agent is currently above it until the cell is "mapped"
  // Current possible variations are:
    // The number of weeds per cell (fixed maximum value is 12)
    // The uncertainty threshold for the cell to be mapped
  // The test objective is to observe the obtained IG values for the cell after each scan
  TestFunction_IG();


  std::cout << "## ...Exiting" << std::endl;
  return 0;
}
