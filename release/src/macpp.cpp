#include "sim/engine.hpp"
#include "util/randomgenerator.hpp"

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

/**
 * Main file
 *
 * @author Dario Albani
 * @email albani@diag.uniroma1.it
 */

/**
 * Method used to merge different yaml files from different modules into
 * a single temporary YAML file
 */
std::string mergeAdditionalFiles(const std::string& inputFile, const std::string& additionalFiles) {
  // generate temp file with all merged information
  size_t lastindex = inputFile.find_last_of(".");
  std::string toReturn = inputFile.substr(0, lastindex);
  toReturn = toReturn + "_temp.yaml";

  if( remove( toReturn.c_str() ) != 0 )
    printf( "Error deleting %s",toReturn.c_str() );
  else
    printf( "%s successfully deleted\n",toReturn.c_str() );


  // temporary file with all merged information
  std::ofstream ifile(toReturn, std::ios::app);
  if (!ifile.is_open()) {
    std::cerr << "###### unable to write the temporary configuration file " << toReturn << std::endl;
    exit(-1);
  }

  // split by spaces
  std::istringstream iss(additionalFiles);
  std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
                                   std::istream_iterator<std::string>());
  // append also the original file
  results.push_back(inputFile);

  for(auto otherFile : results) {
    // open other file with append flag
    std::ifstream ofile(otherFile);

    if (!ofile.is_open()) {
      std::cerr << "###### unable to read the temporary configuration file " << otherFile << std::endl;
      exit(-1);
    } else {
      ifile << ofile.rdbuf();
    }

    ofile.close();
  }

  return toReturn;
}

/**
 * Entry point of the simulation.
 * Declare here your specific classes and parameters if you need to
 */
int main(int argc, char *argv[]) {
  // Declare the supported options
  namespace po =   boost::program_options;
  po::options_description desc("allowed options for the program");
  std::string inputFile; /** main input yaml file */
  std::string outputFile; /** optional output file name */
  std::string additionalFiles; /** additional yaml files to be merged with the input file */
  desc.add_options()("help", "produce the help message")
    ("input,i", po::value<std::string>(&inputFile)->required(),"input file (YAML)")
    ("output,o", po::value<std::string>(&outputFile)->default_value("output_.yaml")->implicit_value(""), "outputfile (YAML)")
    ("addition,a", po::value<std::string>(&additionalFiles)->default_value("")->implicit_value(""), "additional files (YAML)")
    ;

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << "\n";
    std::cerr << desc << "\n";
    return 1;
  }

  // starting up
  std::cout << inputFile << " " << outputFile << std::endl;
  std::cout << "MACPP" << std::endl;
  std::cout << "## starting up..." << std::endl;
  // check the additional file flag and generate a new temporary file
  if (!additionalFiles.empty()) {
    std::cout << "#### merging additional files..." << std::endl;
    inputFile = mergeAdditionalFiles(inputFile, additionalFiles);
  }
  std::cout << "#### ...done" << std::endl;

  // read the merged configuration file
  std::cout << "#### loading configuration file..." << std::endl;
  YAML::Node config = YAML::LoadFile(inputFile);
  std::cout << "#### ...done" << std::endl;

  // initialize the random generator
  std::cout << "#### initializing the random generator..." << std::endl;
  RandomGenerator::getInstance().init(config["seed"].as<unsigned>());
  std::cout << "#### ...done" << std::endl;

  // initialize the engine
  std::cout << "#### initializing the engine..." << std::endl;
  Engine::getInstance().init(config);
  std::cout << "#### ...done" << std::endl;

  std::cout << "## Running..." << std::endl;
  Engine::getInstance().run();
  std::cout << "## ...done" << std::endl;

  std::cout << "## Saving Results..." << std::endl;
  //TODO save results
  std::cout << "## ...done" << std::endl;
}
