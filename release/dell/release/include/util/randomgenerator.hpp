#ifndef RANDOMGENERATOR_HPP
#define RANDOMGENERATOR_HPP

#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/seed_seq.hpp>

/**
 * Singleton
 *
 * Auxiliary class for generating random numbers.
 * It makes use of boost and it is used to share the same generator among the whole simulation
 */

class RandomGenerator {

public:
  /* Singleton specific method */
  static RandomGenerator& getInstance() {
    static RandomGenerator instance;
    return instance;
  }

private:
  unsigned seed; /** < random seed for the mersenne twister */
  boost::random::mt19937 generator; /** < share the same random generator */

  RandomGenerator() {}

public:
  RandomGenerator(RandomGenerator const&) = delete;
  void operator=(RandomGenerator const&) = delete;

  inline void init(unsigned seed) {
    this->seed = seed;
    this->generator = boost::random::mt19937(seed);
  }

  inline boost::random::mt19937 getGenerator() {
    return generator;
  }

  inline int nextInt(int max) {
    return nextInt(0, max);
  }

  inline int nextInt(int min, int max) {
    boost::random::uniform_real_distribution<float> dis(min, max);
    return int(dis(generator));
  }

  inline float nextFloat(float max) {
    return nextFloat(0, max);
  }

  inline float nextFloat(float min, float max) {
    boost::random::uniform_real_distribution<float> dis(min, max);
    return dis(generator);
  }

  inline double nextDouble(double max) {
    return nextDouble(0, max);
  }

  inline double nextDouble(double min, double max) {
    boost::random::uniform_real_distribution<double> dis(min, max);
    return dis(generator);
  }

  inline double nextNormal(double mean, double sigma) {
    boost::random::normal_distribution<double> dis(mean, sigma);
    return dis(generator);
  }
};
#endif /* RANDOMGENERATOR_HPP */
