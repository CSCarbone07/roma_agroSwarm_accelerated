#ifndef STEPPABLE_HPP
#define STEPPABLE_HPP

#include <iostream>

/**
 * Base class for steppable objects
 * A steppable object is an object over which the simulation iterates during each step
 */

class Steppable {
public:
  virtual bool doStep(unsigned time_step) = 0; /** < execute one step */
  virtual bool getInfo(std::stringstream& ss) = 0; /** < use this to retrieve information about last doStep call */
};

#endif /* STEPPABLE_HPP */
