#ifndef LINE_HPP
#define LINE_HPP


#include "eigen3/Eigen/Dense"
#include <algorithm>
#include <random>
#include <vector>

class Line
{
public:
  Eigen::Vector2f point;
  Eigen::Vector2f direction;
  Line(Eigen::Vector2f point,Eigen::Vector2f direction){
    this->point = point;
    this->direction = direction;
  }
};
#endif /* !LINE*/

