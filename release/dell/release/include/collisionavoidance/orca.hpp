#ifndef ORCA_HPP
#define ORCA_HPP

#include "collisionavoidance/line.hpp"
#include "agent/agent.hpp"

#include <algorithm>
#include <random>
#include <vector>
#include <limits>
#include <math.h>
#pragma once

class Orca
{

public:
 
  static Eigen::Vector2f compute_orca(Agent* me, Eigen::Vector2f orca_velocity, std::vector<Agent*> others, float t, float dt);

  static std::vector<Eigen::Vector2f> get_avoidance_velocity(Agent* me, Agent* collider, float t, float dt);

  static Eigen::Vector2f halfplane_optimize(std::vector<Line> lines, Eigen::Vector2f optimal_point);

  static float norm_sq(Eigen::Vector2f v);

  static Eigen::Vector2f normalized(Eigen::Vector2f v);

  static float dist_sq(Eigen::Vector2f a, Eigen::Vector2f b);

  static Eigen::Vector2f perp(Eigen::Vector2f v);

  static std::vector<Line> islice(std::vector<Line> lines, int n);

  static std::vector<float> line_halfplane_intersect(Line line,std::vector<Line> other_lines);

  static Eigen::Vector2f point_line_project(Line line, Eigen::Vector2f point, float left_bound, float right_bound);

  static float clamp(float a, float min, float max);

};
#endif /* !ORCA*/
