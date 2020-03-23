#include "collisionavoidance/orca.hpp"
#include "collisionavoidance/line.hpp"
#include "sim/engine.hpp"



Eigen::Vector2f Orca::compute_orca(Agent* me, Eigen::Vector2f orca_velocity, std::vector<Agent*> others, float t, float dt){
  // Compute ORCA solution for agent. NOTE: velocity must be _instantly_
  // changed on tick *edge*, like first-order integration, otherwise the method
  // undercompensates and you will still risk colliding.
  std::vector<Line> lines;
  for(auto other : others){
    if(other->getId() == me->getId() || other->getZ() > 0.5 ) 
      continue;
    std::vector<Eigen::Vector2f> dv_n = get_avoidance_velocity(me, other, t, dt);
    // n is dv_n.at(1)
    // dv is dv_n.at(0)
    // except assertion error
    if(norm_sq(dv_n.at(1)) < 0){
      // "assertion error"
      continue;
    }
    Eigen::Vector2f vel;
    vel <<  me->getVX(), me->getVY();
    Line line = Line((vel + (dv_n.at(0) / 2)), dv_n.at(1));
    lines.push_back(line);
  }
  Eigen::Vector2f ret = halfplane_optimize(lines, orca_velocity);

  // limit bound velocity
  if(std::abs(ret(0)) > 0.1 || std::abs(ret(1)) > 0.1 ){
    if(ret(0) < 0){
      ret(0) = -me->getLinearVelocity();
    }
    if(ret(0) > 0){
      ret(0) = me->getLinearVelocity();
    }
    if(ret(1) < 0){
      ret(1) = -me->getLinearVelocity();
    }
    if(ret(1) > 0){
      ret(1) = me->getLinearVelocity();
    }
  }
  return ret;
}

std::vector<Eigen::Vector2f> Orca::get_avoidance_velocity(Agent* me, Agent* collider, float t, float dt){
  //trasforming position into eigen vectors
  Eigen::Vector2f my_pose;
  my_pose << me->getX(), me->getY();
  Eigen::Vector2f other_pose;
  other_pose << collider->getX(), collider->getY();

  // Agents Position Diff
  Eigen::Vector2f x = -(my_pose - other_pose);

  // Agents Velocity Diff
  Eigen::Vector2f my_vel;
  my_vel << me->getVX(), me->getVY();
  Eigen::Vector2f other_vel;
  other_vel << collider->getVX(), collider->getVY();
  Eigen::Vector2f v = my_vel - other_vel;

  float r = Engine::getInstance().getOrcaRadius()*2;
  float x_len_sq = x.dot(x);
  Eigen::Vector2f w,n,u;

  if (x_len_sq >= (r * r)){
      //std::cout << "We need to decide whether to project onto the disk truncating the VO" << std::endl;
      // We need to decide whether to project onto the disk truncating the VO
      // or onto the sides.
      // The center of the truncating disk doesn't mark the line between
      // projecting onto the sides or the disk, since the sides are not
      // parallel to the displacement. We need to bring it a bit closer. How
      // much closer can be worked out by similar triangles. It works out
      // that the new point is at x/t cos(theta)^2, where theta is the angle
      // of the aperture (so sin^2(theta) = (r/||x||)^2).
    
    Eigen::Vector2f adjusted_center = x*(1/t) * (1 - ((r*r)/x_len_sq));
    if(((v-adjusted_center).dot(adjusted_center)) < 0){
      // v lies in the front part of the cone
      w = (v - x*(1/t));
      n = normalized(w);
      u = (n * r/t ) - w;
    }
    else{
      // v lies in the rest of the cone
      // Rotate x in the direction of v, to make it a side of the cone.
      // Then project v onto that, and calculate the difference.
      float leg_len = sqrt(x_len_sq - r*r);
      // The sign of the sine determines which side to project on.
      Eigen::Matrix2f temp;
      temp << v, x;
      float sine = copysignf(r,temp.determinant());
      Eigen::Matrix2f rot;
      rot <<  leg_len, -sine,
	            sine, leg_len;
      // TODO check it was           rot.dot(x)/x_len_sq
      Eigen::Vector2f rotated_x = rot * x * (1/x_len_sq);
      n = perp(rotated_x);
      if (sine < 0){
        // Need to flip the direction of the line to make the
        // half-plane point out of the cone.
        n = -n;
      }
      u = (rotated_x * (v.dot( rotated_x))) - v;
    }
  }
  else{
    //std::cout << "intersecting" << std::endl;
    // We're already intersecting. Pick the closest velocity to our
    // velocity that will get us out of the collision within the next
    // timestep.
    // print("intersecting")
    w = (v - (x*1/dt));
    n = normalized(w);
    u = (n * (r*(1/dt))) - w;
  }
  std::vector<Eigen::Vector2f> ret;
  ret.push_back(u);
  ret.push_back(n);
  return ret;
}


Eigen::Vector2f Orca::halfplane_optimize(std::vector<Line> lines, Eigen::Vector2f optimal_point){
  //    """Find the point closest to optimal_point in the intersection of the
  //    closed half-planes defined by lines which are in Hessian normal form
  //    (point-normal form)."""
  //    # We implement the quadratic time (though linear expected given randomly
  //    # permuted input) incremental half-plane intersection algorithm as laid
  //    # out in http://www.mpi-inf.mpg.de/~kavitha/lecture3.ps
  Eigen::Vector2f point = optimal_point;
  for (unsigned i = 0; i < lines.size(); i++){
    // If this half-plane already contains the current point, all is well.
    if ((point - lines.at(i).point).dot(lines.at(i).direction) >= 0){
      // assert False, point
      continue;
    }
    //        # Otherwise, the new optimum must lie on the newly added line. Compute
    //        # the feasible interval of the intersection of all the lines added so
    //        # far with the current one.
    std::vector<Line> prev_lines = islice(lines, i);
    std::vector<float> left_right_dist;
    if(prev_lines.size() == 0){
      float left_dist = -(std::numeric_limits<float>::infinity());
      float right_dist = (std::numeric_limits<float>::infinity());
      left_right_dist.push_back(left_dist);
      left_right_dist.push_back(right_dist);
    }else{
      left_right_dist = line_halfplane_intersect(lines.at(i), prev_lines);
    }
    if (left_right_dist.empty()){
      point = optimal_point;
      return point;
    }else{
      // Now project the optimal point onto the line segment defined by the
      // the above bounds. This gives us our new best point.
      point = point_line_project(lines.at(i), optimal_point, left_right_dist.at(0), left_right_dist.at(1));
    }
  }
  return point;
}

Eigen::Vector2f Orca::point_line_project(Line line, Eigen::Vector2f point, float left_bound, float right_bound){
  //    """Project point onto the line segment defined by line, which is in
  //    point-normal form, and the left and right bounds with respect to line'
  //    anchor point."""
  Eigen::Vector2f new_dir = perp(line.direction);
  float proj_len = (point - line.point).dot(new_dir);
  float clamped_len = clamp(proj_len, left_bound, right_bound);
  return (line.point + (new_dir * clamped_len));

}


float Orca::norm_sq(Eigen::Vector2f v){
  return v.dot(v);
}

Eigen::Vector2f Orca::normalized(Eigen::Vector2f v){
  float l = norm_sq(v);
  return v * 1/sqrt(l);
}

float Orca::dist_sq(Eigen::Vector2f a, Eigen::Vector2f b){
  return norm_sq( b - a );
}

Eigen::Vector2f Orca::perp(Eigen::Vector2f v){
  Eigen::Vector2f ret;
  ret << v(1,0), -v(0,0);
  return ret;
}

std::vector<Line> Orca::islice(std::vector<Line> lines, int n){
  std::vector<Line> ret;
  for(unsigned i =0; i < lines.size(); i++){
    if ( i >= unsigned(n)){
      break;
    }
    ret.push_back(lines.at(i));
  }
  return ret;
}
std::vector<float> Orca::line_halfplane_intersect(Line line, std::vector<Line> other_lines)
{
  //    """Compute the signed offsets of the interval on the edge of the
  //    half-plane defined by line that is included in the half-planes defined by
  //    other_lines.
  //    The offsets are relative to line's anchor point, in units of line's
  //    direction.
  // "Left" is the negative of the canonical direction of the line.
  //    # "Right" is positive.
  float left_dist = -(std::numeric_limits<float>::infinity());
  float right_dist = (std::numeric_limits<float>::infinity());
  for(auto prev_line : other_lines)
    {

      float num = prev_line.direction.dot(line.point - prev_line.point);
      Eigen::Matrix2f temp;
      temp << line.direction(0),line.direction(1), 
              float(prev_line.direction(0)),float(prev_line.direction(1));

      float den = temp.determinant();
      // Check for zero denominator, since ZeroDivisionError
      if (den == 0)
      {
        // The half-planes are parallel.
        if (num < 0)
          {
            // The intersection of the half-planes is empty; there is no
            // solution.
            return std::vector<float>();
            //                return left_dist, right_dist
          }else
          {
            //                # The *half-planes* intersect, but their lines don't cross, so
            //                # ignore.
            continue;
          }
      }
      // Signed offset of the point of intersection, relative to the line's
      // anchor point, in units of the line's direction.
      float  offset = num / den;
      if (den > 0)
      {
        // Point of intersection is to the right.
        right_dist = std::fmin(right_dist, offset);
      }
          else
      {
        // Point of intersection is to the left.
        left_dist = std::fmax(left_dist, offset);
      }
      if (left_dist > right_dist)
      {
        // The interval is inconsistent, so the feasible region is empty.
        // raise InfeasibleError
        return std::vector<float>();
      }
    }
  std::vector<float> ret;
  ret.push_back(left_dist);
  ret.push_back(right_dist);
  return ret;
}

float Orca::clamp(float a, float min, float max)
{
  if(a < min) return min;
  else if(a > max) return max;
  else return a;
}



