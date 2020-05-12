#pragma once

#ifndef WMATH_HPP
#define WMATH_HPP

#include <cmath>
#include <vector>

float MapRangeClamped (float in, float inMinRange, float inMaxRange, float outMinRange, float outMaxRange);

float FloatLerp (float in, float inMin, float inMax);
std::vector<float> VectorLerp (float in, std::vector<float> minVector, std::vector<float> maxVector);

int factorial(int n);






#endif /* WMATH_HPP */
