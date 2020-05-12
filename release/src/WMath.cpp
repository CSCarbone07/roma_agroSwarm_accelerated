#include "libraries/WMath.h"
#include <cmath>


float MapRangeClamped (float in, float inMinRange, float inMaxRange, float outMinRange, float outMaxRange)
{
    if(in >= inMaxRange)
    {return outMaxRange;}

    if(in <= inMinRange)
    {return outMinRange;}

    float alpha =(in-inMinRange)/(inMaxRange-inMinRange);
    float out = FloatLerp(alpha, outMinRange, outMaxRange);
    

    return out;

}

float FloatLerp (float in, float inMin, float inMax)
{
    /*
    if(in >= 1)
    {return inMax;}
    if(in <= 0)
    {return inMin;}
*/
    return (1-in)*inMin + in*inMax;
}

std::vector<float> VectorLerp (float in, std::vector<float> minVector, std::vector<float> maxVector)
{
    return std::vector<float> {FloatLerp(in, minVector.at(0),maxVector.at(0)), 
    FloatLerp(in, minVector.at(1),maxVector.at(1)), FloatLerp(in, minVector.at(2),maxVector.at(2))};
}


int factorial(int n)
{
       for( int i = 1; i < n+1; i = i++ ) 
       {
            n=n*i;
       }
}


