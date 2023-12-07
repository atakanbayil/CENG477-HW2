#ifndef __SCALING_H__
#define __SCALING_H__

#include <iostream>
#include "Vec4.h"
#include "Matrix4.h"
using namespace std;
class Scaling
{
public:
    int scalingId;
    double sx, sy, sz;

    Scaling();
    Scaling(int scalingId, double sx, double sy, double sz);
    Matrix4 getScalingMatrix(Vec4 v);

    friend std::ostream &operator<<(std::ostream &os, const Scaling &s);
};

#endif