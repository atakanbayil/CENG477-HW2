#include <iomanip>
#include "Rotation.h"
#include <iostream>
#include "Helpers.h"
using namespace std;

Rotation::Rotation() {
    this->rotationId = -1;
    this->angle = 0;
    this->ux = 0;
    this->uy = 0;
    this->uz = 0;
}

Rotation::Rotation(int rotationId, double angle, double x, double y, double z)
{
    this->rotationId = rotationId;
    this->angle = angle;
    this->ux = x;
    this->uy = y;
    this->uz = z;
}
//değiştir
Matrix4 Rotation::getRotationMatrix() {
    Vec3 v;
    Vec3 u = Vec3(this->ux, this->uy, this->uz, -1);
    u = normalizeVec3(u);
    if (abs(u.x) < abs(u.y) && abs(u.x) < abs(u.z)) {
        v = Vec3(0, (-1.0)*u.z, u.y, -1);
    } else if (abs(u.y) < abs(u.x) && abs(u.y) < abs(u.z)) {
        v = Vec3((-1.0)*u.z, 0, u.x, -1);
    } else {
        v = Vec3((-1.0)*u.y, u.x, 0, -1);
    }
    v = normalizeVec3(v);
    Vec3 w = crossProductVec3(u, v);
    w = normalizeVec3(w);
    Matrix4 M = getIdentityMatrix();
    M.values[0][0] = u.x;
    M.values[0][1] = u.y;
    M.values[0][2] = u.z;
    M.values[1][0] = v.x;
    M.values[1][1] = v.y;
    M.values[1][2] = v.z;
    M.values[2][0] = w.x;
    M.values[2][1] = w.y;
    M.values[2][2] = w.z;

    Matrix4 M_inverse = getIdentityMatrix();
    M_inverse.values[0][0] = u.x;
    M_inverse.values[0][1] = v.x;
    M_inverse.values[0][2] = w.x;
    M_inverse.values[1][0] = u.y;
    M_inverse.values[1][1] = v.y;
    M_inverse.values[1][2] = w.y;
    M_inverse.values[2][0] = u.z;
    M_inverse.values[2][1] = v.z;
    M_inverse.values[2][2] = w.z;

    Matrix4 R = getIdentityMatrix();
    R.values[1][1] = cos(this->angle*M_PI/180.0);
    R.values[1][2] = -sin(this->angle*M_PI/180.0);
    R.values[2][1] = sin(this->angle*M_PI/180.0);
    R.values[2][2] = cos(this->angle*M_PI/180.0);

    Matrix4 temp = multiplyMatrixWithMatrix(M_inverse, R);
    Matrix4 result = multiplyMatrixWithMatrix(temp, M);
    return result;

}
//buraya kadar 
std::ostream &operator<<(std::ostream &os, const Rotation &r)
{
    os << std::fixed << std::setprecision(3) << "Rotation " << r.rotationId << " => [angle=" << r.angle << ", " << r.ux << ", " << r.uy << ", " << r.uz << "]";
    return os;
}