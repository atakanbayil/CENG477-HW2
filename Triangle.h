#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

class Triangle
{
public:
    int vertexIds[3];

    Triangle();
    Triangle(int vid1, int vid2, int vid3);
    Triangle(const Triangle &other);
    int getFirstVertexId();
    int getSecondVertexId();
    int getThirdVertexId();

    //Vec3* applyTransformations(Matrix4 transformations,Scene scene);
    void setFirstVertexId(int vid);
    void setSecondVertexId(int vid);
    void setThirdVertexId(int vid);
    friend std::ostream &operator<<(std::ostream &os, const Triangle &t);
};

#endif