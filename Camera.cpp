#include <iomanip>
#include "Camera.h"

Camera::Camera() {}

Camera::Camera(int cameraId,
               int projectionType,
               Vec3 position, Vec3 gaze,
               Vec3 u, Vec3 v, Vec3 w,
               double left, double right, double bottom, double top,
               double near, double far,
               int horRes, int verRes,
               std::string outputFilename)
{

    this->cameraId = cameraId;
    this->projectionType = projectionType;
    this->position = position;
    this->gaze = gaze;
    this->u = u;
    this->v = v;
    this->w = w;
    this->left = left;
    this->right = right;
    this->bottom = bottom;
    this->top = top;
    this->near = near;
    this->far = far;
    this->horRes = horRes;
    this->verRes = verRes;
    this->outputFilename = outputFilename;
}

Camera::Camera(const Camera &other)
{
    this->cameraId = other.cameraId;
    this->projectionType = other.projectionType;
    this->position = other.position;
    this->gaze = other.gaze;
    this->u = other.u;
    this->v = other.v;
    this->w = other.w;
    this->left = other.left;
    this->right = other.right;
    this->bottom = other.bottom;
    this->top = other.top;
    this->near = other.near;
    this->far = other.far;
    this->horRes = other.horRes;
    this->verRes = other.verRes;
    this->outputFilename = other.outputFilename;
}
//değiştir

Matrix4 Camera::getViewportTransformationMatrix() {
    Matrix4 viewportMatrix = getIdentityMatrix();
    viewportMatrix.values[0][0] = (this->horRes) / 2.0;
    viewportMatrix.values[0][3] = (this->horRes - 1.0) / 2.0;
    viewportMatrix.values[1][1] = (this->verRes) / 2.0;
    viewportMatrix.values[1][3] = (this->verRes - 1.0) / 2.0;
    viewportMatrix.values[2][2] = 0.5;
    viewportMatrix.values[2][3] = 0.5;
    return viewportMatrix;
}


Matrix4 Camera::getProjectionTransformationMatrix(int projectionType) {
    Matrix4 projectionMatrix = getIdentityMatrix();

    projectionMatrix.values[0][0] = 2.0 / (this->right - this->left);
    projectionMatrix.values[0][3] = -(this->right + this->left) / (this->right - this->left);
    projectionMatrix.values[1][1] = 2.0 / (this->top - this->bottom);
    projectionMatrix.values[1][3] = -(this->top + this->bottom) / (this->top - this->bottom);
    projectionMatrix.values[2][2] = -(2.0 / (this->far - this->near));
    projectionMatrix.values[2][3] = -(this->near + this->far) / (this->far - this->near);

    if (projectionType) {
        Matrix4 pers2orth = getIdentityMatrix();
        pers2orth.values[0][0] = this->near;
        pers2orth.values[1][1] = this->near;
        pers2orth.values[2][2] = this->near + this->far;
        pers2orth.values[2][3] = this->near * this->far;
        pers2orth.values[3][2] = -1.0;
        pers2orth.values[3][3] = 0.0;
        projectionMatrix = multiplyMatrixWithMatrix(projectionMatrix, pers2orth);
    }

    return projectionMatrix;
}

Matrix4 Camera::getCameraTransformationMatrix(){

    Matrix4 cameraTransformationMatrix;

    cameraTransformationMatrix.values[0][0] = u.x;
    cameraTransformationMatrix.values[0][1] = u.y;
    cameraTransformationMatrix.values[0][2] = u.z;
    cameraTransformationMatrix.values[0][3] = -1.0 * dotProductVec3(position,u);

    cameraTransformationMatrix.values[1][0] = v.x;
    cameraTransformationMatrix.values[1][1] = v.y;
    cameraTransformationMatrix.values[1][2] = v.z;
    cameraTransformationMatrix.values[1][3] = -1.0 * dotProductVec3(position,v);

    cameraTransformationMatrix.values[2][0] = w.x;
    cameraTransformationMatrix.values[2][1] = w.y;
    cameraTransformationMatrix.values[2][2] = w.z;
    cameraTransformationMatrix.values[2][3] = -1.0 * dotProductVec3(position,w);

    cameraTransformationMatrix.values[3][0] = 0.0;
    cameraTransformationMatrix.values[3][1] = 0.0;
    cameraTransformationMatrix.values[3][2] = 0.0;
    cameraTransformationMatrix.values[3][3] = 1.0;

    return cameraTransformationMatrix;
}
//buraya kadar
std::ostream &operator<<(std::ostream &os, const Camera &c)
{
    const char *camType = c.projectionType ? "perspective" : "orthographic";

    os << std::fixed << std::setprecision(6) << "Camera " << c.cameraId << " (" << camType << ") => pos: " << c.position << " gaze: " << c.gaze << std::endl
       << "\tu: " << c.u << " v: " << c.v << " w: " << c.w << std::endl
       << std::fixed << std::setprecision(3) << "\tleft: " << c.left << " right: " << c.right << " bottom: " << c.bottom << " top: " << c.top << std::endl
       << "\tnear: " << c.near << " far: " << c.far << " resolutions: " << c.horRes << "x" << c.verRes << " fileName: " << c.outputFilename;

    return os;
}