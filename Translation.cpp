#include <iomanip>
#include "Translation.h"

Translation::Translation()
{
    this->translationId = -1;
    this->tx = 0.0;
    this->ty = 0.0;
    this->tz = 0.0;
}

Translation::Translation(int translationId, double tx, double ty, double tz)
{
    this->translationId = translationId;
    this->tx = tx;
    this->ty = ty;
    this->tz = tz;
}
Matrix4 Translation::getTranslationMatrix()
{
    Matrix4 result;
    result.values[0][0] = 1.0;
    result.values[0][1] = 0.0;
    result.values[0][2] = 0.0;
    result.values[0][3] = tx;
    result.values[1][0] = 0.0;
    result.values[1][1] = 1.0;
    result.values[1][2] = 0.0;
    result.values[1][3] = ty;
    result.values[2][0] = 0.0;
    result.values[2][1] = 0.0;
    result.values[2][2] = 1.0;
    result.values[2][3] = tz;
    result.values[3][0] = 0.0;
    result.values[3][1] = 0.0;
    result.values[3][2] = 0.0;
    result.values[3][3] = 1.0;
    return result;
}

std::ostream &operator<<(std::ostream &os, const Translation &t)
{
    os << std::fixed << std::setprecision(3) << "Translation " << t.translationId << " => [" << t.tx << ", " << t.ty << ", " << t.tz << "]";
    return os;
}