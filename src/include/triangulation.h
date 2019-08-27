#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <include/common.h>

class Triangulation
{
public:
    Triangulation();
    static Vec3 triangulationPt(Vec2 pt1,
                                Vec2 pt2,
                                SE3 T_c_w1,
                                SE3 T_c_w2,
                                double fx, double fy, double cx, double cy);
    static Vec3 triangulationPt(Vec2 pt1,
                                Vec2 pt2,
                                Mat3x4 projection_matrix1,
                                Mat3x4 projection_matrix2);
    static Vec2 reProjection(Vec3 pt, SE3 T_c_w, double fx, double fy, double cx, double cy);
};

#endif // TRIANGULATION_H
