#ifndef DEPTHcam0_H
#define DEPTHcam0_H

#include "include/common.h"


enum TYPEOFCAMERA{DEPTH_D435I,
                  STEREO_EuRoC_MAV};
//DepthCamera Class
//Support Z16 Depth Format (RealSense d435, d435i)
//Support Stero Camera
//fx,fy,cx,cy are rectified cam para

class DepthCamera
{
public:
    typedef std::shared_ptr<DepthCamera> Ptr;
    //parameter
    enum TYPEOFCAMERA cam_type;
    double cam0_fx;
    double cam0_fy;
    double cam0_cx;
    double cam0_cy;
    Mat3x3 cam0_matrix;
    cv::Mat K0,D0;
    double cam_scale_factor;

    DepthCamera();
    void setDepthCamInfo(double fx, double fy, double cx, double cy, double scale_factor=1);
    static Vec3 world2cameraT_c_w( const Vec3& p_w, const SE3& T_c_w );
    static Vec3 camera2worldT_c_w( const Vec3& p_c, const SE3& T_c_w );
    Vec2 camera2pixel( const Vec3& p_c );
    Vec3 pixel2camera( const Vec2& p_p, double depth=1 );
    Vec3 pixel2worldT_c_w( const Vec2& p_p, const SE3& T_c_w, double depth=1 );
    Vec2 world2pixelT_c_w( const Vec3& p_w, const SE3& T_c_w );

    double cam1_fx;
    double cam1_fy;
    double cam1_cx;
    double cam1_cy;
    Mat3x3 cam1_matrix;
    cv::Mat K1,D1;
    SE3    T_cam0_cam1;
    SE3    T_cam1_cam0;
    void setSteroCamInfo(const cv::Mat cam0_camera_matrix_in, const cv::Mat cam0_distCoeffs_in,
                         const cv::Mat cam1_camera_matrix_in, const cv::Mat cam1_distCoeffs_in,
                         const SE3 T_cam0_cam1_in);

private:

};

#endif // DEPTHcam0_H
