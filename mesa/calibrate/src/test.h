/*
 * David Z, Apr 15, 2015 
 * 
 * all the collected test functions to probe the camera model of swiss ranger 
 * including the extrinsic matrix [R|t] convert (X,Y,Z) -> (-X, -Y, Z+0.01m)
 * and, the intrinsic matrix A[fx, fy, cx, cy, 1], 
 * and, the distortion parameters, k1, k2, p1, p2, k3, k4, k5, k6, 
 *
 * */

#ifndef TEST_H
#define TEST_H

#include <vector>
#include <iostream>
#include <cmath>
#include "libMesaSR.h"
#include "opencv2/opencv.hpp"
#include <fstream>

using namespace cv; 
using namespace std; 

#define SQ(x) ((x)*(x))

class CamModel
{
  public:
    CamModel(Mat& mat)
    {
      _toCam(mat);
    }
    CamModel(){}
    CamModel(double fx1, double fy1, double cx1, double cy1, double k11, double k22, double k33, double k44): 
    fx(fx1), fy(fy1), cx(cx1), cy(cy1), k1(k11), k2(k22), k3(k33), k4(k44)
  {}
    void _toMat(Mat& mat)
    {
       double * _cm = (double*)mat.ptr(); 
      // camera matrix, fx, cx, fy, cy, 
      _cm[0] = fx; _cm[2] = cx; 
      _cm[4] = fy; _cm[5] = cy; 
      _cm[8] = 1.0; 
    }
    void _toCam(Mat& mat)
    {
      double * _cm = (double*)mat.ptr(); 
      fx = _cm[0]; cx = _cm[2] ; 
      fy = _cm[4]; cy = _cm[5] ; 
    }
    double fx; 
    double fy; 
    double cx; 
    double cy; 
    double k1; 
    double k2; 
    double k3; 
    double k4;
};

double computeReprojectionErrors(
    const vector<vector<Vec3f> >& objectPoints,
    const vector<vector<Vec2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors );

double projectPointsError(InputArray _opoints,
    InputArray _rvec,
    InputArray _tvec,
    InputArray _cameraMatrix,
    InputArray _distCoeffs,
    OutputArray _ipoints);

double computeReprojectionErrors_new(
    const vector<vector<Vec3f> >& objectPoints,
    const vector<vector<Vec2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors 
    );

void getSRParams(SRCAM cam, Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rvec, cv::Mat& tvec);
void testCalibrate(SRCAM, Mat& cam_M);
void generateRandomFrame(SRCAM, vector<float>& xyz);
void genrateRandomCam(SRCAM);
void getXYZ(SRCAM cam, vector<float>& xyz);

void testCamModel(SRCAM, Mat& cameraMatrix);

// try to find a way to compute XYZ from distance, actually SR_CoordTrfUint16 return the z value, surprise,
void testDis2XYZ(SRCAM);

// try to probe the relationship between the intensity value of the img and the distance value
// R(x,y) = 0.3551, seems not linear dependent
void testDisandIntensity(SRCAM);

// 
void testCamModelProj(SRCAM, CamModel& );

double cost_function(SRCAM, CamModel& );

// using opencv calibrate to compute the camera model, let' go 
void testOpenCVCalibrate(SRCAM);
void calibrateOpenCV(SRCAM cam, CamModel& ini, CamModel& out);
void transformXYZ(vector<float>& xyz, vector<float>& xyz_t);

#endif
