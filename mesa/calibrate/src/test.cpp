/*
 * David Z, Apr 15, 2015 
 * 
 * all the collected test functions to probe the camera model of swiss ranger 
 * including the extrinsic matrix [R|t] convert (X,Y,Z) -> (-X, -Y, Z+0.01m)
 * and, the intrinsic matrix A[fx, fy, cx, cy, 1], 
 * and, the distortion parameters, k1, k2, p1, p2, k3, k4, k5, k6, 
 *
 * */

#include "test.h"

double computeReprojectionErrors(
    const vector<vector<Vec3f> >& objectPoints,
    const vector<vector<Vec2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors )
{
  vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
        cameraMatrix, distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }

  return std::sqrt(totalErr/totalPoints);
}

double projectPointsError(InputArray _opoints,
    InputArray _rvec,
    InputArray _tvec,
    InputArray _cameraMatrix,
    InputArray _distCoeffs,
    OutputArray _ipoints)
{
  Mat opoints = _opoints.getMat();
  int npoints = opoints.checkVector(3), depth = opoints.depth();
  CV_Assert(npoints >= 0 && (depth == CV_32F || depth == CV_64F));

  _ipoints.create(npoints, 1, CV_MAKETYPE(depth, 2), -1, true);
  CvMat c_imagePoints = _ipoints.getMat();
  CvMat c_objectPoints = opoints;
  Mat cameraMatrix = _cameraMatrix.getMat();

  Mat rvec = _rvec.getMat(), tvec = _tvec.getMat();
  CvMat c_cameraMatrix = cameraMatrix;
  CvMat c_rvec = rvec, c_tvec = tvec;

  double dc0buf[5]={0};
  Mat dc0(5,1,CV_64F,dc0buf);
  Mat distCoeffs1 = _distCoeffs.getMat();
  if( distCoeffs1.empty() )
    distCoeffs1 = dc0;
  CvMat c_distCoeffs = distCoeffs1;
  int ndistCoeffs = distCoeffs1.rows + distCoeffs1.cols - 1;

  // cvProjectPoints2( &c_objectPoints, &c_rvec, &c_tvec, &c_cameraMatrix, &c_distCoeffs,
  // &c_imagePoints, pdpdrot, pdpdt, pdpdf, pdpdc, pdpddist, aspectRatio );

  Ptr<CvMat> matM, _m;
  int i, j, count;
  int calc_derivatives;
  const CvPoint3D64f* M;
  CvPoint2D64f* m;
  double r[3], R[9], dRdr[27], t[3], a[9], k[8] = {0,0,0,0,0,0,0,0}, fx, fy, cx, cy;
  CvMat _r, _t, _a = cvMat( 3, 3, CV_64F, a ), _k;
  CvMat matR = cvMat( 3, 3, CV_64F, R ), _dRdr = cvMat( 3, 9, CV_64F, dRdr );
  
  CvMat* objectPoints = &c_objectPoints;
  int total = c_objectPoints.rows * c_objectPoints.cols * CV_MAT_CN(c_objectPoints.type);
  count = total / 3;
  matM = cvCreateMat( objectPoints->rows, objectPoints->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(objectPoints->type)) );
  cvConvert(objectPoints, matM);

  CvMat* imagePoints = &c_imagePoints;
  _m = cvCreateMat( imagePoints->rows, imagePoints->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(imagePoints->type)) );
  cvConvert(imagePoints, _m);

  M = (CvPoint3D64f*)matM->data.db;
  m = (CvPoint2D64f*)_m->data.db;

  CvMat* r_vec = &c_rvec;
  CvMat* t_vec = &c_tvec;
  CvMat* A = &c_cameraMatrix;
  CvMat* distCoeffs = &c_distCoeffs;

  if( r_vec->rows == 3 && r_vec->cols == 3 )
  {
    _r = cvMat( 3, 1, CV_64FC1, r );
    cvRodrigues2( r_vec, &_r );
    cvRodrigues2( &_r, &matR, &_dRdr );
    cvCopy( r_vec, &matR );
  }else{
    _r = cvMat( r_vec->rows, r_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(r_vec->type)), r );
    cvConvert( r_vec, &_r );
    cvRodrigues2( &_r, &matR, &_dRdr );
  }

  _t = cvMat( t_vec->rows, t_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(t_vec->type)), t );
  cvConvert( t_vec, &_t );
  cvConvert( A, &_a );
  fx = a[0]; fy = a[4];
  cx = a[2]; cy = a[5];

  if( distCoeffs )
  {
    _k = cvMat( distCoeffs->rows, distCoeffs->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), k );
    cvConvert( distCoeffs, &_k );
  }
  for( i = 0; i < count; i++ )
  {
    double X = M[i].x, Y = M[i].y, Z = M[i].z;
    double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
    double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
    double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
    double r2, r4, r6, a1, a2, a3, cdist, icdist2;
    double xd, yd;

    z = z ? 1./z : 1;
    x *= z; y *= z;

    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
    icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
    xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
    yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;

    m[i].x = xd*fx + cx;
    m[i].y = yd*fy + cy;
  }
  if( _m != imagePoints )
    cvConvert( _m, imagePoints );
}

double computeReprojectionErrors_new(
    const vector<vector<Vec3f> >& objectPoints,
    const vector<vector<Vec2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors 
    )
{
  vector<Vec2f> imagePoints2; 
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());
 
  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    /*projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
        cameraMatrix, distCoeffs, imagePoints2);*/
    projectPointsError(Mat(objectPoints[i]), rvecs[i], tvecs[i],
        cameraMatrix, distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
 return std::sqrt(totalErr/totalPoints);
}

// 6  using opencv calibrate to compute the camera model, let' go 
void testOpenCVCalibrate(SRCAM cam)
{
   // 1, test the initial error function 
   CamModel ini_model;
   testCamModelProj(cam, ini_model);
   
   // 2, using this ini_model as input to calibrate camera using opencv library 
   CamModel cal_model; 
   calibrateOpenCV(cam, ini_model, cal_model); 
   cout<<"mesaCalibrate.cpp: using the calibrated model: "<<endl;
   cost_function(cam, cal_model); 

   // compare using another data 
   genrateRandomCam(cam); 
   cout<<"mesaCalibrate.cpp: comapare with another data: "<<endl
     <<"first ini_model: "<<endl;
   cost_function(cam, ini_model);
   cout<<"second cal_model: "<<endl;
   cost_function(cam, cal_model);
   return ;
}
 

double cost_function(SRCAM cam, CamModel& model)
{
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam);
  vector<float> xyz; 
  getXYZ(cam, xyz);
  
  double x,y,z;
  unsigned int i; 
  ofstream comp_xy("comp_xy.log"); 
  float ux, vy, xd, yd; 
  double r, D; 

  // error statistics 
  double err_sum_x = 0;
  double err_sum_y = 0;
  double err_sum = 0;
  unsigned int num = 0;

  double z_offset = 0.01; // SR 4000, 1cm
  CamModel& m = model;

  for(int v=0; v < rows; v++)
  {
    for(int u= 0; u<cols; u++)
    {
      i = v*cols + u;
      
      x = xyz[3*i]; 
      y = xyz[3*i+1];
      z = xyz[3*i+2]; 
      if(abs(z) < 0.01 || z<0)
        continue;
      ++num;
      
      double x1 = -x; 
      double y1 = -y; 
      double z1 = z + z_offset;

      double rx = x1/z1; 
      double ry = y1/z1; 
  
      double r2 = SQ(rx) + SQ(ry); 
      double r4 = SQ(r2); 

      ux = m.cx + m.fx*(rx*(1+m.k1*r2 + m.k2*r4 + 2*m.k3*rx*ry + m.k4*(r2+2*SQ(rx))));
      vy = m.cy + m.fy*(ry*(1+m.k1*r2 + m.k2*r4 + 2*m.k4*rx*ry + m.k3*(r2+2*SQ(ry)))); 
      
      comp_xy<< u <<", "<<v<<" "<<ux<<", "<<vy<<" x y z: "<<x<<" "<<y<<" "<<z<<endl;
      err_sum_x += SQ(ux - u); 
      err_sum_y += SQ(vy - v); 
      err_sum = err_sum_x + err_sum_y; 
    }
  }
  
  cout<<"err_sum_x: "<<err_sum_x<<" err_sum_y: "<<err_sum_y<<endl
    <<"err_sum: "<<err_sum<<" mean_square_error: "<<err_sum/num<<endl;
  return err_sum/num;
}

void testCamModelProj(SRCAM cam, CamModel& model_out)
{
  // randomly generate a frame
  
  // get a frame 
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  int total = rows*cols;

  // this random generate or as 1,2 
  vector<float> xyz; 
  generateRandomFrame(cam, xyz);

  // 1, get image pointer and fill with random values 
  // unsigned short* values = (unsigned short*)SR_GetImage(cam, 0);
  // const int s = 3*sizeof(float); 
  // xyz.resize(rows*cols*3, 0); 

  // 2, convert to an XYZ image 
  // SR_CoordTrfFlt(cam, &xyz[0], &xyz[1], &xyz[2], s, s, s); 
 
  // mrpt default parameters, 
  
  double fx = 262.9201; // 250.21; 
  double fy = 262.9218; //250.21; 
  double cx = 87.99958; // 87.23; 
  double cy = 68.99957; // 69.64; 
  double k1 = -8.258543e-01; //2.525;
  double k2 = 6.561022e-01; // -1.117;
  double k3 = 2.699818e-04;
  double k4 = -3.263559e-05;
  
/*
  // jen's program 
  double fx = 250.2105; // 250.21; 
  double fy = 250.2105; //250.21; 
  double cx = 87.2255; // 87.23; 
  double cy = 69.6392; // 69.64; 
  double k1 = 36.9046; //2.525;
  double k2 = -33.31925; // -1.117;
  double k3 = -8.6899547e-05;
  double k4 = -4.9603016e-05;
*/
  CamModel model(fx, fy, cx, cy, k1, k2, k3, k4); 
  cost_function(cam, model);
  model_out = model;
}


template<typename T>
void computeMeanSigma(T* ptr, int n, float& mean, double& sigma)
{
  double sum = 0; 
  T* p = ptr;
  for(int i=0; i<n; i++)
  {
    T tmp = *p; 
    sum += tmp; 
    ++p; 
  }
  mean = sum/(float)(n); 

  p = ptr; 
  sum = 0; 
  for(int i=0; i<n; i++)
  {
    T tmp = *p; 
    sum += SQ((double)(tmp - mean));
    ++p;
  }
  sigma = sum/(double)(n-1);
}

template<typename T>
void computeEXY(T* pX, T* pY, int n, double& EXY)
{
  double sum = 0; 
  T* p1 = pX; 
  T* p2 = pY;
  for(int i=0; i<n; i++)
  {
    T tmp1 = *p1;
    T tmp2 = *p2;
    sum += double(tmp1*tmp2); 
    ++p1; ++p2; 
  }
  EXY = sum/(double)(n); 
}

void testDisandIntensity(SRCAM cam)
{
  // get a frame 
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  int total = rows*cols;
  vector<float> xyz; 
  
  unsigned short* values = (unsigned short*)SR_GetImage(cam, 0);
  RNG rng; 
  Mat _values(rows, cols, CV_16UC1, values); 
  // rng.fill(_values, RNG::UNIFORM, Scalar::all(10), Scalar::all(65000)); 
  rng.fill(_values, RNG::UNIFORM, Scalar::all(10), Scalar::all(16382)); 

  // get the original distance value 
  vector<short> ux (total, 0); 
  vector<short> uy (total, 0); 
  vector<unsigned short> d(total, 0); 
  const int s1 = sizeof(short); 
  SR_CoordTrfUint16(cam, &ux[0], &uy[0], &d[0], s1, s1, s1);
  
  // compute mean and sigma EX EY, VX VY
  float EX, EY; 
  double VX, VY; 
  unsigned short* pX = values;
  unsigned short* pY = (unsigned short*)d.data();
  computeMeanSigma<unsigned short>(pX, total, EX, VX); 
  computeMeanSigma<unsigned short>(pY, total, EY, VY); 
  
  // compute Cov(X,Y) = E[XY] - EX*EY; 
  double EXY = 0;
  computeEXY<unsigned short>(pX, pY, total, EXY); 
  
  cout<<"statistics: EX "<<EX<<" EY: "<<EY<<" VX "<<VX<<" VY "<<VY<<" EXY "<<EXY<<endl;
  // 
  double r = (EXY-EX*EY)/(sqrt(VX*VY)); 
  cout<<"corr(X,Y) = "<<r<<endl;

  ofstream OX("X.txt"); 
  ofstream OY("Y.txt"); 
  unsigned short * p1 = pX; 
  unsigned short * p2 = pY; 
  for(int i=0; i<total; i++)
  {
    OX<<*p1++<<endl;
    OY<<*p2++<<endl;
  }
}

void calculateXYZ(float xyz[3], short ux, short vy, unsigned int mmd, float c, float cx, float cy)
{
  float Xq = cx - ux; 
  float Yq = cy - vy;
  float r = 1./sqrt(SQ(Xq) + SQ(Yq) + SQ(c));
  float d = mmd * 0.001; // mm -> m
  xyz[0] = Xq*d*r - Xq; 
  xyz[1] = Yq*d*r - Yq; 
  xyz[2] = c*d*r; 
}

void testDis2XYZ(SRCAM cam)
{
  // get a frame 
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  int total = rows*cols;

  // 1, generate a frame, calculate using the 
  unsigned short* values = (unsigned short*)SR_GetImage(cam, 0); // this is distance 
  
  cout<<"mesaCalibrate.cpp: testDis2XYZ, rows: "<<rows<<" cols: "<<cols<<endl;

  RNG rng; 
  Mat _values(rows, cols, CV_16UC1, values); 
  rng.fill(_values, RNG::UNIFORM, Scalar::all(10), Scalar::all(16382)); 
  
  // vector<unsigned short> tmp_buf(total, 10); 
  // memcpy(values, tmp_buf.data(), total*sizeof(WORD));
  
  // get the original distance value 
  vector<short> ux (total, 0); 
  vector<short> uy (total, 0); 
  vector<unsigned short> d(total, 0); 
  const int s1 = sizeof(short); 
  SR_CoordTrfUint16(cam, &ux[0], &uy[0], &d[0], s1, s1, s1);

  // using the disfunction 
  vector<unsigned char> ix(total, 0); 
  vector<unsigned char> iy(total, 0); 
  vector<unsigned short> iDst(total, 0); 
  unsigned short* pD = values; 
  
  vector<short> tx = ux; vector<short> ty = uy; vector<unsigned short > tz = d;

  int k=0; 
  for(int j=0; j<rows; j++)
    for(int i=0; i<cols; i++)
    { 
      iy[k] = j;
      ix[k] = i;
      iDst[k] = pD[j*cols +i]; 
      ++k;
    }
  SR_CoordTrfPntUint16(cam, &ix[0], &iy[0], &iDst[0], &tx[0], &ty[0], &tz[0], total);

  ofstream comp_f("comp_out.txt");
  for(int j=0; j<rows; j++)
    for(int i=0; i<cols; i++)
    {
      k = j*cols + i; 
      comp_f<<ux[k]<<"\t"<<uy[k]<<"\t"<<d[k]<<"\t"<<tx[k]<<"\t"<<ty[k]<<"\t"<<tz[k]<<endl;
    }

  /*
  // try the phi, not work  
  unsigned short *p = values; 
  unsigned short max_shift = 1<<14; // 2^14 
  unsigned short dmax = 4999; // 5m
  for(int i=0; i<total; i++)
  {
    unsigned short tmp = *p++; 
    d[i] = tmp*dmax/max_shift;
  }
  */

  /*
  // get the converted XYZ 
  const int s = 3*sizeof(float); 
  vector<float> xyz(rows*cols*3, 0);
  SR_CoordTrfFlt(cam, &xyz[0], &xyz[1], &xyz[2], s, s, s); 
  Vec3f * ptr = (Vec3f*)&xyz[0];

  // compare with the calculated one 
  float xyz3[3];
  ofstream comp_f("comp_out.txt");
  
  // camera model
  float c = 250.21; 
  float cx = 87.23; 
  float cy = 69.64;

  int i=0, j, k, index; 
  unsigned char ix ; 
  unsigned char iy ; 

  for(j=0; j<rows; j++)
    for(k=0; k<cols; k++, i++)
    {
      index = j*cols + k; 
      Vec3f pt_obs = ptr[index]; 
      // Vec3f pixel = Vec3f((float)k + 0.5f, (float)j + 0.5f, 1.0); 
      // calculateXYZ(xyz3, ux[index], uy[index], d[index], c, cx, cy);
      
      ix = k; 
      iy = j;
      // SR_CoordTrfPntFlt(cam, &ix, &iy, &d[index], &xyz3[0], &xyz3[1], &xyz3[2], 1);
      // try the API function 
      comp_f<<pt_obs<<"\t"<<d[index]<<"\t"<<xyz3[0]<<"\t"<<xyz3[1]<<"\t"<<xyz3[2]<<endl;
    }*/
}

void testCamModel(SRCAM srCam, Mat& cameraMatrix)
{
  vector<float> xyz; 
  generateRandomFrame(srCam, xyz); 

  int rows = SR_GetRows(srCam); 
  int cols = SR_GetCols(srCam);

  Vec3f *ptr = (Vec3f*)&xyz[0]; 
  Mat inv_cam = cameraMatrix; // cameraMatrix.inv();
  
  double fx = 1./cameraMatrix.at<double>(0,0);
  double fy = 1./cameraMatrix.at<double>(1,1); 
  double cx = cameraMatrix.at<double>(0,2); 
  double cy = cameraMatrix.at<double>(1,2);
  
  cout<<"calibrate: fx: "<<fx<<" fy: "<<fy<<endl
    <<" cx: "<<cx<<" cy: "<<cy<<endl;

  float x, y, z;
  ofstream comp_f("comp_out.txt");

  int i=0, j, k, index; 
  for(j=0; j<rows; j++)
    for(k=0; k<cols; k++, i++)
    {
      index = j*cols + k; 
      Vec3f pt_obs = ptr[index]; 
      Vec3f pixel = Vec3f((float)k + 0.5f, (float)j + 0.5f, 1.0); 
      // Mat pt_est = inv_cam*Mat(pixel);
      z = pt_obs(2); 
      x = (pixel(0)-cx)*z*fx; 
      y = (pixel(1)-cy)*z*fy;
      comp_f<<pt_obs<<"\t"<<x<<"\t"<<y<<"\t"<<z<<endl;
    }
  return ;
}

void getXYZ(SRCAM cam, vector<float>& xyz)
{
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  const int s = 3*sizeof(float); 
  xyz.resize(rows*cols*3, 0); 
  // 2, convert to an XYZ image 
  SR_CoordTrfFlt(cam, &xyz[0], &xyz[1], &xyz[2], s, s, s); 
}

void genrateRandomCam(SRCAM cam)
{
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 

  // 1, get image pointer and fill with random values 
  unsigned short* values = (unsigned short*)SR_GetImage(cam, 0);
  RNG rng; 
  Mat _values(rows, cols, CV_16UC1, values); 
  rng.fill(_values, RNG::UNIFORM, Scalar::all(10), Scalar::all(16382)); 
  return ;
}

void generateRandomFrame(SRCAM cam, vector<float>& xyz)
{
  // 1 random cam image
  genrateRandomCam(cam);

  // 2 getXYZ data
  getXYZ(cam, xyz);
}

void testCalibrate(SRCAM cam, Mat& cam_M)
{
  Mat cameraMatrix; 
  Mat distCoeffs; 
  Mat rvec; 
  Mat tvec; 
  
  getSRParams(cam, cameraMatrix, distCoeffs, rvec, tvec); 
  ofstream outf("output.txt"); 
  outf<<"camera matrix: "<<endl<<cameraMatrix<<endl
    <<"distorion coefficients: "<<endl<<distCoeffs<<endl
    <<"translation offset: "<<endl<<tvec<<endl
    <<"rotation vector: "<<endl<<rvec<<endl; 
  cam_M = cameraMatrix; 
}

void transformXYZ(vector<float>& xyz, vector<float>& xyz_t)
{
  int total = 144*176;
  assert(xyz.size() == 3*total);
  xyz_t = xyz; 
  for(int i=0; i<total; i++)
  {
    float tx = xyz[i*3]; 
    float ty = xyz[i*3+1]; 
    float tz = xyz[i*3+2]; 
    xyz_t[i*3] = -tx; 
    xyz_t[i*3+1] = -ty; 
    xyz_t[i*3+2] = tz + 0.01; // z_offset = 1cm = 0.01m
  }
}

void calibrateOpenCV(SRCAM cam, CamModel& ini_model, CamModel& out_model)
{
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  int total = rows*cols;

  vector<float> xyz; 
  // generateRandomFrame(cam, xyz);
  getXYZ(cam, xyz);

  // 3, transform camera model 
  vector<float> xyz_t; 
  transformXYZ(xyz, xyz_t);
  
  // 4, initialization 
  Vec3f *ptr = (Vec3f*)&xyz_t[0]; 
  vector<Vec3f> objectPoints(rows*cols); 
  vector<Vec2f> imagePoints(rows*cols);
  vector<Vec2f> normalizedImPointsX(rows*cols);
  vector<Vec2f> normalizedImPointsY(rows*cols);

  int i=0, j, k, index; 
  for(j=0; j<rows; j++)
    for(k=0; k<cols; k++, i++)
    {
      index = j*cols + k; 
      objectPoints[i] = ptr[index]; 
      imagePoints[i] = Vec2f((float)k + 0.5f, (float)j + 0.5f); 
      normalizedImPointsX[i] = Vec2f(ptr[index][0]/ptr[index][2], imagePoints[i][0]); 
      normalizedImPointsY[i] = Vec2f(ptr[index][1]/ptr[index][2], imagePoints[i][1]); 
    }

  Mat cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0)); 
  ini_model._toMat(cameraMatrix);
  const int distCoeffsNum = 8; 
  Mat distCoeffs = Mat(1, distCoeffsNum, CV_64FC1, Scalar::all(0)); 
  distCoeffs.at<double>(0,0) = ini_model.k1; distCoeffs.at<double>(0,1) = ini_model.k2; 
  distCoeffs.at<double>(0,2) = ini_model.k3; distCoeffs.at<double>(0,3) = ini_model.k4;
  Mat rvec = Mat(1, 3, CV_32FC1, Scalar::all(0)); 
  Mat tvec = Mat(1, 3, CV_32FC1, Scalar::all(0));

  // 4, calibrate 
  vector<vector<Vec3f> > _op(1, objectPoints); 
  vector<vector<Vec2f> > _ip(1, imagePoints); 
  vector<Mat> _rvec, _tvec; 

  // this is not the right function to call, weather to use the calibrated extrinsic model, we will see 
  // calibrateCamera(_op, _ip, Size(cols, rows), cameraMatrix, distCoeffs, _rvec, _tvec, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_RATIONAL_MODEL); 
  // calibrateCamera(_op, _ip, Size(cols, rows), cameraMatrix, distCoeffs, _rvec, _tvec, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6); 
  rvec = _rvec[0]; 
  tvec = _tvec[0]; 

  // 5, put it to output model 
  out_model._toCam(cameraMatrix); 
  out_model.k1 = distCoeffs.at<double>(0,0); out_model.k2 = distCoeffs.at<double>(0,1); 
  out_model.k3 = distCoeffs.at<double>(0,3); out_model.k4 = distCoeffs.at<double>(0,4);
 
  return ;
}

void getSRParams(SRCAM cam, Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rvec, cv::Mat& tvec)
{
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  int total = rows*cols;

  // 1,2 in this below function
  vector<float> xyz; 
  generateRandomFrame(cam, xyz);
 
  // 3, transform camera model, this is actually extrinsic matrix [R|t], leave it to OpenCV
  /*vector<float> xyz_t = xyz; 
  for(int i=0; i<total; i++)
  {
    float tx = xyz[i*3]; 
    float ty = xyz[i*3+1]; 
    float tz = xyz[i*3+2]; 
    xyz_t[i*3] = -tx; 
    xyz_t[i*3+1] = -ty; 
    xyz_t[i*3+2] = tz + 0.01; // z_offset = 1cm = 0.01m
  }*/

  // 3, initialize camera matrix 
  Vec3f *ptr = (Vec3f*)&xyz[0]; 
  // Vec3f *ptr = (Vec3f*)&xyz_t[0]; 

  vector<Vec3f> objectPoints(rows*cols); 
  vector<Vec2f> imagePoints(rows*cols);
  vector<Vec2f> normalizedImPointsX(rows*cols);
  vector<Vec2f> normalizedImPointsY(rows*cols);

  int i=0, j, k, index; 
  for(j=0; j<rows; j++)
    for(k=0; k<cols; k++, i++)
    {
      index = j*cols + k; 
      objectPoints[i] = ptr[index]; 
      imagePoints[i] = Vec2f((float)k + 0.5f, (float)j + 0.5f); 
      normalizedImPointsX[i] = Vec2f(ptr[index][0]/ptr[index][2], imagePoints[i][0]); 
      normalizedImPointsY[i] = Vec2f(ptr[index][1]/ptr[index][2], imagePoints[i][1]); 
    }
  
  double p=0, reps = 0.01, aeps = 0.01, ax, bx, ay, by; 
  Vec4f linex, liney; 
  fitLine(normalizedImPointsX, linex, CV_DIST_L2, 0, reps, aeps); 
  fitLine(normalizedImPointsY, liney, CV_DIST_L2, 0, reps, aeps); 
  ax = linex[1]/linex[0]; bx = cols/2; 
  ay = liney[1]/liney[0]; by = rows/2; 
  cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0)); 
  double * _cm = (double*)cameraMatrix.ptr(); 

  // camera matrix, fx, cx, fy, cy, 
  _cm[0] = fabs(ax); _cm[2] = fabs(bx); 
  _cm[4] = fabs(ay); _cm[5] = fabs(by); 
  _cm[8] = 1.0; 

 // _cm[0] = ax; _cm[2] = bx; 
 // _cm[4] = ay; _cm[5] = by; 
 // _cm[8] = 1.0; 

  cout<<"initial guess: fx: "<<_cm[0]<<" fy: "<<_cm[4]<<endl
    <<" cx: "<<_cm[2]<<" cy: "<<_cm[5]<<endl;

  const int distCoeffsNum = 8; 
  distCoeffs = Mat(1, distCoeffsNum, CV_64FC1, Scalar::all(0)); 
  rvec = tvec = Mat(1, 3, CV_32FC1, Scalar::all(0)); 

  // 4, calibrate 
  vector<vector<Vec3f> > _op(1, objectPoints); 
  vector<vector<Vec2f> > _ip(1, imagePoints); 
  vector<Mat> _rvec, _tvec; 
  // calibrateCamera(_op, _ip, Size(cols, rows), cameraMatrix, distCoeffs, _rvec, _tvec, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_RATIONAL_MODEL ); 
  calibrateCamera(_op, _ip, Size(cols, rows), cameraMatrix, distCoeffs, _rvec, _tvec, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6); 

  rvec = _rvec[0]; 
  tvec = _tvec[0]; 
  
  vector<float> perview_err;
  // in opencv
  double err_sq_mean = computeReprojectionErrors(_op, _ip, _rvec, _tvec, cameraMatrix, distCoeffs, perview_err);
  cout<<"mesaCalibrate.cpp: OpenCV calibrate sq_mean_error method1 : "<<err_sq_mean<<endl;
  err_sq_mean = computeReprojectionErrors_new(_op, _ip, _rvec, _tvec, cameraMatrix, distCoeffs, perview_err);
  cout<<"mesaCalibrate.cpp: OpenCV calibrate sq_mean_error method2 : "<<err_sq_mean<<endl; 

  /*
  // test the 
  double fx = cameraMatrix.at<double>(0,0); 
  double fy = cameraMatrix.at<double>(1,1); 
  double cx = cameraMatrix.at<double>(0,2); 
  double cy = cameraMatrix.at<double>(1,2);
  
  double err_sum = 0;
  ofstream comp_f("comp_f2.txt"); 
  i = 0;
  float ux, uy; 
  for(j=0; j<rows; j++)
    for(k=0; k<cols; k++, i++)
    {
      index = j*cols + k; 
      // objectPoints[i] = ptr[index]; 
      // imagePoints[i] = Vec2f((float)k + 0.5f, (float)j + 0.5f); 

      ux = -fx*normalizedImPointsX[i][0]  + cx; 
      uy = -fy*normalizedImPointsY[i][0]  + cy; 
      comp_f<<imagePoints[i][0]<<" "<<imagePoints[i][1]<<" "<<ux<<" "<<uy<<endl;
      // normalizedImPointsX[i] = Vec2f(ptr[index][0]/ptr[index][2], imagePoints[i][0]); 
      // normalizedImPointsY[i] = Vec2f(ptr[index][1]/ptr[index][2], imagePoints[i][1]); 
    }
  */
}



  /* mesa memcpy way fail!
  SRCAM srCam2; 
  int mesaDevice_size = sizeof(CMesaDevice); 
  cout<<"mesaCalibrate.cpp: mesaDevice size: "<<mesaDevice_size<<endl;

  vector<unsigned char> buf(mesaDevice_size, 0); 
  memcpy(buf.data(), &(*srCam), buf.size()); 
  // srCam2 = reinterpret_cast<SRCAM>(buf.data());

  // memcpy(&srCam2, buf.data(), buf.size());
  // memcpy(&srCam2, &srCam, sizeof(SRCAM));
  
  // try virtual camera method 
  

  // 1, write a stream 
  // const char * tmp_file = "whatever_i_am_a_midian.srs"; 
  // SR_StreamToFile(srCam, tmp_file, 0); // mode, 0 create a new file, 1 append, 2 close
  
  // 2, open it with the new cam handle
  // SR_OpenFile(srCam2, tmp_file);

  // delete the first cam handle
  // SR_Close(srCam);

  // SR_OpenDlg (SRCAM*, mode, HWND), , registry way only works in windows
  // //!the <tt>mode</tt> is used to select how the dialog is handled<br>
  //!the mode bits are:
  //! - 0 use registry if existing: tries to open the same device as last time
  //! - 1 open dialog
  //! - 2 open without configuration (internal usage only)
  //!
  //!so following modes make sense:
  //! - 1 use registry, if it failed error is returned, no GUI opened
  //! - 2 always open dialog
  //! - 3 use registry, if it failed open the GUI dialog
  // SR_OpenDlg(&srCam2, 2, 0); 
  */

