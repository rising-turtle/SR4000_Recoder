
#include "test.h"


int main(int argc, char* argv[])
{
  SRCAM srCam; 
  // SR_OpenETH(&srCam, "192.168.0.11"); // open camera 
  SR_OpenETH(&srCam, "192.168.1.4"); // open camera 
  SR_SetIntegrationTime(srCam, 30); 
  // SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP); 
  SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP | AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF | AM_CONV_GRAY 
              | AM_DENOISE_ANF | AM_MEDIANCROSS );

  // 1, calibrate camera model 
  // Mat cam_M; 
  // testCalibrate(srCam, cam_M); 

  // 2, test this model
  // testCamModel(srCam, cam_M);
 
  // 3 test the distance to XYZ model 
  testDis2XYZ(srCam);

  // 4 test the relationship between Distance and Values 
  // testDisandIntensity(srCam); 
  //
  // 5 
  // testCamModelProj(srCam);
  
  // 6  using opencv calibrate to compute the camera model, let' go 
  // testOpenCVCalibrate(srCam);
  
  cout<<"finished!"<<endl;

  return 0;
}


