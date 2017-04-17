#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/io/pcd_io.h"

#include "libMesaSR.h"

#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <vector>
#include <typeinfo>
#include "plane_extract.h"

using namespace std; 

typedef pcl::PointXYZ point_type; 
typedef pcl::PointCloud<point_type> cloud_type; 
typedef typename cloud_type::Ptr  cloudPtr; 

#define S2F 0.001

void showCam(SRCAM);

void testBase(SRCAM);

void showCloud(cloudPtr&);

template<typename T, typename T2>
void genCloud(cloudPtr&, int n, T* x, T* y, T2* z);


bool openSR(SRCAM& cam, string mode) // USB first, TODO: add tcp SR 
{ 
  int ret ; 
  if(mode == string("TCP"))
  {
    // ret = SR_OpenETH(&cam, "192.168.0.11"); // open camera 
    ret = SR_OpenETH(&cam, "192.168.1.42"); // open camera 
    cout<<"pcl_mesa.cpp: open camera using TCP"<<endl;
  }else if(mode == string("USB"))
  {
    ret = SR_OpenUSB(&cam, 0x4000397); 
    cout<<"pcl_mesa.cpp: open camera using USB"<<endl;
  }else{
    ret = SR_OpenUSB(&cam, 0x4000397); 
    cout<<"pcl_mesa.cpp: unknown model: "<<mode<<" using USB default!"<<endl;
  }
  if(ret <= 0)
  {
    cout<<"test_writer.cpp: failed to open SRCAM"<<endl; 
    return false;
  }
  SR_SetIntegrationTime(cam, 30); 
  SR_SetMode(cam, AM_HW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN 
      | AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS); 
  return true;
}

/*
void openSR(SRCAM& srCam)
{
  SR_OpenETH(&srCam, "192.168.0.11"); // open camera 
  // SR_OpenETH(&srCam, "192.168.1.42"); // open camera 
  SR_SetIntegrationTime(srCam, 30); 
  SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP | AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF | AM_CONV_GRAY 
              | AM_DENOISE_ANF | AM_MEDIANCROSS ); 
}*/

void write2file(SRCAM& );

void test_file();

int main(int argc, char* argv[])
{
  string mode = "USB"; 
  if(argc >= 2)
  {
    mode = argv[1];
  }
  
  SRCAM srCam; 
  openSR(srCam, mode);
 
  // SR_OpenETH(&srCam, "192.168.0.11"); // open camera 
  // SR_SetIntegrationTime(srCam, 30); 
  // SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP | AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF | AM_CONV_GRAY); 

  SR_Acquire(srCam); 
  SR_Close(srCam);
  // SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP);

  openSR(srCam, mode);

  // testBase(srCam); 
  
  showCam(srCam);
  
  SR_Close(srCam);
  
  // test_file(); 

  return 0;
}

void test_file()
{
  string sr_data_file_dir("/home/davidz/work/data/SwissRanger4000/try");
  typedef float SR_TYPE; 
  const int SR_SIZE = 176*144;
  
  vector<float> x_(SR_SIZE); 
  vector<float> y_(SR_SIZE); 
  vector<float> z_(SR_SIZE);
  
  // PCL point cloud 
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
    cloud(new pcl::PointCloud<point_type>());
  cloud->points.resize(SR_SIZE);

  for(int i=1; i<2; i++)
  {
    stringstream ss ; 
    ss<<sr_data_file_dir<<"/d1_"<<setfill('0')<<setw(4)<<i<<".bdat"; 
      FILE* fid = fopen(ss.str().c_str(), "rb"); 
      if(fid == NULL)
      {
        // cout<<"pcl_mesa.cpp: failed to open file: "<<f_name<<endl; 
        return ;
      }else
      {
        cout<<"pcl_mesa.cpp: open file: "<<ss.str()<<endl;
        // old file version 
        fread(&z_[0], sizeof(SR_TYPE), SR_SIZE, fid); 
        fread(&x_[0], sizeof(SR_TYPE), SR_SIZE, fid);
        fread(&y_[0], sizeof(SR_TYPE), SR_SIZE, fid); 
      }
    for(int j=0; j<SR_SIZE; j++)
    {
      point_type& pt = cloud->points[j];
      pt.x = x_[j]; pt.y = y_[j]; pt.z = z_[j]; 
    }
    cout<<"pcl_mesa.cpp: display the "<<i<<" th cloud!"<<endl;
    showCloud(cloud);
  }
  
  // save pcd file 
  cloud->width = SR_SIZE; 
  cloud->height = 1;
  pcl::io::savePCDFile("tmp.pcd", *cloud);

  return ;
}

void record2FileBin(const char* fname, const char* buf, const size_t size)
{
  ofstream ouf_bin(fname, ios::out | ios::binary);
  // ouf_bin.write((char*)&size, sizeof(size_t));
  ouf_bin.write(buf, size);
  ouf_bin.close();
}


void write2file(SRCAM& cam)
{
  static int rows = 144 ; 
  static int cols = 176 ;
  static int count = 0;
  static int total = rows*cols*sizeof(unsigned short);
  static int cam_offset = sizeof(SRCAM);
  // static vector<char> buf(cam_offset + total*2, 0);
  static vector<char> buf(total*2, 0);
  
  // write camera handle first 
  // cam_offset does not work because the SRCAM is a pointer to an enclosed class
  char* pb = &buf[0]; 
  // memcpy(pb, &cam, cam_offset);

  // write short intensity 
  ImgEntry* imgEntryArray;
  int nImg = SR_GetImageList(cam, &imgEntryArray);
  WORD* p = (WORD*)imgEntryArray[1].data;
  // memcpy(pb + cam_offset, p, total); 
  memcpy(pb, p, total);
  
  // write short distance 
  p = (WORD*)imgEntryArray[0].data; 
  // memcpy((pb + cam_offset + total), p, total); 
  memcpy(pb + total, p, total);

  // filename 
  stringstream ss; 
  ss<<"./sr_data/d1_"<<setfill('0')<<setw(4)<<count++<<".bdat"; 
  // record2FileBin(ss.str().c_str(), &buf[0], cam_offset + total*2);
  record2FileBin(ss.str().c_str(), &buf[0], total*2);
}

void testBase(SRCAM cam)
{
  SR_Acquire(cam); 
  
  // 1, set buffer 
  int rows = SR_GetRows(cam); 
  int cols = SR_GetCols(cam); 
  int total = rows*cols; 
  
  int short_s = sizeof(short); 
  int ushort_s = sizeof(unsigned short);
  int float_s = sizeof(float);
  vector<short> xs16(total, 0); vector<short> ys16(total, 0); vector<unsigned short> zs16(total, 0); 
  vector<float> xf32(total, 0); vector<float> yf32(total, 0); vector<float> zf32(total, 0); 
  vector<unsigned char> xc8(total, 0); vector<unsigned char> yc8(total, 0); vector<unsigned short> iDst(total, 0); 
  WORD* dst; 
  // 2, get XYZ using different functions 
  dst = (WORD*)SR_GetImage(cam, 0); // distance map 

  int k = 0;
  for(int j=0; j<rows; j++)
  {
    for(int i=0; i<cols; i++)
      {
        yc8[k] = j; 
        xc8[k] = i;
        iDst[k] = dst[j*cols + i];
        k++;
      }
  }
  
  // 3, get data 
  SR_CoordTrfUint16(cam, &xs16[0], &ys16[0], &zs16[0], short_s, short_s, ushort_s); 
  SR_CoordTrfFlt(cam, &xf32[0], &yf32[0], &zf32[0], float_s, float_s, float_s); 

  // 4, generate cloud 
  cloudPtr pc(new cloud_type);
  genCloud<short, unsigned short>(pc, total, &xs16[0], &ys16[0], &zs16[0]); 
  cout<<"pcl_mesa.cpp: show cloud short!"<<endl;
  showCloud(pc); 

  // 5, using the other function
  SR_CoordTrfPntUint16(cam, &xc8[0], &yc8[0], &iDst[0], &xs16[0], &ys16[0], &zs16[0], total); 
  genCloud<short, unsigned short>(pc, total, &xs16[0], &ys16[0], &zs16[0]); 
  cout<<"pcl_mesa.cpp: show cloud short using dis function!"<<endl;
  showCloud(pc); 

  // similarly 
  genCloud<float, float>(pc, total, &xf32[0], &yf32[0], &zf32[0]);
  cout<<"pcl_mesa.cpp: show cloud float!"<<endl;
  showCloud(pc);

  SR_CoordTrfPntFlt(cam, &xc8[0], &yc8[0], &iDst[0], &xf32[0], &yf32[0], &zf32[0], total); 
  genCloud<float, float>(pc, total, &xf32[0], &yf32[0], &zf32[0]);
  cout<<"pcl_mesa.cpp: show cloud float using dis function!"<<endl;
  showCloud(pc);

  return;
}

template<typename T, typename T2>
void genCloud(cloudPtr& pc, int n, T* x, T* y, T2* z)
{
  pc->points.resize(n);
  T* px = x; T* py = y; T2* pz = z; 
  for(int i=0; i<n; i++)
  {
    point_type & pt = pc->points[i]; 
    if(typeid(T).name() == typeid(short).name())
    {
      pt.x = *px*S2F; 
      pt.y = *py*S2F; 
      pt.z = *pz*S2F;
    }else
    {
      pt.x = *px; 
      pt.y = *py;
      pt.z = *pz;
    }
    ++px; ++py; ++pz;
  }
}


void showCloud(cloudPtr& pc)
{
  // visualization 
  pcl::visualization::CloudViewer viewer("MesaSR PCL Viewer"); 
  while(!viewer.wasStopped())
  {
    viewer.showCloud(pc);
    usleep(30000); // sleep 30 ms 
  }
}


void showCam(SRCAM srCam)
{
  int rows = SR_GetRows(srCam); 
  int cols = SR_GetCols(srCam); 

  // PCL point cloud 
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
    cloud(new pcl::PointCloud<point_type>(rows, cols));

  pcl::PointCloud<pcl::PointXYZ>::Ptr 
    floor(new pcl::PointCloud<point_type>(rows, cols));

  float* ptrXYZ = (float*)(&cloud->front()); 
  int s = sizeof(point_type); 
  
  // visualization 
  pcl::visualization::CloudViewer viewer("MesaSR PCL Viewer"); 
  
  CPlaneExtract<pcl::PointXYZ> planeExtract;
  while(!viewer.wasStopped())
  {
    SR_Acquire(srCam);
    
    write2file(srCam);

    SR_CoordTrfFlt(srCam, &ptrXYZ[0], &ptrXYZ[1], &ptrXYZ[2], s, s, s); 
    
    // if(planeExtract.extractFloor(cloud, floor))
    //  viewer.showCloud(floor);
    // else
    viewer.showCloud(cloud);
    usleep(30000); // sleep 30 ms 
  }

}

