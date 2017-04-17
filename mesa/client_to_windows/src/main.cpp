#include "SR_tcp_socket.h"
#include "cam_model.h"
#include <stdio.h>
#include <vector>
#include <string.h>
#include "timestamp.h"

using namespace std;

void fromDepth2XYZ(vector<unsigned short>& depth, vector<float>& px, vector<float>& py, vector<float>& pz);

int test1(); 
int test2(); // test wifi delay

int main()
{
  // test1();
  test2(); 
}

int test2()
{
  CSRTcpSocket tcp_client; 
  if(!tcp_client.open())
  {
    printf("main.cpp: failed to connect to the server!\n"); 
    return -1;
  }
  TTimeStamp start_t = getCurrentTime();
  int N = 500; 
  TCP_data tcp_data;
  for(int i=0; i<N;)
  {
    if(tcp_client.get(tcp_data))
    {
      printf("main.cpp: get %d\n", ++i);
      usleep(1000);
    }
  }
  TTimeStamp stop_t = getCurrentTime(); 
  double time_consumption = timeDifference(start_t, stop_t)*1000; 
  printf("main.cpp: total cost %lf ms for 500 frames, mean = %lf\n", time_consumption, time_consumption/500.);

  return 1;
}

int test1()
{
    CSRTcpSocket tcp_client; 
    if(!tcp_client.open())
    {
      printf("main.cpp: failed to connect to the server!\n"); 
      return -1;
    }
    
    int N = 200;
    int rows = 144; 
    int cols = 176;
    int total = rows*cols; 
    vector<unsigned short> dis(total, 0); // z value
    TCP_data tcp_data;

    vector<float> x(total, 0); 
    vector<float> y(total, 0); 
    vector<float> z(total, 0);

    for(int i=0; i< N; )
    {
      if(tcp_client.get(tcp_data))
      {
        printf("main.cpp: get %d frame\n", ++i);

        // get depth data from the raw stream 
        char* p = tcp_data.data.data(); 
        // the first part is intensity , the second part is depth 
        memcpy(dis.data(), p + total*sizeof(unsigned short), total*sizeof(unsigned short));
        
        // from depth to point x, y, z
        fromDepth2XYZ(dis, x, y, z); 
      }
    }
}

void fromDepth2XYZ(vector<unsigned short>& depth, vector<float>& px, vector<float>& py, vector<float>& pz)
{
  static CamModel cam_model(223.9758, 226.7442, 89.361, 75.8112);     
  unsigned short* pD = (unsigned short*)(depth.data());
  int k;
  float z;  
  double ox,oy,oz;
  int sr_rows = 144; 
  int sr_cols = 176;
  for(int i=0; i < sr_rows; i++)
  {
    for(int j=0; j <sr_cols; j++)
    {
      k = j + i*sr_cols;
      z = *pD*0.001; // convert mm to m

      if(z <= 0.01 || z>= 4.95) // 9.5 for 10m range, 4.95 for 5m range
      {
        px[k] = py[k] = pz[k] = 0; 
      }else
      {
        z = z + 0.01; // Z is in the swiss ranger reference, z = Z + shift, to rectified camera reference
        cam_model.convertUVZ2XYZ(j+0.5, i+0.5, z, ox, oy, oz); 
        px[k] = -ox;  py[k] = -oy; pz[k] = oz - 0.01 ; // 1cm shift, transfer to the swiss ranger reference
      }
      ++pD; 
    }
  }
}

