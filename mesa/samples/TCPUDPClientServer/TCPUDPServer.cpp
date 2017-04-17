// TCPUDPClientServer.cpp : Defines the entry point for the console application.
//

#ifdef _WIN32
#include "stdafx.h"

#include <cstdio>
#include <conio.h>
#include <winsock2.h>
#pragma comment(lib,"Ws2_32")
#pragma comment(lib,"libMesaSR")
typedef int socklen_t;
#else

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <arpa/inet.h>
#include <string.h>
#define _getch getchar
#define closesocket close
#define SD_BOTH SHUT_RDWR
//typedef unsigned short WORD;
//typedef unsigned int   DWORD;
typedef unsigned char   BYTE;
typedef int   SOCKET;
#endif

#include "protocolDefines.h"
#include "libMesaSR.h"


int TCPRecv(SOCKET s, void *buf, int len, int flags);
int TCPServer1();

#ifdef _WIN32
int _tmain(int argc, _TCHAR* argv[])
#else
//dummy implementation because it is not linked with pthread library
#ifdef __cplusplus
extern "C"
{
#endif
int pthread_mutex_lock(pthread_mutex_t*){return 0;}
int pthread_mutex_unlock(pthread_mutex_t*){return 0;}
int pthread_mutex_init(pthread_mutex_t*, const pthread_mutexattr_t*){return 0;}
#ifdef __cplusplus
};
#endif

int main(int argc, char* argv[])
#endif
{
#ifdef _WIN32
  {
    WSADATA wsa;
    int result;
    // starting Winsock
    result = WSAStartup(0x0002, &wsa); // 0x0002 for Winsock 2.0, 0x0101 for WS 1.1
  }
#endif
  TCPServer1();
  puts("TCPServer finished.");
#ifdef _WIN32
  puts("press key");_getch();
  WSACleanup();
#endif
  return 0;
}

int TCPServer1()
{
  WORD port=htons(22222);
  struct sockaddr_in saLoc, saRmt;
  SOCKET sLoc,sRmt;
  socklen_t saLen=sizeof(saRmt);
  int res;
  SRCAM srCam;
  int w,h;
  puts("starting TCPServer");
#ifdef __BFIN__
  SR_Open(&srCam);
#else
  SR_OpenDlg(&srCam,3,0);
#endif
  w=SR_GetCols(srCam);
  h=SR_GetRows(srCam);


  if ((sLoc=socket(AF_INET, SOCK_STREAM, 0))==-1)
  { 
    perror("TCPListen: socket() failed.");
    return -1;
  }
  memset((char *)&saLoc, 0, sizeof(saLoc));
  saLoc.sin_family = AF_INET;
  saLoc.sin_port = port;
  saLoc.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sLoc, (struct sockaddr*)&saLoc, sizeof(saLoc))==-1)
  {
    perror("TCPListen: bind() failed.");
    return -1;
  }
  listen(sLoc,0);
  printf("bind and listen on %s:%d\n",inet_ntoa(saLoc.sin_addr), ntohs(saLoc.sin_port));

  IPObj rx;
  for(;;)
  {
    sRmt=accept(sLoc,(struct sockaddr*)&saRmt,&saLen);
    if(sRmt<0)
      perror("TCPAccept failed");

    printf("accept %s:%u\n",inet_ntoa(saRmt.sin_addr), htons(saRmt.sin_port));
    //READ WRITE
    for(;;)
    {
      res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), MSG_PEEK);
      printf("TCPRecv peek %d\n",res);
      if(res<=0)
        break;
      switch(rx.id)
      {
        case RQ_GetImage:   
        {
          res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), 0);
          printf("RQ_GetImage consume %d\n",res);
          char* dst;
          int sz=w*h*2;
          SR_Acquire(srCam);
          dst = (char*)SR_GetImage(srCam, 0);
          IPRSGetImage tx;
          tx._p.id=RS_GetImage;
          tx._p.size=sizeof(tx)+sz;
          res=send(sRmt,(char*)&tx,sizeof(tx),0);
          printf("sent %d bytes\n",res);
          res=send(sRmt,(char*)dst,sz,0);
          printf("sent %d bytes\n",res);
          break;
        }
        case RQ_GetVersion:   
        {
          res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), 0);
          puts("RQ_GetVersion");
          IPRSGetVersion tx;
          tx._p.id=RS_GetVersion;
          res=SR_GetVersion(tx.ver);
          res=send(sRmt,(char*)&tx,sizeof(tx),0);
          printf("sent %d bytes\n",res);
          break;
        }
        case RQ_EchoString:   
        {
          //IPObj tx;
          IPRQEchoString rx;
          res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), 0);
          puts("RQ_EchoString");
          printf("received string '%s'\n",rx.string);
          break;
        }
        default:
        {
          res=TCPRecv(sLoc, (char*)&rx, sizeof(rx), 0);//consume
          fprintf(stderr, "id %u size %u\n",rx.id,rx.size);
          break;
        }
      }
    }
    puts("shutdown");
    goto shutdown;
  }
shutdown:
  shutdown(sRmt, SD_BOTH);//SHUT_RDWR);
  closesocket(sRmt);
  shutdown(sLoc, SD_BOTH);//SHUT_RDWR);
  closesocket(sLoc);
  puts("ending TCPServer");
  return 0;
}

int TCPRecv(SOCKET s, void *buf, int len, int flags)
{
  int pos,res;
  BYTE* p=(BYTE*)buf;

  for(pos=0,res=0;res<len;)
  {
    res = recv(s, (char*)&p[pos], len-pos, flags);
    if ( res <= 0)
    {
      printf("TCPRecv: recv[] failed %d. received %d/%d\n",res,pos,len);
      break;
    }
    else
    {
      if(!(flags&MSG_PEEK))
      {
        pos+=res;
        //Printf(MK_DEBUG_STRING|MC_ETH,"received %d/%d (+%d) bytes\r",pos,len,res);
        res=pos;
      }
      else
      {
        //Printf(MK_DEBUG_STRING|MC_ETH,"peeked %d/%d bytes\r",res,len);
      }
    }
  }
  //Printf(MK_DEBUG_STRING|MC_ETH,"received %d/%d (+%d) bytes done.\n",pos,len,res);
  return res;
}
