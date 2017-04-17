// TCPUDPClientServer.cpp : Defines the entry point for the console application.
//

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#include "stdafx.h"

#include <cstdio>
#include <conio.h>
#include <winsock2.h>
#pragma comment(lib,"Ws2_32")
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
typedef unsigned short WORD;
typedef unsigned int   DWORD;
typedef unsigned char   BYTE;
typedef int   SOCKET;
#endif
#include "protocolDefines.h"

int TCPRecv(SOCKET s, void *buf, int len, int flags);
int TCPClient1(const char* ip);

static const char* separator="\n----------------------------------------------------";
//static const char* menu="1: TCPClient\n"
//                        "x: exit\n"
//                        "\npress a key";

static const char* menuClnt=
    "1: RQ_GetImage\n"
    "2: RQ_GetVersion\n"
    "3: RQ_EchoString\n"
    "4: shutdown\n"
    "5: closesocket\n"
    "x: exit\n"
    "\npress a key";

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
  //int c;
#ifdef _WIN32
  {
    WSADATA wsa;
    int result;
    // starting Winsock
    result = WSAStartup(0x0002, &wsa); // 0x0002 for Winsock 2.0, 0x0101 for WS 1.1
  }
#endif
//  for(;;)
//  {
//    puts(separator);
//    puts(menu);
//    c=_getch();
//    switch(c)
//    {
//      case '1':
//      {
//        const char* ip="127.0.0.1";
//        //const char* ip="10.0.1.181";
//        TCPClient1(ip);
//        break;
//      }
//      case 'x':
//        goto exit;
//    }
//  }
//exit:
  const char* ip="127.0.0.1";
  //const char* ip="10.0.1.133";
  TCPClient1(ip);
  puts("TCPClient finished.");
#ifdef _WIN32
  puts("press key");_getch();
  WSACleanup();
#endif
  return 0;
}


int TCPClient1(const char* ip)
{
  WORD rmtPort=htons(22222);
  int timeout=1000;//ms
  struct sockaddr_in saRmt;       // Information about the saRmt
  int saLen=sizeof(saRmt);        // Length of struct sockaddr_in
  DWORD rmtAddr=inet_addr(ip);
  SOCKET s;
  puts("connect TCPClient...");
  if ((s=socket(AF_INET, SOCK_STREAM, 0))<0)
  { 
    puts("TCPConnect: socket() failed.");
  }
  memset((char *)&saRmt, 0, sizeof(saRmt));
  saRmt.sin_family = AF_INET;
  saRmt.sin_port = rmtPort;
  saRmt.sin_addr.s_addr=rmtAddr;
  if (connect(s, (struct sockaddr*)&saRmt, sizeof(saRmt))!=0)
  {
    puts("TCPConnect: connect() failed.");
    return -1;
  }
  printf("TCPConnect connect on %s:%d\n",inet_ntoa(saRmt.sin_addr), ntohs(saRmt.sin_port));
  int to=3000;//3000ms timeout
  setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,(char*)&to,sizeof(to));

  int c,res;
  for(;;)
  {
    puts(separator);
    puts(menuClnt);
    c=_getch();
    switch(c)
    {
    case '1'://    "1: RQ_GetImage\n"
      {
        IPObj tx;
        tx.id=RQ_GetImage;
        tx.size=sizeof(tx);
        res=send(s,(char*)&tx,sizeof(tx),0);
        printf("sent %d bytes\n",res);
        
        IPRSGetImage rx;
        char* data;
        res=TCPRecv(s,(char*)&rx,sizeof(rx),0);
        printf("TCPRecv consume %d\n",res);
        if(res<=0)
        {
          puts("TCPRecv error");break;
        }
        int size=rx._p.size-sizeof(IPRSGetImage);
        data=(char*)malloc(size);
        if(data)
        {
          unsigned char w=176,h=144;
          unsigned short* img=(unsigned short*)data;
          res=TCPRecv(s,data,size,0);
          printf("TCPRecv consume %d\n",res);
          printf("image received: %d",(int)img[w*h/2+w/2]);
          free(data);
        }
        else
        {
          puts("malloc failed");
        }
        break;
      }
    case '2'://    "2: RQ_GetVersion\n"
      {
        IPObj tx;
        tx.id=RQ_GetVersion;
        tx.size=sizeof(tx);
        res=send(s,(char*)&tx,sizeof(tx),0);
        printf("sent %d bytes\n",res);
        IPRSGetVersion rx;
        res=TCPRecv(s,(char*)&rx,sizeof(rx),0);
        printf("TCPRecv consume %d\n",res);
        printf("Version %d.%d.%d.%d\n",(int)rx.ver[3],(int)rx.ver[2],(int)rx.ver[1],(int)rx.ver[0]);
        break;
      }
    case '3'://    "3: RQ_EchoString\n"
      {
        IPRQEchoString tx;
        tx._p.id=RQ_EchoString;
        tx._p.size=sizeof(tx);
        strcpy(tx.string,"my test string by client.");
        res=send(s,(char*)&tx,sizeof(tx),0);
        printf("sent %d bytes\n",res);
        //do not wait for response
        break;
      }
    case '4':
      {
        shutdown(s,SD_BOTH);
        break;
      }
    case '5':
      {
        closesocket(s);
        break;
      }
    case 'x':
      goto exit;
    }
  }
exit:
  closesocket(s);
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
