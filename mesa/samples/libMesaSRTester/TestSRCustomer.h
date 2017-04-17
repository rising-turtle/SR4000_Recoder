/*------------------------------------------------------------------------*/
/*                                                                        */
/*  Copyright (c) 2008 by MESA Imaging SA                                 */
/*                     http://www.mesa-imaging.ch                         */
/*------------------------------------------------------------------------*/
// $Author$
//    $URL$
//    $Rev$
//   $Date$
/*------------------------------------------------------------------------*/
#pragma once
#ifdef _WIN32
#include <sys/stat.h> //stat
#include <conio.h> //_putch,_getch
#include <Windows.h> //WORD, DWORD etc
#else
#define _putch putchar
#define _vsnprintf vsnprintf
#define _unlink unlink
#define MAX_PATH 260
typedef unsigned char  BYTE;
typedef unsigned long  DWORD;
int _kbhit();
#endif

#include "TestSuite.h"
#include "libMesaSR.h"
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
  #pragma comment( lib, "libMesaSR.lib" )
#endif

#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

void ShowImg(SRCAM srCam, ImgEntry* imgEntryArray); //need to be implemented in main code

class CTestSRCustomer :public CTestSuite
{
public:
  enum CamIf {IF_NONE, IF_ETH, IF_USB};
  CTestSRCustomer(){_lastInstance=this; _camIf=IF_NONE;}
  static CTestSRCustomer* GetLastInstance(){return _lastInstance;}
  //Test Functions:
  int LibusbCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data);
  void TestDataDump();
  void TestBasic();
  void TestContinousAcquire();
  void TestUserSrvCmd();
  void TestTrial();
  bool SetTestCam(const char* ctype, const char* caddr);
  const char* getCamIp(){return this->_camIp;}
  unsigned int getCamSn() {return this->_camSn;}
  
private:
  static CTestSRCustomer* _lastInstance;

protected:  
  CamIf _camIf;
  unsigned int _camSn;
  char _camIp[30];
};

__inline int MyLibusbCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data)
{
  CTestSRCustomer* testSR=CTestSRCustomer::GetLastInstance();
  if(testSR)
    return testSR->LibusbCallback(srCam, msg, param, data);
  return 0;
}
