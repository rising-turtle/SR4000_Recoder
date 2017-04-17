/*------------------------------------------------------------------------*/
/*                                                                        */
/*  Copyright (c) 2008 by MESA Imaging SA,                                */
/*                     http://www.mesa-imaging.ch                         */
/*------------------------------------------------------------------------*/
// $Author$
//    $URL$
//    $Rev$
//   $Date$
/*------------------------------------------------------------------------*/
//!\file
//!Example console application
// libMesaSRTester.cpp : Defines the entry point for the console application.
//
//!\addtogroup libMesaSRTester
//!@{

#include "TestSRCustomer.h"

#ifdef _WIN32
#define set_canon(x) //!< macro for windows/linux compatibility
#else
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#include <stdlib.h>
#include <linux/sockios.h>
#include <asm/ioctls.h>
#include <sys/select.h>

inline int set_canon(int flag)
{
        struct termios t;
        tcgetattr( fileno(stdin), &t);
        if( flag)
                t.c_lflag |= (ICANON|ECHO);
        else
                t.c_lflag &= ~(ICANON|ECHO);
        tcsetattr( fileno(stdin), TCSANOW, &t); 

        return( 1);
}

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


#define _getch getchar
#endif

static const char* menu=
"CTestSRCustomer:\n"
" 1: TestBasic\n"
" 2: TestDataDump\n"
" 3: TestContinousAcquire\n"
" 4: TestUserSrvCmd\n"
" 5: TestTrial\n"
"----------------\n"
"x: exit\n"
"\npress a key";

static const char* usage_error=
#ifdef __BFIN__
"Error: Invalid argument line!\n"
"Usage: libMesaSRTester [-t 'test']\n"
"'test' : the name of the test to run\n";
#else
"Error: Invalid argument line!\n"
"Usage: libMesaSRTester [-i 'camiface' -a 'camaddr' -t 'test']\n"
"'camiface' : either 'usb' or 'eth'\n"
"'camaddr'  : in case of -i='eth', the cam ip address\n"
"             in case of -i='usb', the last 8 digits of the cam serial number\n"
"'test'     : the name of the test to run\n"; 
#endif

static const char* iface_error=
"Error: Passed in 'interface' not valid, must be either 'usb' or 'eth'\n"
"exiting ...\n";

static const char* test_error=
"Error: Passed in test name not available!\n"
"Run the program without any arguments to see valid names\n"
"exiting ...\n";

//! User Callback function
int MyLibusbCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data);

//! User function to show image
void ShowImg(SRCAM srCam, ImgEntry* imgEntryArray)
{
  //does nothing here
}

int DoSwitch(CTestSRCustomer* srCustTester, int c)
{
  switch(c)
  {
  case '1':
    srCustTester->TestBasic();
    break;
  case '2':
    srCustTester->TestDataDump();
    break;
  case '3':
    srCustTester->TestContinousAcquire();
    break;
  case '4':
    srCustTester->TestUserSrvCmd();
    break;
  case '5':
    srCustTester->TestTrial();
    break;
  case 'x':
    return -1;
  }
  return 0;
}

bool checkArgs(int argc,char** argv)
{
  bool c,a,t;
  switch(argc) 
  {
    case 1:
      return true;
    case 3:
      #ifdef __BFIN__
        t = (strcmp(argv[1],"-t") == 0);
        return t;
      #else
        return false;
      #endif
    case 7:
      #ifdef __BFIN__
        return false;
      #else
        c = (strcmp(argv[1],"-i") == 0);
        a = (strcmp(argv[3],"-a") == 0);
        t = (strcmp(argv[5],"-t") == 0);
        return (c && a && t);
      #endif
    default:
      return false;
  }
}

//!main function
int main(int argc, char* argv[])
{ 
  if(!checkArgs(argc,argv))
  {
    puts(usage_error);
    return 0;
  }
  set_canon(0);
  CTestSRCustomer srTester;
  
  int c;
  SR_SetCallback(MyLibusbCallback);//set global callback
  if(argc == 1)
  {
    for(;;)
    {
      puts(CTestSuite::_separator);
      puts(menu);
      c=_getch();
      if(DoSwitch(&srTester,c))
        break;
      puts(CTestSuite::_separator);
    }
  }
  else
  {
    #ifdef __BFIN__
    const char* found = strstr(menu,argv[2]);
    #else
    const char* found = strstr(menu,argv[6]);
    #endif
    
    if(found)
    {
      c = (int) *(found - 3);
      #ifdef __BFIN__
        DoSwitch(&srTester, c);
      #else
        if(srTester.SetTestCam(argv[2],argv[4]))
          DoSwitch(&srTester, c);
	      else
		      puts(iface_error);
      #endif
    }
    else
      puts(test_error);
  }
  SR_SetCallback(SR_GetDefaultCallback());//set default global callback
  set_canon(1);
  return 0;
}

//!@}
