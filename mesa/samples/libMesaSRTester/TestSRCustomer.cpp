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
#include "TestSRCustomer.h"
#include "UserProtocol.h"

#ifndef _WIN32
  #include <stdio.h>
  #include <stdlib.h>
  #include <termios.h>
  #include <linux/sockios.h>
  #include <asm/ioctls.h>
  #include <sys/select.h>
  #ifndef __BFIN__
    #include <stropts.h>
  #else
    #include <sys/ioctl.h>
  #endif
  #include <unistd.h>
  #define _getch getchar

int _kbhit() {
  static const int STDIN = 0;
  static bool initialized = false;

  if (! initialized) {
        // Use termios to turn off line buffering
    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = true;
  }

  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return bytesWaiting;
}
#endif
CTestSRCustomer* CTestSRCustomer::_lastInstance=0;

#ifndef HIWORD
# define HIWORD(X) ((unsigned short)((unsigned long)(X)>>16))
# define LOWORD(X) ((unsigned short)((unsigned long)(X)&0xFFFF))
#endif
//! local used message callback function to redirect output messages to console

bool CTestSRCustomer::SetTestCam(const char* ctype, const char* caddr)
{
  bool is_eth = (strcmp(ctype,"eth") == 0);
  bool is_usb = (strcmp(ctype,"usb") == 0);
  if(is_eth)
  {
    this->_camIf = IF_ETH;
    strcpy(this->_camIp,caddr);
	return true;
  }
  else if(is_usb)
  {
	this->_camIf = IF_USB;
    unsigned long val = strtoul(caddr,NULL,16);
    this->_camSn = (unsigned int) val;
	return true;
  }
  else
    return false;
}

int CTestSRCustomer::LibusbCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data)
{
  switch(msg)
  {
  case CM_MSG_DISPLAY: // redirects all output to console
    {
      if (param==MC_ETH)
        return 0;
      char*p=(char*)data,*q;
      if(_afterTest)
        _putch('\n');
      while(q=strchr(p,'\n'))
      {
        fputs(_sResStr,stdout);
        fwrite(p,q-p+1,1,stdout);
        p=&q[1];
      }
      fputs(_sResStr,stdout);
      puts((char*)p);
      _afterTest=false;
      return 0;
    }
  case CM_PROGRESS:
    {
      int state   =LOWORD(param);
      int progress=HIWORD(param);
      switch(state)
      {
      case CP_FLASH_ERASE:
        printf("Erasing flash (%d%%)...\n",progress);break;
      case CP_FLASH_WRITE:
        printf("Writing flash (%d%%)...\n",progress);break;
      case CP_FLASH_READ:
        printf("Reading flash (%d%%)...\n",progress);break;
      case CP_FPGA_BOOT:
        printf("Boot FPGA (%d%%)...\n",progress);break;
      case CP_CAM_REBOOT:
        printf("Reboot camera (%d%%)...\n",progress);break;
      case CP_DONE:
        puts("\ndone.");
      }
      return 0;
    }
  default:
    {
      //default handling
      return SR_GetDefaultCallback()(0,msg,param,data);
    }
  }
}

//!function to test various libMesaSR functions.
//!at the end a report shows how many functions runned successfully and how many failed
void CTestSRCustomer::TestBasic()
{
  Reset();
  CMesaDevice* srCam=0;

  int res, numImg, i, defaultMode;
  unsigned int serialNumber=0;
  BYTE uc[4];
  WORD us[4];
  char bufC[1024];
  ImgEntry* imgEntryArray;
  void* data;

  //--------- SR_GetVersion ---------
  {
    unsigned short version[4];
    Run("SR_GetVersion");
    res=SR_GetVersion(version);
    Trace("Version %d.%d.%d.%d",version[3],version[2],version[1],version[0]);
    Result(res==0);
  }
  //--------- SR_CheckForNewDllVersion ---------
#ifdef _WIN32
  Run("SR_CheckForNewDllVersion");
  res=SR_CheckForNewDllVersion(0);     Result(res>=0);
  res=SR_CheckForNewDllVersion(1);     Result(res>=0);
  res=SR_CheckForNewDllVersion(2);     Result(res>=0);
#endif
  //--------- SR_Open ---------
#ifdef __BFIN__
  Run("SR_Open");
  res=SR_Open(&srCam);
#else
  switch(_camIf)
  {
    case IF_NONE:
      Run("SR_OpenDlg");
      res=SR_OpenDlg(&srCam,3,0);
      break;
    case IF_ETH:
      Run("SR_OpenETH");
      res=SR_OpenETH(&srCam,this->_camIp);
      break;
    case IF_USB:
      Run("SR_OpenUSB");
      res=SR_OpenUSB(&srCam,this->_camSn);
      break;
  }  
#endif
  ABORT_ON(res<=0);
  //--------- SR_GetDeviceString ---------
#ifndef __BFIN__
  Run("SR_GetDeviceString");
  res=SR_GetDeviceString(srCam,bufC,_countof(bufC));//returns the device ID used in other calls
  Trace("%s",bufC);
  Result(res>0);
#endif //__BFIN__
  //--------- SR_SetTimeout ---------
  Run("SR_SetTimeout (BLIND)");
  SR_SetTimeout(srCam,100);
  SR_SetTimeout(srCam,1000);    Result(1);
  //--------- Get Rows/Cols ---------
  Run("Get Rows/Cols");
  res=SR_GetRows(srCam);              Result(res==144 || res==160);
  res=SR_GetCols(srCam);              Result(res==176 || res==124);
  //--------- GetImageList ---------
  Run("GetImageList");
  numImg=SR_GetImageList(srCam, &imgEntryArray);   Result(numImg>1 && imgEntryArray!=0);
  //--------- GetImage ---------
  Run("GetImage");
  for(i=0;i<numImg;i++)
  {
    data=SR_GetImage(srCam, i);   Result(data!=0);
  }
  //--------- SR_SetMode ---------
  Run("SR_SetMode");
  defaultMode=SR_GetMode(srCam);
#ifdef __BFIN__
  res=SR_SetMode(srCam,0);   Result(res==0);
#else //__BFIN__
  res=SR_SetMode(srCam,AM_COR_FIX_PTRN|AM_MEDIAN|AM_TOGGLE_FRQ|AM_CONV_GRAY|AM_SW_ANF);   Result(res==0);
  res=SR_Acquire(srCam);
  res=SR_SetMode(srCam,defaultMode);   Result(res==0);
#endif //__BFIN__
  //--------- SR_Acquire ---------
  Run("SR_Acquire");
  {
    const int numTest=10;
    int w=SR_GetCols(srCam);int h=SR_GetRows(srCam);
    int numPix=w*h;
    int numBytes=numImg*numPix*sizeof(WORD);
    for(i=0;i<numTest;i++)
    {
      res=SR_Acquire(srCam);
      Result(res==numBytes);
    }


    //--------- SR_CoordTrfXXX ---------
    {
      size_t bufSz=numPix*3*sizeof(double);
      void *buf=malloc(bufSz);   memset(buf,0xaf,bufSz);

      short *xS16=(short*)buf, *yS16=&xS16[numPix]; WORD *zU16=(WORD*)&yS16[numPix];
      float  *xFlt=(float*)buf,  *yFlt=&xFlt[numPix], *zFlt=&yFlt[numPix];
      double *xDbl=(double*)buf, *yDbl=&xDbl[numPix], *zDbl=&yDbl[numPix];
      int pitchS16X=sizeof(short),  pitchS16Y=sizeof(short), pitchU16Z=sizeof(WORD);
      int pitchFltX=sizeof(float),  pitchFltY=sizeof(float), pitchFltZ=sizeof(float);
      int pitchDblX=sizeof(double), pitchDblY=sizeof(double), pitchDblZ=sizeof(double);

      Run("SR_CoordTrfUint16");
      for(i=0;i<numTest;i++)
      {
        res=SR_Acquire(srCam);
        res=SR_CoordTrfUint16(srCam, 0,    0,    zU16, pitchS16X, pitchS16Y, pitchU16Z);     Result(res==0);
        res=SR_CoordTrfUint16(srCam, xS16, yS16, zU16, pitchS16X, pitchS16Y, pitchU16Z);     Result(res==0);
      }
      Run("SR_CoordTrfFlt");
      for(i=0;i<numTest;i++)
      {
        res=SR_Acquire(srCam);
        res=SR_CoordTrfFlt(srCam, 0,    0,    zFlt, pitchFltX, pitchFltY, pitchFltZ);     Result(res==0);
        res=SR_CoordTrfFlt(srCam, xFlt, yFlt, zFlt, pitchFltX, pitchFltY, pitchFltZ);     Result(res==0);
      }
      Run("SR_CoordTrfDbl");
      for(i=0;i<numTest;i++)
      {
        res=SR_Acquire(srCam);
        res=SR_CoordTrfDbl(srCam, 0,    0,    zDbl, pitchDblX, pitchDblY, pitchDblZ);     Result(res==0);
        res=SR_CoordTrfDbl(srCam, xDbl, yDbl, zDbl, pitchDblX, pitchDblY, pitchDblZ);     Result(res==0);
      }

      free(buf);
    }

    //--------- SR_CoordTrfPntXXX ---------
    {
      const unsigned char iX[9]={0, 0, 0, (w-1)/2, (w-1)/2, (w-1)/2, w-1, w-1, w-1};
      const unsigned char iY[9]={0, (h-1)/2, h-1, 0, (h-1)/2, h-1, 0, (h-1)/2, h-1};
      unsigned short iDst[9];
      WORD* dst;
      int j,numPnt=_countof(iDst);

      size_t bufSz=numPnt*3*sizeof(double);
      void *buf=malloc(bufSz);   memset(buf,0xaf,bufSz);
      short *xS16=(short*)buf, *yS16=&xS16[numPnt]; WORD *zU16=(WORD*)&yS16[numPnt];
      float  *xFlt=(float*)buf,  *yFlt=&xFlt[numPnt], *zFlt=&yFlt[numPnt];
      double *xDbl=(double*)buf, *yDbl=&xDbl[numPnt], *zDbl=&yDbl[numPnt];

      Run("SR_CoordTrfPntUint16");
      for(i=0;i<numTest;i++)
      {
        res=SR_Acquire(srCam); dst=(WORD*)SR_GetImage(srCam,0); for(j=0;j<numPnt;j++){iDst[j]=dst[iX[j]+iY[j]*w];}
        res=SR_CoordTrfPntUint16(srCam, iX, iY, iDst, 0,    0,    zU16, numPnt);     Result(res==0);
        res=SR_CoordTrfPntUint16(srCam, iX, iY, iDst, xS16, yS16, zU16, numPnt);     Result(res==0);
      }

      Run("SR_CoordTrfPntFlt");
      for(i=0;i<numTest;i++)
      {
        res=SR_Acquire(srCam); dst=(WORD*)SR_GetImage(srCam,0); for(j=0;j<numPnt;j++){iDst[j]=dst[iX[j]+iY[j]*w];}
        res=SR_CoordTrfPntFlt(srCam, iX, iY, iDst, 0,    0,    zFlt, numPnt);     Result(res==0);
        res=SR_CoordTrfPntFlt(srCam, iX, iY, iDst, xFlt, yFlt, zFlt, numPnt);     Result(res==0);
      }

      Run("SR_CoordTrfPntDbl");
      for(i=0;i<numTest;i++)
      {
        res=SR_Acquire(srCam); dst=(WORD*)SR_GetImage(srCam,0); for(j=0;j<numPnt;j++){iDst[j]=dst[iX[j]+iY[j]*w];}
        res=SR_CoordTrfPntDbl(srCam, iX, iY, iDst, 0,    0,    zDbl, numPnt);     Result(res==0);
        res=SR_CoordTrfPntDbl(srCam, iX, iY, iDst, xDbl, yDbl, zDbl, numPnt);     Result(res==0);
      }

      free(buf);
    }
  }


#ifdef __BFIN__
  {
    const int numTest=10;
    int j,k;
    Run("SR_IsFrameAvailable");
	  uc[0]=SR_GetIntegrationTime(srCam);
  	uc[1]=uc[0]/2;
  	uc[2]=uc[0]*2;
    for(k=0;k<3;k++)
    {
		  Trace("SetIntegrationTime %d",uc[k]);
      SR_SetIntegrationTime(srCam,uc[k]);
		  for(i=0;i<numTest;i++)
		  {
	      printf("SR_IsFrameAvailable ...");
		    for(j=0;j<100;j++)
		    {
		      res=SR_IsFrameAvailable(srCam);
		      printf("%d",res);
		      if(res)
		      {
			      puts("... SR_Acquire");
		        res=SR_Acquire(srCam);
		        break;
		      }
		      else
		        usleep(1000);
		    }
		  }
    }
  SR_SetIntegrationTime(srCam,uc[0]);
  }
#endif

  //--------- Get/Set Reg ---------
  Run("Get/Set Reg");
  uc[0]=SR_GetReg(srCam,20);
  uc[1]=(uc[0]+0x3f)^0x73;           Result(uc[0]!=uc[1]);  //an other value...
  res =SR_SetReg(srCam,20,uc[1]);    Result(res==2);
  uc[2]=SR_GetReg(srCam,20);         Result(uc[1]==uc[2]);
  res =SR_SetReg(srCam,20,uc[0]);    Result(res==2);
  uc[3]=SR_GetReg(srCam,20);         Result(uc[0]==uc[3]);

  //--------- Get/Set IntegrationTime ---------
  Run("Get/Set IntegrationTime");
  uc[0]=SR_GetIntegrationTime(srCam);
  uc[1]=(uc[0]+0x3f)^0x73;                    Result(uc[0]!=uc[1]);  //an other value...
  res =SR_SetIntegrationTime(srCam,uc[1]);    Result(res==2);
  uc[2]=SR_GetIntegrationTime(srCam);         Result(uc[1]==uc[2]);
  res =SR_SetIntegrationTime(srCam,uc[0]);    Result(res==2);
  uc[3]=SR_GetIntegrationTime(srCam);         Result(uc[0]==uc[3]);

  Run("SR_SetDualIntegrationTime");
  res =SR_SetDualIntegrationTime(srCam, 50);
  uc[0]=SR_GetReg(srCam,46);
  if(uc[0] >= 0x73)
    Result(res==2);
  else
    Result(res==-1);
  res =SR_SetIntegrationTime(srCam,32);       Result(res==2);
  if(uc[0] >= 0x73)
  {
    uc[0]=SR_GetReg(srCam,15);                
    Result(uc[0]==16);   // second int. time must be at 50%
  }
  res =SR_SetDualIntegrationTime(srCam, 0);
  uc[0]=SR_GetReg(srCam,46);
  if(uc[0] >= 0x73)
    Result(res==2);
  else
    Result(res==-1);

  //--------- Get/SetAmplitudeThreshold ---------
  Run("Get/SetAmplitudeThreshold");
  us[0]=SR_GetAmplitudeThreshold(srCam);
  us[1]=(us[0]+0x523f)^0x13c9;                   Result(us[0]!=us[1]);  //an other value...
  res =SR_SetAmplitudeThreshold(srCam,us[1]);    Result(res==4);
  us[2]=SR_GetAmplitudeThreshold(srCam);         Result(us[1]==us[2]);
  res =SR_SetAmplitudeThreshold(srCam,us[0]);    Result(res==4);
  us[3]=SR_GetAmplitudeThreshold(srCam);         Result(us[0]==us[3]);

  //--------- Get/SetDistanceOffset ---------
  Run("Get/SetDistanceOffset");
  us[0]=SR_GetDistanceOffset(srCam);
  us[1]=(us[0]+0x523f)^0x13c9;               Result(us[0]!=us[1]);  //an other value...
  res =SR_SetDistanceOffset(srCam,us[1]);    Result(res>=2);
  us[2]=SR_GetDistanceOffset(srCam);         Result((us[1]&0xff)==(us[2]&0xff)); //check only lsb (sr3k)
  res =SR_SetDistanceOffset(srCam,us[0]);    Result(res>=2);
  us[3]=SR_GetDistanceOffset(srCam);         Result(us[0]==us[3]);

  //--------- SR_SetAutoExposure ---------
  Run("SR_SetAutoExposure");
  res=SR_SetAutoExposure(srCam, 0,255,10,45);
  Result(res==0);

  //--------- Non Ambiguity test---------
  Run("Non Ambiguity test");
  uc[0]=SR_GetReg(srCam,46);
  defaultMode=SR_GetMode(srCam);
  defaultMode=defaultMode|AM_NO_AMB;
  res=SR_SetMode(srCam,defaultMode); 
  if(uc[0] >= 0x97)
    Result(res==0);
  else
    Result(res==-1);
  defaultMode=SR_GetMode(srCam);
  defaultMode=defaultMode&~AM_NO_AMB;
  res=SR_SetMode(srCam,defaultMode); 
  Result(res==0);

#ifdef _WIN32
  Run("SR_StreamToFile");
  {
    char tmpName[MAX_PATH];//alternative _tempnam
    GetTempPath(_countof(tmpName),tmpName);
    res=GetTempFileName(tmpName,"tmpSR",0,tmpName);     Result(res!=0);
    //char tmpName[]={"//tmp//libMesaSR.XXXXXX"};
    //mkstemp(tmpName);
    SR_StreamToFile(srCam,tmpName,0);
    for(i=1;i<5;i++)
    {
      res=SR_Acquire(srCam);
    }
    SR_StreamToFile(srCam,tmpName,2);
    struct _stat st;
    int numPix=SR_GetRows(srCam)*SR_GetCols(srCam);
    res=_stat(tmpName,&st);   Result(res==0&& st.st_size>=numPix*5*2);
    res=_unlink(tmpName);      Result(res==0);
  }
#endif
  //--------- SR_Close ---------
  Run("SR_Close");
  res=SR_Close(srCam);
  Result(res==0);
  //--------- RESULTS -----------------------
  ReportSummary();
}

//!function to test basic operation of the libMesaSR interface.
//!it opens the device with SR_OpenUSB() then acquires one images and dumps its data on request
void CTestSRCustomer::TestDataDump()
{
  char cmd;
  CMesaDevice* srCam;
  int res;

  printf("This Testcode uses the new libMesaSR interface with the libusb device driver\n");

#ifdef __BFIN__
  Run("SR_Open");
  res=SR_Open(&srCam);
#else
  Run("SR_OpenDlg");
  res=SR_OpenDlg(&srCam,3,0);
#endif
  //res=SR_OpenUSB(&srCam,0);//returns the device ID used in other calls.
  //res=SR_OpenETH(&srCam,"10.0.1.146");//returns the device ID used in other calls.
  printf("libusbTester: SR_OpenXXX() called result:%d\n",res);
  if(res<0)
  {
    printf("libusbTester: SR_OpenXXX() falied. abort Test.");
    return;
  }

  for(cmd='\n';cmd!='x';)
  {
    res=SR_Acquire(srCam);
    ImgEntry* imgEntryArray;
    SR_GetImageList(srCam, &imgEntryArray);
    void* buf=imgEntryArray[0].data;
    int w,h;
    w=SR_GetCols(srCam);
    h=SR_GetRows(srCam);
  
    printf("libusbTester: SR_Acquire() called result:%d\n",res);
    //-----------------------------------------------------------
    //insert code here to process the acquired 3D data.
    //To query the format of the acquired data use the functions
    //SR_GetRows(), SR_GetCols(), SR_GetNumImg(), SR_GetBytePerPix()
    //that are described in the Swissranger.chm help file
    {
      printf("continue acquire?<any key>, dump data?<d>, exit?<x>:",cmd);cmd=_getch();
      putchar(cmd);
      switch(cmd)
      {
        case 'd':
          Dump((unsigned char*)buf,(int)w*h*2);
          //no break;
       default:
          break;
        case 'x':
          goto exitLoop;
          puts("unknown cmd");
      }
    }
  }
exitLoop:
  //-----------------------------------------------------------

  res=SR_Close(srCam);
  printf("libusbTester: SR_Close() called result:%d\n",res);
}


void CTestSRCustomer::TestContinousAcquire()
{
  CMesaDevice* srCam=0;
  int res, numImg, i;
  ImgEntry* imgEntryArray;
  Reset();

#ifdef __BFIN__
  Run("SR_Open");
  res=SR_Open(&srCam);
#else
  Run("SR_OpenDlg");
  res=SR_OpenDlg(&srCam,3,0);
#endif
  ABORT_ON(res<=0);
#ifdef __BFIN__ //avoid additional processing
  res=SR_SetMode(srCam,0);   Result(res==0);
#endif
  //--------- SetTimeout ---------
  Run("SetTimeout");
  SR_SetTimeout(srCam, 3000);
  //--------- GetImageList ---------
  Run("GetImageList");
  numImg=SR_GetImageList(srCam, &imgEntryArray);   Result(numImg>1 && imgEntryArray!=0);
  //--------- GetImage ---------
  Run("SR_Acquire");
  {
    int numPix=SR_GetRows(srCam)*SR_GetCols(srCam);
    int numBytes=numImg*numPix*2;
    unsigned short *data;
    SR_SetReg(srCam, 4,0);//0=distance data, 1=test data
    for(i=0;i<0xffff;i++)
    {
      res=SR_Acquire(srCam);
      if(res<0)
        Trace("SR_Acquire res=%d ",res);
      data=(unsigned short*)SR_GetImage(srCam, 0);
      printf("%d ",data[176*144-1]);
      if(!(i&0x0f))
      {
#ifndef __BFIN__
        ShowImg(srCam, imgEntryArray);
#endif
        putchar('\n');fflush(stdout);
      }
      if(!(i&0xff))
      {
        Trace("... %d images acquired",i);
#ifdef __BFIN__
        Dump((BYTE*)data,256);
#endif
      }

      //Sleep(2000);

      if(_kbhit())
        break;
    }
  }
  //--------- SR_Close ---------
  Run("SR_Close");
  res=SR_Close(srCam);
  Result(res==0);
  ReportSummary();
  _getch();
}

void CTestSRCustomer::TestUserSrvCmd()
{
#ifndef __BFIN__
  CMesaDevice* srCam;
  int res,i;
  Reset();
  puts("On the Camera the mesaserverCC\npress:\n x to stop\n 0: UsrRequest0\n 1: UsrRequest1\n");
  Run("CTestSRCustomer::TestUserSrvCmd");
  Run("SR_OpenDlg");
  res=SR_OpenDlg(&srCam,3,0);

  for(i=0;;i++)
  {
    SR_Acquire(srCam);
    if(_kbhit())
    {
      char c=_getch();
      switch(c)
      {
      case 'x':
        goto exitfor;
      case '0':
        {
          UsrCmd_Request0 tx;
          tx._p.cmd=CP_UsrCmd_Request0;
          tx._p.size=sizeof(UsrCmd_Request0);
          strcpy(tx.data,"HelloWorld");
          int res=SR_TCPSend(srCam, &tx, sizeof(tx), 0);
          printf("SR_TCPSend UsrCmd_Request0 returned %d %s\n",res,tx.data);
          break;
        }
      case '1':
        {
          UsrCmd_Request1 tx;
          UsrCmd_Acknowledge1 rx;
          tx._p.cmd=CP_UsrCmd_Request1;
          tx._p.size=sizeof(UsrCmd_Request1);
          strcpy(tx.data,"Ping...");
          int res=SR_TCPSend(srCam, &tx, sizeof(tx), 0);
          printf("SR_TCPSend returned %d %s\n",res,tx.data);
          res=SR_TCPRecv(srCam, &rx, sizeof(rx), 0);
          printf("SR_TCPRecv returned %d %s\n",res,rx.data);
          break;
        }
      }
    }
  }
exitfor:
  SR_Close(srCam);
#endif
}

//Repeated CoordTrf Tests
void CTestSRCustomer::TestTrial()
{
  CMesaDevice* srCam;
  int res;
  int w,h,i,j,idx;
  Reset();
  Run("CTestSRCustomer::Trial");

#ifdef __BFIN__
  Run("SR_Open");
  res=SR_Open(&srCam);
#else
  Run("SR_OpenDlg");
  res=SR_OpenDlg(&srCam,3,0);
#endif
  ImgEntry* imgEntryArray;
  //SR_SetReg(srCam,4,1);//Debug Data
  SR_SetIntegrationTime(srCam,5);
  w=SR_GetCols(srCam);
  h=SR_GetRows(srCam);
  SR_GetImageList(srCam, &imgEntryArray);

  const unsigned char iX[9]={0, 0, 0, (w-1)/2, (w-1)/2, (w-1)/2, w-1, w-1, w-1};
  const unsigned char iY[9]={0, (h-1)/2, h-1, 0, (h-1)/2, h-1, 0, (h-1)/2, h-1};
  unsigned short iDst[9];
  WORD* dst;
  int numPnt=_countof(iDst);
  int numPix=w*h;
  size_t bufSz=numPix*3*sizeof(double);
  void *buf=malloc(bufSz);   memset(buf,0xaf,bufSz);
  short *xS16=(short*)buf, *yS16=&xS16[numPix]; WORD *zU16=(WORD*)&yS16[numPix];
  float  *xFlt=(float*)buf,  *yFlt=&xFlt[numPix], *zFlt=&yFlt[numPix];
  double *xDbl=(double*)buf, *yDbl=&xDbl[numPix], *zDbl=&yDbl[numPix];
  int pitchS16X=sizeof(short),  pitchS16Y=sizeof(short), pitchU16Z=sizeof(WORD);
  int pitchFltX=sizeof(float),  pitchFltY=sizeof(float), pitchFltZ=sizeof(float);
  int pitchDblX=sizeof(double), pitchDblY=sizeof(double), pitchDblZ=sizeof(double);

  Run("SR_CoordTrf");putchar('\n');
  for(i=0;;)
  {
    idx=rand()%(numPix);

    printf("%d pixel data index %d (%d/%d)\n",i,idx,idx/w,idx%w);
    res=SR_Acquire(srCam); dst=(WORD*)SR_GetImage(srCam,0); for(j=0;j<numPnt;j++){iDst[j]=dst[iX[j]+iY[j]*w];}
    res=SR_CoordTrfPntUint16(srCam, iX, iY, iDst, 0,    0,    zU16, numPnt);//     Result(res==0);
    res=SR_CoordTrfPntUint16(srCam, iX, iY, iDst, xS16, yS16, zU16, numPnt);//     Result(res==0);
    res=SR_CoordTrfUint16(srCam, 0,    0,    zU16, pitchS16X, pitchS16Y, pitchU16Z);//     Result(res==0);
    res=SR_CoordTrfUint16(srCam, xS16, yS16, zU16, pitchS16X, pitchS16Y, pitchU16Z);//     Result(res==0);
    printf("x %d y %d z %d\n",(int)xS16[idx],(int)yS16[idx],(int)zU16[idx]);

    res=SR_CoordTrfPntFlt(srCam, iX, iY, iDst, 0,    0,    zFlt, numPnt);//     Result(res==0);
    res=SR_CoordTrfPntFlt(srCam, iX, iY, iDst, xFlt, yFlt, zFlt, numPnt);//     Result(res==0);
    res=SR_CoordTrfFlt(srCam, 0,    0,    zFlt, pitchFltX, pitchFltY, pitchFltZ);//     Result(res==0);
    res=SR_CoordTrfFlt(srCam, xFlt, yFlt, zFlt, pitchFltX, pitchFltY, pitchFltZ);//     Result(res==0);
    printf("x %f y %f z %f\n",xFlt[idx],yFlt[idx],zFlt[idx]);

    res=SR_CoordTrfPntDbl(srCam, iX, iY, iDst, 0,    0,    zDbl, numPnt);//     Result(res==0);
    res=SR_CoordTrfPntDbl(srCam, iX, iY, iDst, xDbl, yDbl, zDbl, numPnt);//     Result(res==0);
    res=SR_CoordTrfDbl(srCam, 0,    0,    zDbl, pitchDblX, pitchDblY, pitchDblZ);//     Result(res==0);
    res=SR_CoordTrfDbl(srCam, xDbl, yDbl, zDbl, pitchDblX, pitchDblY, pitchDblZ);//     Result(res==0);
    printf("x %f y %f z %f\n",xDbl[idx],yDbl[idx],zDbl[idx]);

    if(_kbhit())
      goto exitLoop;
  }
exitLoop:

  res=SR_Close(srCam);
  printf("libusbTester: SR_Close() called result:%d\n",res);

  Result(1);
  ReportSummary();
}

/*
//Repeated Acquire Tests
void CTestSRCustomer::TestTrial()
{
  CMesaDevice* srCam;
  int res;
  int w,h,i,j;
  Reset();
  Run("CTestSRCustomer::Trial");

#ifdef __BFIN__
  Run("SR_Open");
  res=SR_Open(&srCam);
#else
  Run("SR_OpenDlg");
  res=SR_OpenDlg(&srCam,3,0);
  //res=SR_OpenETH(&srCam,"10.0.1.109");
#endif
  ImgEntry* imgEntryArray;
  SR_SetReg(srCam,4,1);//Debug Data
  SR_SetIntegrationTime(srCam,5);
  w=SR_GetCols(srCam);
  h=SR_GetRows(srCam);
  SR_GetImageList(srCam, &imgEntryArray);
  unsigned short* buf=(unsigned short*)imgEntryArray[0].data;

  for(j=0;;)
  {
    for(i=0;i<1000;i++,j++)
    {
      res=SR_Acquire(srCam);
      if(res!=w*h*2*2)
      {
        printf("libusbTester ERROR: SR_Acquire() called result:%d\n",res);
      }
      printf("%d pixel data : %.4x %.4x %.4x %.4x %.4x %.4x %.4x %.4x\r",j+1,
             (int)(buf[0]),(int)(buf[1]),(int)(buf[2]),(int)(buf[3]),
             (int)(buf[4]),(int)(buf[5]),(int)(buf[6]),(int)(buf[7]));fflush(stdout);
      if(_kbhit())
        goto exitLoop;
    }
    putchar('\n');
  }
exitLoop:

  res=SR_Close(srCam);
  printf("libusbTester: SR_Close() called result:%d\n",res);

  Result(1);
  ReportSummary();
}*/

/*
//Repeated Open Close Tests
void CTestSRCustomer::TestTrial()
{
  CMesaDevice* srCam;
  int res;
  int i;
  Reset();
  Run("CTestSRCustomer::Trial");

  for(i=0;i<0xffff;i++)
  {
    Run("SR_OpenDlg");
    res=SR_OpenETH(&srCam,"192.168.1.33");
    Sleep(50);
    Trace("OpenClose %d %d",i,res);
    if(res!=0)
    {
      Run("SR_Close");
      res=SR_Close(srCam);srCam=0;
    }
      if(_kbhit())
        goto exitLoop;

  }
  exitLoop:
    Result(res==0);
    ReportSummary();
    _getch();

}
*/
//!@} libMesaSRTester
