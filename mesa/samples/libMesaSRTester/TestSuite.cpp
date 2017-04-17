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
//#include "stdafx.h"
#include "TestSuite.h"
#ifdef _WIN32
  #include <conio.h>//_putch
  #include <Windows.h>//max
#else
  #include <stdio.h>
  #include <string.h>
  #include <stdarg.h>
  #define max(a,b) ((a)>(b)?(a):(b))
  #define _putch putchar
  #define _snprintf  snprintf
  #define _vsnprintf vsnprintf
#endif
#ifndef _countof
  #define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif
const char* CTestSuite::_sTesting  ="****%.3d**** Testing %s\t";
const char* CTestSuite::_sResult   ="**** SR-Handle 0x%p: Result %6d\t%s\n";
const char* CTestSuite::_sSuccess  ="SUCCESS";
const char* CTestSuite::_sFailed   ="FAILED";
const char* CTestSuite::_sSkip     ="SKIPPED";
const char* CTestSuite::_sAbortTest="!!!!!! Test Aborted !!!!!!!!";
const char* CTestSuite::_sResStr   =">>>>   >>>> ";
const char* CTestSuite::_separator="-------------------------------------------------------";

//!function to test basic operation of the libMesaSR interface
//!function to dump the binary data
void CTestSuite::Dump(unsigned char* p,int num)
{
  puts("Dump of memory byte by byte regrouped in blocks of 8 byte\n"\
    "   bytes:  0 1 2 3 4 5 6 7| 8 9 a b c d e f|1011121314151617|18191a1b1c1d1e1f|\n"\
    "--------:-----------------+----------------+----------------+----------------|");
  for(int i=0;i<num;i++)
  {
    if(!(i&0x1f))
    {
      printf("%08x: ",i);
    }
    printf("%02x",p[i]);

    if(!((i+1)&0x07))
    {
      putchar('|');
      if(!((i+1)&0x1f))
      {
        putchar('\n');
      }
    }
  }
  putchar('\n');
}


bool CTestSuite::Run(const char* str, bool doit)
{
  _curTest=str;
  int i,len=printf(_sTesting,_tstCnt+1,_curTest);
  len=6-((len+7)>>3);
  for(i=0;i<len;i++)
    putchar('\t');
  _afterTest=true;

  if(!doit)
  {
    _tstSkip++;
    puts(_sSkip);
    _afterTest=false;
    _skipLst.push_back(str);
  }
  return doit;
}

void CTestSuite::Trace(const char* fmt,...)
{
  va_list marker;
  va_start( marker, fmt);
  int len=0;
  char buf[1024];
  len+=_vsnprintf(&buf[len], _countof(buf)-len, fmt, marker);
  char*p=(char*)buf,*q;
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
}

bool CTestSuite::Result(bool test)
{
  if(!_afterTest)
    Run(_curTest);
  const char* p;
  if(test)
  {
    p=_sSuccess;_tstCnt++;_tstPassed++;
  }
  else
  {
    p=_sFailed; _tstCnt++;
  }
  puts(p);
  _afterTest=false;
  return test;
}

void CTestSuite::ReportSummary()
{
  puts(_separator);
  printf("LibMesaSR_InterfaceTest Result: %d/%d passed, %d skipped\n", _tstPassed, _tstCnt, _tstSkip);
  if(!_skipLst.empty())
  {
    char buf[256];
    int len,pos;
    len  = _snprintf(buf,_countof(buf),_separator);
    pos  = max(len/2-4,0);
    pos +=_snprintf(& buf[pos],_countof(buf)-pos, " SKIPPED");
    if(pos<len)
      buf[pos]=' ';
    puts(buf);
    std::list<const char*>::iterator itr,itrEnd=_skipLst.end();
    for(itr=_skipLst.begin();itr!=itrEnd;itr++)
    {
      puts(*itr);
    }
  }
}
