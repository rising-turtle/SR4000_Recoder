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

#include <list>

class CTestSuite
{
public:
  CTestSuite();
  void Dump(unsigned char* p,int num);
  void Reset();
  bool Run(const char* str, bool doit=true);
  void Trace(const char* fmt,...);
  bool Result(bool test);
  void ReportSummary();

  static const char* _sTesting;
  static const char* _sSkip;
  static const char* _sResult;
  static const char* _sSuccess;
  static const char* _sFailed;
  static const char* _sAbortTest;
  static const char* _sResStr;
  static const char* _separator;
protected:
  bool _afterTest;
private:
  int _tstCnt, _tstSkip, _tstPassed;
  char _dump;
  const char* _curTest;
  std::list<const char*> _skipLst;
};

inline CTestSuite::CTestSuite():_curTest(0),_tstCnt(0),_tstPassed(0),_tstSkip(0),_dump(0),_afterTest(false)
{
}
inline void CTestSuite::Reset()
{
  _tstCnt=_tstSkip=_tstPassed=0;
  _skipLst.clear();
}

#define ABORT_ON(TEST)\
  if(!Result(!(TEST))){\
  puts(_sAbortTest);\
  ReportSummary();\
  return;}
