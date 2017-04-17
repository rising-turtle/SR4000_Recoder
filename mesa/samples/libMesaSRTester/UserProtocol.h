#pragma once
#include "MesaProtocol.h"

typedef enum {
  CP_UsrCmd_Request0=CP_USER,
  CP_UsrCmd_Acknowledge0,
  CP_UsrCmd_Request1,
  CP_UsrCmd_Acknowledge1,
}CPCmdCMK;

typedef struct
{
  CamUser _p;
  char data[20];
}UsrCmd_Request0;

typedef struct
{
  CamUser _p;
  char data[24];
}UsrCmd_Acknowledge0;

typedef struct
{
  CamUser _p;
  char data[28];
}UsrCmd_Request1;

typedef struct
{
  CamUser _p;
  char data[32];
}UsrCmd_Acknowledge1;

