#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include "ros/ros.h"

typedef void* HANDLE;
typedef int BOOL;

#ifndef MMC_SUCCESS
  #define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
  #define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
  #define MMC_MAX_LOG_MSG_SIZE 512
#endif

class EposMotion
{
private:
  void* g_pKeyHandle = 0;
  unsigned short g_usNodeId = 1;
  std::string g_deviceName = "EPOS2";
  std::string g_protocolStackName = "MAXON SERIAL V2";
  std::string g_interfaceName = "USB";
  std::string g_portName = "USB0";
  int g_baudrate = 1000000;

public:
  EposMotion(unsigned short id = 1, std::string device = "EPOS2", std::string protocol = "MAXON SERIAL V2",
             std::string interface = "USB", std::string port = "USB0", int baudrate = 1000000);
  ~EposMotion();
  void SetDefaultParameters(unsigned short id = 1, std::string device = "EPOS2", std::string protocol = "MAXON SERIAL V2",
                            std::string interface = "USB", std::string port = "USB0", int baudrate = 1000000);
  int OpenDevice(unsigned int* p_pErrorCode);
  int CloseDevice(unsigned int* p_pErrorCode);
  void PrintSettings();
  int ResetController(unsigned int* p_pErrorCode);
  int MotorSetting(unsigned int* p_pErrorCode);
  bool ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
  void Move(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long speed, unsigned int & p_rlErrorCode);
  void Halt(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
  void MotionMove(const long speed);
};