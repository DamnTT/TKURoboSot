#include "epos_motion.hpp"

typedef void* HANDLE;
typedef int BOOL;

EposMotion::EposMotion(unsigned short id, std::string device, std::string protocol,
                       std::string interface, std::string port, int baudrate)
{
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  SetDefaultParameters(id, device, protocol, interface, port, baudrate);

  if ((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS) {
    std::cerr << "Controller " << g_usNodeId << ": " << "OpenDevice" << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ulErrorCode << ")"<< std::endl;
  }
  if ((lResult = ResetController(&ulErrorCode))!=MMC_SUCCESS) {
    std::cerr << "Controller " << g_usNodeId << ": " << "ResetController" << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ulErrorCode << ")"<< std::endl;
  }
  if ((lResult = MotorSetting(&ulErrorCode))!=MMC_SUCCESS) {
    std::cerr << "Controller " << g_usNodeId << ": " << "MotorSetting" << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ulErrorCode << ")"<< std::endl;
  }
  if ((lResult = ProfileVelocityMode(g_pKeyHandle, g_usNodeId, ulErrorCode))!=MMC_SUCCESS) {
    std::cerr << "Controller " << g_usNodeId << ": " << "ProfileVelocityMode" << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ulErrorCode << ")"<< std::endl;
  }
}
EposMotion::~EposMotion()
{
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  if (VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &ulErrorCode) == 0) {
    std::cerr << "Controller " << g_usNodeId << ": " << "VCS_SetDisableState" << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ulErrorCode << ")"<< std::endl;
  }
  if ((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS) {
    std::cerr << "Controller " << g_usNodeId << ": " << "CloseDevice" << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ulErrorCode << ")"<< std::endl;
  }
}

void EposMotion::SetDefaultParameters(unsigned short id, std::string device, std::string protocol,
                                      std::string interface, std::string port, int baudrate)
{
  //USB
  g_usNodeId = id;
  g_deviceName = device;
  g_protocolStackName = protocol;
  g_interfaceName = interface;
  g_portName = port;
  g_baudrate = baudrate;
}

int EposMotion::OpenDevice(unsigned int* p_pErrorCode)
{
  int lResult = MMC_FAILED;

  char* pDeviceName = new char[255];
  char* pProtocolStackName = new char[255];
  char* pInterfaceName = new char[255];
  char* pPortName = new char[255];

  strcpy(pDeviceName, g_deviceName.c_str());
  strcpy(pProtocolStackName, g_protocolStackName.c_str());
  strcpy(pInterfaceName, g_interfaceName.c_str());
  strcpy(pPortName, g_portName.c_str());

  ROS_INFO("Open device...");
  std::cout<<"Node Id : "<<g_usNodeId<<", Device : "<<g_deviceName<<", Protocol : "
           <<g_protocolStackName<<", Interface : "<<g_interfaceName<<", Port : "<<g_portName<<", Baudrate : "<<g_baudrate<<std::endl;

  g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

  if (g_pKeyHandle!=0 && *p_pErrorCode == 0) {
    unsigned int lBaudrate = 0;
    unsigned int lTimeout = 0;

    if (VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0) {
      if (VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0) {
        if (VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0) {
          if (g_baudrate==(int)lBaudrate) {
            lResult = MMC_SUCCESS;
          }
        }
      }
    }
  }else {
    g_pKeyHandle = 0;
  }

  delete []pDeviceName;
  delete []pProtocolStackName;
  delete []pInterfaceName;
  delete []pPortName;

  return lResult;
}

int EposMotion::CloseDevice(unsigned int* p_pErrorCode)
{
  int lResult = MMC_FAILED;

  *p_pErrorCode = 0;

  ROS_INFO("Close device");

  if (VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0) {
    lResult = MMC_SUCCESS;
  }

  return lResult;
}

void EposMotion::PrintSettings()
{
  std::stringstream msg;

  msg << "default settings:" << std::endl;
  msg << "node id             = " << g_usNodeId << std::endl;
  msg << "device name         = '" << g_deviceName << "'" << std::endl;
  msg << "protocal stack name = '" << g_protocolStackName << "'" << std::endl;
  msg << "interface name      = '" << g_interfaceName << "'" << std::endl;
  msg << "port name           = '" << g_portName << "'"<< std::endl;
  msg << "baudrate            = " << g_baudrate;

  std::cout << msg.str() << std::endl;
}

int EposMotion::ResetController(unsigned int* p_pErrorCode)
{
  int lResult = MMC_SUCCESS;
  BOOL oIsFault = 0;

  // std::cerr << "Controller " << g_usNodeId << "errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
  if (VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0) {
    std::cerr << "Controller " << g_usNodeId << ": VCS_GetFaultState failed (result=" << lResult << ", errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
    lResult = MMC_FAILED;
  }

  if (lResult==0) {
    if (oIsFault) {
      ROS_INFO("clear fault, node = '%d'", g_usNodeId);

      if (VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0) {
        std::cerr << "Controller " << g_usNodeId << ": VCS_ClearFault failed (result=" << lResult << ", errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
        lResult = MMC_FAILED;
      }
    }

    if (lResult==0) {
      BOOL oIsEnabled = 0;

      if (VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0) {
        std::cerr << "Controller " << g_usNodeId << ": VCS_GetEnableState failed (result=" << lResult << ", errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
        lResult = MMC_FAILED;
      }

      if (lResult==0) {
        if (!oIsEnabled) {
          if (VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0) {
            std::cerr << "Controller " << g_usNodeId << ": VCS_SetEnableState failed (result=" << lResult << ", errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
            lResult = MMC_FAILED;
          }
        }
      }
    }
  }
  return lResult;
}

int EposMotion::MotorSetting(unsigned int* p_pErrorCode)
{
  int lResult = MMC_SUCCESS;
  if ((lResult = VCS_SetMotorType(g_pKeyHandle, g_usNodeId, MT_DC_MOTOR, p_pErrorCode)) != MMC_SUCCESS) {
    std::cerr << "Controller " << g_usNodeId << ": VCS_SetMotorType failed (result=" << lResult << ", errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
    return lResult;
  }else {
    if ((lResult = VCS_SetSensorType(g_pKeyHandle, g_usNodeId, ST_INC_ENCODER_2CHANNEL, p_pErrorCode)) != MMC_SUCCESS) {
      std::cerr << "Controller " << g_usNodeId << ": VCS_SetSensorType failed (result=" << lResult << ", errorCode=0x" << std::hex << p_pErrorCode << ")"<< std::endl;
      return lResult;
    }
  }
  return lResult;
}

bool EposMotion::ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
  int lResult = MMC_SUCCESS;
  std::cout << "set profile velocity mode, node = " << p_usNodeId << std::endl;

  if (VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0) {
    std::cerr << "Controller " << g_usNodeId << ": VCS_ActivateProfileVelocityMode failed (result=" << lResult << ", errorCode=0x" << std::hex << p_rlErrorCode << ")"<< std::endl;
    lResult = MMC_FAILED;
  }else {
    unsigned int max_acceleration = 10000;
    unsigned int profile_dacc = 5000;
    VCS_SetMaxAcceleration(p_DeviceHandle, p_usNodeId, max_acceleration, &p_rlErrorCode);
    VCS_SetVelocityProfile(p_DeviceHandle, p_usNodeId, max_acceleration, profile_dacc, &p_rlErrorCode);
  }
  return lResult;
}

void EposMotion::Move(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long speed, unsigned int & p_rlErrorCode)
{
  if (VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, speed, &p_rlErrorCode) == 0) {
    std::cerr << "Controller " << g_usNodeId << ": VCS_MoveWithVelocity failed (errorCode=0x" << std::hex << p_rlErrorCode << ")"<< std::endl;
  }
}
void EposMotion::Halt(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
  if (VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0) {
    std::cerr << "Controller " << g_usNodeId << ": VCS_HaltVelocityMovement failed (errorCode=0x" << std::hex << p_rlErrorCode << ")"<< std::endl;
  }
}

void EposMotion::MotionMove(const long speed)
{ 
  unsigned int ulErrorCode = 0;
  Move(g_pKeyHandle, g_usNodeId, speed, ulErrorCode);
}