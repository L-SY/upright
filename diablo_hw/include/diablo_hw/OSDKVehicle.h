/**
 * @file OSDK_Vehicle.hpp
 * @brief main interface to interact with robot
 */
#pragma once

#include "OSDKHAL.h"
#include "OSDKVehicle.h"
#include "OSDKMovement.h"
#include "OSDKTelemetry.h"

namespace DIABLO
{
namespace OSDK
{

class Vehicle
{
public:
  Vehicle(HAL* hal) : hal(hal), movement_ctrl(NULL), telemetry(NULL)
  {
  }

  ~Vehicle()
  {
    if (movement_ctrl)
      delete movement_ctrl;
    if (telemetry)
      delete telemetry;
  }

  /**
   * @brief   Initialize SDK port to vehicle
   */
  uint8_t init(void);

public:
  HAL* hal;

public:
  Movement_Ctrl* movement_ctrl;
  Telemetry* telemetry;
};

}  // namespace OSDK
}  // namespace DIABLO
