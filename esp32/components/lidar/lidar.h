#pragma once

#include <stdint.h>
#include "esp_timer.h"

#include "VL53L1X.h"

/*
 * I2C ZUMO Command Codes
 */
#define CMD_LIDAR_SET_PWM       0x05

//#define NUM_LIDAR_SCANS   35
//#define CFG_LIDAR_OFFSET  2
#define NUM_LIDAR_SCANS   33
#define CFG_LIDAR_OFFSET  3

typedef struct
{
  int64_t  t_first;
  int64_t  t_last;
  uint16_t n;
  int8_t dir;
  uint16_t ranges[NUM_LIDAR_SCANS];
} LidarScan;

class lidar
{
  public:
    lidar(int I2CPortNumber);
    void start();
    void handle();

    bool getFrozenScan(LidarScan *scan)
    {
        if( scan!=NULL && frozen_scan.n!=0 && frozen_scan.t_first!=0 && frozen_scan.t_last!=0 )
        {
          *scan = frozen_scan;
          frozen_scan.t_first = 0;
          frozen_scan.t_last = 0;
          frozen_scan.n = 0;
          return true;
        }
        return false;
    }

    uint16_t getNumRanges() {
      return NUM_LIDAR_SCANS;
    }

    uint16_t getRange(int idx) {
      if( idx < NUM_LIDAR_SCANS )
      {
        return active_scan.ranges[idx];
      }
      return 0xffff;
    }

  private:
    int I2CPortNumber;

    VL53L1X sensor;

    int8_t lidar_motor_steps;
    int8_t lidar_motor_dir;
    int8_t lidar_motor_phase_offset;
    LidarScan active_scan;
    LidarScan frozen_scan;
    //uint16_t ranges[NUM_LIDAR_SCANS];
    uint16_t last_range_mm;
    uint16_t min_range_mm;
    uint16_t mean_min_range_mm;

};
