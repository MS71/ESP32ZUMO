// Most of the functionality of this library is based on the VL53L1X API
// provided by ST (STSW-IMG007), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2356), and
// VL53L1X datasheet.

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lidar.h"

static const char * TAG = "LIDAR";

lidar::lidar(int _I2CPortNumber)
  : sensor(_I2CPortNumber)
{
  I2CPortNumber = _I2CPortNumber;
  lidar_motor_steps = 0;
  lidar_motor_dir = 1;
  lidar_motor_phase_offset = CFG_LIDAR_OFFSET;
  last_range_mm = 0;
  min_range_mm = 0xffff;
  mean_min_range_mm = 0xffff;

  memset(&active_scan,0,sizeof(active_scan));
  frozen_scan = active_scan;
}

void lidar::start()
{
  ESP_LOGI(TAG, "start() ...");
  if (sensor.init())
  {
    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
  	sensor.setDistanceMode(VL53L1X::Medium);
  	sensor.setMeasurementTimingBudget(30000);
  	sensor.startContinuous(30);

    {
      i2c_cmd_handle_t CommandHandle = NULL;
      if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
      {
        uint16_t pwm = 1000;
        //uint16_t pwm = 0;
        i2c_master_start( CommandHandle );
        i2c_master_write_byte( CommandHandle, ( 8 << 1 ) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte( CommandHandle, CMD_LIDAR_SET_PWM, true);
        i2c_master_write_byte( CommandHandle, ((pwm)>>8)&0xff, true);
        i2c_master_write_byte( CommandHandle, ((pwm)>>0)&0xff, true);
        i2c_master_stop( CommandHandle );
        if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
        {
        }
        i2c_cmd_link_delete( CommandHandle );
      }
    }

  }
}

void lidar::handle()
{
  if(sensor.dataReady())
  {
    sensor.read();
    if( sensor.ranging_data.range_status == VL53L1X::RangeValid )
    {
      ESP_LOGI(TAG, "handle() step=%d range_mm=%d (%f,%f)",
      lidar_motor_steps,
      sensor.ranging_data.range_mm,
      sensor.ranging_data.peak_signal_count_rate_MCPS,
      sensor.ranging_data.ambient_count_rate_MCPS);

      active_scan.ranges[lidar_motor_steps] = sensor.ranging_data.range_mm;
    }
    else
    {
      active_scan.ranges[lidar_motor_steps] = 0xffff;
    }

    lidar_motor_steps += lidar_motor_dir;

    const uint8_t phasetable[] = {
       0b00001000,
       0b00001001,
       0b00000001,
       0b00000101,
       0b00000100,
       0b00000110,
       0b00000010,
       0b00001010,
    };

    uint16_t v = 0;
    v |= ((phasetable[(lidar_motor_steps+lidar_motor_phase_offset)&(sizeof(phasetable)-1)]>>0)&1)<<1;
    v |= ((phasetable[(lidar_motor_steps+lidar_motor_phase_offset)&(sizeof(phasetable)-1)]>>1)&1)<<2;
    v |= ((phasetable[(lidar_motor_steps+lidar_motor_phase_offset)&(sizeof(phasetable)-1)]>>2)&1)<<3;
    v |= ((phasetable[(lidar_motor_steps+lidar_motor_phase_offset)&(sizeof(phasetable)-1)]>>3)&1)<<4;

    {
      i2c_cmd_handle_t CommandHandle = NULL;
      if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
      {
        i2c_master_start( CommandHandle );
        i2c_master_write_byte( CommandHandle, ( 0x38 << 1 ) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte( CommandHandle, v|0x80, true);
        i2c_master_stop( CommandHandle );
        if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
        {
        }
        i2c_cmd_link_delete( CommandHandle );
      }
    }

    if( lidar_motor_dir == 1 )
    {
      if( lidar_motor_steps >= (NUM_LIDAR_SCANS-1) )
      {
        active_scan.t_last = esp_timer_get_time();
        active_scan.n = NUM_LIDAR_SCANS;
        active_scan.dir = lidar_motor_dir;
        frozen_scan = active_scan;
        active_scan.t_first = active_scan.t_last;
        lidar_motor_dir = -1;

        if( min_range_mm != 0xffff )
        {
          mean_min_range_mm = ((3*mean_min_range_mm)+min_range_mm)/4;
        }
        min_range_mm = 0xffff;
      }
    }
    else
    {
      if( lidar_motor_steps <= 0 )
      {
        active_scan.t_last = esp_timer_get_time();
        active_scan.n = NUM_LIDAR_SCANS;
        active_scan.dir = lidar_motor_dir;
        frozen_scan = active_scan;
        active_scan.t_first = active_scan.t_last;
        lidar_motor_dir = 1;
      }
    }

  }
}
