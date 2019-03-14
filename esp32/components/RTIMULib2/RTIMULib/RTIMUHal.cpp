////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The MPU-9250 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

#ifdef ESP_PLATFORM
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#include "esp_log.h"
static const char * TAG = "RTIMU_HAL";

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

static const int I2CPortNumber = CONFIG_SSD1306_DEFAULT_I2C_PORT_NUMBER;
#endif

#include "IMUDrivers/RTIMU.h"

#if !defined(WIN32) && !defined(__APPLE__) && !defined(ESP_PLATFORM)

#include <linux/spi/spidev.h>

RTIMUHal::RTIMUHal()
{
    I2CPortNumberBus = 255;
    m_currentSlave = 255;
    I2CPortNumber = -1;
    m_SPI = -1;
    m_SPISpeed = 500000;
}

RTIMUHal::~RTIMUHal()
{
    HALClose();
}

bool RTIMUHal::HALOpen()
{
    char buf[32];
    unsigned char SPIMode = SPI_MODE_0;
    unsigned char SPIBits = 8;
    uint32_t SPISpeed = m_SPISpeed;

    if (m_busIsI2C) {
        if (I2CPortNumber >= 0)
            return true;

        if (I2CPortNumberBus == 255) {
            HAL_ERROR("No I2C bus has been set\n");
            return false;
        }
        sprintf(buf, "/dev/i2c-%d", I2CPortNumberBus);
        I2CPortNumber = open(buf, O_RDWR);
        if (I2CPortNumber < 0) {
            HAL_ERROR1("Failed to open I2C bus %d\n", I2CPortNumberBus);
            I2CPortNumber = -1;
            return false;
        }
    } else {
        if (m_SPIBus == 255) {
            HAL_ERROR("No SPI bus has been set\n");
            return false;
        }

        sprintf(buf, "/dev/spidev%d.%d", m_SPIBus, m_SPISelect);
        m_SPI = open(buf, O_RDWR);
        if (m_SPI < 0) {
            HAL_ERROR2("Failed to open SPI bus %d, select %d\n", m_SPIBus, m_SPISelect);
            m_SPI = -1;
            return false;
        }

        if (ioctl(m_SPI, SPI_IOC_WR_MODE, &SPIMode) < 0) {
            HAL_ERROR1("Failed to set WR SPI_MODE0 on bus %d", m_SPIBus);
            close(m_SPIBus);
            return false;
        }

        if (ioctl(m_SPI, SPI_IOC_RD_MODE, &SPIMode) < 0) {
            HAL_ERROR1("Failed to set RD SPI_MODE0 on bus %d", m_SPIBus);
            close(m_SPIBus);
            return false;
        }

        if (ioctl(m_SPI, SPI_IOC_WR_BITS_PER_WORD, &SPIBits) < 0) {
            HAL_ERROR1("Failed to set WR 8 bit mode on bus %d", m_SPIBus);
            close(m_SPIBus);
            return false;
        }

        if (ioctl(m_SPI, SPI_IOC_RD_BITS_PER_WORD, &SPIBits) < 0) {
            HAL_ERROR1("Failed to set RD 8 bit mode on bus %d", m_SPIBus);
            close(m_SPIBus);
            return false;
        }

        if (ioctl(m_SPI, SPI_IOC_WR_MAX_SPEED_HZ, &SPISpeed) < 0) {
             HAL_ERROR2("Failed to set WR %dHz on bus %d", SPISpeed, m_SPIBus);
             close(m_SPIBus);
             return false;
        }

        if (ioctl(m_SPI, SPI_IOC_RD_MAX_SPEED_HZ, &SPISpeed) < 0) {
             HAL_ERROR2("Failed to set RD %dHz on bus %d", SPISpeed, m_SPIBus);
             close(m_SPIBus);
             return false;
        }
    }
    return true;
}

void RTIMUHal::HALClose()
{
    I2CClose();
    SPIClose();
}

void RTIMUHal::I2CClose()
{
    if (I2CPortNumber >= 0) {
        close(I2CPortNumber);
        I2CPortNumber = -1;
        m_currentSlave = 255;
    }
}

void RTIMUHal::SPIClose()
{
    if (m_SPI >= 0) {
        close(m_SPI);
        m_SPI = -1;
    }
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char const data, const char *errorMsg)
{
    return HALWrite(slaveAddr, regAddr, 1, &data, errorMsg);
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char length, unsigned char const *data, const char *errorMsg)
{
    int result;
    unsigned char txBuff[MAX_WRITE_LEN + 1];
    char *ifType;

    if (m_busIsI2C) {
        if (!I2CSelectSlave(slaveAddr, errorMsg))
            return false;
        ifType = (char *)"I2C";
    } else {
        ifType = (char *)"SPI";
    }

    if (length == 0) {
        result = ifWrite(&regAddr, 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("%s write of regAddr failed - %s\n", ifType, errorMsg);
            return false;
        } else if (result != 1) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("%s write of regAddr failed (nothing written) - %s\n", ifType, errorMsg);
            return false;
        }
    } else {
        txBuff[0] = regAddr;
        memcpy(txBuff + 1, data, length);

        result = ifWrite(txBuff, length + 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("%s data write of %d bytes failed - %s\n", ifType, length, errorMsg);
            return false;
        } else if (result < (int)length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR4("%s data write of %d bytes failed, only %d written - %s\n", ifType, length, result, errorMsg);
            return false;
        }
    }
    return true;
}

bool RTIMUHal::ifWrite(unsigned char *data, unsigned char length)
{
    struct spi_ioc_transfer wrIOC;

    if (m_busIsI2C) {
        return write(I2CPortNumber, data, length);
    } else {
        memset(&wrIOC, 0, sizeof(wrIOC));
        wrIOC.tx_buf = (unsigned long) data;
        wrIOC.rx_buf = 0;
        wrIOC.len = length;
        return ioctl(m_SPI, SPI_IOC_MESSAGE(1), &wrIOC);
    }
}


bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    int tries, result, total;
    unsigned char rxBuff[MAX_READ_LEN + 1];
    struct spi_ioc_transfer rdIOC;

    if (m_busIsI2C) {
        if (!HALWrite(slaveAddr, regAddr, 0, NULL, errorMsg))
            return false;

        total = 0;
        tries = 0;

        while ((total < length) && (tries < 5)) {
            result = read(I2CPortNumber, data + total, length - total);

            if (result < 0) {
                if (strlen(errorMsg) > 0)
                    HAL_ERROR3("I2C read error from %d, %d - %s\n", slaveAddr, regAddr, errorMsg);
                return false;
            }

            total += result;

            if (total == length)
                break;

            delayMs(10);
            tries++;
        }

        if (total < length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("I2C read from %d, %d failed - %s\n", slaveAddr, regAddr, errorMsg);
            return false;
        }
    } else {
        rxBuff[0] = regAddr | 0x80;
        memcpy(rxBuff + 1, data, length);
        memset(&rdIOC, 0, sizeof(rdIOC));
        rdIOC.tx_buf = (unsigned long) rxBuff;
        rdIOC.rx_buf = (unsigned long) rxBuff;
        rdIOC.len = length + 1;

        if (ioctl(m_SPI, SPI_IOC_MESSAGE(1), &rdIOC) < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("SPI read error from %d - %s\n", regAddr, errorMsg);
            return false;
        }
        memcpy(data, rxBuff + 1, length);
    }
    return true;
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    int tries, result, total;
    unsigned char rxBuff[MAX_READ_LEN + 1];
    struct spi_ioc_transfer rdIOC;

    if (m_busIsI2C) {
        if (!I2CSelectSlave(slaveAddr, errorMsg))
            return false;

        total = 0;
        tries = 0;

        while ((total < length) && (tries < 5)) {
            result = read(I2CPortNumber, data + total, length - total);

            if (result < 0) {
                if (strlen(errorMsg) > 0)
                    HAL_ERROR2("I2C read error from %d - %s\n", slaveAddr, errorMsg);
                return false;
            }

            total += result;

            if (total == length)
                break;

            delayMs(10);
            tries++;
        }

        if (total < length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("I2C read from %d failed - %s\n", slaveAddr, errorMsg);
            return false;
        }
    } else {
        memset(&rdIOC, 0, sizeof(rdIOC));
        rdIOC.tx_buf = 0;
        rdIOC.rx_buf = (unsigned long) rxBuff;
        rdIOC.len = length;

        if (ioctl(m_SPI, SPI_IOC_MESSAGE(1), &rdIOC) < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR1("SPI read error from - %s\n", errorMsg);
            return false;
        }
        memcpy(data, rxBuff, length);
    }
    return true;
}


bool RTIMUHal::I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg)
{
    if (m_currentSlave == slaveAddr)
        return true;

    if (!HALOpen()) {
        HAL_ERROR1("Failed to open I2C port - %s\n", errorMsg);
        return false;
    }

    if (ioctl(I2CPortNumber, I2C_SLAVE, slaveAddr) < 0) {
        HAL_ERROR2("I2C slave select %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }

    m_currentSlave = slaveAddr;

    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
    usleep(1000 * milliSeconds);
}

#elif defined(ESP_PLATFORM)
RTIMUHal::RTIMUHal()
{
}

RTIMUHal::~RTIMUHal()
{
}


bool RTIMUHal::HALOpen()
{
    ESP_LOGV(TAG, "RTIMUHal::HALOpen() I2CPortNumber=%d",I2CPortNumber);
    return true;
}

void RTIMUHal::HALClose()
{
    ESP_LOGV(TAG, "RTIMUHal::HALClose()");
}

void RTIMUHal::I2CClose()
{
    ESP_LOGV(TAG, "RTIMUHal::I2CClose()");
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char const data, const char *errorMsg)
{
    return HALWrite(slaveAddr,regAddr,1,&data,errorMsg);
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char length, unsigned char const *data, const char *errorMsg)
{
    ESP_LOGV(TAG, "RTIMUHal::HALWrite()");

    bool retval = false;
    i2c_cmd_handle_t CommandHandle = NULL;
  	if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
  	{
  		i2c_master_start( CommandHandle );
  		i2c_master_write_byte( CommandHandle, ( slaveAddr << 1 ) | I2C_MASTER_WRITE, true);
      i2c_master_write_byte( CommandHandle, regAddr, true );
  		i2c_master_write( CommandHandle, (uint8_t*)&data[0], length-1, I2C_MASTER_ACK);
      if( length > 0 )
      {
        i2c_master_write( CommandHandle, (uint8_t*)&data[length-1], 1, I2C_MASTER_NACK);
      }
  		i2c_master_stop( CommandHandle );
  		if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
  		{
  			retval = true;
  		}
  		i2c_cmd_link_delete( CommandHandle );
  	}

    return true;
}


bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    ESP_LOGV(TAG, "RTIMUHal::HALRead() #1");

    bool retval = false;
    i2c_cmd_handle_t CommandHandle = NULL;
  	if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
  	{
  		i2c_master_start( CommandHandle );
  		i2c_master_write_byte( CommandHandle, ( slaveAddr << 1 ) | I2C_MASTER_WRITE, true);
      i2c_master_write_byte( CommandHandle, regAddr, true );
      i2c_master_start( CommandHandle );
      i2c_master_write_byte( CommandHandle, ( slaveAddr << 1 ) | I2C_MASTER_READ, true);
      if( length > 1 )
      {
  		    i2c_master_read( CommandHandle, &data[0], length-1, I2C_MASTER_ACK);
      }
      i2c_master_read( CommandHandle, &data[length-1], 1, I2C_MASTER_NACK);
  		i2c_master_stop( CommandHandle );
  		if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
  		{
  			retval = true;
  		}
  		i2c_cmd_link_delete( CommandHandle );
  	}

    return retval;
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    ESP_LOGV(TAG, "RTIMUHal::HALRead() #2");

    bool retval = false;
    i2c_cmd_handle_t CommandHandle = NULL;
  	if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
  	{
      i2c_master_start( CommandHandle );
      i2c_master_write_byte( CommandHandle, ( slaveAddr << 1 ) | I2C_MASTER_READ, true);
      if( length > 1 )
      {
  		    i2c_master_read( CommandHandle, &data[0], length-1, I2C_MASTER_ACK);
      }
      i2c_master_read( CommandHandle, &data[length-1], 1, I2C_MASTER_NACK);
  		i2c_master_stop( CommandHandle );
  		if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
  		{
  			retval = true;
  		}
  		i2c_cmd_link_delete( CommandHandle );
  	}

    return retval;
}


bool RTIMUHal::I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg)
{
    ESP_LOGV(TAG, "RTIMUHal::I2CSelectSlave()");
    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
    vTaskDelay(milliSeconds / portTICK_PERIOD_MS);
}
#else
//  just dummy routines for Windows

RTIMUHal::RTIMUHal()
{
}

RTIMUHal::~RTIMUHal()
{
}


bool RTIMUHal::HALOpen()
{
    return true;
}

void RTIMUHal::HALClose()
{
}

void RTIMUHal::I2CClose()
{
}

bool RTIMUHal::HALWrite(unsigned char , unsigned char ,
                   unsigned char const , const char *)
{
    return true;
}

bool RTIMUHal::HALWrite(unsigned char , unsigned char ,
                   unsigned char , unsigned char const *, const char *)
{
    return true;
}


bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    return true;
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    return true;
}


bool RTIMUHal::I2CSelectSlave(unsigned char , const char *)
{
    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
}
#endif
