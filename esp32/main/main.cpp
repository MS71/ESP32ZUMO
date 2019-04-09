/******************************************************************************
*
*
******************************************************************************
*/
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdarg.h>
#include <stdio.h>

static const char * TAG = "MAIN";
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#define ENABLE_WIFI
#define ENABLE_OTA
#define ENABLE_I2C
#undef ENABLE_DISPLAY
#define ENABLE_LIDAR
#define ENABLE_IMU_BNO055
#undef USE_RAWIMU
#undef ENABLE_RTIMULIB
#undef ENABLE_TCPLOG
#define ENABLE_WATCHDOG

#define CMD_BEEP                0x01
#define CMD_BATLEVEL            0x02
#define CMD_MOTORS_SET_SPEED    0x03
#define CMD_ENCODERS            0x04
#define CMD_GET_STATUS          0x06

#include "MiniPID.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_task_wdt.h"
#include "lwip/apps/sntp.h"
#include "esp_int_wdt.h"

#ifdef ENABLE_DISPLAY
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"
#endif

#ifdef ENABLE_IMU_BNO055
#include "BNO055ESP32.h"
#endif

#include "ros.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "ros/service_server.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "nvs_flash.h"

#include "ota_server.h"
#ifdef ENABLE_LIDAR
#include "lidar.h"
#endif

#ifdef USE_RAWIMU
#include "l3gd20h.h"
#include "lsm303d.h"
#endif

#ifdef ENABLE_RTIMULIB
#undef ENABLE_RTIMULIB_CAL
#include "RTIMULib.h"
#ifdef ENABLE_RTIMULIB_CAL
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"
#endif
#endif

#define WIFI_SSID 		CONFIG_WIFI_SSID
#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD

static const int I2CPortNumber = CONFIG_SSD1306_DEFAULT_I2C_PORT_NUMBER;
static const int SCLPin = CONFIG_SSD1306_DEFAULT_I2C_SCL_PIN;
static const int SDAPin = CONFIG_SSD1306_DEFAULT_I2C_SDA_PIN;

char rosCoreHostName[] = "z600.fritz.box";
ros::NodeHandle  nh;

char imucalib_buf[256] = "";
std_msgs::String imucalib_msg;
ros::Publisher pub_imucalib("imucalib", &imucalib_msg);

std_msgs::Float32 headingx_msg;
std_msgs::Float32 headingy_msg;
std_msgs::Float32 headingz_msg;
ros::Publisher pub_headingx("headingx", &headingx_msg);
ros::Publisher pub_headingy("headingy", &headingy_msg);
ros::Publisher pub_headingz("headingz", &headingz_msg);

std_msgs::Float32 ubat_msg;
ros::Publisher pub_ubat("ubat", &ubat_msg);

std_msgs::Int8 wifirssi_msg;
ros::Publisher pub_wifirssi("wifi_rssi", &wifirssi_msg);

std_msgs::UInt16MultiArray linesensor_msg;
ros::Publisher pub_linesensor("linesensor", &linesensor_msg);

std_msgs::Int32MultiArray encoder_msg;
ros::Publisher pub_encoder("encoder", &encoder_msg);

#ifdef ENABLE_LIDAR
std_msgs::UInt16MultiArray lidar_msg;
ros::Publisher pub_lidar("lidar", &lidar_msg);
#endif

sensor_msgs::LaserScan scan_msg;
ros::Publisher pub_scan("scan", &scan_msg);

//nav_msgs::Odometry odom_msg;
nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("odom_raw",&odom_msg);

void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
ros::Subscriber<geometry_msgs::Twist> sub_cmdvel("cmd_vel", cmd_vel_callback);

tf::TransformBroadcaster tf_broadcaster;

#ifdef ENABLE_RTIMULIB
RTIMUSettings *rtimu_settings = NULL;
RTIMU *rtimu = NULL;
RTIMU_DATA rtimu_data = {0};

#ifdef ENABLE_RTIMULIB_CAL
RTIMUMagCal 	*rtimu_magcal = NULL;
RTIMUAccelCal	*rtimu_acccal = NULL;
#endif
#endif

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu_data",&imu_msg);

#ifdef ENABLE_IMU_BNO055
BNO055* bno055 = NULL;
bno055_calibration_t bno055_calib;
bool bno055_configured = false;
#endif

#ifdef ENABLE_WIFI
char wifistate[4] = "---";
bool wificonnected = false;
static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;
#endif

#ifdef ENABLE_LIDAR
lidar mylidar(CONFIG_SSD1306_DEFAULT_I2C_PORT_NUMBER);
#endif

struct
{
	uint32_t 	loopcnt;
	uint16_t 	bat;
	int32_t 	enc_l;
	int32_t 	enc_r;
	uint16_t  linesensors[5];
} m32u4_status;

extern "C" {
	void app_main(void);
}

void vBeep(int frequency, int duration, int volume)
{
	i2c_cmd_handle_t CommandHandle = NULL;
	if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
	{
		i2c_master_start( CommandHandle );
		i2c_master_write_byte( CommandHandle, ( 8 << 1 ) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte( CommandHandle, CMD_BEEP, true);
		i2c_master_write_byte( CommandHandle, ((frequency)>>8)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((frequency)>>0)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((duration)>>8)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((duration)>>0)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((volume)>>0)&0xff, true);
		i2c_master_stop( CommandHandle );
		if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
		{
		}
		i2c_cmd_link_delete( CommandHandle );
	}
}

#ifdef ENABLE_DISPLAY
static const int I2CDisplayAddress = 0x3C;
static const int I2CDisplayWidth = 128;
static const int I2CDisplayHeight = 64;
static const int I2CResetPin = -1;
struct SSD1306_Device I2CDisplay;

void vUpdateDisplay(struct SSD1306_Device* DisplayHandle)
{
	int64_t t = esp_timer_get_time();
	static int64_t _t = 0;
	if( t > _t )
	{
		_t = t + 1000000;
		wifi_ap_record_t ap_info;
		char tmpstr[64];
		static uint8_t alivecnt = 0;
		int8_t wifi_rssi = -127;

		if( wificonnected == true )
		{
			if( ESP_OK == esp_wifi_sta_get_ap_info( &ap_info) )
			{
				wifi_rssi = ap_info.rssi;
			}
		}

		SSD1306_Clear( DisplayHandle, SSD_COLOR_BLACK );

		SSD1306_SetFont( DisplayHandle, &Font_droid_sans_mono_13x24 );
		sprintf(tmpstr,"%s|%d%d|%1.1f",
			wifistate,
			(alivecnt++)%10,
			(m32u4_status.loopcnt/100)%10,
			((float)m32u4_status.bat/1000.0));
		SSD1306_FontDrawString( DisplayHandle, 0, 0, tmpstr, SSD_COLOR_WHITE );

		sprintf(tmpstr,"%02d|%03d|%03d",
		  (99*(128-(0-wifi_rssi)))/128,
			(int)((m32u4_status.enc_r/100)%100),
		  (int)((m32u4_status.enc_l/100)%100));
		SSD1306_FontDrawString( DisplayHandle, 0/*(I2CDisplayWidth-9*13)/2*/, 24, tmpstr, SSD_COLOR_WHITE );

		SSD1306_DrawLine(DisplayHandle,0,23,I2CDisplayWidth-1,23,SSD_COLOR_WHITE);

		SSD1306_DrawLine(DisplayHandle,0,47,I2CDisplayWidth-1,47,SSD_COLOR_WHITE);

#ifdef ENABLE_LIDAR
		{
			int n = mylidar.getNumRanges();
			int bw = I2CDisplayWidth/n;
			int x = (I2CDisplayWidth-(n*bw))/2;
			int y = 48;
			SSD1306_DrawLine(DisplayHandle,x-1,48,x-1,(I2CDisplayHeight-1),SSD_COLOR_WHITE);
			SSD1306_DrawLine(DisplayHandle,x+n*bw+1,48,x+n*bw+1,(I2CDisplayHeight-1),SSD_COLOR_WHITE);
			for (int i=0;i<n;i++)
			{
				int v = mylidar.getRange(i)/50;
				if( v > ((I2CDisplayHeight-1) - y) ) {
					v = ((I2CDisplayHeight-1) - y);
				}
				if( (y+v) < (I2CDisplayHeight-1) )
				{
					SSD1306_DrawBox( DisplayHandle, x+i*bw, y+v, (x+(i+1)*bw)-1, (I2CDisplayHeight-1), SSD_COLOR_WHITE, true );
				}
			}
		}
#endif

		SSD1306_Update( DisplayHandle );
	}
}
#endif

int64_t odo_last_time = 0;

void cmd_vel_motor_update();

double motor_speed_l = 0.0;
double motor_speed_r = 0.0;

void vHandleEncoderSteps(int16_t enc_l,int16_t enc_r)
{
	int64_t current_time = esp_timer_get_time();
	double dt = 0.000001*(current_time - odo_last_time);
	odo_last_time = current_time;

	/*
	 * https://answers.ros.org/question/207392/generating-odom-message-from-encoder-ticks-for-robot_pose_ekf/
	 * http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm
	 */
	static double x = 0;
	static double y = 0;
	static double th = 0;

	/*
	 * Pololu Wheel Encoder
	 */
	double gear = 100.0/1.0;
	double steps_per_revolution = 12.0;
	double wheel_diameter = 0.038; // 38mm
	double lengthBetweenTwoWheels = 0.09; // 9cm
	double pi = 3.14159265359;
	//double DistancePerCount = 0.0001;
	double DistancePerCount = (pi*wheel_diameter) / (steps_per_revolution*gear);

	//extract the wheel velocities from the tick signals count
	double deltaLeft = enc_l;
	double deltaRight = enc_r;

	double v_left = (deltaLeft * DistancePerCount) / dt;
	double v_right = (deltaRight * DistancePerCount) / dt;

	//motor_md[MOTOR_L].odom_rate = (4.0 * motor_md[MOTOR_L].odom_rate + v_left) / 5.0;
	//motor_md[MOTOR_R].odom_rate = (4.0 * motor_md[MOTOR_R].odom_rate + v_right) / 5.0;

	double vx = ((v_right + v_left) / 2);
	double vy = 0;
	double vth = ((v_right - v_left)/lengthBetweenTwoWheels);

	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	motor_speed_l = v_left;
	motor_speed_r = v_right;
	cmd_vel_motor_update();

#if 0
ESP_LOGI(TAG, "vHandleEncoderSteps(%d,%d) dT=%f v=(%f,%f)|(%f,%f) x=%f y=%f th=%f",
	enc_l,enc_r,
	dt,
	v_left,v_right,
	motor_speed_l,motor_speed_r,
	x,y,th);
#endif

	if( nh.connected() )
	{
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

	#if 0
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = nh.now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		tf_broadcaster.sendTransform(odom_trans);
	#endif

		//Odometry message
		odom_msg.header.stamp = nh.now();
		odom_msg.header.seq = odom_msg.header.seq + 1;
		odom_msg.header.frame_id = "odom";

		//set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		//set the velocity
		odom_msg.child_frame_id = "base_link";
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.linear.y = vy;
		odom_msg.twist.twist.angular.z = vth;

#if 1
		odom_msg.pose.covariance[0] = -1;
		odom_msg.twist.covariance[0] = -1;
#else
		if (v_left == 0 && v_right == 0){
			odom_msg.pose.covariance[0] = 1e-9;
			odom_msg.pose.covariance[7] = 1e-3;
			odom_msg.pose.covariance[8] = 1e-9;
			odom_msg.pose.covariance[14] = 1e6;
			odom_msg.pose.covariance[21] = 1e6;
			odom_msg.pose.covariance[28] = 1e6;
			odom_msg.pose.covariance[35] = 1e-9;
			odom_msg.twist.covariance[0] = 1e-9;
			odom_msg.twist.covariance[7] = 1e-3;
			odom_msg.twist.covariance[8] = 1e-9;
			odom_msg.twist.covariance[14] = 1e6;
			odom_msg.twist.covariance[21] = 1e6;
			odom_msg.twist.covariance[28] = 1e6;
			odom_msg.twist.covariance[35] = 1e-9;
		}
		else
		{
			odom_msg.pose.covariance[0] = 1e-3;
			odom_msg.pose.covariance[7] = 1e-3;
			odom_msg.pose.covariance[8] = 0.0;
			odom_msg.pose.covariance[14] = 1e6;
			odom_msg.pose.covariance[21] = 1e6;
			odom_msg.pose.covariance[28] = 1e6;
			odom_msg.pose.covariance[35] = 1e3;
			odom_msg.twist.covariance[0] = 1e-3;
			odom_msg.twist.covariance[7] = 1e-3;
			odom_msg.twist.covariance[8] = 0.0;
			odom_msg.twist.covariance[14] = 1e6;
			odom_msg.twist.covariance[21] = 1e6;
			odom_msg.twist.covariance[28] = 1e6;
			odom_msg.twist.covariance[35] = 1e3;
		}
#endif

		//publish the message
		pub_odom.publish(&odom_msg);
	}
}

bool nReadM32U4Status()
{
	bool res = false;
	i2c_cmd_handle_t CommandHandle = NULL;
	if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
	{
		uint8_t buf[20];
		int n = sizeof(buf);
		i2c_master_start( CommandHandle );
		i2c_master_write_byte( CommandHandle, ( 8 << 1 ) | I2C_MASTER_READ, true);
		i2c_master_read( CommandHandle, &buf[0], n-1, I2C_MASTER_ACK);
		i2c_master_read( CommandHandle, &buf[n-1], 1, I2C_MASTER_NACK);
		i2c_master_stop( CommandHandle );
		if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
		{
			//memcpy(&m32u4_status,buf,sizeof(m32u4_status));
			int j=0;
			m32u4_status.loopcnt = 0;
			m32u4_status.loopcnt |= (buf[j++]<<24);
			m32u4_status.loopcnt |= (buf[j++]<<16);
			m32u4_status.loopcnt |= (buf[j++]<<8);
			m32u4_status.loopcnt |= (buf[j++]<<0);
			m32u4_status.bat = 0;
			m32u4_status.bat |= (buf[j++]<<8);
			m32u4_status.bat |= (buf[j++]<<0);
			int16_t enc_l = 0;
			enc_l |= (buf[j++]<<8);
			enc_l |= (buf[j++]<<0);
			int16_t enc_r = 0;
			enc_r |= (buf[j++]<<8);
			enc_r |= (buf[j++]<<0);
			vHandleEncoderSteps(enc_l,enc_r);
			m32u4_status.enc_l += enc_l;
			m32u4_status.enc_r += enc_r;
			for(int i=0;i<5;i++)
			{
				m32u4_status.linesensors[i] = 0;
				m32u4_status.linesensors[i] |= (buf[j++]<<8);
				m32u4_status.linesensors[i] |= (buf[j++]<<0);
			}
			res = true;
		}
		i2c_cmd_link_delete( CommandHandle );
	}
	return res;
}

MiniPID motor_pid_l(.02,.1,0.5);
MiniPID motor_pid_r(.02,.1,0.5);

void cmd_vel_motor_update()
{
	i2c_cmd_handle_t CommandHandle = NULL;
	if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
	{
		int16_t l = motor_pid_l.getOutput(motor_speed_l)*1000.0;
		int16_t r = motor_pid_r.getOutput(motor_speed_r)*1000.0;

#if 0
		ESP_LOGI(TAG, "cmd_vel_motor_update L(%f,%f=>%d) R(%f,%f=>%d)",
		motor_pid_l.getSetpoint(),motor_speed_l,l,
		motor_pid_r.getSetpoint(),motor_speed_r,r);
#endif

		i2c_master_start( CommandHandle );
		i2c_master_write_byte( CommandHandle, ( 8 << 1 ) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte( CommandHandle, CMD_MOTORS_SET_SPEED, true);
		i2c_master_write_byte( CommandHandle, ((l)>>8)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((l)>>0)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((r)>>8)&0xff, true);
		i2c_master_write_byte( CommandHandle, ((r)>>0)&0xff, true);
		i2c_master_stop( CommandHandle );
		if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
		{
		}
		i2c_cmd_link_delete( CommandHandle );
	}
}

void cmd_vel_motor_stop()
{
  motor_pid_l.setSetpoint(0);
  motor_pid_r.setSetpoint(0);
  cmd_vel_motor_update();
}

#ifdef USE_RAWIMU
l3gd20h_sensor_t* l3gd20h;
l3gd20h_float_data_t  data;

lsm303d_sensor_t* lsm303d;
lsm303d_float_a_data_t  a_data;
lsm303d_float_m_data_t  m_data;
float lsm303d_temp;

void vHandle_l3gd20h_and_lsm303d()
{
	if (l3gd20h_new_data (l3gd20h))
	{
		int64_t current_time = esp_timer_get_time();
		static int64_t previous_time = 0;

		l3gd20h_get_float_data(l3gd20h, &data);
		if( previous_time != 0 )
		{
			double dt = 0.000001*(current_time - previous_time);
			static double gyroXangle = 0.0;
			static double gyroYangle = 0.0;
			static double gyroZangle = 0.0;

			gyroXangle += data.x * dt;
			gyroYangle += data.y * dt;
			gyroZangle += data.z * dt;

			// max. full scale is +-2000 dps and best sensitivity is 1 mdps, i.e. 7 digits
			ESP_LOGE(TAG, "L3GD20H (xyz)[dps]: %+6.1f %+6.1f %+6.1f %+6.1f %+6.1f %+6.1f",
				data.x, gyroXangle,
				data.y, gyroYangle,
				data.z, gyroZangle);
		}
		previous_time = current_time;
	}

	// test for new accelerator data and fetch them
	if (lsm303d_new_a_data (lsm303d) &&
			lsm303d_get_float_a_data (lsm303d, &a_data))

	// max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
	ESP_LOGV(TAG, "LSM303D (xyz)[g]  ax=%+7.3f ay=%+7.3f az=%+7.3f",
				 a_data.ax, a_data.ay, a_data.az);

	// test for new magnetometer data and fetch them
	if (lsm303d_new_m_data (lsm303d) &&
			lsm303d_get_float_m_data (lsm303d, &m_data))

	// max. full scale is +-12 Gs and best resolution is 1 mGs, i.e. 5 digits
	ESP_LOGV(TAG,"LSM303D (xyz)[Gs] mx=%+7.3f my=%+7.3f mz=%+7.3f",
				 m_data.mx, m_data.my, m_data.mz);

	lsm303d_temp = lsm303d_get_temperature (lsm303d);
	ESP_LOGV(TAG,"LSM303D (tmp)[°C] %+7.3f", lsm303d_temp);
}
#endif

#ifdef ENABLE_RTIMULIB
#if 0
void svcCallbackRTIIMUCallibrate(const std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response)
{
	ESP_LOGI(TAG,"svcCallbackRTIIMUCallibrate %d",request.data);
	response.message = "ok";
}

ros::ServiceServer<std_srvs::SetBoolRequest,std_srvs::SetBoolResponse> service_rtiimu_callibrate("/rtiimu_callibrate_srv",&svcCallbackRTIIMUCallibrate);
#endif
#endif

void I2CThread(void *pvParameters)
{
	bool published = false;

	ESP_LOGI(TAG, "I2CThread() ...");

#ifdef ENABLE_IMU_BNO055
	bno055 = new BNO055((i2c_port_t)I2CPortNumber,0x28);
	if( bno055 != NULL )
	{
		try {
			bno055_configured = false;
			ESP_LOGI(TAG, "I2CThread() BNO055 init ...");
			bno055->begin();  // BNO055 is in CONFIG_MODE until it is changed
			bno055->setOprModeConfig();
			bno055->enableExternalCrystal();
			//bno.setSensorOffsets(storedOffsets);
			//bno055->setAxisRemap(BNO055_REMAP_CONFIG_P0, BNO055_REMAP_SIGN_P0); // see datasheet, section 3.4
			//bno055->setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
			bno055->setAxisRemap(BNO055_REMAP_CONFIG_P2, BNO055_REMAP_SIGN_P2); // see datasheet, section 3.4
			//xxbno055->setAxisRemap(BNO055_REMAP_CONFIG_P3, BNO055_REMAP_SIGN_P3); // see datasheet, section 3.4

			bno055->setUnits(BNO055_UNIT_ACCEL_MS2,
										BNO055_UNIT_ANGULAR_RATE_RPS,
										BNO055_UNIT_EULER_DEGREES,
										BNO055_UNIT_TEMP_C,
										BNO055_DATA_FORMAT_ANDROID);

			#if 0
			bno055->setAccelConfig(BNO055_CONF_ACCEL_RANGE_4G,
														 BNO055_CONF_ACCEL_BANDWIDTH_7_81HZ,
 													 	 BNO055_CONF_ACCEL_MODE_NORMAL);
			#endif
			/* you can specify a PoWeRMode using:
						- setPwrModeNormal(); (Default on startup)
						- setPwrModeLowPower();
						- setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
						*/
			//bno055->setOprModeNdof();
			ESP_LOGI(TAG, "I2CThread() BNO055 init done");
		} catch ( BNO055BaseException ex ) {
			ESP_LOGI(TAG, "I2CThread() BNO055 exception %s",ex.what());
		}
	}
#endif

#ifdef ENABLE_RTIMULIB
	rtimu_settings  = new RTIMUSettings("RTIMULib");
	rtimu_settings->m_imuType = RTIMU_TYPE_AUTODISCOVER;
	rtimu_settings->m_I2CSlaveAddress = 0;
	rtimu_settings->m_busIsI2C = true;
	rtimu_settings->m_I2CBus = (i2c_port_t)I2CPortNumber;
	rtimu_settings->m_SPIBus = 0;
	rtimu_settings->m_SPISelect = 0;
	rtimu_settings->m_SPISpeed = 400000;
	rtimu_settings->m_fusionType = RTFUSION_TYPE_RTQF;
	rtimu_settings->m_axisRotation = RTIMU_XNORTH_YEAST;
	rtimu_settings->m_pressureType = RTPRESSURE_TYPE_AUTODISCOVER;
	rtimu_settings->m_I2CPressureAddress = 0;
	rtimu_settings->m_humidityType = RTHUMIDITY_TYPE_AUTODISCOVER;
	rtimu_settings->m_I2CHumidityAddress = 0;
	rtimu_settings->m_compassCalValid = false;
	rtimu_settings->m_compassCalEllipsoidValid = false;
	for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
					rtimu_settings->m_compassCalEllipsoidCorr[i][j] = 0;
			}
	}
	rtimu_settings->m_compassCalEllipsoidCorr[0][0] = 1;
	rtimu_settings->m_compassCalEllipsoidCorr[1][1] = 1;
	rtimu_settings->m_compassCalEllipsoidCorr[2][2] = 1;

	rtimu_settings->m_compassAdjDeclination = 0;

	rtimu_settings->m_accelCalValid = false;
	rtimu_settings->m_gyroBiasValid = false;

	//  MPU9150 defaults

	rtimu_settings->m_MPU9150GyroAccelSampleRate = 50;
	rtimu_settings->m_MPU9150CompassSampleRate = 25;
	rtimu_settings->m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
	rtimu_settings->m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
	rtimu_settings->m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;

	//  MPU9250 defaults

	rtimu_settings->m_MPU9250GyroAccelSampleRate = 80;
	rtimu_settings->m_MPU9250CompassSampleRate = 40;
	rtimu_settings->m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;
	rtimu_settings->m_MPU9250AccelLpf = MPU9250_ACCEL_LPF_41;
	rtimu_settings->m_MPU9250GyroFsr = MPU9250_GYROFSR_1000;
	rtimu_settings->m_MPU9250AccelFsr = MPU9250_ACCELFSR_8;

	//  GD20HM303D defaults

	rtimu_settings->m_GD20HM303DGyroSampleRate = L3GD20H_SAMPLERATE_50;
	rtimu_settings->m_GD20HM303DGyroBW = L3GD20H_BANDWIDTH_1;
	rtimu_settings->m_GD20HM303DGyroHpf = L3GD20H_HPF_4;
	rtimu_settings->m_GD20HM303DGyroFsr = L3GD20H_FSR_500;

	rtimu_settings->m_GD20HM303DAccelSampleRate = LSM303D_ACCEL_SAMPLERATE_50;
	rtimu_settings->m_GD20HM303DAccelFsr = LSM303D_ACCEL_FSR_8;
	rtimu_settings->m_GD20HM303DAccelLpf = LSM303D_ACCEL_LPF_50;

	rtimu_settings->m_GD20HM303DCompassSampleRate = LSM303D_COMPASS_SAMPLERATE_50;
	rtimu_settings->m_GD20HM303DCompassFsr = LSM303D_COMPASS_FSR_2;

	//  GD20M303DLHC defaults

	rtimu_settings->m_GD20M303DLHCGyroSampleRate = L3GD20_SAMPLERATE_95;
	rtimu_settings->m_GD20M303DLHCGyroBW = L3GD20_BANDWIDTH_1;
	rtimu_settings->m_GD20M303DLHCGyroHpf = L3GD20_HPF_4;
	rtimu_settings->m_GD20M303DLHCGyroFsr = L3GD20H_FSR_500;

	rtimu_settings->m_GD20M303DLHCAccelSampleRate = LSM303DLHC_ACCEL_SAMPLERATE_50;
	rtimu_settings->m_GD20M303DLHCAccelFsr = LSM303DLHC_ACCEL_FSR_8;

	rtimu_settings->m_GD20M303DLHCCompassSampleRate = LSM303DLHC_COMPASS_SAMPLERATE_30;
	rtimu_settings->m_GD20M303DLHCCompassFsr = LSM303DLHC_COMPASS_FSR_1_3;

	//  GD20HM303DLHC defaults

	rtimu_settings->m_GD20HM303DLHCGyroSampleRate = L3GD20H_SAMPLERATE_50;
	rtimu_settings->m_GD20HM303DLHCGyroBW = L3GD20H_BANDWIDTH_1;
	rtimu_settings->m_GD20HM303DLHCGyroHpf = L3GD20H_HPF_4;
	rtimu_settings->m_GD20HM303DLHCGyroFsr = L3GD20H_FSR_500;

	rtimu_settings->m_GD20HM303DLHCAccelSampleRate = LSM303DLHC_ACCEL_SAMPLERATE_50;
	rtimu_settings->m_GD20HM303DLHCAccelFsr = LSM303DLHC_ACCEL_FSR_8;

	rtimu_settings->m_GD20HM303DLHCCompassSampleRate = LSM303DLHC_COMPASS_SAMPLERATE_30;
	rtimu_settings->m_GD20HM303DLHCCompassFsr = LSM303DLHC_COMPASS_FSR_1_3;

	//  LSM9DS0 defaults

	rtimu_settings->m_LSM9DS0GyroSampleRate = LSM9DS0_GYRO_SAMPLERATE_95;
	rtimu_settings->m_LSM9DS0GyroBW = LSM9DS0_GYRO_BANDWIDTH_1;
	rtimu_settings->m_LSM9DS0GyroHpf = LSM9DS0_GYRO_HPF_4;
	rtimu_settings->m_LSM9DS0GyroFsr = LSM9DS0_GYRO_FSR_500;

	rtimu_settings->m_LSM9DS0AccelSampleRate = LSM9DS0_ACCEL_SAMPLERATE_50;
	rtimu_settings->m_LSM9DS0AccelFsr = LSM9DS0_ACCEL_FSR_8;
	rtimu_settings->m_LSM9DS0AccelLpf = LSM9DS0_ACCEL_LPF_50;

	rtimu_settings->m_LSM9DS0CompassSampleRate = LSM9DS0_COMPASS_SAMPLERATE_50;
	rtimu_settings->m_LSM9DS0CompassFsr = LSM9DS0_COMPASS_FSR_2;

	//  LSM9DS1 defaults

	rtimu_settings->m_LSM9DS1GyroSampleRate = LSM9DS1_GYRO_SAMPLERATE_119;
	rtimu_settings->m_LSM9DS1GyroBW = LSM9DS1_GYRO_BANDWIDTH_1;
	rtimu_settings->m_LSM9DS1GyroHpf = LSM9DS1_GYRO_HPF_4;
	rtimu_settings->m_LSM9DS1GyroFsr = LSM9DS1_GYRO_FSR_500;

	rtimu_settings->m_LSM9DS1AccelSampleRate = LSM9DS1_ACCEL_SAMPLERATE_119;
	rtimu_settings->m_LSM9DS1AccelFsr = LSM9DS1_ACCEL_FSR_8;
	rtimu_settings->m_LSM9DS1AccelLpf = LSM9DS1_ACCEL_LPF_50;

	rtimu_settings->m_LSM9DS1CompassSampleRate = LSM9DS1_COMPASS_SAMPLERATE_20;
	rtimu_settings->m_LSM9DS1CompassFsr = LSM9DS1_COMPASS_FSR_4;
	// BMX055 defaults

	rtimu_settings->m_BMX055GyroSampleRate = BMX055_GYRO_SAMPLERATE_100_32;
	rtimu_settings->m_BMX055GyroFsr = BMX055_GYRO_FSR_500;

	rtimu_settings->m_BMX055AccelSampleRate = BMX055_ACCEL_SAMPLERATE_125;
	rtimu_settings->m_BMX055AccelFsr = BMX055_ACCEL_FSR_8;

	rtimu_settings->m_BMX055MagPreset = BMX055_MAG_REGULAR;

	rtimu = RTIMU::createIMU(rtimu_settings);
	rtimu->IMUInit();

		// Set the Fusion coefficient
	rtimu->setSlerpPower(0.02);
	// Enable the sensors
	rtimu->setGyroEnable(true);
	rtimu->setAccelEnable(true);
	rtimu->setCompassEnable(true);

#ifdef ENABLE_RTIMULIB_CAL
	while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
#endif

#endif

motor_pid_l.setOutputLimits(-255,255);
motor_pid_r.setOutputLimits(-255,255);

#ifdef USE_RAWIMU
	// init the sensor with slave address L3GD20H_I2C_ADDRESS_2 connected to I2C_BUS.
	l3gd20h = l3gd20h_init_sensor ((i2c_port_t)I2CPortNumber, L3GD20H_I2C_ADDRESS_2, 0);

	// select LPF/HPF, configure HPF and reset the reference by dummy read
	l3gd20h_select_output_filter (l3gd20h, l3gd20h_hpf_and_lpf2);
	l3gd20h_config_hpf (l3gd20h, l3gd20h_hpf_normal, 0);
	l3gd20h_get_hpf_ref (l3gd20h);

	// LAST STEP: Finally set scale and sensor mode to start measurements
	l3gd20h_set_scale(l3gd20h, l3gd20h_scale_245_dps);
	l3gd20h_set_mode (l3gd20h, l3gd20h_normal_odr_12_5, 3, true, true, true);

	// init the sensor with slave address LSM303D_I2C_ADDRESS_2 connected to I2C_BUS.
	lsm303d = lsm303d_init_sensor ((i2c_port_t)I2CPortNumber, LSM303D_I2C_ADDRESS_2, 0);

	// configure HPF and implicitly reset the reference by a dummy read
	lsm303d_config_a_hpf (lsm303d, lsm303d_hpf_normal, true, true, true, true);

	// enable the temperature sensor
	lsm303d_enable_temperature (lsm303d, true);

	// LAST STEP: Finally set scale and mode to start measurements
	lsm303d_set_a_scale(lsm303d, lsm303d_a_scale_2_g);
	lsm303d_set_m_scale(lsm303d, lsm303d_m_scale_4_Gs);
	lsm303d_set_a_mode (lsm303d, lsm303d_a_odr_12_5, lsm303d_a_aaf_bw_773, true, true, true);
	lsm303d_set_m_mode (lsm303d, lsm303d_m_odr_12_5, lsm303d_m_low_res, lsm303d_m_continuous);
#endif

	nh.initNode(rosCoreHostName);
	tf_broadcaster.init(nh);

	scan_msg.ranges = NULL;
	scan_msg.intensities = NULL;

	while(1)
	{
		{
			static bool _connected = false;
			if( _connected != nh.connected() )
			{
				_connected = nh.connected();
				ESP_LOGI(TAG, "I2CThread loop ros(%d,%d,%d)",
					nh.connected(),published,nh.getHardware()->ok());
			}
		}

#ifdef ENABLE_WATCHDOG
		esp_task_wdt_feed();
#endif

		if( nh.connected() && published==false )
		{
			published = true;

			ESP_LOGI(TAG, "ROS publish ...");

			nh.advertise(pub_ubat);
			nh.advertise(pub_imucalib);
			nh.advertise(pub_headingx);
			nh.advertise(pub_headingy);
			nh.advertise(pub_headingz);
			nh.advertise(pub_wifirssi);
			linesensor_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension)*2);
		  linesensor_msg.layout.dim[0].label = "height";
		  linesensor_msg.layout.dim[0].size = 5;
		  linesensor_msg.layout.dim[0].stride = 1;
		  linesensor_msg.layout.data_offset = 0;
		  linesensor_msg.data = (uint16_t*)malloc(sizeof(uint16_t)*5);
		  linesensor_msg.data_length = 5;
			nh.advertise(pub_linesensor);
			encoder_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension)*2);
		  encoder_msg.layout.dim[0].label = "height";
		  encoder_msg.layout.dim[0].size = 2;
		  encoder_msg.layout.dim[0].stride = 1;
		  encoder_msg.layout.data_offset = 0;
		  encoder_msg.data = (int32_t*)malloc(sizeof(int32_t)*2);
		  encoder_msg.data_length = 2;
			nh.advertise(pub_encoder);
#ifdef ENABLE_LIDAR
			lidar_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension)*2);
			lidar_msg.layout.dim[0].label = "height";
			lidar_msg.layout.dim[0].size = mylidar.getNumRanges();
			lidar_msg.layout.dim[0].stride = 1;
			lidar_msg.layout.data_offset = 0;
			lidar_msg.data = (uint16_t*)malloc(sizeof(uint16_t)*mylidar.getNumRanges());
			lidar_msg.data_length = mylidar.getNumRanges();
			nh.advertise(pub_lidar);
#endif
			nh.advertise(pub_scan);
			nh.advertise(pub_odom);
			nh.subscribe(sub_cmdvel);

#if defined(ENABLE_RTIMULIB) || defined(ENABLE_IMU_BNO055)
			nh.advertise(pub_imu);
			//nh.advertiseService(service_rtiimu_callibrate);
#endif

			ESP_LOGI(TAG, "ROS publish ... done");
		}

		if( !nh.connected() && published==true )
		{
			ESP_LOGI(TAG, "ROS disconnected");
			published = false;
		}

#ifdef ENABLE_LIDAR
		mylidar.handle();
#endif

#ifdef USE_RAWIMU
		vHandle_l3gd20h_and_lsm303d();
#endif

#ifdef ENABLE_IMU_BNO055
		if( bno055 != NULL )
		{
			try {
				if( /*nh.connected() && published==true &&*/ bno055_configured==false )
				{
#if 1
					ESP_LOGI(TAG, "BNO055 try get BNO055Callibration param ...");

					//bno055->setOprModeConfig();
					int p[3+3+3+2] = {8,7,1,180,-695,574,-2,-2,-1,1000,372};
					//nh.getParam("/BNO055Callibration", p, 11, 100);

					bno055_offsets_t o;
					o.accelOffsetX = p[0];
					o.accelOffsetY = p[1];
					o.accelOffsetZ = p[2];
					o.magOffsetX = p[3];
					o.magOffsetY = p[4];
					o.magOffsetZ = p[5];
					o.gyroOffsetX = p[6];
					o.gyroOffsetY = p[7];
					o.gyroOffsetZ = p[8];
					o.accelRadius = p[9];
					o.magRadius = p[10];
					bno055->setSensorOffsets(o);
					o = bno055->getSensorOffsets();
					bno055_calibration_t calib = bno055->getCalibration();
					ESP_LOGI(TAG, "BNO055 SET calib(%d,%d,%d,%d) %d,%d,%d %d,%d,%d %d,%d,%d %d,%d",
						bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel,
						o.accelOffsetX,o.accelOffsetY,o.accelOffsetZ,
						o.magOffsetX,o.magOffsetY,o.magOffsetZ,
						o.gyroOffsetX,o.gyroOffsetY,o.gyroOffsetZ,
						o.accelRadius,o.magRadius);
#endif
					bno055->setOprModeNdof();
					bno055_configured = true;
				}

				if( bno055_configured==true )
				{
					bno055_calibration_t calib = bno055->getCalibration();

					if( memcmp(&bno055_calib,&calib,sizeof(bno055_calibration_t)) != 0 )
					{
						bno055->setOprModeConfig();
						bno055_offsets_t bno055_offsets = bno055->getSensorOffsets();
						bno055_calib = calib;
						bno055->setOprModeNdof();

						ESP_LOGI(TAG, "BNO055 UPT calib(%d,%d,%d,%d) [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
							bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel,
							bno055_offsets.accelOffsetX,bno055_offsets.accelOffsetY,bno055_offsets.accelOffsetZ,
							bno055_offsets.magOffsetX,bno055_offsets.magOffsetY,bno055_offsets.magOffsetZ,
							bno055_offsets.gyroOffsetX,bno055_offsets.gyroOffsetY,bno055_offsets.gyroOffsetZ,
							bno055_offsets.accelRadius,bno055_offsets.magRadius);

						if( nh.connected() && published==true )
						{
							sprintf(imucalib_buf,"BNO055 calib(%d,%d,%d,%d) [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
								bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel,
								bno055_offsets.accelOffsetX,bno055_offsets.accelOffsetY,bno055_offsets.accelOffsetZ,
								bno055_offsets.magOffsetX,bno055_offsets.magOffsetY,bno055_offsets.magOffsetZ,
								bno055_offsets.gyroOffsetX,bno055_offsets.gyroOffsetY,bno055_offsets.gyroOffsetZ,
								bno055_offsets.accelRadius,bno055_offsets.magRadius);
							imucalib_msg.data = imucalib_buf;
							pub_imucalib.publish( &imucalib_msg );
						}
					}
				}
				if( nh.connected() && published==true && bno055_configured==true )
				{
					static ros::Time _prev;
					ros::Time now = nh.now();
					if( now.toSec() > (_prev.toSec() + 0.01) )
					{
						_prev = now;
						bno055_quaternion_t quaternion = bno055->getQuaternion();
						bno055_vector_t vector_angvel = bno055->getVectorGyroscope();
						bno055_vector_t vector_linaccl = bno055->getVectorLinearAccel();
						int8_t temperature = bno055->getTemp();

						double cov_orientation = 0.08;
						double cov_velocity = 0.02;
						double cov_acceleration = 0.04;

						bno055_vector_t euler = bno055->getVectorEuler();
						headingx_msg.data = (float)euler.x;
						pub_headingx.publish( &headingx_msg );
						headingy_msg.data = (float)euler.y;
						pub_headingy.publish( &headingy_msg );
						headingz_msg.data = (float)euler.z;
						pub_headingz.publish( &headingz_msg );

						imu_msg.header.frame_id = "imu_link";
						imu_msg.header.stamp = now;
						imu_msg.header.seq = imu_msg.header.seq+1;
						imu_msg.orientation.x = quaternion.y;
						imu_msg.orientation.y = -quaternion.x;
						imu_msg.orientation.z = quaternion.z;
						imu_msg.orientation.w = quaternion.w;
#if 0
						imu_msg.orientation_covariance[0] = -1.0;
#else
						const double orientation_covariance[9] = {
								cov_orientation, 0.0, 0.0,
								0.0, cov_orientation,	0.0,
								0.0, 0.0,	cov_orientation };
						memcpy(imu_msg.orientation_covariance,
							orientation_covariance,
							sizeof(imu_msg.orientation_covariance));
#endif

						imu_msg.angular_velocity.x = vector_angvel.y/* * M_PI / 180.0*/;
						imu_msg.angular_velocity.y = -vector_angvel.x/* * M_PI / 180.0*/;
						imu_msg.angular_velocity.z = vector_angvel.z/* * M_PI / 180.0*/;
#if 0
						imu_msg.angular_velocity_covariance[0] = -1.0;
#else
						const double angular_velocity_covariance[9] = {
							cov_velocity, 0.0, 0.0,
							0.0, cov_velocity,	0.0,
							0.0, 0.0,	cov_velocity };
						memcpy(imu_msg.angular_velocity_covariance,
							angular_velocity_covariance,
							sizeof(imu_msg.angular_velocity_covariance));
#endif

						imu_msg.linear_acceleration.x = vector_linaccl.y/* / 100.0*/;
						imu_msg.linear_acceleration.y = -vector_linaccl.x/* / 100.0*/;
						imu_msg.linear_acceleration.z = vector_linaccl.z/* / 100.0*/;
#if 0
						imu_msg.linear_acceleration_covariance[0] = -1.0;
#else
						const double linear_acceleration_covariance[9] = {
							cov_acceleration, 0.0, 0.0,
							0.0, cov_acceleration,	0.0,
							0.0, 0.0,	cov_acceleration };
						memcpy(imu_msg.linear_acceleration_covariance,
							linear_acceleration_covariance,
							sizeof(imu_msg.linear_acceleration_covariance));
#endif

						pub_imu.publish(&imu_msg);
					}
				}
			} catch ( BNO055BaseException ex ) {
				ESP_LOGI(TAG, "I2CThread() BNO055 exception %s",ex.what());
			}
		}
#endif

#ifdef ENABLE_RTIMULIB
		if (rtimu->IMUType() != RTIMU_TYPE_NULL)
		{
			rtimu->IMURead();
		  rtimu_data = rtimu->getIMUData();

			ESP_LOGV(TAG, "IMU: %d FP:%d[%s]",
				(int)(rtimu_data.timestamp),
				(int)(rtimu_data.fusionPoseValid),
				RTMath::displayDegrees("fusionPose",rtimu_data.fusionPose));

				if( rtimu_data.fusionQPoseValid  &&
					  rtimu_data.gyroValid &&
					  rtimu_data.accelValid )
				{
					RTQuaternion fusionQPose;

					imu_msg.header.frame_id = "imu_link";
					imu_msg.header.stamp = nh.now();
					imu_msg.header.seq = imu_msg.header.seq + 1;
					imu_msg.orientation.x = rtimu_data.fusionQPose.x();
					imu_msg.orientation.y = rtimu_data.fusionQPose.y();
					imu_msg.orientation.z = rtimu_data.fusionQPose.z();
					imu_msg.orientation.w = rtimu_data.fusionQPose.scalar();
					const double orientation_covariance[9] = {
							1.0, 0.0, 0.0,
							0.0, 1.0,	0.0,
							0.0, 0.0,	1.0 };
					memcpy(imu_msg.orientation_covariance,
						orientation_covariance,
						sizeof(imu_msg.orientation_covariance));

					const double G_TO_MPSS = 9.80665;
					imu_msg.angular_velocity.x = rtimu_data.gyro.x() * G_TO_MPSS;
					imu_msg.angular_velocity.y = rtimu_data.gyro.y() * G_TO_MPSS;
					imu_msg.angular_velocity.z = rtimu_data.gyro.z() * G_TO_MPSS;
					const double angular_velocity_covariance[9] = {
						1.0, 0.0, 0.0,
						0.0, 1.0,	0.0,
						0.0, 0.0,	1.0 };
					memcpy(imu_msg.angular_velocity_covariance,
						angular_velocity_covariance,
						sizeof(imu_msg.angular_velocity_covariance));

					imu_msg.linear_acceleration.x = rtimu_data.accel.x();
					imu_msg.linear_acceleration.y = rtimu_data.accel.y();
					imu_msg.linear_acceleration.z = rtimu_data.accel.z();
					const double linear_acceleration_covariance[9] = {
						1.0, 0.0, 0.0,
						0.0, 1.0,	0.0,
						0.0, 0.0,	1.0 };
					memcpy(imu_msg.linear_acceleration_covariance,
						linear_acceleration_covariance,
						sizeof(imu_msg.linear_acceleration_covariance));

						if( nh.connected() && published==true )
						{
							pub_imu.publish(&imu_msg);
						}
				}
		}
#endif

#if 0
		{
			int64_t t = esp_timer_get_time();
			static int64_t _t = 0;
			static double v = 0;
			if( t > _t )
			{
				_t = t + 3000000;
				double _v = 0.05;

				if( v == _v )
				{
					v = -_v;
					motor_pid_l.setSetpoint(v);
					motor_pid_r.setSetpoint(v);
				}
				else
				{
					v = _v;
					motor_pid_l.setSetpoint(v);
					motor_pid_r.setSetpoint(v);
				}
			}
		}
#endif

#if 1
		if( nReadM32U4Status() == true )
		{
			if( nh.connected() && published==true )
			{
				int64_t t = esp_timer_get_time();
				static int64_t _t = 0;
				if( t > _t )
				{
					_t = t + 100000;
					ubat_msg.data = ((float)m32u4_status.bat/1000.0);
					pub_ubat.publish( &ubat_msg );

					ubat_msg.data = ((float)m32u4_status.bat/1000.0);
					pub_ubat.publish( &ubat_msg );

					linesensor_msg.data[0] = m32u4_status.linesensors[0];
					linesensor_msg.data[1] = m32u4_status.linesensors[1];
					linesensor_msg.data[2] = m32u4_status.linesensors[2];
					linesensor_msg.data[3] = m32u4_status.linesensors[3];
					linesensor_msg.data[4] = m32u4_status.linesensors[4];
					pub_linesensor.publish( &linesensor_msg );

					encoder_msg.data[0] = m32u4_status.enc_l;
					encoder_msg.data[1] = m32u4_status.enc_r;
					pub_encoder.publish( &encoder_msg );
				}
			}
		}
#endif

		if( nh.connected() && published==true )
		{
#ifdef ENABLE_LIDAR
			{
				LidarScan scan;
				if( mylidar.getFrozenScan(&scan) == true )
				{
					for(int i=0;i<scan.n;i++)
					{
						lidar_msg.data[i] = scan.ranges[i];
					}
					pub_lidar.publish( &lidar_msg );

					{
						ros::Time time = nh.now();
						ros::Duration dT;
						dT.fromSec(0.000001*(esp_timer_get_time()-scan.t_first));
						time -= dT;

						const float lidar_angle_min = -(360.0*(NUM_LIDAR_SCANS)/40)/2.0;
						const float lidar_angle_max = +(360.0*(NUM_LIDAR_SCANS)/40)/2.0;
						scan_msg.ranges = (float*)realloc(scan_msg.ranges,sizeof(float)*scan.n);
						scan_msg.intensities = (float*)realloc(scan_msg.intensities,sizeof(float)*scan.n);
						scan_msg.ranges_length = scan.n;
						scan_msg.intensities_length = scan.n;
						scan_msg.header.frame_id = "laser_link";
						scan_msg.scan_time = (0.000001*(scan.t_last-scan.t_first));
						if( scan.dir == -1 )
						{
							scan_msg.header.stamp = time;
							scan_msg.header.seq = scan_msg.header.seq + 1;
							scan_msg.angle_min = (2.0*M_PI * lidar_angle_min/360.0);
							scan_msg.angle_max = (2.0*M_PI * lidar_angle_max/360.0);
							scan_msg.angle_increment = ((scan_msg.angle_max-scan_msg.angle_min) / (scan.n-1));
							for( int i=0;i<scan.n; i++ )
							{
								if( scan.ranges[i] == 0xffff )
								{
									scan_msg.intensities[i] = 0.0;
									scan_msg.ranges[i] = 0.0;
								}
								else
								{
									scan_msg.intensities[i] = 1.0;
									scan_msg.ranges[i] = 0.001*scan.ranges[i];
								}
							}
							scan_msg.time_increment = (0.000001*(scan.t_last-scan.t_first)) / (scan.n-1);
						}
						else
						{
							scan_msg.header.stamp = time;
							scan_msg.header.seq = scan_msg.header.seq + 1;
							scan_msg.angle_min = (2.0*3.14 * lidar_angle_max/360.0);
							scan_msg.angle_max = (2.0*3.14 * lidar_angle_min/360.0);
							scan_msg.angle_increment = ((scan_msg.angle_max-scan_msg.angle_min) / (scan.n-1));
							for( int i=0;i<scan.n; i++ )
							{
								if( scan.ranges[scan.n-1-i] == 0xffff )
								{
									scan_msg.intensities[i] = 0.0;
									scan_msg.ranges[i] = 0.0;
								}
								else
								{
									scan_msg.intensities[i] = 1.0;
									scan_msg.ranges[i] = 0.001*scan.ranges[scan.n-1-i];
								}
							}
							scan_msg.time_increment = (0.000001*(scan.t_last-scan.t_first)) / (scan.n-1);
						}

						scan_msg.range_min = 0.0;
						scan_msg.range_max = 2.0;
						pub_scan.publish(&scan_msg);

					}
				}
			}
#endif
		}

		{
			int64_t t = esp_timer_get_time();
			static int64_t _t = 0;
			if( t > _t )
			{
				_t = t + 500000;

				if( wificonnected==true && nh.connected() && published==true )
				{
					wifi_ap_record_t ap_info;
					if( ESP_OK == esp_wifi_sta_get_ap_info( &ap_info) )
					{
						wifirssi_msg.data = ap_info.rssi;
						pub_wifirssi.publish( &wifirssi_msg );
					}
				}
			}
		}

#ifdef ENABLE_DISPLAY
		vUpdateDisplay(&I2CDisplay);
#endif

		nh.spinOnce();
	}
}

/*
 * called by other ROS nodes
 */
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
#if 1
	double wheel0_speed = 0;
	double wheel1_speed = 0;

	double wheelbase_ = 0.09;

	// *** Compute the current wheel speeds ***
	// First compute the Robot's linear and angular speeds
	double xspeed = vel_cmd.linear.x;
	double yspeed = vel_cmd.linear.y;
	double linear_speed = sqrt (xspeed * xspeed + yspeed * yspeed);
	double angular_speed = vel_cmd.angular.z;

	if( vel_cmd.linear.x >= 0 )
	{
		// robot is moving forward
		wheel0_speed = linear_speed + angular_speed * wheelbase_ / 2.0;
		wheel1_speed = linear_speed - angular_speed * wheelbase_ / 2.0;
	}
	else
	{
		// robot is backing up
		wheel0_speed = -linear_speed + angular_speed * wheelbase_ / 2.0;
		wheel1_speed = -linear_speed - angular_speed * wheelbase_ / 2.0;
	}

	motor_pid_l.setSetpoint(wheel1_speed);
	motor_pid_r.setSetpoint(wheel0_speed);

#else
	double x = (double)1.0*vel_cmd.linear.x;
	double y = (double)1.0*vel_cmd.angular.z;

	double vmax = 0.1;

	//x = (x>vmax)?vmax:((x<-vmax)?-vmax:x);
	//y = (y>vmax)?vmax:((y<-vmax)?-vmax:y);

	if((fabs(x)+fabs(y))>1.0)
	{
		double k = (fabs(x)+fabs(y));
		x = x/k;
		y = y/k;
	}

	motor_pid_r.setSetpoint((double)(x)+(y));
	motor_pid_l.setSetpoint((double)(x)-(y));
#endif

	ESP_LOGI(TAG, "cmd_vel_callback Twist(x:%f,y:%f,z:%f) M(%f,%f)",
	  vel_cmd.linear.x,vel_cmd.linear.y,vel_cmd.angular.z,
		motor_pid_l.getSetpoint(),motor_pid_r.getSetpoint());

	cmd_vel_motor_update();
}

int my_nulllog(const char *format, va_list args);
int my_tcplog(const char *format, va_list args);

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
		case SYSTEM_EVENT_STA_START:
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
		wificonnected = false;
		esp_wifi_connect();
		sprintf(wifistate,"???");
		break;

		case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
#ifdef ENABLE_TCPLOG
		esp_log_set_vprintf(my_tcplog);
#endif
		wificonnected = true;
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		sprintf(wifistate,"%03d",(event->event_info.got_ip.ip_info.ip.addr>>24)&0xff);
		initialize_sntp();
		break;

		case SYSTEM_EVENT_STA_LOST_IP:
#ifdef ENABLE_TCPLOG
		esp_log_set_vprintf(my_nulllog);
#endif
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_LOST_IP");
		break;

		case SYSTEM_EVENT_STA_DISCONNECTED:
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		sprintf(wifistate,"???");
		wificonnected = false;
		esp_wifi_connect();
		break;

		case SYSTEM_EVENT_SCAN_DONE:
		ESP_LOGI(TAG, "SYSTEM_EVENT_SCAN_DONE");
		break;

		default:
		break;
	}
	return ESP_OK;
}

#ifdef ENABLE_WIFI
static void initialise_wifi()
{
		wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
						{.ssid = WIFI_SSID},
		        {.password = WIFI_PASSWORD},
        },
    };
		wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
		wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
#endif

#ifdef ENABLE_OTA
static void ota_server_task(void * param)
{
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	ota_server_start();
	vTaskDelete(NULL);
}
#endif

/*
 * vResetBNO055
 */
void vResetBNO055()
{
	{
		i2c_cmd_handle_t CommandHandle = NULL;
		if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
		{
			i2c_master_start( CommandHandle );
			i2c_master_write_byte( CommandHandle, ( 0x38 << 1 ) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte( CommandHandle, 0x00, true);
			i2c_master_stop( CommandHandle );
			if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
			{
			}
			i2c_cmd_link_delete( CommandHandle );
		}
	}

	vTaskDelay(50 / portTICK_PERIOD_MS);

	{
		i2c_cmd_handle_t CommandHandle = NULL;
		if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
		{
			i2c_master_start( CommandHandle );
			i2c_master_write_byte( CommandHandle, ( 0x38 << 1 ) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte( CommandHandle, 0x80, true);
			i2c_master_stop( CommandHandle );
			if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
			{
			}
			i2c_cmd_link_delete( CommandHandle );
		}
	}

	/* wait until BNO055 is ready */
	vTaskDelay(750 / portTICK_PERIOD_MS);
}

#ifdef ENABLE_TCPLOG
int my_nulllog(const char *format, va_list args)
{
	return 0;
}

int my_tcplog(const char *format, va_list args)
{
	static int sock = -1;
	if( sock == -1 )
	{
		sock =  socket(AF_INET, SOCK_STREAM, 0);
		if( sock != -1 )
		{
			struct sockaddr_in destAddr;
			struct addrinfo* result;

			//long on = 0;
			//ioctl(sock, (int)FIONBIO, (char *)&on);

	    /* resolve the domain name into a list of addresses */
	    int error = getaddrinfo(rosCoreHostName, NULL, NULL, &result);
	    if(error != 0 || result == NULL)
	    {
				close(sock);
				sock = -1;
	      return 0;
	    }
	    destAddr.sin_addr = ((struct sockaddr_in *)result->ai_addr)->sin_addr;
			//destAddr.sin_addr.s_addr = inet_addr("192.168.1.45");
			destAddr.sin_family = AF_INET;
			destAddr.sin_port = htons(20001);
			int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
			if (err != 0) {
				close(sock);
				sock = -1;
				return 0;
			}
		}
	}
	if( sock != -1 )
	{
			char line[256];
			vsnprintf (line, sizeof(line), format, args);
			int n = strlen(line);
			int err = send(sock, line, n, 1);
			if (err != 0) {
				close(sock);
				sock = -1;
			}
			return n;
	}
	return 0;
}
#endif

void app_main(void)
{
	ESP_ERROR_CHECK( nvs_flash_init() );
	ESP_LOGI(TAG, "main() ...");

#ifdef ENABLE_WATCHDOG
	esp_int_wdt_init();
	esp_task_wdt_init(1000,true);
#endif

#ifdef ENABLE_I2C
	ESP_LOGI(TAG, "main() i2c init ...");
	static i2c_config_t Config;
	memset( &Config, 0, sizeof( i2c_config_t ) );
	Config.mode = I2C_MODE_MASTER;
	Config.sda_io_num = (gpio_num_t)SDAPin;
	Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	Config.scl_io_num = (gpio_num_t)SCLPin;
	Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	Config.master.clk_speed = 500000;
	i2c_param_config( (i2c_port_t)I2CPortNumber, &Config );
	i2c_driver_install( (i2c_port_t)I2CPortNumber, Config.mode, 0, 0, 0 );
	i2c_set_timeout((i2c_port_t)I2CPortNumber, (I2C_APB_CLK_FREQ / Config.master.clk_speed)*1024);

	vBeep(330,500,200);
#ifdef ENABLE_IMU_BNO055
	vResetBNO055();
#endif

	{
		i2c_cmd_handle_t CommandHandle = NULL;
		if ( ( CommandHandle = i2c_cmd_link_create( ) ) != NULL )
		{
			int16_t l = 0;
			int16_t r = 0;
			i2c_master_start( CommandHandle );
			i2c_master_write_byte( CommandHandle, ( 8 << 1 ) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte( CommandHandle, CMD_MOTORS_SET_SPEED, true);
			i2c_master_write_byte( CommandHandle, ((l)>>8)&0xff, true);
			i2c_master_write_byte( CommandHandle, ((l)>>0)&0xff, true);
			i2c_master_write_byte( CommandHandle, ((r)>>8)&0xff, true);
			i2c_master_write_byte( CommandHandle, ((r)>>0)&0xff, true);
			i2c_master_stop( CommandHandle );
			if( ESP_OK  == i2c_master_cmd_begin((i2c_port_t)I2CPortNumber, CommandHandle, pdMS_TO_TICKS( 10 )) )
			{
			}
			i2c_cmd_link_delete( CommandHandle );
		}
	}
#endif

#ifdef ENABLE_WIFI
	ESP_LOGI(TAG, "main() wifi init ...");
	initialise_wifi();
#endif

#ifdef ENABLE_I2C
#ifdef ENABLE_DISPLAY
	ESP_LOGI(TAG, "main() display init ...");
	assert( SSD1306_I2CMasterAttachDisplayDefault( &I2CDisplay,
		I2CDisplayWidth, I2CDisplayHeight,
		I2CDisplayAddress, I2CResetPin ) == true );
#endif

	ESP_LOGI(TAG, "main() lidar init ...");
#ifdef ENABLE_LIDAR
	mylidar.start();
#endif
#endif

	ESP_LOGI(TAG, "main() start threads ...");
#if defined(ENABLE_WIFI) && defined(ENABLE_OTA)
	xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, 5, NULL);
#endif
#ifdef ENABLE_I2C
	xTaskCreate(&I2CThread, "i2c_task", configMINIMAL_STACK_SIZE + 2048 + 2*2048, NULL, 5, NULL);
#endif

	ESP_LOGI(TAG, "main() ... done");
}

/******************************************************************************
* EOF
******************************************************************************
*/
