#ifndef __FDILINK_DECODE_H
#define __FDILINK_DECODE_H
#include <stdint.h>

typedef uint64_t timestamp_t;

typedef struct
{
	float Gyroscope_X     ;
	float Gyroscope_Y     ;
	float Gyroscope_Z     ;
	float Accelerometer_X ;
	float Accelerometer_Y ;
	float Accelerometer_Z ;
	float Magnetometer_X  ;
	float Magnetometer_Y  ;
	float Magnetometer_Z  ;
	float IMU_Temperature ;
	float Pressure        ;
	float Pressure_Temperature;
	timestamp_t Timestamp;
}FDILink_IMUData_Packet_t;
#define FDILINK_IMUDATA_PACKET_ID 0x40

typedef struct
{
	float RollSpeed;
	float PitchSpeed;
	float HeadingSpeed;
	float Roll;
	float Pitch;
	float Heading;
	float Q1;
	float Q2;
	float Q3;
	float Q4;
	timestamp_t Timestamp;
}FDILink_AHRSData_Packet_t;
#define FDILINK_AHRSDATA_PACKET_ID 0x41

typedef struct
{
	float BodyVelocity_X           ;
	float BodyVelocity_Y           ;
	float BodyVelocity_Z           ;
	float BodyAcceleration_X       ;
	float BodyAcceleration_Y       ;
	float BodyAcceleration_Z       ;
	double Location_North          ;
	double Location_East           ;
	double Location_Down           ;
	float Velocity_North           ;
	float Velocity_East            ;
	float Velocity_Down            ;
	float Acceleration_North       ;
	float Acceleration_East        ;
	float Acceleration_Down        ;
	float Pressure_Altitude        ;
	timestamp_t Timestamp          ;
}FDILink_INSGPSData_Packet_t;
#define FDILINK_INSGPSDATA_PACKET_ID 0x42

typedef struct
{
	union{
		uint8_t GyroStatus;
		struct 
		{
			uint8_t GyroErrorUnchanged 	:1;
			uint8_t GyroErrorFrame 		:1;
			uint8_t GyroErrorOverrange 	:1;
			uint8_t GyroReserved 		:5;
		};
	};
	union{
		uint8_t AccStatus;
		struct 
		{
			uint8_t AccErrorUnchanged 	:1;
			uint8_t AccErrorFrame 		:1;
			uint8_t AccErrorOverrange 	:1;
			uint8_t AccReserved 		:5;
		};
	};
	union{
		uint8_t MagStatus;
		struct 
		{
			uint8_t MagErrorUnchanged 	:1;
			uint8_t MagErrorFrame 		:1;
			uint8_t MagErrorOverrange 	:1;
			uint8_t MagReserved 		:5;
		};
	};
	uint8_t Reserved;
}System_State_Packet_t;
#define System_State_Packet_ID 0x50

 typedef struct
{
	uint32_t Unix_time;
	uint32_t Microseconds;
}Unix_Time_Packet_t;
#define Unix_Time_Packet_ID 0x51

 typedef struct
{
	uint32_t   Microseconds;
	uint16_t   Year;
	uint16_t   Year_day;
	uint8_t   Month;
	uint8_t   Month_Day;
	uint8_t   Week_Day;
	uint8_t   Hour;
	uint8_t   Minute;
	uint8_t   Second;
}Formatted_Time_Packet_t;
#define Formatted_Time_Packet_ID 0x52


 typedef struct
{
	float   Latitude_standard_deviation;
	float   Longitude_standard_deviation;
	float   Height_standard_deviation;
}Position_Standard_Deviation_Packet_t;
#define Position_Standard_Deviation_Packet_ID 0x54

 typedef struct
{
	float   Velocity_north_standard_deviation;
	float   Velocity_east_standard_deviation;
	float   Velocity_down_standard_deviation;
}Velocity_Standard_Deviation_Packet_t; 
#define Velocity_Standard_Deviation_Packet_ID 0x55

 typedef struct
{
	float   Roll_standard_deviation;
	float   Pitch_standard_deviation;
	float   Heading_standard_deviation;
}Euler_Orientation_Standard_Deviation_Packet_t;
#define Euler_Orientation_Standard_Deviation_Packet_ID 0x56

 typedef struct
{
	float   Q0_standard_deviation;
	float   Q1_standard_deviation;
	float   Q2_standard_deviation;
	float   Q3_standard_deviation;

}Quaternion_Orientation_Standard_Deviation_Packet_t;
#define Quaternion_Orientation_Standard_Deviation_Packet_ID 0x57

 typedef struct
{
	float   Accelerometer_X;
	float   Accelerometer_Y;
	float   Accelerometer_Z;
	float   Gyroscope_X;
	float   Gyroscope_Y;
	float   Gyroscope_Z;
	float   Magnetometer_X;
	float   Magnetometer_Y;
	float   Magnetometer_Z;
	float   IMU_Temperature;
	float   Pressure;
	float   Pressure_Temperature;

}Raw_Sensors_Packet_t;
#define Raw_Sensors_Packet_ID 0x58

typedef struct
{
		uint16_t  GNSS_Fix_Status : 4;
		uint16_t  Doppler_velocity_valid: 1;
		uint16_t  Time_valid : 1;
		uint16_t  External_GNSS : 1;
		uint16_t  Tilt_valid : 1;
		uint16_t  Heading_valid : 1;
		uint16_t  Floating_ambiguity_heading : 1;
		uint16_t  Reserved : 6;
}RAW_GNSS_Status_t;

typedef struct
{
	uint32_t   Unix_time_stamp;
	uint32_t   Microseconds;
	double   Latitude;
	double   Longitude;
	double   Height;
	float   Velocity_north;
	float   Velocity_east;
	float   Velocity_down;
	float   Latitude_standard_deviation;
	float   Longitude_standard_deviation;
	float   Height_standard_deviation;
	float   Reserved1;
	float   Reserved2;
	float   Reserved3;
	float   Reserved4;
	RAW_GNSS_Status_t   Status;

}Raw_GNSS_Packet_t;
#define Raw_GNSS_Packet_ID 0x59

 typedef struct
{
	float   HDOP;
	float   VDOP;
	uint8_t   GNSS_satellites;
}Satellites_Packet_t;
#define Satellites_Packet_ID 0x5A

 typedef struct
{
	uint8_t   Satellite_system;
	uint8_t   Satellite_number[20];
	int8_t   Satellite_frequencies[20];
	uint8_t   Elevation[20];
	uint16_t   Azimuth[20];
	uint8_t   SNR[20];
}Detailed_Satellites_Packet_t;
#define Detailed_Satellites_Packet_ID 0x5B

 typedef struct
{
	double   Latitude;
	double   Longitude;
	double   Height;
	float   hAcc;
	float   vAcc;
}Geodetic_Position_Packet_t;
#define Geodetic_Position_Packet_ID 0x5C

 typedef struct
{
	double   ECEF_X;
	double   ECEF_Y;
	double   ECEF_Z;
}ECEF_Position_Packet_t;
#define ECEF_Position_Packet_ID 0x5D

 typedef struct
{
	float   Northing;
	float   Easting;
	float   Height;
	uint8_t   Zone_number;
	int8_t   Zone_character;
}UTM_Position_Packet_t;
#define UTM_Position_Packet_ID 0x5E

 typedef struct
{
	float   Velocity_north;
	float   Velocity_east;
	float   Velocity_down;
}NED_Velocity_Packet_t;
#define NED_Velocity_Packet_ID 0x5F

 typedef struct
{
	float   Velocity_X;
	float   Velocity_Y;
	float   Velocity_Z;
}Body_Velocity_Packet_t;
#define Body_Velocity_Packet_ID 0x60

 typedef struct
{
	float   Acceleration_X;
	float   Acceleration_Y;
	float   Acceleration_Z;
}Acceleration_Packet_t;
#define Acceleration_Packet_ID 0x61

 typedef struct
{
	float   Body_acceleration_X;
	float   Body_acceleration_Y;
	float   Body_acceleration_Z;
	float   G_force;
}Body_Acceleration_Packet_t;
#define Body_Acceleration_Packet_ID 0x62

 typedef struct
{
	float   Roll;
	float   Pitch;
	float   Heading;
}Euler_Orientation_Packet_t;
#define Euler_Orientation_Packet_ID 0x63

 typedef struct
{
	float   Q0;
	float   Q1;
	float   Q2;
	float   Q3;
}Quaternion_Orientation_Packet_t;
#define Quaternion_Orientation_Packet_ID 0x64

 typedef struct
{
	float DCM[3][3];
}DCM_Orientation_Packet_t;
#define DCM_Orientation_Packet_ID 0x65

 typedef struct
{
	float   Angular_velocity_X;
	float   Angular_velocity_Y;
	float   Angular_velocity_Z;
}Angular_Velocity_Packet_t;
#define Angular_Velocity_Packet_ID 0x66

 typedef struct
{
	float   Angular_acceleration_X;
	float   Angular_acceleration_Y;
	float   Angular_acceleration_Z;
}Angular_Acceleration_Packet_t;
#define Angular_Acceleration_Packet_ID 0x67

 typedef struct
{
	uint32_t   Running_time_seconds;
	uint32_t   Microseconds;
}Running_Time_Packet_t;
#define Running_Time_Packet_ID 0x6D

 typedef struct
{
	float   Local_magnetic_field_X;
	float   Local_magnetic_field_Y;
	float   Local_magnetic_field_Z;
}Local_Magnetic_Field_Packet_t;
#define Local_Magnetic_Field_Packet_ID 0x6E

 typedef struct
{
	int32_t   Odometer_pulse_count;
	float   Odometer_distance;
	float   Odometer_speed;
	float   Odometer_slip;
	uint8_t   Odometer_active;
	uint8_t  Reserved[3];
}Odometer_State_Packet_t;
#define Odometer_State_Packet_ID 0x6F

 typedef struct
{
	float   Geoid_height;
}Geoid_Height_Packet_t;
#define Geoid_Height_Packet_ID 0x72

 typedef struct
{
	uint8_t RTCM_corrections_data[255];
}RTCM_Corrections_Packet_t;
#define RTCM_Corrections_Packet_ID 0x73

 typedef struct
{
	float   Wind_velocity_north;
	float   Wind_velocity_east;
	float   Wind_velocity_standard_deviation;
}Wind_Packet_t;
#define Wind_Packet_ID 0x75

 typedef struct
{
	float   Heave_point_1_PosN;
	float   Heave_point_1_PosE;
	float   Heave_point_1_PosD;
	float   Heave_point_1_VelN;
	float   Heave_point_1_VelE;
	float   Heave_point_1_VelD;
	float   Heave_point_2_PosN;
	float   Heave_point_2_PosE;
	float   Heave_point_2_PosD;
	float   Heave_point_2_VelN;
	float   Heave_point_2_VelE;
	float   Heave_point_2_VelD;
	float   Heave_point_3_PosN;
	float   Heave_point_3_PosE;
	float   Heave_point_3_PosD;
	float   Heave_point_3_VelN;
	float   Heave_point_3_VelE;
	float   Heave_point_3_VelD;
	float   Heave_point_4_PosN;
	float   Heave_point_4_PosE;
	float   Heave_point_4_PosD;
	float   Heave_point_4_VelN;
	float   Heave_point_4_VelE;
	float   Heave_point_4_VelD;
}Heave_Packet_t;
#define Heave_Packet_ID 0x76

 typedef struct
{
	uint32_t   Unix_time;
	uint32_t   Nanoseconds;
	int32_t   Receiver_clock_offset;
	uint8_t   Receiver_number;
	uint8_t   Packet_number;
	uint8_t   Total_packets;
	uint8_t   Number_of_satellites;
	uint8_t   Satellite_system;
	uint8_t   PRN_or_satellite_number;
	uint8_t   Elevation;
	uint16_t   Azimuth;
	uint8_t   Number_of_frequencies;
	uint8_t   Satellite_frequency;
	uint8_t   Tracking_status;
	double   Carrier_phase;
	double   Pseudo_range;
	float   Doppler_frequency;
	float   Signal_to_noise_ratio;
}Raw_Satellite_Data_Packet_t;
#define Raw_Satellite_Data_Packet_ID 0x77

 typedef struct
{
	uint32_t  Microseconds;
  double RoverRtkRefPosN;
	double RoverRtkRefPosE; 
	double RoverRtkRefPosD; 
	double MBRtkRefPosN; 
	double MBRtkRefPosE;
	double MBRtkRefPosD; 		
	double RoverLat; 
	double RoverLon;
	float RoverAlt;
	float Rover_hAcc; 
	float Rover_vAcc;							   
	double MBLat;
	double MBLon; 
	float MBAlt; 
	float MBhAcc; 
	float MBvAcc; 
	double RoverRtkPosLength; 
	float RoverRtkAccuracyLength; 							   
	double RoverRtkPosHeading; 
	float RoverRtkAccuracyHeading;
	uint8_t   MBfixtype;
	uint8_t   Roverfixtype;
}GNSS_DUAL_ANT_Data_Packet_t;
#define GNSS_DUAL_ANT_Data_Packet_ID 0x78

 typedef struct
{
	float   Current_angle;
	uint32_t   Reserved;
}Gimbal_State_Packet_t;
#define Gimbal_State_Packet_ID 0x7A

 typedef struct
{
	float   Virtual_odometer_distance;
	float   Slip_Angle;
	float   Velocity_X;
	float   Velocity_Y;
	float   Distance_standard_deviation;
	uint32_t Reserved;
}Automotive_Packet_t;
#define Automotive_Packet_ID 0x7B

 typedef struct
{
	uint8_t   Permanent;
	uint8_t   UTC_synchronisation;
	uint16_t   Packet_timer_period;
}Packet_Timer_Period_Packet_t;
#define Packet_Timer_Period_Packet_ID 0x7C

//该结构体包含Packet_ID，在代码中搜索Packet_ID进行参数数量校验会差一个
//应该搜索 _Packet_ID
 typedef struct
{
	uint8_t   Packet_ID;
	uint32_t   Packet_period;
}Packets_Period_Packet_Sub_t;

 typedef struct
{
	uint8_t   Permanent;
	uint8_t   Clear_existing_packet_periods;
	Packets_Period_Packet_Sub_t periods[5];
}Packets_Period_Packet_t;
#define Packets_Period_Packet_ID 0x7D

 typedef struct
{
	uint8_t   Permanent;
	float   Alignment_DCM[3][3];
	float   GNSS_antenna_offset_X;
	float   GNSS_antenna_offset_Y;
	float   GNSS_antenna_offset_Z;
	float   Odometer_offset_X;
	float   Odometer_offset_Y;
	float   Odometer_offset_Z;
	float   External_data_offset_X;
	float   External_data_offset_Y;
	float   External_data_offset_Z;
}Installation_Alignment_Packet_t;
#define Installation_Alignment_Packet_ID 0x80

 typedef struct
{
	uint8_t   Permanent;
	uint8_t   Vehicle_type;
	uint8_t   Internal_GNSS_enabled;
	uint8_t   Magnetometers_enabled;
	uint8_t   Atmospheric_altitude_enabled;
	uint8_t   Velocity_heading_enabled;
	uint8_t   Reversing_detection_enabled;
	uint8_t   Motion_analysis_enabled;
	uint8_t   Automatic_magnetic_calibration_enabled;
}Filter_Options_Packet_t;
#define Filter_Options_Packet_ID 0x81

 typedef struct
{
	uint8_t   Permanent;
	uint8_t   GPIO1_Function;
	uint8_t   GPIO2_Function;
	uint8_t   Auxiliary_RS232_transmit_function;
	uint8_t   Auxiliary_RS232_receive_function;
}GPIO_Configuration_Packet_t;
#define GPIO_Configuration_Packet_ID 0x82

 typedef struct
{
	uint8_t   Permanent;
	float   Hard_iron_bias_X;
	float   Hard_iron_bias_Y;
	float   Hard_iron_bias_Z;
	float   Soft_iron_transformation_XX;
	float   Soft_iron_transformation_XY;
	float   Soft_iron_transformation_XZ;
	float   Soft_iron_transformation_YX;
	float   Soft_iron_transformation_YY;
	float   Soft_iron_transformation_YZ;
	float   Soft_iron_transformation_ZX;
	float   Soft_iron_transformation_ZY;
	float   Soft_iron_transformation_ZZ;
}Magnetic_Calibration_Values_Packet_t;
#define Magnetic_Calibration_Values_Packet_ID 0x83

 typedef struct
{
	uint8_t   Magnetic_calibration_action;
}Magnetic_Calibration_Configuration_Packet_t;
#define Magnetic_Calibration_Configuration_Packet_ID 0x84

 typedef struct
{
	uint8_t   Magnetic_calibration_status;
	uint8_t   Magnetic_calibration_progress;
	uint8_t   Local_magnetic_error;
}Magnetic_Calibration_Status_Packet_t;
#define Magnetic_Calibration_Status_Packet_ID 0x85

 typedef struct
{
	uint8_t   Permanent;
	uint8_t   Automatic_pulse_measurement_active;
	uint16_t   Reserved;
	float   Pulse_length;
}Odometer_Configuration_Packet_t;
#define Odometer_Configuration_Packet_ID 0x86

 typedef struct
{
	uint8_t   Permanent;
	uint32_t   Verification_sequence;
}Set_Zero_Orientation_Alignment_Packet_t;
#define Set_Zero_Orientation_Alignment_Packet_ID 0x87

 typedef struct
{
	uint8_t   Permanent;
	float   Primary_reference_point_offset_X;
	float   Primary_reference_point_offset_Y;
	float   Primary_reference_point_offset_Z;
	float   Heave_point_2_offset_X;
	float   Heave_point_2_offset_Y;
	float   Heave_point_2_offset_Z;
	float   Heave_point_3_offset_X;
	float   Heave_point_3_offset_Y;
	float   Heave_point_3_offset_Z;
	float   Heave_point_4_offset_X;
	float   Heave_point_4_offset_Y;
	float   Heave_point_4_offset_Z;
}Reference_Point_Offsets_Packet_t;
#define Reference_Point_Offsets_Packet_ID 0x88


 typedef struct
{
	uint8_t User_data[64];
}User_Data_Packet_t;
#define User_Data_Packet_ID 0x8A


 typedef struct
{
	uint8_t Permanent;
	uint32_t Primary_RS232_baud_rate;
	uint32_t GPIO_1_And_2_baud_rate;
	uint32_t Auxiliary_RS232_Baud_Rate;
	uint32_t Reserved;
}Baud_Rates_Packet_t;
#define Baud_Rates_Packet_ID 0xA0

 typedef struct
{
	uint8_t Permanent;
	uint8_t Accelerometers_range;
	uint8_t Gyroscopes_range;
	uint8_t Magnetometers_range;
}Sensor_Ranges_Packet_t;
#define Sensor_Ranges_Packet_ID 0xA1

 typedef struct
{
	uint8_t Permanent;
	uint8_t NMEA_fix_behaviour;
	uint16_t GPZDA_Rates;
	uint16_t GPGGA_Rates;
	uint16_t GPVTG_Rates;
	uint16_t GPRMC_Rates;
	uint16_t GPHDT_Rates;
	uint16_t GPGLL_Rates;
	uint16_t PASHR_Rates;
	uint16_t TSS1_Rates;
	uint16_t Simrad_Rates;
	uint16_t GPROT_Rates;
	uint16_t GPHEV_Rates;
	uint16_t GPGSV_Rates;
	uint16_t PFECAtt_Rates;
	uint16_t PFECHve_Rates;
	uint8_t Reserved[3];
}GPIO_Output_Configuration_Packet_t;
#define GPIO_Output_Configuration_Packet_ID 0xA2

 typedef struct
{
	uint8_t   Permanent;
	float   Gimbal_radians_per_encoder_tick;
	uint8_t   Reserved[60];
}GPIO_Input_Configuration_Packet_t;
#define GPIO_Input_Configuration_Packet_ID 0xA3

 typedef struct
{
	uint8_t Permanent;
	uint16_t Offset_type;
	uint8_t Automatic_offset_orientation;
	uint8_t Reserved;
	float Manual_offset_X;
	float Manual_offset_Y;
	float Manual_offset_Z;
}Dual_Antenna_Configuration_Packet_t;
#define Dual_Antenna_Configuration_Packet_ID 0xA4

/*********************************** 输入 *****************************/
 typedef struct
{
	double   Latitude;
	double   Longitude;
	double   Height;
	float   Velocity_north;
	float   Velocity_east;
	float   Velocity_down;
	float   Latitude_standard_deviation;
	float   Longitude_standard_deviation;
	float   Height_standard_deviation;
	float   Velocity_north_standard_deviation;
	float   Velocity_east_standard_deviation;
	float   Velocity_down_standard_deviation;
}External_Position_And_Velocity_Packet_t;
#define External_Position_And_Velocity_Packet_ID 0x68

 typedef struct
{
	double   Latitude;
	double   Longitude;
	double   Height;
	float   Latitude_standard_deviation;
	float   Longitude_standard_deviation;
	float   Height_standard_deviation;
}External_Position_Packet_t;
#define External_Position_Packet_ID 0x69

 typedef struct
{
	float   Velocity_north;
	float   Velocity_east;
	float   Velocity_down;
	float   Velocity_north_standard_deviation;
	float   Velocity_east_standard_deviation;
	float   Velocity_down_standard_deviation;
}External_Velocity_Packet_t;
#define External_Velocity_Packet_ID 0x6A

 typedef struct
{
	float   Velocity_X;
	float   Velocity_Y;
	float   Velocity_Z;
	float   Velocity_standard_deviation;
}External_Body_Velocity_Packet_t;
#define External_Body_Velocity_Packet_ID 0x6B

 typedef struct
{
	float   Heading;
	float   Heading_standard_deviation;
}External_Heading_Packet_t;
#define External_Heading_Packet_ID 0x6C

 typedef struct
{
	uint32_t   Unix_time_seconds;
	uint32_t   Microseconds;
}External_Time_Packet_t;
#define External_Time_Packet_ID 0x70

 typedef struct
{
	float   Depth;
	float   Depth_standard_deviation;
}External_Depth_Packet_t;
#define External_Depth_Packet_ID 0x71

 typedef struct
{
	float   Differential_pressure;
	float   Outside_air_temperature;
}External_Pitot_Pressure_Packet_t;
#define External_Pitot_Pressure_Packet_ID 0x74

 typedef struct
{
	float   Barometric_altitude_delay;
	float   Airspeed_delay;
	float   Barometric_altitude;
	float   Airspeed;
	float   Barometric_altitude_standard_deviation;
	float   Airspeed_standard_deviation;
	uint8_t   Flags;
}External_Air_Data_Packet_t;
#define External_Air_Data_Packet_ID 0x79

 typedef struct
{
	float   Position_X;
	float   Position_Y;
	float   Heading;
	float   Position_X_standard_deviation;
	float   Position_Y_standard_deviation;
	float   Heading_standard_deviation;
}External_LIDAR_Packet_t;
#define External_LIDAR_Packet_ID 0x91

 typedef struct
{
	float   Position_X;
	float   Position_Y;
	float   Position_Z;
	float   Velocity_X;
	float   Velocity_Y;
	float   Velocity_Z;
	float   Roll;
	float   Pitch;
	float   Yaw;
	
	float   Position_X_standard_deviation;
	float   Position_Y_standard_deviation;
	float   Position_Z_standard_deviation;
	float   Velocity_X_standard_deviation;
	float   Velocity_Y_standard_deviation;
	float   Velocity_Z_standard_deviation;
	float   Roll_standard_deviation;
	float   Pitch_standard_deviation;
	float   Yaw_standard_deviation;
}External_SLAM1_Packet_t;
#define External_SLAM1_Packet_ID 0x92
typedef External_SLAM1_Packet_t External_SLAM2_Packet_t;
#define External_SLAM2_Packet_ID 0x93

 typedef struct
{
	uint8_t Type;
	uint32_t Microseconds;
	float Wheel_rate[4];
}External_Odom_Data_Packet_t;
#define External_Odom_Data_Packet_ID 0x90

 typedef struct
{
	uint8_t    ID;
	uint8_t    CMD;
	uint16_t   Reserved;
}Packet_Requst_Packet_t;
#define Packet_Requst_Packet_ID 0xA0

#endif //__FDILINK_DECODE_H
