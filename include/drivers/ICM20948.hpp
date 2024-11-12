#pragma once

#include "i2c_helpers.hpp"


class ICM20948 {
    public:
    	typedef struct imu_st_angles_data_tag
	{
		  float fYaw;
		  float fPitch;
		  float fRoll;
	}IMU_ST_ANGLES_DATA;

	typedef struct imu_st_sensor_data_tag
	{
		  int16_t s16X;
		  int16_t s16Y;
		  int16_t s16Z;
	}IMU_ST_SENSOR_DATA;

	typedef struct icm20948_st_avg_data_tag
	{
	  uint8_t u8Index;
		int16_t s16AvgBuffer[8];
	}ICM20948_ST_AVG_DATA;

    bool init(int i2c, uint8_t addr);
    bool read_data(IMU_ST_ANGLES_DATA &ang, IMU_ST_SENSOR_DATA& acc, IMU_ST_SENSOR_DATA &mag, IMU_ST_SENSOR_DATA &pos);
    
    private:
    int i2c;
    uint8_t addr;
    uint8_t read_byte(uint8_t reg);
    void read_bytes(uint8_t reg, uint8_t *data, size_t len);
    void GyroRead(int16_t*, int16_t*, int16_t*);
    void CalAvgValue(uint8_t*, int16_t*, int16_t, int32_t*);
    void GyroOffset(void);
    bool MagCheck(void);
    void MagRead(int16_t*, int16_t*, int16_t*);
    void ReadSecondary(uint8_t, uint8_t, uint8_t, uint8_t*);
    void WriteSecondary(uint8_t, uint8_t, uint8_t);
    void AccelRead(int16_t*, int16_t*, int16_t*);
    uint8_t I2C_ReadOneByte(uint8_t u8RegAddr);
    void I2C_WriteOneByte(uint8_t u8RegAddr, uint8_t u8data);
    void calAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal);
    IMU_ST_SENSOR_DATA gstGyroOffset ={0,0,0};  

};
