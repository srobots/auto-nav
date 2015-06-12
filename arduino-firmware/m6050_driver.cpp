
#include "m6050_driver.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

mpu6050 mpu

void dmpDataReady()
{
    mpu6050.mpuInterrupt = true;
}

void m6050Driver::init()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true);
#endif

    mpu.initialize();
    mpu6050.devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (mpu6050.devStatus == 0)
    {
	mpu.setDMPEnabled(true);
	attachInterrupt(0, dmpDataReady, RISING);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packageSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
	//error
    }
}

void m6050Driver::processRawData()
{
}

void m6050Driver::getYPR(float *stroeArr, uint8_t len)
{
}

void m6050TaskScheduler()
{
}
