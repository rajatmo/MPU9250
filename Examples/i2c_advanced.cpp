#include "MPU9250.h"
#include <cstdio>

//Tested on F411RE blackpill minimum dev board, compiled for Nucleo F411RE.
#define I2C1_SCL PB_6
#define I2C1_SDA PB_7

BufferedSerial serial(USBTX, USBRX, 115200);
FILE* pc;

I2C i2c(I2C1_SDA, I2C1_SCL);

MPU9250 mpu(i2c, 0x68 << 1);

int main() {
	i2c.frequency(400 * 1000);

	int status = mpu.begin();
	if (status < 0) {
		while (1) {}
	}
	// setting the accelerometer full scale range to +/-8G 
	mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
	// setting the gyroscope full scale range to +/-500 deg/s
	mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
	// setting DLPF bandwidth to 20 Hz
	mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
	// setting SRD to 19 for a 50 Hz update rate
	mpu.setSrd(19);
	pc = fdopen(&serial, "r+");
	fprintf(pc, "Begin code: %i\n", status);
	while (true) {
		mpu.readSensor();
		fprintf(pc, "AX:%f AY:%f AZ:%f GX:%f GY:%f GZ:%f MX:%f MY:%f MZ:%f T:%f\n", mpu.getAccelX_mss(), mpu.getAccelY_mss(), mpu.getAccelZ_mss(), mpu.getGyroX_rads(), mpu.getGyroY_rads(), mpu.getGyroZ_rads(), mpu.getMagX_uT(), mpu.getMagY_uT(), mpu.getMagZ_uT(), mpu.getTemperature_C());
		ThisThread::sleep_for(20ms);
	}

}