#include "MPU9250FIFO.h"
#include <cstdio>

//Tested on F411RE blackpill minimum dev board, compiled for Nucleo F411RE.
#define I2C1_SCL PB_6
#define I2C1_SDA PB_7

BufferedSerial serial(USBTX, USBRX, 115200);
FILE* pc;

I2C i2c(I2C1_SDA, I2C1_SCL);

MPU9250FIFO mpu(i2c, 0x68 << 1);
int status;

// variables to hold FIFO data, these need to be large enough to hold the data
float ax[100], ay[100], az[100];
size_t fifoSize;

int main() {
	i2c.frequency(400 * 1000);

	status = mpu.begin();
	if (status < 0) {
		while (1) {}
	}
	// setting DLPF bandwidth to 20 Hz
	mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
	// setting SRD to 19 for a 50 Hz update rate
	mpu.setSrd(19);
	// enabling the FIFO to record just the accelerometers
	mpu.enableFifo(true, false, false, false);
	// gather 50 samples of data
	ThisThread::sleep_for(980ms);
	// read the fifo buffer from the IMU
	mpu.readFifo();
	// get the X, Y, and Z accelerometer data and their size
	mpu.getFifoAccelX_mss(&fifoSize, ax);
	mpu.getFifoAccelY_mss(&fifoSize, ay);
	mpu.getFifoAccelZ_mss(&fifoSize, az);

	pc = fdopen(&serial, "r+");
	fprintf(pc, "The FIFO buffer is %u samples long.\n", fifoSize);
	for (size_t i = 0; i < fifoSize; i++) {
		fprintf(pc, "%f\t%f\t%f\n", ax[i], ay[i], az[i]);
	}
	while (true) {

	}

}