#include "MPU9250.h"
#include <cstdio>
#include <string>

//Tested on F411RE blackpill minimum dev board, compiled for Nucleo F411RE.
#define I2C1_SCL PB_6
#define I2C1_SDA PB_7

BufferedSerial serial(USBTX, USBRX, 115200);
FILE * pc;

I2C i2c(I2C1_SDA, I2C1_SCL);

MPU9250 mpu(i2c, 0x68<<1);

int main(){
    int status = mpu.begin();
    if (status < 0) {
        while(1) {}
    }
    pc = fdopen(&serial, "r+");
    fprintf(pc, "Begin code: %i\n", status);
    while (true) {
        mpu.readSensor();
        fprintf(pc, "%f %f %f %f %f %f %f %f %f %f\n", mpu.getAccelX_mss(), mpu.getAccelY_mss(), mpu.getAccelZ_mss(), mpu.getGyroX_rads(), mpu.getGyroY_rads(), mpu.getGyroZ_rads(), mpu.getMagX_uT(), mpu.getMagY_uT(), mpu.getMagZ_uT(), mpu.getTemperature_C());
        ThisThread::sleep_for(100ms);
    }
    
}