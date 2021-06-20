//#include "mbed.h"

//#include "Examples/LEDPwm.h"

//#include "Examples/servo1.h"

//#include "Examples/Bluetooth_Display.h"

//#include "Examples/ADXL_345_Display.h"

//#include "aileron-pid.h"

//#include "controller_input.h"

//#include "I2C-test.h"

#include "MPU9250.h"
#include "pin_io.h"
#include <cstdio>
#include <string>


DigitalOut led(LED_PIN);

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