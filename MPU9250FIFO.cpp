#include "MPU9250FIFO.h"

/* configures and enables the FIFO buffer  */
int MPU9250FIFO::enableFifo(bool accel, bool gyro, bool mag, bool temp) {
	if (writeRegister(USER_CTRL, (0x40 | I2C_MST_EN)) < 0) {
		return -1;
	}
	if (writeRegister(FIFO_EN, (accel * FIFO_ACCEL) | (gyro * FIFO_GYRO) | (mag * FIFO_MAG) | (temp * FIFO_TEMP)) < 0) {
		return -2;
	}
	_enFifoAccel = accel;
	_enFifoGyro = gyro;
	_enFifoMag = mag;
	_enFifoTemp = temp;
	_fifoFrameSize = accel * 6 + gyro * 6 + mag * 7 + temp * 2;
	return 1;
}

/* reads data from the MPU9250 FIFO and stores in buffer */
int MPU9250FIFO::readFifo() {
	// get the fifo size
	readRegisters(FIFO_COUNT, 2, _buffer);
	_fifoSize = (((uint16_t)(_buffer[0] & 0x0F)) << 8) + (((uint16_t)_buffer[1]));
	// read and parse the buffer
	for (size_t i = 0; i < _fifoSize / _fifoFrameSize; i++) {
		// grab the data from the MPU9250
		if (readRegisters(FIFO_READ, _fifoFrameSize, _buffer) < 0) {
			return -1;
		}
		if (_enFifoAccel) {
			// combine into 16 bit values
			_axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
			_aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
			_azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
			// transform and convert to float values
			_axFifo[i] = (((float)(tX[0] * _axcounts + tX[1] * _aycounts + tX[2] * _azcounts) * _accelScale) - _axb) * _axs;
			_ayFifo[i] = (((float)(tY[0] * _axcounts + tY[1] * _aycounts + tY[2] * _azcounts) * _accelScale) - _ayb) * _ays;
			_azFifo[i] = (((float)(tZ[0] * _axcounts + tZ[1] * _aycounts + tZ[2] * _azcounts) * _accelScale) - _azb) * _azs;
			_aSize = _fifoSize / _fifoFrameSize;
		}
		if (_enFifoTemp) {
			// combine into 16 bit values
			_tcounts = (((int16_t)_buffer[0 + _enFifoAccel * 6]) << 8) | _buffer[1 + _enFifoAccel * 6];
			// transform and convert to float values
			_tFifo[i] = ((((float)_tcounts) - _tempOffset) / _tempScale) + _tempOffset;
			_tSize = _fifoSize / _fifoFrameSize;
		}
		if (_enFifoGyro) {
			// combine into 16 bit values
			_gxcounts = (((int16_t)_buffer[0 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) | _buffer[1 + _enFifoAccel * 6 + _enFifoTemp * 2];
			_gycounts = (((int16_t)_buffer[2 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) | _buffer[3 + _enFifoAccel * 6 + _enFifoTemp * 2];
			_gzcounts = (((int16_t)_buffer[4 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) | _buffer[5 + _enFifoAccel * 6 + _enFifoTemp * 2];
			// transform and convert to float values
			_gxFifo[i] = ((float)(tX[0] * _gxcounts + tX[1] * _gycounts + tX[2] * _gzcounts) * _gyroScale) - _gxb;
			_gyFifo[i] = ((float)(tY[0] * _gxcounts + tY[1] * _gycounts + tY[2] * _gzcounts) * _gyroScale) - _gyb;
			_gzFifo[i] = ((float)(tZ[0] * _gxcounts + tZ[1] * _gycounts + tZ[2] * _gzcounts) * _gyroScale) - _gzb;
			_gSize = _fifoSize / _fifoFrameSize;
		}
		if (_enFifoMag) {
			// combine into 16 bit values
			_hxcounts = (((int16_t)_buffer[1 + _enFifoAccel * 6 + _enFifoTemp * 2 + _enFifoGyro * 6]) << 8) | _buffer[0 + _enFifoAccel * 6 + _enFifoTemp * 2 + _enFifoGyro * 6];
			_hycounts = (((int16_t)_buffer[3 + _enFifoAccel * 6 + _enFifoTemp * 2 + _enFifoGyro * 6]) << 8) | _buffer[2 + _enFifoAccel * 6 + _enFifoTemp * 2 + _enFifoGyro * 6];
			_hzcounts = (((int16_t)_buffer[5 + _enFifoAccel * 6 + _enFifoTemp * 2 + _enFifoGyro * 6]) << 8) | _buffer[4 + _enFifoAccel * 6 + _enFifoTemp * 2 + _enFifoGyro * 6];
			// transform and convert to float values
			_hxFifo[i] = (((float)(_hxcounts)*_magScaleX) - _hxb) * _hxs;
			_hyFifo[i] = (((float)(_hycounts)*_magScaleY) - _hyb) * _hys;
			_hzFifo[i] = (((float)(_hzcounts)*_magScaleZ) - _hzb) * _hzs;
			_hSize = _fifoSize / _fifoFrameSize;
		}
	}
	return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU9250FIFO::getFifoAccelX_mss(size_t* size, float* data) {
	*size = _aSize;
	memcpy(data, _axFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU9250FIFO::getFifoAccelY_mss(size_t* size, float* data) {
	*size = _aSize;
	memcpy(data, _ayFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU9250FIFO::getFifoAccelZ_mss(size_t* size, float* data) {
	*size = _aSize;
	memcpy(data, _azFifo, _aSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void MPU9250FIFO::getFifoGyroX_rads(size_t* size, float* data) {
	*size = _gSize;
	memcpy(data, _gxFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void MPU9250FIFO::getFifoGyroY_rads(size_t* size, float* data) {
	*size = _gSize;
	memcpy(data, _gyFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void MPU9250FIFO::getFifoGyroZ_rads(size_t* size, float* data) {
	*size = _gSize;
	memcpy(data, _gzFifo, _gSize * sizeof(float));
}

/* returns the magnetometer FIFO size and data in the x direction, uT */
void MPU9250FIFO::getFifoMagX_uT(size_t* size, float* data) {
	*size = _hSize;
	memcpy(data, _hxFifo, _hSize * sizeof(float));
}

/* returns the magnetometer FIFO size and data in the y direction, uT */
void MPU9250FIFO::getFifoMagY_uT(size_t* size, float* data) {
	*size = _hSize;
	memcpy(data, _hyFifo, _hSize * sizeof(float));
}

/* returns the magnetometer FIFO size and data in the z direction, uT */
void MPU9250FIFO::getFifoMagZ_uT(size_t* size, float* data) {
	*size = _hSize;
	memcpy(data, _hzFifo, _hSize * sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void MPU9250FIFO::getFifoTemperature_C(size_t* size, float* data) {
	*size = _tSize;
	memcpy(data, _tFifo, _tSize * sizeof(float));
}
