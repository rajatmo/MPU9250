
#ifndef MPU9250FIFO_h
#define MPU9250FIFO_h

#include "MPU9250.h"
class MPU9250FIFO : public MPU9250 {
public:
	using MPU9250::MPU9250;
	int enableFifo(bool accel, bool gyro, bool mag, bool temp);
	int readFifo();
	void getFifoAccelX_mss(size_t* size, float* data);
	void getFifoAccelY_mss(size_t* size, float* data);
	void getFifoAccelZ_mss(size_t* size, float* data);
	void getFifoGyroX_rads(size_t* size, float* data);
	void getFifoGyroY_rads(size_t* size, float* data);
	void getFifoGyroZ_rads(size_t* size, float* data);
	void getFifoMagX_uT(size_t* size, float* data);
	void getFifoMagY_uT(size_t* size, float* data);
	void getFifoMagZ_uT(size_t* size, float* data);
	void getFifoTemperature_C(size_t* size, float* data);

protected:
	// fifo
	bool _enFifoAccel, _enFifoGyro, _enFifoMag, _enFifoTemp;
	size_t _fifoSize, _fifoFrameSize;
	float _axFifo[85], _ayFifo[85], _azFifo[85];
	size_t _aSize;
	float _gxFifo[85], _gyFifo[85], _gzFifo[85];
	size_t _gSize;
	float _hxFifo[73], _hyFifo[73], _hzFifo[73];
	size_t _hSize;
	float _tFifo[256];
	size_t _tSize;
};

#endif