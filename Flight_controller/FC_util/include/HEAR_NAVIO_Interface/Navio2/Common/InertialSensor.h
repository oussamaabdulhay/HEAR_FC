#ifndef _INERTIAL_SENSOR_H
#define _INERTIAL_SENSOR_H

class InertialSensor {
public:
    virtual bool initialize() = 0;
    virtual bool probe() = 0;
    virtual void update() = 0;

    float read_temperature() {return temperature;};
    void read_accelerometer(float *ax, float *ay, float *az) {*ax = _ax; *ay = _ay; *az = _az;};
    void read_raw_accelerometer(int *rax, int *ray, int *raz) {*rax = _rax; *ray = _ray; *raz = _raz;};
    void read_gyroscope(float *gx, float *gy, float *gz) {*gx = _gx; *gy = _gy; *gz = _gz;};
    void read_raw_gyroscope(int *rgx, int *rgy, int *rgz) {*rgx = _rgx; *rgy = _rgy; *rgz = _rgz;};
    void read_magnetometer(float *mx, float *my, float *mz) {*mx = _mx; *my = _my; *mz = _mz;};
    void read_raw_magnetometer(int *rmx, int *rmy, int *rmz) {*rmx = _rmx; *rmy = _rmy; *rmz = _rmz;};

protected:
    float temperature;
    float _ax, _ay, _az;
    int _rax, _ray, _raz;

    float _gx, _gy, _gz;
    int _rgx, _rgy, _rgz;

    float _mx, _my, _mz;
    int _rmx, _rmy, _rmz;
};

#endif //_INERTIAL_SENSOR_H
