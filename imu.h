#ifndef _IMU_H_
#define _IMU_H_

#define BMX055_A  0x18
#define BMX055_G  0x68
#define BMX055_M  0x10

void imu_initialize(void);
int get_mag_id(void);
#endif