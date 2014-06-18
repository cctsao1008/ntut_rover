/*********************************************************************
 *
 *   File : 
 *       imu.c
 *
 *   Date : 
 *       11/05/2014
 *
 *   Contributors : 
 *       TSAO, CHIA-CHENG <chiacheng.tsao@gmail.com> , From NTUT
 *
 *********************************************************************/
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include "imu.h"

static int fd_a, fd_g, fd_m;

void imu_initialize(void)
{
    if (wiringPiSetup() == -1)
        exit (1);

    fd_a = wiringPiI2CSetup (BMX055_A);

    if (fd_a < 0 )
    {
        printf(" Unable to initialise I2C(A): \n");
    }

    fd_g = wiringPiI2CSetup (BMX055_G);

    if (fd_g < 0 )
    {
        printf(" Unable to initialise I2C(G): \n");
    }
    
    fd_m = wiringPiI2CSetup (BMX055_M);

    if (fd_m < 0 )
    {
        printf(" Unable to initialise I2C(M): \n");
    }

    wiringPiI2CWriteReg8 (fd_m, 0x4b, 0x01);
}

int get_mag_id(void)
{
    if (fd_m < 0 )
        return (-1);

    return wiringPiI2CReadReg8(fd_m, 0x40);
}
 