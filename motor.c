/*********************************************************************
 *
 *   File : 
 *       motor.c
 *
 *   Date : 
 *       11/05/2014
 *
 *   Contributors : 
 *       TSAO, CHIA-CHENG <chiacheng.tsao@gmail.com> , From NTUT
 *
 *********************************************************************/
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <softPwm.h>
#include "motor.h"

#define L298N_ENA 0
#define L298N_ENB 2
#define L298N_IN1 1
#define L298N_IN2 4
#define L298N_IN3 5
#define L298N_IN4 6

int l298n_test (void)
{
    int ena = L298N_ENA;
    int enb = L298N_ENB;
    int in1 = L298N_IN1;
    int in2 = L298N_IN2;
    int in3 = L298N_IN3;
    int in4 = L298N_IN4;

    if (wiringPiSetup() == -1)
        exit (1);

    printf ("Raspberry Pi SoftwarePWM HBridge Test\n");

    softPwmCreate (ena, 0,255);
    softPwmCreate (enb, 0,255);
    pinMode (in1, OUTPUT);
    pinMode (in2, OUTPUT);
    pinMode (in3, OUTPUT);
    pinMode (in4, OUTPUT);

    for (;;)
    {
        printf ("Forward\n");
        digitalWrite (in1, 1);
        digitalWrite (in2, 0);
        softPwmWrite (ena, 250);
        digitalWrite (in3, 1); 
        digitalWrite (in4, 0);
        softPwmWrite (enb, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (in1, 1);
        digitalWrite (in2, 0);
        softPwmWrite (ena, 0);
        digitalWrite (in3, 1); 
        digitalWrite (in4, 0);
        softPwmWrite (enb, 0);
        delay (250);

        printf ("Reverse\n");
        digitalWrite (in1, 0); 
        digitalWrite (in2, 1);
        softPwmWrite (ena, 250);
        digitalWrite (in3, 0); 
        digitalWrite (in4, 1);
        softPwmWrite (enb, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (in1, 1);
        digitalWrite (in2, 0);
        softPwmWrite (ena, 0);
        digitalWrite (in3, 1); 
        digitalWrite (in4, 0);
        softPwmWrite (enb, 0);
        delay (250);

        printf ("Right\n");
        digitalWrite (in1, 0); 
        digitalWrite (in2, 1);
        softPwmWrite (ena, 250);
        digitalWrite (in3, 1); 
        digitalWrite (in4, 0);
        softPwmWrite (enb, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (in1, 1);
        digitalWrite (in2, 0);
        softPwmWrite (ena, 0);
        digitalWrite (in3, 1); 
        digitalWrite (in4, 0);
        softPwmWrite (enb, 0);
        delay (250);
        printf ("Left\n");
        digitalWrite (in1, 1); 
        digitalWrite (in2, 0);
        softPwmWrite (ena, 250);
        digitalWrite (in3, 0); 
        digitalWrite (in4, 1);
        softPwmWrite (enb, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (in1, 1);
        digitalWrite (in2, 0);
        softPwmWrite (ena, 0);
        digitalWrite (in3, 1); 
        digitalWrite (in4, 0);
        softPwmWrite (enb, 0);
        delay (1250);
    }
    return 0;
}

typedef struct _motor
{
    int ena;
    int enb;
    int in1;
    int in2;
    int in3;
    int in4;

}motor_t;

void motor_initialize(void)
{
    //setreuid(geteuid(), getuid());
    //seteuid(0);

    if (wiringPiSetup() == -1)
        exit (1);

    softPwmCreate (L298N_ENA, 0,255);
    softPwmCreate (L298N_ENB, 0,255);
    pinMode (L298N_IN1, OUTPUT);
    pinMode (L298N_IN2, OUTPUT);
    pinMode (L298N_IN3, OUTPUT);
    pinMode (L298N_IN4, OUTPUT);
}

void motor_update(m_ctrl_t mode, int pwm1, int pwm2)
{
    pwm1 = abs(pwm1);
    pwm2 = abs(pwm2);

    switch(mode)
    {
        case M_FWD :

            digitalWrite (L298N_IN1, 1);
            digitalWrite (L298N_IN2, 0);
            softPwmWrite (L298N_ENA, pwm1);
            digitalWrite (L298N_IN3, 1); 
            digitalWrite (L298N_IN4, 0);
            softPwmWrite (L298N_ENB, pwm2);

            break;

        case M_BWD :

            digitalWrite (L298N_IN1, 0); 
            digitalWrite (L298N_IN2, 1);
            softPwmWrite (L298N_ENA, pwm1);
            digitalWrite (L298N_IN3, 0); 
            digitalWrite (L298N_IN4, 1);
            softPwmWrite (L298N_ENB, pwm2);

            break;

        case M_TNL :

            digitalWrite (L298N_IN1, 1); 
            digitalWrite (L298N_IN2, 0);
            softPwmWrite (L298N_ENA, pwm1);
            digitalWrite (L298N_IN3, 0); 
            digitalWrite (L298N_IN4, 1);
            softPwmWrite (L298N_ENB, pwm2);

            break;

        case M_TNR :

            digitalWrite (L298N_IN1, 0); 
            digitalWrite (L298N_IN2, 1);
            softPwmWrite (L298N_ENA, pwm1);
            digitalWrite (L298N_IN3, 1); 
            digitalWrite (L298N_IN4, 0);
            softPwmWrite (L298N_ENB, pwm2);

            break;

        case M_BRK :

            #if 0 /* Motor Off */
            digitalWrite (L298N_IN1, 1);
            digitalWrite (L298N_IN2, 0);
            softPwmWrite (L298N_ENA, 0);
            digitalWrite (L298N_IN3, 1); 
            digitalWrite (L298N_IN4, 0);
            softPwmWrite (L298N_ENB, 0);
            #else /* Motor Break */
            digitalWrite (L298N_IN1, 1);
            digitalWrite (L298N_IN2, 1);
            softPwmWrite (L298N_ENA, 1);
            digitalWrite (L298N_IN3, 1); 
            digitalWrite (L298N_IN4, 1);
            softPwmWrite (L298N_ENB, 1);
            #endif

            break;

        default :

            break;
    }
}

