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


/*
 +test of Gordon Henderson's Projects softwarePWM implementation of the wiringPi library
 +https://projects.drogon.net 
 +code is presented as open source with no promises or reservations. 
 +claim you wrote it for all I fucking care.
 -JSH
 */

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <softPwm.h>

int l298n_test (void)
{
    int enableA = 1;
    int enableB = 6;
    int h1 = 16;
    int h2 = 15;
    int h3 = 11;
    int h4 = 10;

    if (wiringPiSetup() == -1)
        exit (1);

    printf ("Raspberry Pi SoftwarePWM HBridge Test\n");

    softPwmCreate (enableA, 0,255);
    softPwmCreate (enableB, 0,255);
    pinMode (h1, OUTPUT);
    pinMode (h2, OUTPUT);
    pinMode (h3, OUTPUT);
    pinMode (h4, OUTPUT);

    for (;;)
    {
        printf ("Forward\n");
        digitalWrite (h1, 1);
        digitalWrite (h2, 0);
        softPwmWrite (enableA, 250);
        digitalWrite (h3, 1); 
        digitalWrite (h4, 0);
        softPwmWrite (enableB, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (h1, 1);
        digitalWrite (h2, 0);
        softPwmWrite (enableA, 0);
        digitalWrite (h3, 1); 
        digitalWrite (h4, 0);
        softPwmWrite (enableB, 0);
        delay (250);

        printf ("Reverse\n");
        digitalWrite (h1, 0); 
        digitalWrite (h2, 1);
        softPwmWrite (enableA, 250);
        digitalWrite (h3, 0); 
        digitalWrite (h4, 1);
        softPwmWrite (enableB, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (h1, 1);
        digitalWrite (h2, 0);
        softPwmWrite (enableA, 0);
        digitalWrite (h3, 1); 
        digitalWrite (h4, 0);
        softPwmWrite (enableB, 0);
        delay (250);

        printf ("Right\n");
        digitalWrite (h1, 0); 
        digitalWrite (h2, 1);
        softPwmWrite (enableA, 250);
        digitalWrite (h3, 1); 
        digitalWrite (h4, 0);
        softPwmWrite (enableB, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (h1, 1);
        digitalWrite (h2, 0);
        softPwmWrite (enableA, 0);
        digitalWrite (h3, 1); 
        digitalWrite (h4, 0);
        softPwmWrite (enableB, 0);
        delay (250);
        printf ("Left\n");
        digitalWrite (h1, 1); 
        digitalWrite (h2, 0);
        softPwmWrite (enableA, 250);
        digitalWrite (h3, 0); 
        digitalWrite (h4, 1);
        softPwmWrite (enableB, 250);
        delay (1000);

        printf ("Stop\n");
        digitalWrite (h1, 1);
        digitalWrite (h2, 0);
        softPwmWrite (enableA, 0);
        digitalWrite (h3, 1); 
        digitalWrite (h4, 0);
        softPwmWrite (enableB, 0);
        delay (1250);
    }
    return 0;
}
