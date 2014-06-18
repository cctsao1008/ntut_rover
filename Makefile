CC=gcc
CC_OPTIONS=-Wall -lgps -lncurses -lwiringPi -lpthread -L./xively -lxively -I xively -std=gnu99

ntut-rover : all

# core
all : core.o gps.o imu.o motor.o socket.o
	$(CC) -o ntut-rover core.o gps.o imu.o motor.o socket.o $(CC_OPTIONS)

# core
core.o : core.c
	$(CC) $(CC_OPTIONS) -c core.c -o core.o

# gps
gps.o : gps.c
	$(CC) $(CC_OPTIONS) -c gps.c -o gps.o

# imu
imu.o : imu.c
	$(CC) $(CC_OPTIONS) -c imu.c -o imu.o

# motor
motor.o : motor.c
	$(CC) $(CC_OPTIONS) -c motor.c -o motor.o

# socket
socket.o : socket.c
	$(CC) $(CC_OPTIONS) -c socket.c -o socket.o

# cleaning
clean :
	rm -f *.o ntut-rover
