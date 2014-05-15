CC=gcc
CC_OPTIONS=-Wall -lgps -lncurses -lwiringPi -lpthread

ntut-rover : core

# core
core : core.o gps.o motor.o
	$(CC) -o ntut-rover core.o gps.o motor.o $(CC_OPTIONS)

core.o : core.c
	$(CC) $(CC_OPTIONS) -c core.c -o core.o

# gps support
gps.o : gps.c
	$(CC) $(CC_OPTIONS) -c gps.c -o gps.o

# motor control
	$(CC) $(CC_OPTIONS) -c motor.c -o motor.o

# cleaning
clean :
	rm -f *.o ntut-rover
