#include <stdio.h>
#include <stdlib.h> /* for exit(EXIT_SUCCESS) or exit(EXIT_FAILURE)*/
#include <ncurses.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <gps.h>
#include <assert.h>
#include <unistd.h>

int     gps_start(void);
void    gps_stop(void);
//int     get_gps_data(struct gps_fix_t *);
int     get_gps_data(struct gps_data_t *gd);
