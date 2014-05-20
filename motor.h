#ifndef _MOTOR_H_
#define _MOTOR_H_

typedef enum _m_ctrl{M_FWD, M_BWD, M_TNL, M_TNR, M_BRK}m_ctrl_t;

void motor_initialize(void);
void motor_update(m_ctrl_t mode, int pwm1, int pwm2);

#endif
