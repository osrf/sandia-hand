#ifndef CONTROL_H
#define CONTROL_H

void control_init();
void control_systick();
void control_set_motorspace(int16_t *targets);
void control_set_jointspace(float *targets);
void control_set_jointspace_fp(int16_t *targets);
void control_halt();
void control_systick_disable();
void control_systick_enable();

extern volatile float   g_control_joint_tgt[3];
extern volatile int32_t g_control_hall_tgt [3];

enum control_mode_t 
{ 
  CM_IDLE = 0, 
  CM_MOTOR_SPACE, 
  CM_JOINT_SPACE,
  CM_JOINT_SPACE_FP   // fixed-point
};

extern volatile enum control_mode_t g_control_mode;

#endif

