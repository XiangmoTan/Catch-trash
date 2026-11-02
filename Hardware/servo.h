#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(void);
void Servo_SetAngle(float Angle, float Angle_left);
void Servo360_SetAngle_Fake(float target_angle);

#endif
