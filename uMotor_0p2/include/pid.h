#ifndef PID_H_ /* include guard */
#define PID_H_

#include <stdio.h>

typedef struct PID_Control
{
	double kP;
	double kI;
	double kD;
	double deltaTime;
	double setPoint;

	uint32_t updates; /*I know you wont listen, but don't ever touch these members.*/
	double sampleError;/*I know you wont listen, but don't ever touch these members.*/
	double deltaInput;/*I know you wont listen, but don't ever touch these members.*/
	double lastError;/*I know you wont listen, but don't ever touch these members.*/
	double PTerm;/*I know you wont listen, but don't ever touch these members.*/
	double ITerm;/*I know you wont listen, but don't ever touch these members.*/
	double DTerm;/*I know you wont listen, but don't ever touch these members.*/
	double controllerOutput;/*I know you wont listen, but don't ever touch these members.*/
	double windupMax;/*I know you wont listen, but don't ever touch these members.*/
	double windupMin;/*I know you wont listen, but don't ever touch these members.*/
} PID_Controller;

void PID_Initialize	(PID_Controller * _PID);
double PID_Update	(PID_Controller * _PID, double systemFeedback);
void PID_SetSetpoint(PID_Controller * _PID, double newSetpoint);
void PID_SetKp		(PID_Controller * _PID, double proportional_gain);
void PID_SetKi		(PID_Controller * _PID, double integral_gain);
void PID_SetKd		(PID_Controller * _PID, double derivative_gain);
void PID_Reset		(PID_Controller * _PID);
double PID_GetOutput	(PID_Controller * _PID);
void PID_SetOutput	(PID_Controller * _PID, double newOutput);
double PID_GetSetpoint(PID_Controller * _PID);
double PID_GetLastInput(PID_Controller * _PID);

#endif /* PID_H_ */
