#ifndef FOC_H_ /* include guard */
#define FOC_H_

#include "system.h"

#define PI					3.141592654f
#define TWOPI				6.283185307f
#define FOURPI				12.566370614f
#define SHIFT_120			2.094395102f
#define SHIFT_240 			4.188790205f
#define _PI_3				1.047197551f    // pi / 3
#define	SQRT_3				1.732050808f    // sqrt(3)
#define SQR_THREE_TWO 		0.866025404f    // sqrt(3) / 2
#define ONE_THIRD			0.333333333f    // 1 / 3
#define ONE_SQR_THREE		0.577350269f    // 1 / sqrt(3)
#define	V_SUPPLY			48.0f		    //
#define	DEG_RAD				0.017453293f    // degrees to radians
#define ONE_2PI				0.159154943f    // 1 / (2pi)
#define PI_2				1.570796327f    // pi / 2

#define DPS_TO_RPM(xx)		((xx) / 6.0f)
#define MAX_SPEED			5000.0f			// Maximum RPM



#define POS_CONTROL
//#define SPD_CONTROL
//#define TRQ_CONTROL

//#define	FAST_TRIG
//#define	SINE_MODULATION

extern float m_fRotorThetaInit;
extern float m_fMechAngle;
extern float m_fMechAngleOffset;
extern float m_fCurrentA;
extern float m_fCurrentB;
extern float m_fId;
extern float m_fIq;
extern float m_fSpeed;
extern float m_fRotorTheta;

extern PID_Controller pi_axis_d;
extern PID_Controller pi_axis_q;
extern PID_Controller pi_speed;
extern PID_Controller pi_pos;

void FOC_Init(void);
void Run_SVM(void);

#endif /* FOC_H_ */
