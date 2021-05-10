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
#define MAX_SPEED			500.0f			// Maximum speed (RPM)
#define MAX_CURRENT			10.0f			// Maximum current per phase (A)

#define Q_WIND		        1.0f
#define D_WIND		        1.0f
#define SPEED_WIND	        (MAX_CURRENT * 0.75f) // amps
#define POS_WIND	        8000.0f // deg/s
#define Kp			        0.050f
#define Ki			        10.74399996f
#define PID_FREQ	        4000.0f	// Hz


#define POS_CONTROL
//#define SPD_CONTROL
//#define TRQ_CONTROL

//#define	FAST_TRIG
//#define	SINE_MODULATION

extern float m_fRotorThetaInit;
extern double m_fMechAngle;
extern double m_fHomeOffset;
extern double m_fMechAngleLast;
extern float m_fCurrentA;
extern float m_fCurrentB;
extern float m_fCurrentC;
extern float m_fId;
extern float m_fIq;
extern float m_fSpeed;
extern float m_fSpeedFilt;
extern double velest;
extern double velintegrator;
extern float m_fRotorTheta;
extern float m_fSupplyVolt;

extern float rotor_theta_init_L;
extern int reversing;
extern int a_state;
extern int b_state;
extern int c_state;
extern double mechAngleoffset;

extern float m_fTrapzPwmVal;

extern float systemParams[256];

extern PID_Controller pi_axis_d;
extern PID_Controller pi_axis_q;
extern PID_Controller pi_speed;
extern PID_Controller pi_pos;;
extern PID_Controller pi_speed_trapz;




void FOC_Init(void);
void Run_SVM(void);

#endif /* FOC_H_ */
