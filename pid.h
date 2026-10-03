#ifndef PID_H
#define PID_H

/* Classical back-calculation keeps the existing integral update.
   General back-calculation splits the PID into a direct term and a dynamic
   term, and feeds (u_sat - u) back at the input of the dynamic term. */
enum PID_BackCalculation
{
    PID_BACK_CALCULATION_CLASSIC = 0,
    PID_BACK_CALCULATION_GENERAL = 1
};

struct PID
{
    float Kp;               /* Proportional gain constant */
    float Ki;               /* Integral gain constant */
    float Kd;               /* Derivative gain constant */
    float Kaw;              /* Anti-windup gain. Classic: added to the integral state.
                               General: added to the dynamic-part input (k_bc). */
    float T_C;              /* Time constant for derivative filtering */
    float T;                /* Time step */
    float max;              /* Max command */
    float min;              /* Min command */
    float max_rate;         /* Max rate of change of the command */
    float integral;         /* Integral term */
    float err_prev;         /* Previous error */
    float deriv_prev;       /* Previous derivative */
    float command_sat_prev; /* Previous saturated command */
    float command_prev;     /* Previous unsaturated command */
    enum PID_BackCalculation back_calculation;
    float c1_in_prev;       /* Previous input of the general-method filter */
    float c1_out_prev;      /* Previous output of the general-method filter */
};

float PID_Step(struct PID *pid, float measurement, float setpoint);

#endif
