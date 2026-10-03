#include "pid.h"

static float PID_Limit_Command(struct PID *pid, float command)
{
    float command_sat;

    /* Saturate command */
    if (command > pid->max)
    {
        command_sat = pid->max;
    }
    else if (command < pid->min)
    {
        command_sat = pid->min;
    }
    else
    {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > pid->command_sat_prev + pid->max_rate * pid->T)
    {
        command_sat = pid->command_sat_prev + pid->max_rate * pid->T;
    }
    else if (command_sat < pid->command_sat_prev - pid->max_rate * pid->T)
    {
        command_sat = pid->command_sat_prev - pid->max_rate * pid->T;
    }

    return command_sat;
}

static float PID_Classic_Command(struct PID *pid, float err)
{
    float deriv_filt;

    /* Integral term calculation - including anti-windup.
       Kaw multiplies (u_sat - u) and is added directly to the integral state. */
    pid->integral += pid->Ki * err * pid->T + pid->Kaw * (pid->command_sat_prev - pid->command_prev) * pid->T;

    /* Derivative term calculation using filtered derivative method */
    deriv_filt = (err - pid->err_prev + pid->T_C * pid->deriv_prev) / (pid->T + pid->T_C);
    pid->err_prev = err;
    pid->deriv_prev = deriv_filt;

    /* Summing the 3 terms */
    return pid->Kp * err + pid->integral + pid->Kd * deriv_filt;
}

/* General back-calculation for C(s) = Kp + Ki/s + Kd s / (T_C s + 1).
   C(inf) = Kp + Kd/T_C is applied directly to the error.
   The dynamic part is C1(z), Tustin, followed by a forward-Euler integrator.
   Kaw*(u_sat - u) is added to the error at the input of that dynamic part. */
static float PID_General_Command(struct PID *pid, float err)
{
    return (pid->Kp + pid->Kd / pid->T_C) * err + pid->integral;
}

static void PID_General_Update(struct PID *pid, float err, float command, float command_sat)
{
    float K;
    float b0;
    float b1;
    float a0;
    float a1;
    float dynamic_in;
    float c1_out;

    K = pid->Ki * pid->T_C - pid->Kd / pid->T_C;
    b0 = 2.0f * K / pid->T + pid->Ki;
    b1 = pid->Ki - 2.0f * K / pid->T;
    a0 = 2.0f * pid->T_C / pid->T + 1.0f;
    a1 = 1.0f - 2.0f * pid->T_C / pid->T;

    dynamic_in = err + pid->Kaw * (command_sat - command);
    c1_out = (b0 * dynamic_in + b1 * pid->c1_in_prev - a1 * pid->c1_out_prev) / a0;

    /* Forward Euler so the integrator output used this step does not depend on c1_out. */
    pid->integral += pid->T * c1_out;
    pid->c1_in_prev = dynamic_in;
    pid->c1_out_prev = c1_out;
}

float PID_Step(struct PID *pid, float measurement, float setpoint)
{
    /* This function implements a PID controller.
     *
     * Inputs:
     *   measurement: current measurement of the process variable
     *   setpoint: desired value of the process variable
     *   pid: a pointer to a PID struct containing the controller parameters
     *
     * Returns:
     *   command_sat: the control output of the PID controller (saturated based on max, min, max_rate)
     */

    float err;
    float command;
    float command_sat;

    err = setpoint - measurement;

    if (pid->back_calculation == PID_BACK_CALCULATION_GENERAL)
    {
        command = PID_General_Command(pid, err);
    }
    else
    {
        command = PID_Classic_Command(pid, err);
    }

    command_sat = PID_Limit_Command(pid, command);

    if (pid->back_calculation == PID_BACK_CALCULATION_GENERAL)
    {
        PID_General_Update(pid, err, command, command_sat);
    }

    /* Remember command at previous step */
    pid->command_prev = command;

    /* Remember saturated command at previous step */
    pid->command_sat_prev = command_sat;

    return command_sat;
}
