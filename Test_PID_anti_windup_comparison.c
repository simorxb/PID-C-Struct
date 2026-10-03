#include <stdio.h>
#include <stdlib.h>

#include "object.h"
#include "pid.h"

#define LENGTH 800
#define TIME_STEP 0.1f

/* Same gains on both controllers. The anti-windup gain is placed where each
   method uses it:
   - general: Kaw = 1/C(inf), the back-calculation gain k_bc
   - classic: Kaw = Ki*k_bc, because this implementation adds Kaw*(u_sat-u)
     directly onto the integral state, after Ki
   The position reference is a 5 s ramp to 20 m, a hold, then a ramp back to 0.
   Force is limited to +/- 25 N. The command rate limit is left wide open so the
   comparison is about amplitude saturation. */
static float Setpoint(float t)
{
    const float ramp = 5.0f;
    const float hold = 40.0f;
    const float target = 20.0f;

    if (t < ramp)
    {
        return target * t / ramp;
    }
    if (t < hold)
    {
        return target;
    }
    if (t < hold + ramp)
    {
        return target * (1.0f - (t - hold) / ramp);
    }
    return 0.0f;
}

int main()
{
    const float Kp = 8.0f;
    const float Ki = 0.5f;
    const float Kd = 40.0f;
    const float T_C = 0.5f;
    const float force_limit = 25.0f;
    const float C_inf = Kp + Kd / T_C;
    const float k_bc = 1.0f / C_inf;

    float t = 0.0f;
    int i = 0;
    float setpoint;
    float command_classic;
    float command_general;
    float z_classic = 0.0f;
    float z_general = 0.0f;

    struct PID pid_classic = {Kp, Ki, Kd, Ki * k_bc, T_C, TIME_STEP, force_limit, -force_limit, 1000.0f, 0, 0, 0, 0, 0, PID_BACK_CALCULATION_CLASSIC, 0, 0};
    struct PID pid_general = {Kp, Ki, Kd, k_bc, T_C, TIME_STEP, force_limit, -force_limit, 1000.0f, 0, 0, 0, 0, 0, PID_BACK_CALCULATION_GENERAL, 0, 0};

    struct Object obj_classic = {10, 0.5f, force_limit, -force_limit, TIME_STEP, 0, 0};
    struct Object obj_general = {10, 0.5f, force_limit, -force_limit, TIME_STEP, 0, 0};

    FILE *file = fopen("data_PID_anti_windup_comparison.txt", "w");

    /* Columns: t, setpoint, z_classic, u_classic, u_unsat_classic, z_general, u_general, u_unsat_general */
    while (i < LENGTH)
    {
        setpoint = Setpoint(t);

        command_classic = PID_Step(&pid_classic, z_classic, setpoint);
        z_classic = Object_Step(&obj_classic, command_classic);

        command_general = PID_Step(&pid_general, z_general, setpoint);
        z_general = Object_Step(&obj_general, command_general);

        fprintf(file, "%f %f %f %f %f %f %f %f\n",
                t,
                setpoint,
                z_classic,
                command_classic,
                pid_classic.command_prev,
                z_general,
                command_general,
                pid_general.command_prev);

        t = t + TIME_STEP;
        i = i + 1;
    }

    fclose(file);
    exit(0);
}
