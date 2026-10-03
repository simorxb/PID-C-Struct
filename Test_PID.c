#include <stdio.h>
#include <stdlib.h>

#include "object.h"
#include "pid.h"

#define LENGTH 1200
#define TIME_STEP 0.1

int main()
{
    // Current simulation time
    float t = 0;

    // Iteration counter
    int i = 0;

    // Setpoint and output of the first control loop
    float command1 = 0;
    float stp1 = 100;
    float z1 = 0;

    // Setpoint and output of the second control loop
    float command2 = 0;
    float stp2 = 50;
    float z2 = 0;

    // PID controller parameters for the first control loop.
    // back_calculation selects classical back-calculation, matching the original test.
    struct PID pid1 = {1, 0.1, 5, 0.1, 1, TIME_STEP, 100, -100, 40, 0, 0, 0, 0, 0, PID_BACK_CALCULATION_CLASSIC, 0, 0};

    // Object parameters for the first control loop
    struct Object obj1 = {10, 0.5, 100, -100, TIME_STEP, 0, 0};

    // PID controller parameters for the second control loop
    struct PID pid2 = {1.8, 0.3, 7, 0.3, 1, TIME_STEP, 100, -100, 40, 0, 0, 0, 0, 0, PID_BACK_CALCULATION_CLASSIC, 0, 0};

    // Object parameters for the second control loop
    struct Object obj2 = {10, 0.5, 100, -100, TIME_STEP, 0, 0};

    // Open a file for logging simulation data
    FILE *file = fopen("data_PID_C.txt", "w");

    /* Implement iteration using a while loop */
    while (i < LENGTH)
    {
        /* Change setpoint at t = 60 seconds */
        if (t < 60)
        {
            stp1 = 100;
            stp2 = 50;
        }
        else
        {
            stp1 = 200;
            stp2 = 150;
        }

        // Execute the first control loop
        command1 = PID_Step(&pid1, z1, stp1);
        z1 = Object_Step(&obj1, command1);

        // Execute the second control loop
        command2 = PID_Step(&pid2, z2, stp2);
        z2 = Object_Step(&obj2, command2);

        // Log the current time and control loop values to the file
        fprintf(file, "%f %f %f %f %f %f %f\n", t, command1, z1, stp1, command2, z2, stp2);

        // Increment the time and iteration counter
        t = t + TIME_STEP;
        i = i + 1;
    }

    // Close the file and exit the program
    fclose(file);
    exit(0);
}
