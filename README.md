The code shows the implementation of a PID controller in C. The PID controller includes:

- Filtered derivative for the derivative term
- Saturation of the command
- Rate limiter of the command
- Anti-windup technique: back-calculation

and it is implemented using a structure, to make the code reusable.

Code created to support a Linkedin post. Follow me on Linkedin! https://www.linkedin.com/in/simone-bertoni-control-eng/

Link to the original post: https://www.linkedin.com/posts/simone-bertoni-control-eng_pid-c-activity-7033344503226753024-wJZp?utm_source=share&utm_medium=member_desktop

---------------------------------------------------------

Don't be a slave to model-based code generation.

You can generate great code too.

And you know exactly what it does.

This PID controller is implemented in C, based on a structure, so it can be reused multiple times.

The architecture of the program consists in:

✅ The model of an object of mass m = 10 kg, sliding on a surface where it's subject to viscous friction of coefficient k = 0.5 N*s/m, pushed by a force F

✅ A PID controller

You can use this framework to test pieces of code written in C, running them in a simulation environment also written in C.

The function below PID_Step is a step of a PID controller that is supposed to run on a microcontroller, in a task scheduled every 100 ms, for example.

The PID_Step function includes (code on the 4th slide):

✅ Filtered derivative for the derivative term

✅ Saturation of the command

✅ Rate limiter of the command

✅ Anti-windup technique: back-calculation

To test the function in a simulation environment, we need a model of the plant. In this case, the model is the object described above, with the following differential equation:

d^2z/dt^2 * m = F - k * dz/dt

where

✅ z: position of the object in meters

✅ m: mass of the object, 10 kg

✅ k: coefficient of viscous friction, 0.5 N*s/m

✅ F: force in N, limited to [-100, 100] N

The model of the object is implemented using Euler forward in the code on the 5th slide, where we use v = dz/dt from the previous step to calculate the new value of d^2z/dt^2 and then we integrate it twice to find the new value of v = dz/dt and z.

The main function (6th slide) implements the simulation, based on two slightly different PIDs that control the position of two identical objects:

✅ Creates the structures that represent the two PIDs and the two objects

✅ Iterates 1200 times = 120 seconds (every step represents 0.1 seconds in this example) using a while loop

✅ Generates the two different setpoints: 100 m and 50 m for the first 60 seconds, then changed to 200 m and 150 m

✅ Runs PID_Step using the value of z from the previous step (for the two different PIDs)

✅ Runs Object_Step using the value of the command defined by PID_Step

The result of the simulation is shown on the 7th slide.

This approach (using structures) allows you to have as many different PIDs as you need (creating one structure per PID).

The link to the code is in the comments.

If you enjoyed this follow me for more tips on control and embedded software engineering.

Hit the 🔔 on my profile to get a notification for all my new posts.

Feel free to ask anything in the comments, I'll do my best to answer.

#controlsystems #embeddedsystems #softwareengineering #embeddedsoftware #coding #controltheory #pid
