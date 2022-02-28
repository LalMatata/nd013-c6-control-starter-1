# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Project Instructions

In the previous project you built a path planner for the autonomous vehicle. Now you will build the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

In the directory [/pid_controller](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller)  you will find the files [pid.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid.cpp)  and [pid.h](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid.h). This is where you will code your pid controller.
The function pid is called in [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp).

### Step 1: Build the PID controller object
Complete the TODO in the [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid_controller.h) and [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid_controller.cpp).

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.
### Step 2: PID controller for throttle:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp), complete the TODO (step 2) to compute the error for the throttle pid. The error is the speed difference between the actual speed and the desired speed.

Useful variables:
- The last point of **v_points** vector contains the velocity computed by the path planner.
- **velocity** contains the actual velocity.
- The output of the controller should be inside [-1, 1].

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 3: PID controller for steer:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp), complete the TODO (step 3) to compute the error for the steer pid. The error is the angle difference between the actual steer and the desired steer to reach the planned position.

Useful variables:
- The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
- **yaw** gives the actual rotational angle of the car.
- The output of the controller should be inside [-1.2, 1.2].
- If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 4: Evaluate the PID efficiency
The values of the error and the pid command are saved in thottle_data.txt and steer_data.txt.
Plot the saved values using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You might need to install a few additional python modules: 

```
pip3 install pandas
pip3 install matplotlib
```
### Solution

PID (proportional integral derivative) controllers use a control loop feedback mechanism to control process variables. While optimizing the PID parameter, the optimal values were found by trying first the P, then the I and then the D values one by one. In the simulation, the reactions of the vehicle were monitored.

https://user-images.githubusercontent.com/32412808/155986405-f34c2bcb-3d39-45fc-9c09-398eb2c6c809.mov


### Evaluation

A PID controller is implemented and integrated into the provided framework for throttle and steering control. Several fixes are added to the framework and simulation client to make the control smoother and more stable. PID controller (proportional–integral–derivative controller) parameters are handled separately for steering and throttle. Errors are also calculated separately. I tuned the parameters for the PID by observing the responses. When calculating the error, the yaw angle of the vehicle and the closest x and y points to the vehicle in the simulation and where the vehicle is at x and y were taken as reference.

The following figure shows the values in case of the simulation, the reference speed is calculated by the behavior planner. The x axis shows iterations. It can be seen that it takes a long time to reach the desired speed (the controller is damped) and even despite this, some oscillation is clearly visible.  It can be seen that initially the error is reduced quickly (caused by the proportional term), but then the equalibrium is reached slowly - this is caused by the slow buildup of the integral term.

![](project/pid_controller/screenshot/Figure_2.png)

The Figure below shows the steering control error and its output in a scene with obstacles. There are three parts in the timeline where the error is huge, these are the 3 moments when the car runs through obstacles and has to change lanes. Part of the error here may be due to latency or other incompatibilities between the controller and simulation - for example, the root cause may be that the controller is planning too far ahead (takes the last waypoint).

![](project/pid_controller/screenshot/Figure_3.png)

### Questions

Parameter optimization is performed using the twiddle algorithm, which is based on making iterative changes to each parameter while an improvement is seen, reducing the magnitude of the change if better values are not found.
Answer the following questions:

- What is the effect of the PID according to the plots, how each part of the PID affects the control command?
  * The proportional term moves the car towards the target.
  

- How would you design a way to automatically tune the PID parameters?
    * Parameter optimization is performed using the twiddle algorithm, which is based on making iterative changes to each parameter while an improvement is seen, reducing the magnitude of the change if better values are not found.


- PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
    * The P Controller stabilizes the gain but produces a constant steady-state error.
    * The I Controller reduces or eliminates the steady-state error.
    * The D Controller reduces the rate of change of error.
    * The only disadvantage is the tuning methodology.


- (Optional) What would you do to improve the PID controller?
  * I changed pid values.

### Tips:

- When you wil be testing your c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.
- When you will be tuning the PID parameters, try between those values:


