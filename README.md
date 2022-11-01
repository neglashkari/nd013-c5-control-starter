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

Answer the following questions:
- Add the plots to your report and explain them (describe what you see)
- What is the effect of the PID according to the plots, how each part of the PID affects the control command?
- How would you design a way to automatically tune the PID parameters?
- PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
- (Optional) What would you do to improve the PID controller?


### Tips:

- When you wil be testing your c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.
- When you will be tuning the PID parameters, try between those values:

# Results
Below some screenshots of Carla Simulator is provided showing the car is successfully tracking the desired trajectory (i.e., moving between lanes, bypassing other cars and making right turn at the intersectio):
![Capture](https://user-images.githubusercontent.com/109758200/199138425-1bd1698c-14fc-432b-99e8-08d8f7518d2a.PNG)
![Capture2](https://user-images.githubusercontent.com/109758200/199138438-4e1f2522-b1c1-477c-b9ac-1b0e7c127639.PNG)
![Capture3](https://user-images.githubusercontent.com/109758200/199138451-3f1ec2f8-8c84-4246-b225-eea80c48f851.PNG)
![Capture4](https://user-images.githubusercontent.com/109758200/199138462-181499c3-9366-4470-a722-322dd1b0cd37.PNG)
![Capture5](https://user-images.githubusercontent.com/109758200/199138405-3fd19e9f-ee0a-44e7-b561-41c9c655c234.PNG)

Performance plots are shown below which helps evaluating the efficieny of our PID controller: 
![plot_1_new](https://user-images.githubusercontent.com/109758200/199138804-367e984d-36c9-407e-86a4-a786ea53fc9e.PNG)
![plot_2_new](https://user-images.githubusercontent.com/109758200/199138818-21e7dd19-ca29-400f-8448-6e5b62c5944e.PNG)

### Add the plots to your report and explain them (describe what you see)
From the steer plots, steering error is still oscillating which indicates more PID tuning is required. Also, this can be because of the method that desired yaw is calculated, i.e., the last points of the path. To improve steering control performance, we can use the closest point of the reference path to the ego vehicle (instead of the last point of the reference path).

From the throttle plots (i.e., throttle error measured as the desired vs actual velocity, and corresponding throttle and break output), one can see it's initially oscilating, and then the throttle output settles to a near constant thanks to our PID controller. Exception is at sharp turns (e.g. iteration ~100 that vehicle turns right at intersection), where the throttle and break output vary. 

### What is the effect of the PID according to the plots, how each part of the PID affects the control command?
From the above plots, car's oscillation gets worse with the increase of proportional gain ``kp``; this oscillation can be controlled with increasing derivative gain ``kd``. Moreover, the integral gain ``ki`` helps reducing offset.

### How would you design a way to automatically tune the PID parameters?
Twiddle algorithm can be used to find optimized PID coefficients.

### PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
Pros are it does not require deriving kinematic model or dynamic model of the system in order to control the system. Its fast to calculate (high compuation) and it does not suffer from design complexity. 

Cons are PID controllers are hard to tune and they do not guarantee robustness in presence of noises and disturbances.
