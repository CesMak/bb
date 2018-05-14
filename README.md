Hey, I would like to share my code of the simulation of a Ballbot in Gazebo. Unfortunately the Ballbot is currently not balancing that long (10sec at best). I am not sure what the reasons are, but I would be really happy if you just test it and play around with it -maybe you can tell me why it is not balancing.

Please see the [Problems Area](https://github.com/CesMak/bb/wiki/Simulation#Problems) and feel free to write me an Email (MarkusLamprecht@live.de) if you have any improvements. 

![Ballbot Simulation](https://github.com/CesMak/bb/blob/master/img/Ballbot_SimulationRecap.gif)

[**Installation youtube video**](https://www.youtube.com/watch?v=a7RVCsyNebY&feature=youtu.be)

## Prerequisites
* Gazebo 7
* ROS - Kinetic
* Ubuntu 16.04 LTS
* `ros-kinetic-gazebo-ros`
* `ros-kinetic-gazebo-ros-control`

I only tested the simulation with Gazebo 7 and ROS - Kinetic.

## Build and first run
### Install Tool's I use
* `sudo apt-get install ros-kinetic-plotjuggler`
* `sudo apt-get install ros-kinetic-rqt-multiplot `

### Build:
Open a terminal (STR+ALT+T) and paste the following inside:
* `mkdir ballbot`
* `cd ballbot`
* `git clone git@github.com:CesMak/bb.git .`

(yes copy the dot at the end as well and paste the whole command in your terminal)

* `catkin build`

In case of building errors feel free to write me an email: MarkusLamprecht@live.de

* `source devel/setup.bash`

### Run:
Please be sure that if you open a new termial always source your setup. (Inside ballbot: source `devel/setup.bash`)

* `roslaunch ballbot_gazebo ballbot.launch `

![alt text][pic1]

[pic1]: https://github.com/CesMak/bb/blob/master/img/ballbot_gazebo.png "Gazebo+Rviz"


Starts launch file located in: _ballbot/src/ballbot/bb_simulation/launch/ballbot.launch_

In normal mode: Rviz and Gazebo are startet. Note that Gazebo is started in paused mode which means that you have to unpause it for a short time in order to see the robots in RVIZ.

If you want to start also plotjuggler or other nodes or do not load the ball etc. open the _ballbot.launch_ file and set the arguments:

| Argument      | Description   | Initial Value 
| ------------- |-------------| -------------|
| start_rviz     | Start RVIZ | True |
| start_rqt_multiplot     | Start RQT-Multiplot to plot data | False |
| start_plotjuggler     | Start PlotJuggler to plot imu data etc. | False |
| start_rqt_gui     | Start RQT-GUI to show a diagram of used nodes | False |
| use_the_ball     | Load Ball as well? | True |
| controller_type    | Choose which controller you want to use to control the ballbot.  | 2D |
| motors_controller_type  | Choose the controller type of the wheels. | EffortJointInterface |
| wheel_type_single  | If false double_wheel omni wheels are loaded with a 160mm diameter ball. If true single omni_wheels are loaded with a 140mm diameter ball and a different model. | False |

* `roslaunch bb_description bb_rviz_only.launch`

Starts launch file located in: _ballbot/src/ballbot/bb_descrpition/launch/bb_rviz_only.launch_

In normal mode: Just RVIZ is started. This is a fast method to just see the robot without wheels. Wheels are loaded only if gazebo is started as well.


## Problems:
<a name="Problems"></a>
### Balancing Problems:
There are probably many factor's which impact the balancing behaviour of a ballbot. Here is a list of these factor's and some values which I am uncertain how to choose them correctly:

1. The Friction of the ball on the surface and the subwheels on the ground. 
<a name="gazebo_params"></a>
How to choose the KP, KD, Mu1, Mu2 factors of the ball, the surface and the subwheels correctly? I guess the factors(KP,..etc.) of the other links like the wheels for instance do not matter. Here is my current setup:

![alt text][gazebo_params]

[gazebo_params]: https://github.com/CesMak/bb/blob/master/img/Gazebo_Simulation_Params.png "Choose Gazebo Prams"

What I currently found out about these params:

| Gazebo-Refrence-URDF-Value | Description   | Unit | Example |
| ------------- |-------------| -------------| --|
| mu1    | Is the Coulomb friction coefficient for the first friction direction | [dimensionless scalar](https://socratic.org/questions/explain-why-the-coefficient-of-friction-has-no-units) | 0.6 (wood)
| mu2    | Is the Coulomb friction coefficient for the second friction direction | [dimensionless scalar](https://socratic.org/questions/explain-why-the-coefficient-of-friction-has-no-units) | 0.6 (wood)
| kp    |Contact stiffness for rigid body contacts as defined by ODE. ( [ODE uses erp and cfm](http://www.ode.org/ode-latest-userguide.html#sec_7_3_7), (Is this still [true](http://answers.gazebosim.org/question/13807/is-soft_cfm-and-soft_erp-utilized-by-ode/)?), which is then mapped to kp and kd) | - |1+e5...1+e13|
| kd    |Contact damping for rigid body contacts as defined by ODE. | - | 1...100|

Note in my project I use [urdf](http://wiki.ros.org/urdf/XML) for joints and links and for additional gazebo parameters I use the gazebo-reference tag (see for example the _/urdf/bb_double_wheel.gazebo.xacro_ file). Everything within the gazebo-reference tag is then parsed to sdf which is used by gazebo. There are [short forms](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) like the mu1 tag or kp tag which is parsed to [sdf](http://sdformat.org/spec?ver=1.5&elem=collision#ode_fdir1). An example snipped may look like:

>         <surface>
>           <friction>
>               <friction>
>                 <ode>
>                   <mu> 0.6 </mu>
>                   <mu2> 0.6 </mu2>
>                   (... fdir1, slip1, slip2)
>                </ode>
>              </friction>
>           </friction>
>         </surface>

Additionally it says [here](http://gazebosim.org/tutorials?tut=friction):
* ODE will automatically compute the first and second friction directions for us.

* The two objects in collision each specify mu and mu2. Gazebo will choose the smallest mu and mu2 from the two colliding objects. -> This means I have to set the mu1 and mu2 value of the ball and the surface and the subwheels to the same value(mu1=mu2=1+e3)

* The valid range of values for mu and mu2 is any non-negative number, where 0 equates to a friction-less contact and a large value approximates a surface with infinite friction. Tables of friction coefficient values for a variety of materials can be found in engineering handbooks or [online references](https://www.engineeringtoolbox.com/friction-coefficients-d_778.html).

* If you have a ball on a plate then I guess mu1 and mu2 should be chosen equally. 

2. The Inertia's of the Robot
3. The center of gravity 
4. The Controller Tuning Factors K

### Problems with RVIZ:

1. The laserscan data point's are not displayed, although the /ballbot/sensor/scan_raw which publishes the laserscan messages seems to work well.
2. How to display the Depth Information of the Realsense Camera in RVIZ? 
3. If you just `roslaunch bb_description bb_rviz_only.launch` the wheels are not loaded in RVIZ is this due to the fact that there exist's no tf of them unless I also load the transmission link's? BTW start gazebo? 
4. The Ball in RVIZ seems to jump. Do not know if this is because of the tf update rates of the ground truth? Moreover if the ball fells in gazebo it does not fell in rviz to the ground - any ideas why?



