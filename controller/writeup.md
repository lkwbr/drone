## Project: Controller

### Implemented Controller

#### 1. Implemented body rate control in C++

_The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments._ 

We turn the moments of inertia into a length-3 vector, and then multiply by the gain parameter `kpPQR` and the difference between the commanded and actual `p`, `q`, and `r` variables.

#### 2. Implement roll pitch control in C++

_The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles._

We use the rotation matrix to account for the non-linear transformation from local accelerations to body rates, `p`, `q`, and `r`. We also ensure that we never exceed our physical limitations of the `maxTiltAngle`.

It is ensured that negative commanded acceleration is not supported in the drone.

#### 3. Implement altitude controller in C++

_The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles. Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4._

This is a PD controller that controls the altitude of the vehicle by commanding a certain thrust given the current difference in desired z-position and z-derivative.

#### 4. Implement lateral position control in C++

_The controller should use the local NE position and velocity to generate a commanded local acceleration._

This lateral-position controller is located in the `QuadControl::LateralPositionControl` function. It implementes a feed-forward PD controller.

We make sure to limit both the velocity and acceleration according to the `maxSpeedXY` and `maxAccelXY`, respectively.

#### 5. Implement yaw control in C++

_The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required)._

This is a simple P-controller, calculating the positional error in the yaw. 

Here we optimized the yaw to stay in the range `[-PI, +PI]` by adding `2*PI` if it's too low, and subtracting `2*PI` if it's too high. Worked out well in practice.

#### 6. Implement calculating the motor commands given commanded thrust and moments in C++ 
_The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments._

The function for calculating the desired thrust for each rotor is `QuadControl::GenerateMotorCommands`, where we calculate the torques given the commanded moments, and then we combine those in the same way we saw in lecture to determine what thrust each motor should produce to approach the commanded thrust `collThrustCmd` and moments `momentCmd`. Simple math.

### Flight Evaulation

#### 1. Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory

_Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used)._

* Scenario 1: Pass
* Scenario 2: Pass
* Scenario 3: Pass
* Scenario 4: Pass
* Scenario 5: Pass

Fin.