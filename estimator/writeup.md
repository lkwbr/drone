## Project: Building an Estimator

### Implement Estimator

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data 

_The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements._

The standard deviation was computed from the two generated files from the Estimator Visual Studio code: Graph1.txt and Graph2.txt. 

Here's the following Python snippet I used to do this, which relies on the `numpy` package:

```python
import numpy as np

wd = r'C:\Users\lukedottec\Documents\Education\Udacity\Flying Car and Autonmous Vehicles Nanodegree\FCND-Estimation-CPP\config\log'

# Grab data
gps_x = np.loadtxt('{}\Graph1.txt'.format(wd), delimiter=',', dtype='Float64', skiprows=1)[:,1]
acc_x = np.loadtxt('{}\Graph2.txt'.format(wd), delimiter=',', dtype='Float64', skiprows=1)[:,1]

# Compute standard deviation
gps_x_std = np.std(gps_x)
acc_x_std = np.std(acc_x)
```

#### 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.Implement altitude controller in C++ 

_The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme._

This code exists in the `QuadEstimatorEKF::UpdateFromIMU` function. What we're doing here is converting local frame angles into Euler angle derivatives, and approximating the integral by multiplying `dt` by those same derivatives.

#### 3. Implement all of the elements of the prediction step for the estimator 

_The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation._

The prediction is accomplished in the `QuadEstimatorEKF::Predict`, `QuadEstimatorEKF::PredictState`, and `QuadEstimatorEKF::GetRbgPrime` functions. We do a couple of things to implement the prediction step for the estimator: predict the state based on acceleration measurements and update the state covariance matrix to implement the Extended Kalman Filter (EKF) state. For the latter, we used equations provided to us in the Estimation for Quadrotors whitepaper.

#### 4. Implement the magnetometer update 

_The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way)._

This code for updating the state based on magnetometer readings is contained in the `QuadEstimatorEKF::UpdateFromMag` method.

#### 5. Implement the GPS update 

_The estimator should correctly incorporate the GPS information to update the current state estimate._

The code for updating state for GPS readings lies within `QuadEstimatorEKF::UpdateFromGPS`. We've now closed the loop upon removal of the "ideal" estimator.

### Flight Evaulation

#### 1. Meet the performance criteria of each step 

_For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements._

* Scenario 6: Pass
* Scenario 7: Pass
* Scenario 8: N/A
* Scenario 9: N/A
* Scenario 10: Fail/Pass (didn't have enough time to get the first condition to pass!)
* Scenario 11: Pass

#### 2. De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors 

_The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight)._

No detuning seemed to be required for my past controller code. Is this weird? Good news?
