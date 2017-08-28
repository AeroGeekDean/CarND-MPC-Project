# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## My Results


Here's a composit video of my results from this project.

#### 55 mph with 100ms delay in the MPC process

Click image below for YouTube video.

[![55 mph with 100ms delay](http://img.youtube.com/vi/hsZVZ6lffzY/0.jpg)](https://www.youtube.com/watch?v=hsZVZ6lffzY)

The video contains 2 laps around the track with no added latency for baseline performance. It's then followed by 3 laps with 100ms of time delay added to the each frame of the MPC process.

The soundtrack is from an [arcade games](https://en.wikipedia.org/wiki/Spy_Hunter) of my childhood!

## Coordinate / Axis System Definitions

From working with flight simulations in the past, it's usually best to have a clear understanding of the various coodinate systems involved. For this project, there are 2 coordinates used: Map coordinate, and vehicle body coordinate (hence forward, 'body coodinate' for short).

### Map Coordinates

Note - The coordinate follows the "Right hand rule", whereby a positive (+) rotation of `psi` follows the x->y 'curling' direction (of the right hand), which means `z` points upward along the right thumb. Althought we will not be using `z` for this project.
```
//          ^ Y (north)
//          |
//         90
//          |  psi
// ---180---+---0/360---> X (east)
//          |
//         270
//          |
//          |

```

### Body Coordinates

Note - The positive (+) rotation of `psi` does NOT follow the `x->y` curling! Thus we'll need to be diligent in keeping track of the signs (+/-) throught out the development of motion equations.

```
//             ^ X (front)
//             |
//           360/0  psi
// (left)      |
// Y <---270---+---90---
//             |
//            1800
//             |
//             |
```

## C++ Code Structure

The classroom quiz and the starter code provided had many global functions & multiple classes squeezed into few files. I decided to break these up into separate classes (and files) that performs different jobs, as a way to practice my Object Oriented C++ skills... that I have not touched in a while.

#### main.cpp
The main() still does the same function of providing socket communication with the json data via callback methods. Majority of the original global convenience functions have been moved to `UtilFunctions.h`.

The main() instantiates an object of the `car` class, and all car/MPC related operations are encapsulated in this class.

#### UtilFunctions.h
The basic engineering unit conversion, constant definition (pi) have been moved here as `inline` functions, in addition to a new `wrapAngle()` function, and polynomial functions (`polyeval()`, `polyfit()`). The `polyeval()` has been overloaded for use by both regular `double` type and `CppAD::` types to support CppAD's Automatic Differentiation feature. Additionally, a `polyder()` was added to compute the polynomial derivate coefficients, similar to the one from MATLAB.

Finally, a `propagate_state_map_coord()` function is added to support "Dead Reckoning" feature for handling latency (described later). This function propagates the vehicle states forward in time, within the map coordinate axis system.

#### Class Car
The `Car` class holds the state of the vehicle, and has a `map2body()` method for coordinate transformation from map axis to it's own body axis. This is used for the Waypoint coordinate transformations. The class also holds the fitted polynomial coefficients for approximating the Waypoints. From these coefficients, it figures out the navigational error parameters `cte` and `psi_error`.

The class also aggregates a MPC class object, which is the actual core Model Predicive Control algorithm.

Finally, it has an `update()` method that performs the processing required for the MPC. (If there are future other controls or estimation processings, they will be aggregated here besides MPC.)

#### Class MPC
The MPC class handles all the data pre-processing / massaging required for the core optimization algorithm. It reformulates the vehicle control task into an optimization task by defining the various independnet and dependent variables, variable boundaries, and constraint boundaries. It then calls the optimizer.

#### FG_eval.h
This class provides the `operator()` method that the optimizer calls for calculating the relationship constriants amongst all the variables (ie: the vehicle kinematic / dynamic model), and the cost function definition.

Ok, enough about the C++ code structure. Let's get into doing some MPC'ing!

## Process Timing (ie: measuring actual `dt`)
For each frame of the controller process, the actual frame time `dt` (of the previous frame) is measured.
- It is ASSUMED the previous frame `dt` is a decent predictor of the current frame `dt`, in general.

We use this to estimate the elapsed time between when the simulator's outputs (vehicle states, etc) are samples, and the vehicle control inputs are received back by the simulator. This `dt` will allow our controller process to be able to dynamically handle differences in frame duration, thus keeping the overall vehicle behavior dynamics consistent in the presence of these frame time flucturations. (Think fast vs slow computers...)
- `dt` is very critical for any control system that handles time-varying dynamics.

## Adding Control Feedback to Model
The classroom quiz and the sample starter code did NOT previously capture the vehicle control inputs (steering, acceleration/throttle) from the previous frame. The example MPC than makes it's control output determination WITHOUT any knowlege its prior outputs. Thus allowing for potential wide varying fluctuations from frame to frame.

The simulator does sends back the previous control commands, and I captured these.

However, I discovered that... 

### Discovery: Simulator feedback data has a 2-frames delay!
Initially a 1-frame delay was expected. MPC process send out a signal, and expects that signal to show up as feedback input on the next frame.

However, realizing that the simulator and the MPC are 2 time-stepping processes running asynchronously, without any handshaking between the two. Given that, data from one process is likely not immediately sampled & processed by the other process until their next frame. Thus a 2-frame round-trip delay is very reasonable.

This initial surprise turned out to be a good mental refresher on asynchronous processes and inter-process communications.

## Dead Reckoning State Inputs to Account for Actual Latency
Now that we have the latest control inputs the simulator used to propagate / move the vehicle, AND we have a good estimate of the controller process's frame delay `dt`, we could could now estimate / forward propagate / dead reckon the vehicle to its new state... when it will receive our new control inputs.

Thus we do this pre-processing before handing things over to the MPC algorithm.

And since we're measuring `dt` directly, the artificial 100 ms frame delay will be automatically accounted for by this method.

## Coordinate Transformation from Map to Vehicle Body Axis
Once the vehicle is propagated forward, we then tranform the waypoints from map to body axis via `Car::map2body()`.

### Polynomial Fitting of Waypoints
We then apply a 3rd-order polynomial to fit the waypoint data. The simulator continuously send the next six (6) waypoints ahead of the vehicle. Thus sufficient data for a 3rd-order polynominal.

## Model
The "Model" resides within the `FG_eval::operator()` method's constraint equation of the MPC.

Our model equation is very similar to the classroom quiz (simple bicycle model), but adjusted for the body axis system. Thus note some differences in some '+' vs '-' for the `y` and `psi` axis.

```c++
// Setup the rest of the model constraints
// NOTE - *** This is in vehicle body axis!!! ***
// (+) x      = fwd
// (+) y      = left
// (+) psi    = right of nose
// (+) v      = fwd
// (+) delta  = right turn
// (+) cte    = car is right of Ref Trajectory
// (+) epsi   = Ref heading is right of nose

x1    = x0 + v0 * cos(psi0) * dt_model;
y1    = y0 - v0 * sin(psi0) * dt_model;
psi1  = psi0 + v0 * (delta0/Lf) * dt_model;
v1    = v0 + a0 * dt_model;
cte1  = y1_desired - y1;
epsi1 = psi_desired - psi1;
```

## Lookahead Horizon, Model Timestep length and duration
The MPC works by forward predicting / "imaging" where the vehicle will be with the given control inputs, and then varying the inputs to make it "optimal". It predicts forward in time to the `look ahead` horizon, by taking `N` time steps of duration `dt_model`.

Thus the relationship:
```
look_ahead = N * dt_model
```

`N` directly affect the computational intensivelyness, as that's the number for forward-in-time steps the MPC will need to predict.

`dt_model` affects the granuarity of each of these steps. If it's too small, then the `look_ahead` time horizon is shortened and so does MPC's predictive capability. However, if `dt_model` is too large, then we may not capture sharper dynamics, leading to "discretization error".

Thus it's an engineering trade off.

I set `lookahead = 1.5` seconds,

`dt_model = 0.1` seconds, resulting in

`N = (int)(lookahead/dt_model) = 15`

## Cost Functions and their weights

Below are the cost functions that was applied:

```c++
// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
  fg[0] += CppAD::pow(vars[cte_start + t], 2) *3; // <--- Don't fixate over zeroing out CTE. It'll cause oscillation.
  fg[0] += CppAD::pow(vars[epsi_start + t], 2) *100; //<--- More important to nail the heading, to help guide down path
  fg[0] += CppAD::pow(vars[v_start + t] - vref, 2); // <--- Speed is low frequency dynamic, thus low priority gain
}

// Minimize the use of actuators. <--- Don't like these. Penalizes for steady turn input = push car to outside of turn.
//for (int t = 0; t < N - 1; t++) {
//  fg[0] += CppAD::pow(vars[delta_start + t], 2)*0.01;
//  fg[0] += CppAD::pow(vars[a_start + t], 2);
//}

// Minimize the actuation rates.
for (int t = 1; t < N - 1; t++) {
  // use actual rates, not just differences of 'consecutive' elements   ---> vvvv
  AD<double> delta_rate = (vars[delta_start + t] - vars[delta_start + t - 1])/dt_model;
  AD<double> accel_rate = (vars[a_start + t] - vars[a_start + t - 1])/dt_model;
  fg[0] += CppAD::pow(delta_rate, 2) *0.1;  // reduce to previous nominal gain order of magnitude
  fg[0] += CppAD::pow(accel_rate, 2) *0.05; // reduce to previous nominal gain order of magnitude
}

// Also penalize deviation from previous frame's control inputs.  <--- new
fg[0] += CppAD::pow( (vars[delta_start] - delta0)/dt_model, 2) *0.1;
fg[0] += CppAD::pow( (vars[a_start] - accel0)/dt_model, 2) *0.05;

```

## End of My Results

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
