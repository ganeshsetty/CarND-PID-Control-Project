# PID control project

The objective of this project is implementing a PID (Proportional-Integral-Derivative) controller to keep the car driving on a race track. The control input to car is steering value provided by PID output. The PID controller takes the error(difference between the desired and true state) called cross track error(cte) as input and its value, integral and derivative are multiplied by factors Kp,Ki and Kd called proportional ,Integral and derivative gains respectively. The P term  generates a control command -(Kp) x (cte) i.e proportional to 'cte' to reduce the error. The Kp value of 0.25 is wisely choosen as per sebastian lecture. But when Kp of high value say 1.5 the car doesn't settle around the desired path but oscillates (particularly observed during trurns) and the car's behaviour is shown in below video link, even though keeping other optimal values Ki of 0.0005 and Kd of 3.5.
https://youtu.be/wmFUkKIb9Ig

To reduce the wavering during turns, a derivative term is -(Kd) x (d(cte)/dt) which brings back the car to desired state by suppressing the oscillations. The below video shows the behaviour
https://youtu.be/eOvKEAk7uHM

The Ki term called Integral accumulates the error and drives in the direction against the accumulated error (i.e reducing them). This is particularly helpful when there is systematic bias / drift is involved. This is usually kept at very low value 0.0005 as there is no systematic bias in our case. In the case of high Ki say 0.5, at start itself it counter acts to nullify the drift which is not actually present in our case and drives the car out of track which is demoed in the below video link.
https://youtu.be/X5BIY4I6Sn8

## Twiddle implementation
Twiddle algorithm is used for automatic tuning of PID parameters Kp,Ki and Kd. I have used initilal value of Kp = 0.25, Kd = 4.5 and Ki =0.0005 and 'dp' the potential changes as (0.1,0.001,1.0) for Kp,Ki and Kd respectively. A tolerance value of 0.01 is choosen to give enough iterations for tuning so that optimal PID parameters values are arrived. The 'dp' can be choosen as (1,1,1) as mentioned in sebastian lecture but higher values while used in twiddle algo while tuning will make car go out of track within few iterations and one has to restart the simulator again and again. So the potential changes are kept proportional by keeping in mind its impact on car behavior. For example dp for Kp if choosen as 1, during tuning, the Kp value is increased say 0.25 + 1.0 =1.25 and this high gain value makes the car go out of track.Applied same logic for other parameters 'dp' also.

The initial sum of dp is 1+0.1+0.001 = 1.101 and twiddle algo iterartions are run till this value goes below the tolerance threshold value of 0.01. Started with initial value of Kp,Ki,Kd as (0.25,0.0005,4.5) ,after tuning the final parameters  values of Kp,Ki,Kd are converged to (0.250786,0.00150786,3.44084). The video below captured during the initial iterations of twiddle tuning algo  and after approx 30 minutes,  parameters get converged to above values  and the car in simulator runs more stable and keeps well within the track at avearge speed of 40mph. Also the throttle is adjusted as a factor of steering angle.If steering angle is high, the speed redueces and vice versa.
https://youtu.be/BjdA1K5QAZE

I would like to thank the forum members who provided valuable insights in  discussions which helped in driving the work in right direction.




# Setup of enviorenment

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
