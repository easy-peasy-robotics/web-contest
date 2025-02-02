EPR Web Contest Sandbox
=======================

## ℹ Intro
Dear participants, after days of hard work 💪🏻 you reached the final frontier 🌌

To complete the crash course, we would like you to develop a demo to run both on the simulated and
physical robot that requires reusing all the components you have studied thus far, including CAD design,
YARP, Motor Control, and Computer Vision.

You will be working in teams that will compete to win the first prize for the most awesome
and effective demo ✨

## 🎯 Objectives
1. Your team will be given specifications for CAD designing a particular object.
1. Put the mesh of the object in the location [`./gazebo/models/object/.`](./gazebo/models/object), overwriting the existing `mesh.stl`.
1. Adjust the dimensions by fiddling with the parameters in the corresponding [`./gazebo/models/object/model.sdf`](./gazebo/models/object/model.sdf).
1. Your object will be part of a set of 4 objects in total that will be randomly presented to the robot as shown in the figure below.
1. Using your developers' abilities, analyze and expand the code in [`./src/main.cpp`](./src/main.cpp) to let the iCub explore the world
   and detect where your specific object is located, that is whether in **position 1, 2, 3 or 4**.
1. Try to be neat and informative and document within the repository the decisions taken by the team, the progress and the overall solution.

<p align="center">
    <img src="./assets/objects-positions.png">
</p>

## ⚙ Testing your solution
You can test your code by launching the applications available in [`./app/scripts`](./app/scripts), or even better by
launching the script `./test.sh` in the [`./smoke-test`](./smoke-test) directory.

Bear in mind the following important aspects:
- The entry point to your code needs to be the request **`go`** sent to the RPC port `/service`, to which
  you are required to return the position of your object in the allowed set {1, 2, 3, 4}.
- The smoke-test will interrogate your module multiple times in a row by changing the scenario and you will have to
  respond to each request within **240 seconds** to prevent timeout expiration.
  
Moreover, to address the motor control part, you can make use of standard joint interfaces (position, velocity...), although
our recommendation is that you exploit an operational space approach for controlling the gaze as discussed [here](https://robotology.github.io/robotology-documentation/doc/html/icub_gaze_interface.html) 🌐

In particular, the contest code already contains boilerplate snippets to operate with the corresponding [IGazeControl interface](http://yarp.it/classyarp_1_1dev_1_1IGazeControl.html).

### 📐 Objects
- Do not rely on the appearance of fallback objects, as the physical setting where you are going to put
  to test your object as well as the codebase will be different from the sandbox.
- If your object is similar to one particular fallback object, you may consider replacing
  the latter interfering with yours with copies of other more neutral fallback objects. However, object
  names in the simulation shall remain unchanged.
- Remember that you can rotate your object in Gazebo in order to make it visually appear in a given suitable configuration.
  Fiddle only with the [orientation part](./gazebo/models/object/model.sdf#L19).

### ☁ Instructions to run the sandbox on the web
1. This is a Gitpod-enabled sandbox, so to get started click on the badge below (or type the URL `https://gitpod.io/#https://github.com/easy-peasy-robotics/your-sandbox`):

    [![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/from-referrer)

1. Once the sandbox workspace is ready, build and install the project:
    ```console
    cd /workspace/web-contest-sandbox 
    mkdir build && cd build
    cmake ../
    make install
    ```
1. From within Gitpod, open up the browser at the port `6080` to get to the workspace desktop GUI.
1. In the desktop GUI, open a terminal and run the grasping experiment:
   ```console
   cd /workspace/web-contest-sandbox/smoke-test
   ./test.sh
   ```

## 👥 Tips for team working
Don't jump to the code at once! Rather, take your time to plan activities accurately. This stage will pay off.

Optimize the resources by assigning teammates specific tasks so that some will design the object's mesh,
some other will deal with the motor control and others will be responsible for vision recognition.

Coordinating several developers working remotely certainly introduces overhead. To this end:
- Leverage on GitHub tools (issues, PR...).
- Foresee intermediate milestones and frequent meetups to get everyone aligned on the same page,
  and to gauge progress and handle unexpected delays.

Also, you may find the option for [sharing Gitpod workspaces](https://www.gitpod.io/docs/sharing-and-collaboration/#collaboration--sharing-of-workspaces)
quite useful in certain circumstances.

## 🏆 Evaluation Criteria
In descending priority order:
- How the 🤖 behaves overall (e.g. efficiency, stability, robustness, predictability).
- How the CAD design of the object complies with the original requirements.
- How the repository is organized in terms of code and documentation.
- Possibly, mentors will [review your team's code](https://help.github.com/articles/about-pull-request-reviews).
