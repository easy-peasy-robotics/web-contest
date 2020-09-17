EPR Web Contest Sandbox
=======================

## ☁ Instructions to run the sandbox on the web
We make use of the [Gitpod Cloud IDE](https://gitpod.io) as infrastructure. Find out more on [YARP-enabled Gitpod workspaces][1].

1. To get started with the sandbox, click on the following badge:

    [![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)][2]

2. Once the sandbox workspace is ready, build and install the project:
    ```sh
    $ cd /workspace/web-contest-sandbox 
    $ mkdir build && cd build
    $ cmake ../
    $ make install
    ```
3. From within Gitpod, open up the browser at the port `6080` to get to the workspace desktop GUI.
4. In the desktop GUI, open a terminal and run the grasping experiment:
   ```sh
   $ gazebo web-contest-sandbox.sdf
   ```

[1]: https://spectrum.chat/icub/technicalities/yarp-enabled-gitpod-workspaces-available~73ab5ee9-830e-4b7f-9e99-195295bb5e34
[2]: https://gitpod.io/#https://github.com/easy-peasy-robotics/web-contest-sandbox
