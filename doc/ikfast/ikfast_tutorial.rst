IKFast Kinematics Solver
========================

.. image:: openrave-UR3e-urde-preview-docker-TurboVNC.png
   :width: 700px

In this section, we will walk through configuring an IKFast plugin for MoveIt (altered for hubraum purposes).

What is IKFast?
---------------

IKFast, the Robot Kinematics Compiler, is a powerful inverse kinematics solver provided within Rosen Diankov's `OpenRAVE <http://openrave.org>`_ motion planning software. IKFast automatically analyses any complex kinematic chain for common patterns that allow for an analytic solution and generates C++ code to find them.
As a consequence, IKFast provides extremely stable solutions that can be found in a few microseconds on recent processors.

MoveIt IKFast
---------------

MoveIt provides tools to generate an IKFast kinematics plugin for MoveIt using the OpenRAVE generated cpp files.
This tutorial will step you through setting up your robot to utilize the power of IKFast.
MoveIt IKFast is tested on ROS Melodic with a 6DOF and 7DOF robot arm manipulator.
While it works in theory, MoveIt IKFast doesn't currently support >7 degree of freedom arms.

Getting Started with Docker
-----------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should have a MoveIt configuration package for your robot that was created by using the `Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_.

OpenRAVE is a planning framework as complex as MoveIt itself and installing it is tricky -- particularly because its public documentation is not maintained anymore.
Fortunately, personalrobotics provide a `docker image <https://hub.docker.com/r/personalrobotics/ros-openrave>`_ based on Ubuntu 14.04 with OpenRAVE 0.9.0 and ROS Indigo installed, which can be used to generate the solver code once.

So the easiest way to run the IKFast code generator is through this docker image.
For manual building instructions (tailored towards Ubuntu 16.04), please see the `Kinetic version of this tutorial <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html>`_.
To follow the recommended, docker-based approach, ensure you have docker installed and started: ::

 sudo apt-get install docker.io
 sudo service docker start

The following command will ensure that you can run docker with your user account (adding $USER to the docker group): ::

 sudo usermod -a -G docker $USER

You need to log off/log on in order to actually activate this permission change.

  Note: To run docker from `WSL2 <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_, you need to install the `Docker Desktop WSL 2 backend <https://docs.docker.com/docker-for-windows/wsl>`_.

[hubraum note]
  Note:  Be aware, that X11 servers do not allow 3D accelerated graphics forwarding. Thus in later part of this tutorial look for TurboVNC (or VirtualGL) installation tutorial in order to preview openrave generated model of your device. However it is only neccessary if you want to play more with GUI applications assosiated with MoveIT and IKFast environment (like RViz). For sole generation of dedicated kinematics, you might omit this part.
 
Getting Started
-----------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should have MoveIt configuration package for your robot that was created by using the `Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_

Installing OpenRAVE on Ubuntu 16.04 is tricky. Here are 2 blog posts that give slightly different recipes for installing OpenRAVE.

 * `Stéphane Caron's Installing OpenRAVE on Ubuntu 16.04 <https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html>`_
 * `Francisco Suárez-Ruiz's Robotics Workstation Setup in Ubuntu 16.04 <https://fsuarez6.github.io/blog/workstation-setup-xenial>`_

Make sure you have these programs installed: ::

 sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy qt4-dev-tools

You may also need the following libraries: ::

 sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev

To enable the OpenRAVE viewer you may also need to install OpenSceneGraph-3.4 from source: ::

 sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
 git clone https://github.com/openscenegraph/OpenSceneGraph.git --branch OpenSceneGraph-3.4
 cd OpenSceneGraph
 mkdir build; cd build
 cmake .. -DDESIRED_QT_VERSION=4
 make -j$(nproc)
 sudo make install

For IkFast to work correctly, you *must* have the correct version of sympy installed: ::

 pip install --upgrade --user sympy==0.7.1

You should *not* have mpmath installed: ::

 sudo apt remove python-mpmath

MoveIt IKFast Installation
---------------------------
Install the MoveIt IKFast package either from Debian packages or from source.

**Binary Install**: ::

 sudo apt-get install ros-${ROS_DISTRO}-moveit-kinematics

**Source**

Inside your catkin workspace's ``./src`` directory: ::

 git clone https://github.com/ros-planning/moveit.git
 rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
 catkin build

OpenRAVE Installation
----------------------

**Binary Install (only Indigo / Ubuntu 14.04)**: ::

 sudo apt-get install ros-indigo-openrave

Note: you have to set: ::

 export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`

**Source Install**: ::

 git clone --branch latest_stable https://github.com/rdiankov/openrave.git
 cd openrave && mkdir build && cd build
 cmake -DODE_USE_MULTITHREAD=ON -DOSG_DIR=/usr/local/lib64/ ..
 make -j$(nproc)
 sudo make install

Working commit numbers 5cfc7444... confirmed for Ubuntu 14.04 and 9c79ea26... confirmed for Ubuntu 16.04, according to Stéphane Caron.

**Please report your results with this on** `this GitHub repository. <https://github.com/ros-planning/moveit_tutorials>`_

Create Collada File For Use With OpenRAVE
-----------------------------------------

Parameters
^^^^^^^^^^

 * *MYROBOT_NAME* - name of robot as in your URDF
 * *PLANNING_GROUP* - name of the planning group you would like to use this solver for, as referenced in your SRDF and kinematics.yaml
 * *MOVEIT_IK_PLUGIN_PKG* - name of the new package you just created
 * *IKFAST_OUTPUT_PATH* - file path to the location of your generated IKFast output.cpp file

To make using this tutorial copy/paste friendly, set a MYROBOT_NAME environment variable with the name of your robot: ::

  export MYROBOT_NAME="ur3e"
  export MYROBOT_CONFIG_OUTPUT_PATH="root/$MYROBOT_NAME_config/"

Getting UR calibration 
----------------------

Package for extracting the factory calibration from a UR robot and changing it to be used by ``ur_e_description`` to gain a correct URDF model.

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary, to control the robot using `Universal_Robots_ROS_Driver <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>`_ driver, it is highly recommended to do so, as end effector positions might be off in the magnitude of centimeters. Make sure to have the driver installed before your proceed with calibration correction.

** calibration_correction **

This node extracts calibration information directly from a robot, calculates the URDF correction and saves it into a ``.yaml`` file.

In the launch folder of the ``ur_calibration`` package is a helper script: ::

   roslaunch ur_calibration calibration_correction.launch \
   robot_ip:=<robot_ip> target_filename:="${MYROBOT_CONFIG_OUTPUT_PATH}/my_robot_calibration.yaml"


For the parameter ``robot_ip`` insert the IP address on which the ROS pc can reach the robot. As ``target_filename`` provide an absolute path where the result will be saved to.

First you will need robot description file that is in `Collada or OpenRAVE <http://openrave.org/docs/latest_stable/collada_robot_extensions/>`_ robot format.

Fist however let's generate URDF model from calibrated robot. ::

   xacro ~/catkin_ws/src/fmauch_universal_robot/ur_e_description/urdf/$MYROBOT_NAME_robot.urdf.xacro kinematics_config:='$MYROBOT_CONFIG_OUTPUT_PATH/$MYROBOT_NAME_calibration.yaml' >> $MYROBOT_CONFIG_OUTPUT_PATH/$MYROBOT_NAME.urdf

Note: in case ``ur_e_descirption`` catalog was not located in ``fmauch_universal_robot`` repository, check it out from `here <https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_e_description>`_ to ``~/catkin_ws/src/fmauch_universal_robot/.`` prior to step above ::

   cd ~/catkin_ws/src/fmauch_universal_robot/
   git clone https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_e_description.git

    
Once you have your robot in URDF format, you can convert it to Collada (.dae) file using the following command: ::

 rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae

Often floating point issues arise in converting a URDF file to Collada file, so a script has been created to round all the numbers down to x decimal places in your .dae file. Its probably best if you skip this step initially and see if IKFast can generate a solution with your default values, but if the generator takes longer than, say, an hour, try the following: ::

    export IKFAST_PRECISION="5"
    cp "$MYROBOT_NAME".dae "$MYROBOT_NAME".backup.dae  # create a backup of your full precision dae.
    rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"

From experience we recommend 5 decimal places, but if the OpenRAVE IKFast generator takes to long to find a solution, lowering the number of decimal places should help.

To see the links in your newly generated Collada file

You may need to install package **libsoqt4-dev** to have the display working: ::

 openrave-robot.py "$MYROBOT_NAME".dae --info links

This is useful if you have a 7-dof arm and you need to fill in a --freeindex parameter, discussed later.

To test your newly generated Collada file in OpenRAVE: ::

 openrave "$MYROBOT_NAME".dae

You should see your robot.

.. image:: openrave-UR3e-urde-preview-docker-TurboVNC.png
   :width: 500px
   
   Note: In order to display that via X11 server aka. VNC install server and client supporting 3D graphics rendering acceleration `How to setup VirtualGL and TurboVNC on Ubuntu <https://github.com/hub-raum/VNC-with-3D-Acceleration/blob/master/How%20to%20setup%20VirtualGL%20and%20TurboVNC%20on%20Ubuntu.md>`_

Create IKFast Solution CPP File
-------------------------------
Once you have a numerically rounded Collada file its time to generate the C++ .h header file that contains the analytical IK solution for your robot.

Select IK Type
^^^^^^^^^^^^^^
You need to choose which type of IK you want to solve for. See `this page <http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types>`_ for more info.
The most common IK type is *transform6d*.

Choose Planning Group
^^^^^^^^^^^^^^^^^^^^^
If your robot has more than one arm or "planning group" that you want to generate an IKFast solution for, choose one to generate first. The following instructions will assume you have chosen one <planning_group_name> that you will create a plugin for. Once you have verified that the plugin works, repeat the following instructions for any other planning groups you have. For example, you might have 2 planning groups: ::

 <planning_group_name> = "left_arm"
 <planning_group_name> = "right_arm"

To make it easy to use copy/paste for the rest of this tutorial. Set a PLANNING_GROUP environment variable. eg: ::

 export PLANNING_GROUP="ur3e"

Identify Link Numbers
^^^^^^^^^^^^^^^^^^^^^
You also need the link index numbers for the *base_link* and *end_link* between which the IK will be calculated. You can count the number of links by viewing a list of links in your model: ::

 openrave-robot.py "$MYROBOT_NAME".dae --info links

A typical 6-DOF manipulator should have 6 arm links + a dummy base_link as required by ROS specifications.  If no extra links are present in the model, this gives: *baselink=0* and *eelink=6*.  Often, an additional tool_link will be provided to position the grasp/tool frame, giving *eelink=7*.

The manipulator below also has another dummy mounting_link, giving *baselink=1* and *eelink=8*.

==========  ======  ===========
name        index   parents
==========  ======  ===========
link0       0
link1       1       link0
link2       2       link1
link3       3       link2
link4       4       link3
link5       5       link4
link6       6       link5
link7       7       link6
link8       8       link7
==========  ======  ===========

Generate IK Solver (using ikfast’s inversekinematics database)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Many thanks to @`gvdhoorn <https://answers.ros.org/users/5184/gvdhoorn/>`_ for pointing out this method in his posts on ROS answer [1, 2, 3]. His post in [3] gives most of the details to generate the ik solver. The greatest thing for using this method is that it will let ikfast to decide which joint to be set as free joint and experiments has proved that it worked for the 6+1 dof case.

I’ve tested using the inversekinematics database without using the ``personalrobotics/ros-openrave`` Docker image. `OpenRave Installation <http://docs.ros.org/en/kinetic/api/framefab_irb6600_support/html/doc/ikfast_tutorial.html#openraveinstallation>`_ in this tutorial works too.

First, create a xml wrapper for the collada file (``"$MYROBOT_NAME".dae``): ::

   <robot file="$NAME_OF_YOUR_COLLADA_FILE">
        <Manipulator name="NAME_OF_THE_ROBOT_IN_URDF">
          <base>base_link</base>
          <effector>tool0</effector>
        </Manipulator>
   </robot>

And save it as ``"$MYROBOT_NAME"_collada.xml`` in the folder where you saved your collada file. Quote from gvdhoorn in `his post <https://answers.ros.org/question/263925/generating-an-ikfast-solution-for-4-dof-arm/?answer=265625#post-id-265625>`_: ::
   OpenRAVE supports relative filenames for the file attribute of the robot element in our wrapper.xml, so it's easiest if you place wrapper.xml in the same directory that contains the .dae of your robot model.

Then run: ::
   cd /path/to/your/xml_and_collada/file # in your case ``cd "$MYROBOT_CONFIG_OUTPUT_PATH"``
   openrave.py --database inversekinematics --robot=<NAME_OF_YOUR_COLLADA_FILE>.xml --iktype=transform6d --iktests=1000

The iktests parameter value was just a default, you can make it larger or smaller.

Then you can harvest your ikfast.h and ikfast.<random_ikfast_id>.cpp

**Example**

First create a xml wrapper: ::

   <robot file="irb6600_with_linear_track_workspace.dae">
        <Manipulator name="framefab_irb6600_workspace">
          <base>linear_axis_base_link</base>
          <effector>robot_tool0</effector>
        </Manipulator>
   </robot>
Save it as ``irb6600_with_linear_track_workspace.xml``.

Then run: ::

   cd /path/to/your/xml_and_collada/file
   openrave.py --database inversekinematics --robot=irb6600_with_linear_track_workspace.xml --iktype=transform6d --iktests=1000
   % it will run ik test 1000 times, you can change it to whatever number you want

After about 2 minutes (bullshit - it took me approx. an hour), you will see the following in your terminal:

   openravepy.databases.inversekinematics: generate, successfully generated c++ ik in 120.398534s, file=/home/yijiangh/.openrave/kinematics.6749b3e95c92afb4a30628f16aa823de/ikfast0x1000004a.Transform6D.0_1_3_4_5_6_f2.cpp
   openravepy.databases.inversekinematics: generate, compiling ik file to /home/yijiangh/.openrave/kinematics.6749b3e95c92afb4a30628f16aa823de/ikfast0x1000004a.Transform6D.x86_64.0_1_3_4_5_6_f2.so
   openravepy.databases.inversekinematics: save, inversekinematics generation is done, compiled shared object: /home/yijiangh/.openrave/kinematics.6749b3e95c92afb4a30628f16aa823de/ikfast0x1000004a.Transform6D.x86_64.0_1_3_4_5_6_f2.so
   openravepy.databases.inversekinematics: RunFromParser, testing the success rate of robot irb6600_with_linear_track_workspace.xml
   % ....... ikfast test failure warning at some test case No. i
   openravepy.databases.inversekinematics: testik, success rate: 0.986000, wrong solutions: 0.000000, no solutions: 0.014000, missing solution: 0.608000

Yaah! you got your ikfast.h and ikfast<id>.Transform6D.<...>.cpp saved in your $home/<username>/.openrave/<id>/ folder.

[1] https://answers.ros.org/question/285611/set-free_index-for-7-dof-robots-ikfast-moveit-plugin-generation/ [2] https://answers.ros.org/question/263925/generating-an-ikfast-solution-for-4-dof-arm/ [3] https://answers.ros.org/question/196753/generating-ikfast-plugin-for-5-dof-robot/

[below part needst to be checked]

Generate IKFast MoveIt plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To generate the IKFast MoveIt plugin, issue the following command: ::

  rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D $MYROBOT_NAME.urdf <planning_group_name> <base_link> <eef_link>

Replace the last three positional parameters with the correct ``planning_group_name`` as well as the names of the base link and the end-effector link of your robot.
The speed and success of this process will depend on the complexity of your robot. A typical 6 DOF manipulator with 3 intersecting axes at the base or wrist will take only a few minutes to generate the solver code. For a detailed explanation of the creation procedure and additional tweaks of the process, see `Tweaking the creation process`_.

The command above creates a new ROS package named ``$MYROBOT_NAME_<planning_group_name>_ikfast_plugin`` within the current folder.
Thus, you need to rebuild your workspace so the new package is detected: ::

  catkin build

Usage
-----
The IKFast plugin can be used as a drop-in replacement for the default KDL IK Solver, but with greatly increased performance. The MoveIt configuration file should be automatically edited by the generator script but in some cases this might fail. In this situation you can switch between the KDL and IKFast solvers using the *kinematics_solver* parameter in the robot's kinematics.yaml file: ::

  rosed "$MYROBOT_NAME"_moveit_config kinematics.yaml

Edit these parts: ::

 <planning_group>:
   kinematics_solver: <myrobot_name>_<planning_group>/IKFastKinematicsPlugin

Test the Plugin
^^^^^^^^^^^^^^^
Use the MoveIt RViz Motion Planning Plugin and use the interactive markers to see if correct IK Solutions are found: ::

  roslaunch "$MYROBOT_NAME"_moveit_config demo.launch

Updating the Plugin
-------------------

If any future changes occur with MoveIt or IKFast, you might need to re-generate this plugin using our scripts. To facilitate this, a bash script was automatically created in the root of your IKFast MoveIt package, named *update_ikfast_plugin.sh*. This regenerates the plugin from the OpenRAVE-generated .cpp solver file.

Tweaking the creation process
-----------------------------

The process of creating the IKFast MoveIt plugin comprises several steps, performed one-by-one by the creation script:

1. Downloading the docker image provided by `personalrobotics <https://hub.docker.com/r/personalrobotics/ros-openrave>`_
2. Converting the ROS URDF file to Collada required for OpenRAVE: ::

     rosrun collada_urdf urdf_to_collada $MYROBOT_NAME.urdf $MYROBOT_NAME.dae

   Sometimes floating point issues arise in converting a URDF file to Collada, which prevents OpenRAVE to find IK solutions.
   Using a utility script, one can easily round all numbers down to n decimal places in your .dae file.
   From experience we recommend 5 decimal places, but if the OpenRave ikfast generator takes too long to find a solution (say more than an hour), lowering the accuracy should help. For example: ::

     rosrun moveit_kinematics round_collada_numbers.py $MYROBOT_NAME.dae $MYROBOT_NAME.rounded.dae 5

3. Running the OpenRAVE IKFast tool to generate C++ solver code
4. Creating the MoveIt IKFast plugin package wrapping the generated solver

The ``auto_create_ikfast_moveit_plugin.sh`` script evaluates the file extension of the input file to determine which steps to run. To re-run the script from any intermediate step (e.g. after having tweaked the accuracy of the ``.dae`` file), simply provide the corresponding output from the previous step as input (``.dae`` or ``.cpp``) instead of the initial ``.urdf`` file.
