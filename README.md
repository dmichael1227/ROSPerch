# ROSPerch
Greetings human!
Please refer to the README.pdf document found [here](https://github.com/amichael1227/ROSPerch/blob/master/README.pdf) for any specific questions that may not be answered in this README.md document. If you need help, feel free to [create an issue]() and we'll be happy to help!

Thank you!

John & Andrew


**NOTE:** *This ROS Package is currently a work in progress and updates will be coming out as available.* 


## Table of Contents
<ol>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#setting-up-github-getting-the-code-and-testing-it">Setting up GitHub, Getting the Code, and Testing It</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch#auto_squarepy-and-launcher_for_auto_squarepy">auto_square.py and launcher_for_auto_square.py</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#auto_driver-and-mission_commandspy">auto_driver and mission_commands.py</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#9dof_pubpy">9dof_pub.py</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#ssh-into-pi-over-direct-ethernet-connection">SSH into Pi Over Direct Ethernet Connection</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.pdf">README.pdf w/ Additional Information</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#additional-references">Additional References</a></li>
</ol>


## Setting up GitHub, Getting the Code, and Testing It
This is how ROS was integrated with the UTAP 2020 daughterboard and a Raspberry Pi 3B running Ubuntu 20.04 with ROS Noetic. Follow the steps under the <strong>Setting Up Your Raspberry Pi and Configuring Pi for Interfacing</strong> in the README.pdf file to get this setup. Also, please note that only the commands that need to be entered into the terminal will be formatted <code>like this</code> for the entirety of this section. 
<ol>
  <li>Build the daughterboard as described in the UTAP_2020_RPi.pdf document that is available at https://github.com/jdicecco/UTAP. Additionally, build a standard SeaPerch.</li>
  <li>We then took the GPIO pin definitions, the code to spin a motor, and the code to turn on an LED from UTAP_2020.py (from the same repository as above), and merged that with the led_test package we made earlier. This code gets used later on in Step 6.
  <ol>
		<li>Plugged the code to make the motor go full throttle forwards and turn on both LEDs into the led_blinker.py code, so a true input would turn them on, and a false input would turn them off.</li>
	</ol>
	</li>
	<li>Set up GitHub on the Pi. (Be sure to alter the commands so they reflect your name, email, etc. instead of the examples.)
	<ol>
		<li>This step is adapted from wiki.paparazziuav.org/wiki/Github_manual_for_Ubuntu.</li>
		<li><code>cd ~/.ssh</code>
		<ol>
			<li>If there are any warnings or errors about the directory not existing, ignore it for now and move to the next step.</li>
		</ol>
		</li>	
		<li><code>ssh-keygen -t rsa -C "your_email@youremail.com"</code></li>
		<li>Press <code>enter</code> twice</li>
		<li>Input your desired password as prompted
		<ol>
			<li>Afterwards, ensure you are in the correct directory again</li>
			<li><code>cd ~/.ssh</code></li>
		</ol>
		<li><code>mkdir key_backup</code></li>
		<li><code>cp id_rsa* key_backup</code></li>
		<li><code>nano id_rsa.pub</code>
		<ol>
			<li>Copy the contents of the file. You may have to get creative with how you copy it, especially if SSHing into the Pi. Transferring the file to your PC with FileZilla, opening it with Notepad++, and copying it worked when we tried it.</li>
		</ol>
		</li>
		<li>Add it into your GitHub Account Settings
		<ol>
			<li>Sign in to the GitHub website, go to your Account Settings</li>
			<li>Click “SSH and GPG keys”</li>
			<li>Click “New SSH key”</li>
			<li>Paste the contents of the file you copied into the “Key” field and then click “Add SSH key”</li>
		</ol>
		</li>
		<li><code>ssh-add</code></li>
		<li><code>git config --global user.name "Your Name"</code></li>
		<li><code>git config --global user.email you@example.com</code></li>
		<li>Basically what this does is replace you having to input your GitHub username and password every time you push a change to you just having to input a shorter password. Additionally, it opens up a new means of cloning the repository, and also eliminates the need for the multi-factor authentication workaround that we had to do in the <strong>Pushing Changes to the Git from the Raspberry Pi</strong> section of the README.pdf document. This also allows you to run the git push/pull commands without any arguments when you need to get the most updated version of the repository.</li>
		</ol>
	</li>
	<li>Clone the GitHub Repository and link it to your catkin workspace.
		<ol>
			<li><code>cd ~/git</code></li>
			<li><code>git clone git@github.com:amichael1227/ROSPerch.git</code>
			<ol>
				<li>Alternatively, if you do not set up the SSH Keys:</li>
				<li><code>git clone https://github.com/amichael1227/ROSPerch.git</code></li>
			</ol>
			</li>
			<li><code>cd ROSPerch/</code></li>
			<li><code>git init</code>
			<ol>
				<li>This initializes the repository in your current directory.</li>
			</ol>
			</li>
			<li><code>ln -s ~/git/ROSPerch/rosperch ~/catkin_ws/src/</code>
			<ol>
				<li>You can check that it is linked properly by running the following commands:</li>
				<li><code>cd ~/catkin_ws/src</code></li>
				<li><code>ls</code></li>
				<li>You should see the “rosperch” directory listed, usually in a lighter blue.</li>
			</ol>
			</li>
		</ol>
	</li>
	<li>Build the package!
	<ol>
			<li><code>cd ~/catkin_ws</code></li>
			<li><code>catkin_make</code></li>
	</ol>
		</li>
		<li>Run the tester code to make sure everything works.
		<ol>
			<li>Open three separate terminals.</li>
			<li>In terminal one, run <code>roscore</code>.</li>
			<li>In terminal two, run <code>rosrun rosperch receiver.py</code>.</li>
			<li>In terminal three, run <code>rosrun rosperch controller.py</code>.</li>
			<li>If you input a <code>1</code> into the terminal running controller.py, the two side motors on the SeaPerch should start running, and the two LEDs on the daughterboard should light up.</li>
			<li>If you input a <code>0</code> into the terminal running controller.py, the two side motors on the SeaPerch should stop running, and the two LEDs on the daughterboard should light up. (If the motors were not running in the first place, nothing would happen.)</li>
			</ol>
			</li>
	<li>For controlling the ROSPerch via the handheld controller, run the following command:
	<ol>
		<li><code>rosrun rosperch joy_controller.py</code></li>
	</ol>
	</li>
	<li>For the ROSPerch to run a predetermined path, in this case a square, run the following commands in separate terminals:
	<ol>
		<li><code>roscore</code></li>
		<li><code>rosrun rosperch launcher_for_auto_square.py</code></li>
		<li><code>rosrun rosperch auto_square.py</code>
		<ol>
			<li>Please note that this code currently is fine tuned for our ROSPerch, but can be easily updated to work with any ROSPerch. The code is also easily adaptable to any pattern that is desired, so long as the distances are input, thus making this a great framework for sensor integration later on.</li>
			<li><em>Additional information on these scripts is available in the <strong><a href="https://github.com/amichael1227/ROSPerch#auto_squarepy-and-launcher_for_auto_squarepy">ROSPerch - auto_square.py and launcher_for_auto_square.py</a></strong> section below.</em></li>
		</ol>
	</ol>
	</li>
	<li>For the ROSPerch to be controlled from the command line, run the following commands, in this order, in separate terminals:
	<ol>
		<li><code>roscore</code></li>
		<li><code>rosrun rosperch mission_commands.py</code></li>
		<li><code>rosrun rosperch auto_driver.py</code>
		<ol>
			<li>Please note that this code currently is fine tuned for our ROSPerch, but can be easily updated to work with any ROSPerch. The code is also acting as a framework code that is easily adaptable to any sensor integration later on. </li>
			<li><em>Additional information on these scripts is available in the <strong><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#auto_driver-and-mission_commandspy">ROSPerch - auto_square.py and launcher_for_auto_square.py</a></strong> section below.</em></li>
		</ol>
	</ol>
	</li>	
</ol>

## *auto_square.py* and *launcher_for_auto_square.py*
The <code>auto_square.py</code> script and <code>launcher_for_auto_square.py</code> scripts are intended to be run together in ROS (after launching <code>roscore</code>). The <code>launcher_for_auto_square.py</code> script is fairly simple; it queries the user for a <code>Y</code> or <code>N</code> value in order to launch the <code>auto_square.py</code> script. A value of <code>Y</code> will begin the sequence of function calls in the <code>auto_square.py</code> script, while a value of <code>N</code> shuts off the forward propulsion motors. The <code>auto_square.py</code> script contains a function that takes in a set direction for each of the two forward propulsion motors (True = forwards, False = backwards) and a time duration for which the motors should be running. Currently, upon receiving the start signal from the <code>launcher_for_auto_square.py</code> script, the <code>auto_square.py</code> script begins a deterministic sequence of function calls to attempt to have the ROSPerch follow a predetermined path with no feedback.

## *auto_driver* and *mission_commands.py*
The <code>auto_driver.py</code> and <code>mission_commands.py</code> scripts are extensions of the <code>auto_sqaure.py</code> script and <code>launcher_for_auto_square.py</code> script. They share similar functionality, but with several key differences. The <code>mission_commands.py</code> script allows the user to send commands by typing <code>drive</code>, <code>leftturn</code>, <code>rightturn</code>, or  <code>stop</code>. The code is designed to filter out human factors when it comes to the commands, which means that capitalization and spaces between the direction and the turn for the <code>left</code> and <code>right</code> <code>turn</code> commands are taken into account. Additionally, the script is also set up to filter out invalid commands. After entering a mission command, the user is then prompted to enter a parameter: distance (in meters) if the user selected <code>drive</code> and degrees if the user selected <code>left turn</code> or <code>right turn</code>. Should the user select <code>stop</code>, a dummy parameter variable of <code>1</code> is assigned <code>stop</code>, as due to the nature of the scripts, a parameter must be assigned (<em>it is important to note that currently, the stop command does not work and thus the motors will not stop the ROSPerch</em>). The entered command and parameter is sent to the <code>auto_driver.py</code> script in a <code>Commands</code> message. After sending the command, the <code>mission_commands.py</code> script waits for a message on the <code>systemstate</code> topic to prompt the user to input another command.

Upon receiving a command, the <code>auto_driver.py</code> script parses the command and parameter values and executes them by driving the motors in the desired direction for the desired duration. This approach is extremely naive: it assumes that each motor outputs a constant velocity and that there are no changes in the external forces acting on the ROSPerch (no waves, currents, different density liquids, etc.). Stringing these commands together to form a mission also assumes that any resulting drift from the last command is negligible. The ROSPerch has no understanding of its relative position barring the information from previous commands. Upon completion of the last received command, the <code>auto_driver.py</code> script sends a message on the systemstate topic to inform the  <code>mission_commands.py</code> script that the ROSPerch is ready to receive the next command. This structure is the foundation of an automated ROS system. The <code>mission_commands.py</code> script serves as a placeholder for a central decision-making node that, in the future, could integrate sensor data in real-time decision making to determine what the next command will be. Forcing the script to wait for a ready signal ensures that commands will only be sent when the system is ready to receive them. The only downside of this structure is that in order for the scripts to run properly, <code>mission_commands.py</code> <strong>must</strong> be run <strong><em>first</em></strong> and <em>then</em> <code>auto_driver.py</code>. 

The <code>auto_driver2.py</code> and <code>mission_commands2.py</code> scripts (found in the <em>wip</em> section of the scripts directory) attempt to remedy this by introducing a set of handshake functions that establish that both nodes are ready to communicate. The <code>mission_commands2.py</code> script includes a handshake publisher thread that starts a publisher that runs continuously after start-up. This publisher publishes a <code>True</code> boolean value once per second on the <code>command_to_driver</code> topic. Upon receiving this for the first time, the <code>auto_driver2.py</code> script publishes <code>True</code> on the <code>systemstate</code> topic at 10 Hz for one second. The scripts then proceed normally after this sequence; the <code>mission_commands2.py</code> script prompts the user for command input to be sent to the <code>auto_driver2.py</code> script. These scripts do not work consistently; it appears that sometimes after receiving a message on the <code>command_to_driver</code> topic, the <code>auto_driver2.py</code> script can fail to publish the ready signal (tested by monitoring the <code>systemstate</code> topic), causing the  <code>mission_commands2.py</code> script to never query the user for input. It seems that this issue doesn’t usually arise the first time the nodes are launched- getting progressively less likely to work during successive attempts until reboot. It also seems that restarting <code>roscore</code> may help too. We hypothesize that these issues may arise due to improper shutdown of the nodes or involved topics- further investigation is required to be sure.

To launch these scripts, run the following commands in separate terminals, <em>in this order</em>:
<ol>
	<li><code>roscore</code></li>
	<li><code>rosrun rosperch mission_commands.py</code></li>
	<li><code>rosrun rosperch auto_driver.py</code></li>
</ol>

## 9dof_pub.py
The <em>9dof_pub.py</em> script reads from the FXOS8700 3-axis accelerometer/magnetometer and the FXAS21002c gyroscope, converts readings into heading, roll, yaw, and yaw tilt, and publishes heading, roll, yaw, yaw tilt, and acceleration in x, y, and z to a single topic. This code is based largely on the UTAP base code: https://github.com/jdicecco/UTAP as well as the ROS tutorials: http://wiki.ros.org/ROS/Tutorials. 

## SSH into Pi Over Direct Ethernet Connection
In order to establish an ssh connection with the Pi in situations where you may be without a wireless network, a connection between the ethernet port of a laptop and the Pi can be used. (Please note that this assumes that you have SSH enabled on your Pi.)
<ol>
	<li>Plug one end of an ethernet cord into the Pi, and the other into the laptop.</li>
	<li>Press the <code>Windows key</code> on the laptop, type <code>cmd</code> and press <code>enter</code>.</li>
	<li>Enter <code>ssh username@machinename.local</code> into the Command Prompt and press <code>enter</code>.
	<ol>
		<li>Be sure to change the <em>username</em> and <em>machine name</em> to reflect the username and hostname of the Pi.</li>
	</ol>
	</li>
	<li>Enter your password, press <code>enter</code>, and you're in!</li>
</ol>

## Additional References
A majority of our references are included within the README.pdf steps that they were used to create, but here are websites and documents that were used as additional references about ROS, Ubuntu, GitHub, and other project related topics. 

### General ROS References

https://gist.github.com/drmaj/20b365ddd3c4d69e37c79b01ca17587a

*Covers building ROS Melodic with Python 3 rather than Python 2.7, was used during testing different ROS distros.*

https://rsl.ethz.ch/education-students/lectures/ros.html#course_material

*Course materials for a C++ centric ROS course from ETH Zurich, first lecture and exercise are very useful for general ROS intro, other lectures focus more on ROS with C++.*

https://surfertas.github.io/ros/2017/03/06/ros-husky-robocup.html

*Goes over building a .launch file for Exercise Session 1 of the above course website and provides an example solution.*

https://answers.ros.org/question/253445/arduino-cmd_vel-and-odom-with-pololu-motors-w-encoder/

*Example of using ROS with the motor drivers that are used on the UTAP daughterboard, only this example is run on and Arduino with C++ rather than a Raspberry Pi with Python, but still helpful for example integration.*

http://wiki.ros.org/ROS/NetworkSetup

*Goes over different parts of configuring ROS to connect to a network, thus allowing you to communicate with ROS across multiple machines.*

http://wiki.ros.org/ROS/Tutorials/MultipleRemoteMachines

*Information about connecting to machines/robots that are not on the same network, was not very practical to test in the current situation.*

http://wiki.ros.org/Packages

*Outlines what the common files and directories are in a ROS package and defines what they are.* 

https://answers.ros.org/question/217107/does-a-roslaunch-start-roscore-when-needed/

*Clarifies that using roslaunch will autostart roscore if it is needed but not already running.*

### ROS - Launch File

https://answers.ros.org/question/12216/launch-file-how-to-write-it/

*This has an example of how a .launch file would be written.*

http://www.clearpathrobotics.com/assets/guides/kinetic/ros/Launch%20Files.html

*Provides a more detailed picture of what the .launch file should look like. The file starts with <launch> and ends with </launch>. Within those tags, there is a node tag that does essentially the same thing as launching the program with rosrun, including auto launching roscore if it is needed. The pkg in the tag is the ROS package name, the type is the name of the executable file, in our case Python script, that is being launched, and the name part of the tag should be the same as the type part, but can be used to overwrite the name of the node if necessary. Basically, for every Python script you need to call, you need a node tag, since the scripts are technically ROS nodes. Be sure to order the node tags in the order they need to be launched!*

http://wiki.ros.org/roslaunch/XML

*Official documentation on the .launch file’s XML content.*

### ROS - Robot Navigation

https://circuitpython.readthedocs.io/projects/fxas21002c/en/latest/api.html

https://cdn-learn.adafruit.com/assets/assets/000/040/671/original/FXAS21002.pdf?1491475056

*Documentation and datasheet for the NXP FXAS21002C gyroscope, part of the NXP Precision 9DoF Breakout.*

https://circuitpython.readthedocs.io/projects/fxos8700/en/latest/api.html

https://cdn-learn.adafruit.com/assets/assets/000/043/458/original/FXOS8700CQ.pdf?1499125614

*Documentation and datasheet for the NXP FXOS8700 accelerometer and magnetometer, part of the NXP Precision 9DoF Breakout.*

https://learn.adafruit.com/nxp-precision-9dof-breakout/

*Documentation for the NXP Precision 9DoF Breakout.*

https://learn.turtlebot.com/2015/02/01/14/

*Information about using ROS to move a robot to a specific point on a map, seems to need to have some sort of vision sensor on it to work. This tutorial is also specific to the TurtleBot.*

https://www.southampton.ac.uk/~fangohr/teaching/python/book/html/16-scipy.html

*The SciPy library offers a range of numerical methods for Python. When paired with numpy, it can do math with vectors and matrices, and paired with matplotlib it enables you to  plot and visualize the data. This is potentially useful for more “complex” math problems involving integration.*

https://answers.ros.org/question/335062/how-to-create-a-simple-map-for-simulation-in-stdr-simulator/?answer=335079#post-id-335079

http://wiki.ros.org/map_server

*The answer to this forum post (first link) shows how to create a map file that ROS can read. Essentially, you create a simple map in a drawing software like MS Paint or GIMP, save it as a .png, and then create a text file that provides the information ROS needs to know about (examples include image name and the resolution (in meters per pixel)), and save it in the same directory as the image file, with the same name as the image file, but with the .yaml extension. Additional information can be found in the map_server Package Summary (second link) under the Map format header.*

https://www.cs.bham.ac.uk/internal/courses/int-robot/2017/notes/amcl.php

*More information about using ROS to move a robot to a specific point on a map, still needs some sort of vision sensor or environment scanner. If you use the information in the link above to create a map, this tutorial shows how to visualize it using rviz (skip the step about running roscore and the Pioneer driver).*

https://answers.ros.org/question/227390/how-to-bring-map-in-rviz-indigo-gazebo/

*Another way to view a map file in rviz.*

https://answers.ros.org/question/233257/rospack-error-package-map_server-not-found/

*The map_server package might not be installed with the specific version of ROS that you download, so this is the command to download it. Just be sure to change “indigo” to whatever distribution of ROS you are using.*

### Python References

https://pythonprogramminglanguage.com/user-input-python/

*Reviews commands to get raw inputs from users in Python 3 and gives example uses.*

https://stackoverflow.com/questions/22990069/text-game-convert-input-text-to-lowercase-python-3-0

*Covers how to change a user's raw input into all lower-case to eliminate code errors stemming from differences in capitalization. Simply change .lower() to .upper() for upper-case.*

### General Linux References

https://learning.oreilly.com/library/view/robot-operating-system/9781484234051/

*This book provides a brief introduction to Linux operating systems, with most of the focus on Ubuntu, as well as some basic commands. It also covers the absolute basics of ROS with both C++ and Python. *

### Ubuntu References

https://www.digitalocean.com/community/tutorials/how-to-install-and-configure-vnc-on-ubuntu-18-04

*Installing VNC on a Ubuntu system so it can be used when you do not have access to a monitor, mouse, and keyboard for the Raspberry Pi.*

https://linuxize.com/post/how-to-enable-ssh-on-ubuntu-18-04/

*Enabling SSH on a Ubuntu system so it can be used when you do not have access to a monitor, mouse, and keyboard for the Raspberry Pi.*

https://www.howtogeek.com/341944/how-to-clone-your-raspberry-pi-sd-card-for-foolproof-backup/

*Making a copy of the image of Ubuntu that runs on the Raspberry Pi to allow for backups and cloning the SD card.*

https://www.raspberrypi.org/forums/viewtopic.php?t=36856

https://unix.stackexchange.com/questions/118716/unable-to-write-to-a-gpio-pin-despite-file-permissions-on-sys-class-gpio-gpio18

https://stackoverflow.com/questions/30938991/access-gpio-sys-class-gpio-as-non-root

*These three links cover the GPIO issue that we were having, and show other people having the same issues, and how they worked around the issue.*

https://www.cyberciti.biz/faq/ubuntu-change-hostname-command/

*Changing the hostname of a Ubuntu system, useful for the above configuration.*

### Raspbian References

https://www.ionos.com/digitalguide/server/configuration/provide-raspberry-pi-with-a-static-ip-address/

*Setting a static IP address for Raspberry Pi running Raspbian.*

https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/

*Has an image file of Rasbian Buster (and Raspbian Buster Lite) with ROS Melodic installed. Using this image file helps speed up the installation of ROS on the Pi.*


### Virtual Machine References

https://www.liberiangeek.net/2013/09/copy-paste-virtualbox-host-guest-machines/

*Setting up VirtualBox to allow for copy and paste between host and guest machines.*

### GitHub References

http://sethrobertson.github.io/GitBestPractices/

*Some GitHub best practices.*

https://uoftcoders.github.io/studyGroup/lessons/git/branches/lesson/

*Using GitHub on Linux within a terminal.

https://docs.github.com/en/github/using-git/ignoring-files

*Goes over what .gitignore files are and how to use them.*


https://stackoverflow.com/questions/115983/how-can-i-add-an-empty-directory-to-a-git-repository/180917#180917

*Setting up a .gitignore file to add an “empty” directory to a GitHub Repository.*

https://stackoverflow.com/questions/18216991/create-a-tag-in-a-github-repository

*Adding tags to GitHub repositories and specific commits.*

https://www.markdownguide.org/basic-syntax/

https://www.markdownguide.org/extended-syntax/

*Syntax for markdown files, used as formatting guides to make README.md on ROSPerch GitHub Repository. *

https://docs.github.com/en/github/managing-your-work-on-github/linking-a-pull-request-to-an-issue

*Has information on linking pull requests and issues, both manually and automatically with keywords.*

### Underwater Vehicle References

https://ieeexplore.ieee.org/document/6107001

*IEEE paper on implementing ROS on the Yellowfin AUV.*

https://ieeexplore.ieee.org/document/8729755
	
*IEEE presentation report on using ROS on a REMUS 100 AUV, references the Python based libraries and packages that they built to interface with the AUV, however these packages and libraries do not seem to be publicly available.*

https://www.naval-technology.com/projects/remus-100-automatic-underwater-vehicle/

*Article covering the sensors/navigation tools used by the REMUS AUV.*

### Additional GPS Sensor References

https://askubuntu.com/questions/891662/why-does-cgps-s-give-me-no-results

*Tip that helped setting up gpsd when using a USB adapter.*
