# ROSPerch
Greetings human!
Please refer to the README.pdf document found [here](https://github.com/amichael1227/ROSPerch/blob/master/README.pdf) for any specific questions that may not be answered in this README.md document. If you need help, feel free to [create an issue]() and we'll be happy to help!


**NOTE:** *This ROS Package is currently a work in progress and updates will be coming out as available.* 


Thank you!

John & Andrew

# ***Below is said WIP!***

## Table of Contents
<ol>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#setting-up-github-getting-the-code-and-testing-it">Setting up GitHub, Getting the Code, and Testing It</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch#auto_squarepy-and-launcher_for_auto_squarepy">auto_square.py and launcher_for_auto_square.py</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#auto_driver-and-mission_commandspy">auto_driver and mission_commands.py</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#9dof_pubpy">9dof_pub.py</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.md#ssh-into-pi-over-direct-ethernet-connection">SSH into Pi Over Direct Ethernet Connection</a></li>
	<li><a href="https://github.com/amichael1227/ROSPerch/blob/master/README.pdf">README.pdf w/ Additional Information</a></li>
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
Words to be added shortly!

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
