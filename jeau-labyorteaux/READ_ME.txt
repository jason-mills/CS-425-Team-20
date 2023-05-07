Apologies for the messy GitHub, all final files for Jeau Labyorteaux are stored in the folder 
CS-425-Team-20/jeau-labyorteaux/FINAL_CODE_JEAU_LABYORTEAUX/.

This final folder of my code is split into 3 main parts:
	-Scanning 
	-Renderer
	-3D Files


The Scanning folder contains two sub-directories, Arduino and Scanner. 
	
	Arduino contains the .ino file that runs on the arduino inside the hardware. 
	This will control when the hardware starts and stops rotating all based on signals sent from the scanning program.
	
	Scanner contains 4 files, 2 of these are check files that return -1 or 0 based on if the
	camera and arduino are connected or not. 1 is a .json file that contains the optimized
	settings for the Intel Realsense. Finally the last file is Scanning.cpp. This contains
	the main code that dictates when the camera should scan and when the stepper motor should spin.


The Renderer folder contains two sub-directories, New and Original.

	Original contains the original renderer I developed using Tkinter for a GUI and Open3d for
	the visualization.

	New contains the new renderer I developed that scraps the Tkinter GUI in favor for an entirely
	Open3D run applicaiton.

	Both contain a main file that is meant to be run (main.py/RenderDriver.py) and a file that contains
	most of the code (GUI.py/Renderer.py).


The 3D Files folder contains all the custom 3D models I made for the Senior Project.

	This includes the hardware container that all the components are stored in.
	The camera stand on which the camera is mounted.
	The three versions of the baseplate.

All hardware other than the electronic components were designed and printed by myself.
	
