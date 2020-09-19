# Project Name          : Smart Fire Detector
# Start Date            : 2020.08.31
# Git Registration Date : 2020.09.19
# Current Version       : v0.0.0
# Developer             : Jinseong Jeon
# Email Address         : aimer120@nate.com
# Property Rights       : Nau Tech & TelcokoreaIS
# Project Summary
# : The purpose of this project is to continuously detect fire elements
#	so that users can promptly evacuate in case of fire.
#
# Work Flow
 1) In AP Mode

        ___________   Send wifi info for   _____
		|          |  access this device   |__ | An Application
		| Smart    |  ----------------->   || || of user's Mobile phone
		| Fire     |                       ||_||
		| Detector |  <-----------------   |___|
		|__________|  Wifi access &
		              Send access info of nearby AP


 2) In Station Mode
    - Data to be exchanged : Sensor data & Picture & Video


                        Periodically
        ___________     or on request     _____________  Send evacuation alarm _____
		|          |     Send data    	  |            |  or requested data    |__ | An Application
		| Smart    |  ----------------->  | Management |  ------------------>  || || of user's Mobile phone
		| Fire     |                      | & Control  |                       ||_||
		| Detector |  <-----------------  | Server     |  <------------------  |___|
		|__________|    Periodically      |____________|      Request data
                        or on request 
                         Request data 
