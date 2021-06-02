# BSc-Thesis<br/>
Topic: Controlling a group of mobile robots using selected location systems.<br/>
The goal was to synchronize the group of robots following the leader. I used ArUco markers, OpenCV, RPi and Arduino.<br/>
I created a navigation system based on OpenCV running on Raspberry Pi, and DC motor controlling system running on arduino.<br/>
This part of my BSc Thesis was run on Raspberry Pi. I do not add the Arudino code, because it's just simple DC motor controlling.<br/>
The program calibrates the camera, creates ArUco markers, then finds them in the enviroment, and sends the navigation commands to Arduino via bluetooth.<br/>
Credits:<br/>
George Lecakes: https://www.youtube.com/watch?v=l_4fNNyk1aw&list=PLAp0ZhYvW6XbEveYeefGSuLhaPlFML9gP<br/>
OpenCV ArUco manual: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html<br/>
