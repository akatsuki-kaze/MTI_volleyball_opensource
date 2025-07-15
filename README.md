# MTI_volleyball_vision
 It's source code of MTI's robocon volleyball team<br>
# How to use this project
 It's easy to use.<br>
 First,you should have a computer,a xbox(or xInput surpport) joystick,and a realsense d400 series.You'd better prepare a serial device to debug or control your robot(or car)<br>
 Second,install python 3.10(tested).If you want to use other version,you have to take it(X) you can have a try and tell me which version is aviliable.<br>
 Finally,click run.bat to have fun.<br>
 P:you'd better have a nvidia device and install cudnn correctly<br>
# Unique
I have discovered yolo11s sometimes recognized white light as volleyball(blue,white and yellow).<br>
So I cut all yolo targets, then calculate the variance of all grayscale images.We can use the maxium as correect volleyball target<br>
We diden't use ros, but I use async and threading to reuce effect of delay of yolo.we use joystick to remote our robot.
