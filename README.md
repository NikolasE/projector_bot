# Projector Bot

Let humans understand a robot's planned actions and recognized obstacles.

A project at [L&T Technology Services ER&D Hackathon 2019](https://erdhackathon.bemyapp.com/)

See the [Presentation Slides](https://docs.google.com/presentation/d/1CCHufllfJAXIzi3if04-e_dvIUhWsYQIp5_ck_phLnE/edit?ts=5d04d4f2)

The platform consists of a lidar, RGBD camera, and projector mounted to a moveable table.  
<img height=200 src="https://user-images.githubusercontent.com/11611719/59589473-4d970280-90ea-11e9-8e8e-06f470507064.png"/>

After calibration, The `/scan` topic is rendered, prespective-transformed, and projected to its real-world location.  
<img height=200 src="https://user-images.githubusercontent.com/11611719/59589366-0dd01b00-90ea-11e9-944f-022e75217e40.jpg"/>

Not only is the robot's global path now visible to humans, but it is clear that the robot has recognized the person as an obstacle and has adpated its path.  
<img height=200 src="https://user-images.githubusercontent.com/11611719/59588998-39063a80-90e9-11e9-945c-f3615d1d8d13.gif"/>
