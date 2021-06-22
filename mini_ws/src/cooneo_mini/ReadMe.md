1、display.launch 文件只是将添加了传感器样式模型后的urdf文件，在rviz中显示；
2、gazebo.launch 文件是讲添加可传感器仿真插件的urdf文件在gazebo中显示，并且在rviz中显示图像。
注意：由于rviz中加载的urdf文件不能有中文，所以在gazebo.launch文件中：rviz 和 gazebo加载的urdf模型文件不同。感兴趣的人可以分别查看这两个urdf文件的差别。





By lee cooneo.robot2018@gmail.com
