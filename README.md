# 生成pid库和头文件

1. cd pid_lib
2. mkdir build
3. cd build
4. cmake ..
5. make -j
6. make install

# 编译test_pid
1. 修改pid库的头文件和库文件路径
2. 如上创建build编译

# 运行
1. 启动roscore
2. rosrun turtlesim turtlesim_node __name:=my_turtle   #启动乌龟
3. ./devel/lib/pid_contorl/pid_test  
4. rostopic pub /target/pose turtlesim/Pose "{x: 5, y: 10, theta: 0.0, linear_velocity: 0.0, angular_velocity: 0.0}"  #根据需求调节位置