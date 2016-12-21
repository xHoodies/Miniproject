#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

float speed = 0;
float angular = 0;
float cmd_x = 0;
float cmd_y = 0;
float targetspeed;
float targetangular;

const float SPEED = 2;
const float SPEED_INCREMENT = 0.1;
const float ANGULAR_INCREMET = 0.5;

void ControllerCallback(const geometry_msgs::Vector3::ConstPtr & msg) { //Callback function for navigate subscription
    cmd_x = msg->x;
    cmd_y = msg->y;
}

int main(int argc, char * * argv) {
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh; //create the ros node handler

    ros::Subscriber sub = nh.subscribe("direction", 1, ControllerCallback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    ros::Rate loopRate(100);
    while (ros::ok()) { //while statement for when roscore is running
        geometry_msgs::Twist converter;

        if (cmd_x < 0){
            targetspeed = SPEED;
        } else if (cmd_x > 0){
            targetspeed = -SPEED;
        } else {
            targetspeed = 0;
        }

        if (cmd_y < 0){
            targetangular = SPEED;
        } else if (cmd_y > 0){
            targetangular = -SPEED;
        } else {
            targetangular = 0;
        }

        if (speed < targetspeed) {
            speed += SPEED_INCREMENT; //commands for speeding up the turtlebot
        } else if (speed > targetspeed) {
            speed -= SPEED_INCREMENT;  //commands for slowing down the turtlebot
        }

        if (angular < targetangular) {
            angular += ANGULAR_INCREMET; //commands for speeding up the turtlebot
        } else if (angular > targetangular) {
            angular -= ANGULAR_INCREMET;  //commands for slowing down the turtlebot
        }

        converter.linear.x = speed; //go forward
        converter.angular.z = angular;

        pub.publish(converter);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
