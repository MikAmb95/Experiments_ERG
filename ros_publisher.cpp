#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>

using namespace std;


int main(int argc, char **argv) {


	ros::init(argc,argv,"ros_topic_publisher");
	ros::NodeHandle nh;
	ros::Publisher topic_pub = nh.advertise< std_msgs::Float64MultiArray > ("/cmd_input",0);
	//ros::Publisher topic_pub2 = nh.advertise< std_msgs::Float64MultiArray > ("/cmd_input_robot",0);

       ros::Rate rate(10);

        //std_msgs::Float64 cmd[7];
        std_msgs::Float64MultiArray d;

        float inp = 0;
        float cmd_val = 0;

        d.data.resize(2);

        cout<<"New Start"<<endl;
	while (ros::ok()) {
                
                cout<<"1 -> x / 2 -> z :";
                cin>>inp;
                cout<<"insert val in cm :";
                cin>>cmd_val;
                d.data[0] = inp;
                d.data[1] = cmd_val;

		topic_pub.publish(d);
		rate.sleep();
	}
	return 0;
}

