
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>
#include <chrono>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>




using namespace Eigen;
using namespace std;
using namespace KDL;
using namespace std::chrono;


class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void ctrl_loop();
                void robot_status(const std_msgs::Float64MultiArray::ConstPtr& msg);
                void robot_ERG(const std_msgs::Float64MultiArray::ConstPtr& msg);
                void robot_torques(const std_msgs::Float64MultiArray::ConstPtr& msg);
                void crane_status(const std_msgs::Float64MultiArray::ConstPtr& msg);
                void desired_cmd(const std_msgs::Float64MultiArray::ConstPtr& msg);
                

	private:

		ros::NodeHandle _nh;

                ros::Subscriber sub_robot;
                ros::Subscriber sub_robot_tau;
                ros::Subscriber sub_crane;
                ros::Subscriber sub_cmd;
                ros::Subscriber sub_ERG;

 
                _Float64 mex_robot[14];
                VectorXd v_mex_robot = VectorXd::Zero(14);
                _Float64 mex_robot_tau[7];
                VectorXd v_mex_robot_tau = VectorXd::Zero(7);
                _Float64 mex_cmd[2];
                VectorXd v_mex_cmd = VectorXd::Zero(2);
                _Float64 mex_crane[14];
                VectorXd v_mex_crane = VectorXd::Zero(14);
                _Float64 mex_ERG[3];
                VectorXd v_mex_ERG = VectorXd::Zero(3);

};



bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	
	return true;
}

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
        sub_robot = _nh.subscribe("/iiwa_q", 0, &KUKA_INVKIN::robot_status,this); //subscriber to the desired cmd
        sub_robot_tau = _nh.subscribe("/iiwa_tau", 0, &KUKA_INVKIN::robot_torques,this); //subscriber to the desired cmd
        sub_crane = _nh.subscribe("/block_state", 0, &KUKA_INVKIN::crane_status,this); //subscriber to the desired cmd
        sub_cmd = _nh.subscribe("/cmd_input", 0, &KUKA_INVKIN::desired_cmd,this);
        sub_ERG = _nh.subscribe("/param_ERG", 0, &KUKA_INVKIN::robot_ERG,this);
        

}

void KUKA_INVKIN::robot_torques(const std_msgs::Float64MultiArray::ConstPtr& msg) {

 for(int i=0; i<7; i++ ) mex_robot_tau[i] = msg->data[i];
 for(int i=0; i<7; i++ ) v_mex_robot_tau[i] = msg->data[i];

}

void KUKA_INVKIN::robot_ERG(const std_msgs::Float64MultiArray::ConstPtr& msg) {

 for(int i=0; i<3; i++ ) mex_ERG[i] = msg->data[i];
 for(int i=0; i<3; i++ ) v_mex_ERG[i] = msg->data[i];

}

 
void KUKA_INVKIN::robot_status(const std_msgs::Float64MultiArray::ConstPtr& msg) {

 for(int i=0; i<14; i++ ) mex_robot[i] = msg->data[i];
 for(int i=0; i<14; i++ ) v_mex_robot[i] = msg->data[i];

}

void KUKA_INVKIN::crane_status(const std_msgs::Float64MultiArray::ConstPtr& msg) {

 for(int i=0; i<14; i++ ) mex_crane[i] = msg->data[i];
 for(int i=0; i<14; i++ ) v_mex_crane[i] = msg->data[i];

}


void KUKA_INVKIN::desired_cmd(const std_msgs::Float64MultiArray::ConstPtr& msg) {
 //cout<<"cmd"<<endl;
 for(int i=0; i<2; i++ ) mex_cmd[i] = msg->data[i];
 for(int i=0; i<2; i++ ) v_mex_cmd[i] = msg->data[i];


}



void KUKA_INVKIN::ctrl_loop() {

        ros::Rate r(500); // rate ros Node
        

        ofstream file_iiwa;
        ofstream file_ERG;
        ofstream file_iiwa_trq;
        ofstream file_crane;
        ofstream file_cmd;
        ofstream file_time;

        file_iiwa.open("/home/utente/ros_ws/src/iiwa_kdl/src/iiwa.m");
        file_ERG.open("/home/utente/ros_ws/src/iiwa_kdl/src/ERG.m");
        file_iiwa_trq.open("/home/utente/ros_ws/src/iiwa_kdl/src/iiwa_trq.m");
        file_crane.open("/home/utente/ros_ws/src/iiwa_kdl/src/crane.m");
        file_cmd.open("/home/utente/ros_ws/src/iiwa_kdl/src/cmd.m");
        file_time.open("/home/utente/ros_ws/src/iiwa_kdl/src/time.m");

        file_iiwa<<"data_iiwa = [";
        file_ERG<<"data_ERG = [";
        file_iiwa_trq<<"data_iiwa_trq = [";
        file_crane<<"data_crane = [";
        file_cmd<<"data_cmd = [";
        file_time<<"data_time = [";

        double my_time = 0;
        auto start = high_resolution_clock::now();
        while( ros::ok() ) {
 
                        if(my_time<300.0){
                        file_iiwa<<v_mex_robot.transpose()<<" ";
                        file_iiwa<<"\n";
                        file_cmd<<v_mex_cmd.transpose()<<" ";
                        file_cmd<<"\n";
                        file_crane<<v_mex_crane.transpose()<<" ";
                        file_crane<<"\n";
                        file_iiwa_trq<<v_mex_robot_tau.transpose()<<" ";
                        file_iiwa_trq<<"\n";
                        file_time<<my_time<<" ";
                        file_time<<"\n";
                        file_ERG<<v_mex_ERG.transpose()<<" ";
                        file_ERG<<"\n";

                        }
                        else{
                        file_iiwa<<v_mex_robot.transpose()<<"]; \n";
                        file_iiwa.close();
                        file_iiwa_trq<<v_mex_robot_tau.transpose()<<"]; \n";
                        file_iiwa_trq.close();
                        file_cmd<<v_mex_cmd.transpose()<<"]; \n";
                        file_cmd.close();
                        file_crane<<v_mex_crane.transpose()<<"]; \n";
                        file_crane.close();
                        file_time<<my_time<<"]; \n";
                        file_time.close();
                        file_ERG<<v_mex_ERG.transpose()<<"]; \n";
                        file_ERG.close();
                        } 


                auto stop = high_resolution_clock::now();        
                auto duration = duration_cast<microseconds>(stop - start);
                my_time = duration.count()*0.000001;
                cout<<"time: "<<my_time<<endl;

		r.sleep();
	}


}


void KUKA_INVKIN::run() {
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	
}




int main(int argc, char** argv) {
         
        cout<<"Start Fin"<<endl;
	ros::init(argc, argv, "ros_topic_subscriber");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
