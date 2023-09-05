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
#include <Eigen/Dense>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>



#include "my_roscpp_library/my_roscpp_library.h"
#include "my_roscpp_function/my_roscpp_function.h"
#include "my_crane_library/my_crane_library.h"

#include <boost/numeric/odeint.hpp>
#include <boost/phoenix/core.hpp>

#include <boost/phoenix/core.hpp>
#include <boost/phoenix/operator.hpp>


using namespace Eigen;
using namespace std;
using namespace std::chrono;
using namespace boost::numeric::odeint;
namespace phoenix = boost::phoenix;

typedef std::vector< double > state_type;
//[ stiff_system_definition
typedef boost::numeric::ublas::vector< double > vector_type;
typedef boost::numeric::ublas::matrix< double > matrix_type;

VectorXd qr_des = VectorXd::Zero(7);
VectorXd qc_des = VectorXd::Zero(2);

//ROBOT/CRANE LIMITS
double qlim[9] = {170*3.14/180,120*3.14/180,170*3.14/180,120*3.14/180,170*3.14/180,120*3.14/180,175*3.14/180, 4, 4};
double qdlim[9] = {85*3.14/180,85*3.14/180,100*3.14/180,75*3.14/180,130*3.14/180,135*3.14/180,135*3.14/180, 10, 10};
double ulim[9] = {320*4,320*4,170*4,170*4,110*4,40*5,40*5,500000000, 500000000};

int step_dyn = 0;

double max_dim_out = 5000;
MatrixXd q_out_vec = MatrixXd::Zero(5000,28);
int my_flag = 0;
double T_sim = 100.0;




class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void ctrl_loop();
                VectorXd estimated_dynamic(VectorXd q_in);
                VectorXd dynamic_mdl(VectorXd x);
                MatrixXd num_J(VectorXd x);
                VectorXd NF_fcn(VectorXd qr,VectorXd qv);
                double Delta_fcn(VectorXd qv,VectorXd q);

                

	private:

		ros::NodeHandle _nh;  
                ros::Publisher _my_pub;   
                ros::Publisher _my_pub2;
                ros::Publisher _my_pub_ERG;
 
                boost::shared_ptr<std_msgs::Float64MultiArray const>msg_r;
                boost::shared_ptr<std_msgs::Float64MultiArray const>msg_c;


             
};



bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());


	return true;
}



KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
        _my_pub = _nh.advertise< std_msgs::Float64MultiArray > ("/iiwa_cmd",0);
        _my_pub2 =_nh.advertise< std_msgs::Float64MultiArray > ("/cmd_input",0);
        _my_pub_ERG = _nh.advertise< std_msgs::Float64MultiArray > ("/param_ERG",0);


}




VectorXd KUKA_INVKIN::dynamic_mdl(VectorXd x) {
        //cout<<"num system"<<endl; 
        //Robot joint vectors
	VectorXd qr = VectorXd::Zero(7);
	VectorXd dqr = VectorXd::Zero(7);
        VectorXd ddqr = VectorXd::Zero(7);
	
        //Robot dynamic model
        MatrixXd Br = MatrixXd::Zero(7, 7);
	MatrixXd Cr = MatrixXd::Zero(7, 7);
	MatrixXd Gr = VectorXd::Zero(7);
        MatrixXd Kpr = MatrixXd::Identity(7, 7);
	MatrixXd Kdr = MatrixXd::Identity(7, 7);
        VectorXd taur = VectorXd::Zero(7);

        //Crane joint vectors
	VectorXd qc = VectorXd::Zero(7);
	VectorXd dqc = VectorXd::Zero(7);
        VectorXd ddqc = VectorXd::Zero(7);
	
        //Crane dynamic model
        MatrixXd Bc = MatrixXd::Zero(7, 7);
	MatrixXd Cc = MatrixXd::Zero(7, 7);
	MatrixXd Gc = VectorXd::Zero(7);
        MatrixXd Kpc = MatrixXd::Identity(2, 2);
	MatrixXd Kdc = MatrixXd::Identity(2, 2);
        VectorXd tauc = VectorXd::Zero(7);

        //Matrix system 
        MatrixXd B_sys = MatrixXd::Zero(14,14); 
        MatrixXd inv_B_sys = MatrixXd::Zero(14,14); 
        MatrixXd C_sys = MatrixXd::Zero(14,14); 
        MatrixXd G_sys = VectorXd::Zero(14);
        MatrixXd tau_sys = VectorXd::Zero(14);

        //temp matrix
        MatrixXd temp_matrix = MatrixXd::Zero(6,6);
        MatrixXd inv_temp_matrix = MatrixXd::Zero(6,6); 
        MatrixXd I_m = MatrixXd::Identity(14,14);
	
        //Matrix Constraints
        MatrixXd A = MatrixXd::Zero(6,14); //input q
        MatrixXd Ad = MatrixXd::Zero(6,14); //input q_ext
        MatrixXd A_tilda = MatrixXd::Zero(14,6); //input q_ext
        VectorXd q = VectorXd::Zero(14); // q = [qr qc]
        VectorXd q_ext = VectorXd::Zero(28); //q_ext = [qr dqr qc dqc]
        
        //exit vector (vel and acc)
        VectorXd ex = VectorXd::Zero(28);
        VectorXd ddq = VectorXd::Zero(14);
        VectorXd dq_rc = VectorXd::Zero(14);

        //flag for debug
        int cmd = 0;

        //unpack position
	for (int i = 0; i < 7; i++) qr(i) = x(i);
        for (int i = 0; i < 7; i++) qc(i) = x(i+7);
        
        //unpack velocity
	for (int i = 0; i < 7; i++) dqr(i) = x(i+14);
        for (int i = 0; i < 7; i++) dqc(i) = x(i+21);
        for (int i = 0; i < 14; i++) dq_rc(i) = x(i+14);

        //q
        for(int i=0;i<7;i++) q(i) = qr(i)+1E-14;
        for(int i=7;i<14;i++) q(i) = qc(i-7)+1E-14;

        //q_ext
        for(int i=0;i<7;i++) q_ext(i) = qr(i)+1E-14;
        for(int i=7;i<14;i++) q_ext(i) = dqr(i-7)+1E-14;
        for(int i=14;i<21;i++) q_ext(i) = qc(i-14)+1E-14;
        for(int i=21;i<28;i++) q_ext(i) = dqc(i-21)+1E-14;
        


        //qr_des << -0.1252, -0.0295, -0.1236, 1.5410, -1.5670, -1.3221,-1.5713;
        //qr_des << 0, 0, 0, 1.5708, -1.5708, -1.5708,-1.5708;
        //qc_des << 0.1,1.18;

        Cr = my_matrix_C(qr,dqr);
        Gr = my_matrix_G(qr);
        Br = my_matrix_B(qr);
        
        
        //Crane Dyn Mdl
        Bc = B_crane(qc);
	Cc = C_crane(qc,dqc);
        Gc = G_crane(qc);

        //System Dyn Model
        for(int i = 0;i<7;i++){
        for(int j = 0;j<7;j++){
        B_sys(i,j) = Br(i,j);
        }
        }

        for(int i = 0;i<7;i++){
        for(int j = 0;j<7;j++){
        B_sys(i+7,j+7) = Bc(i,j);
        }
        }

        for(int i = 0;i<7;i++){
        for(int j = 0;j<7;j++){
        C_sys(i,j) = Cr(i,j);
        }
        }

        for(int i = 0;i<7;i++){
        for(int j = 0;j<7;j++){
        C_sys(i+7,j+7) = Cc(i,j);
        }
        }
       
	for(int i = 0;i<7;i++)G_sys(i) = Gr(i);
        for(int i = 0;i<7;i++)G_sys(i+7) = Gc(i);

        A = matrix_A(q);
        Ad = matrix_Ad(q_ext);
        inv_B_sys = B_sys.completeOrthogonalDecomposition().pseudoInverse();
        temp_matrix = A*inv_B_sys*A.transpose();
        inv_temp_matrix = temp_matrix.completeOrthogonalDecomposition().pseudoInverse();
        A_tilda = inv_B_sys*A.transpose()*inv_temp_matrix;
        
        //control law 
        Kpr = 200*Kpr;
	Kdr = 50*Kdr;
	Kpc = 100000*Kpc;
        Kdc= 5000*Kdc;

        taur = Kpr * (qr_des - qr) - Kdr * dqr + Gr;
        tauc(0) = Kpc(0,0) * (qc_des(0) - qc(0)) - Kdc(0,0) * dqc(0) + Gc(0);
        tauc(3) = Kpc(1,1) * (qc_des(1) - qc(3)) - Kdc(1,1) * dqc(3) + Gc(3);

        for(int i = 0;i<7;i++)tau_sys(i) = taur(i);
        for(int i = 0;i<7;i++)tau_sys(i+7) = tauc(i);

        ddq = inv_B_sys*(I_m - A.transpose()*A_tilda.transpose())*(tau_sys - C_sys*dq_rc-G_sys)-inv_B_sys*B_sys*A_tilda*Ad*dq_rc;

        for (int i = 0; i < 14; i++) ex(i) = x(i+14);
	for (int i = 0; i < 14; i++) ex(i+14) = ddq(i);
	
        
	return ex;


}


MatrixXd KUKA_INVKIN::num_J(VectorXd x)
{
	//cout<<"num jacobian"<<endl; 
	int dim_s = 28;
	double eps = 0.0000001;
	MatrixXd J2 = MatrixXd::Zero(28, 28);
	VectorXd fx_p = VectorXd::Zero(28);
	VectorXd fx = dynamic_mdl(x);
        //cout<<"f"<<fx.transpose()<<endl;
        //cin>>dim_s;
        KUKA_INVKIN ik;

	//cout <<"fx "<< fx << endl;

	VectorXd x_p = x;

	for (int i = 0; i < dim_s; i++) {
		x_p(i) = x_p(i) + eps;
		fx_p = dynamic_mdl(x_p);
                //cout<<"f2"<<fx_p.transpose()<<endl;
		for (int j = 0; j < dim_s; j++) {
			J2(j, i) = (fx_p(j) - fx(j)) / eps;
		}
		x_p(i) = x(i);
	}

	//cout << J2 << endl;
	//cin >> dim_s;

	return J2;

}



  

void stiff_system(const vector_type& x, vector_type& dxdt, double /* t */)
	{    
                //cout<<"stiff_system"<<endl;
                VectorXd x_dyn = VectorXd::Zero(28);
                VectorXd ex = VectorXd::Zero(28); 

                KUKA_INVKIN ik;

                //cout<<"317"<<endl;
		for (int i = 0; i < 28; i++) x_dyn(i) = x[i];
                //cout<<"319"<<endl;
                //cout<<"dynamic_mdl"<<endl;
                 ex = ik.dynamic_mdl(x_dyn);
                //cout<<"end dynamic_mdl"<<endl;
                for (int i = 0; i < 14; i++) dxdt[i] = x[i+14];
                for (int i = 0; i < 14; i++) dxdt[i+14] = ex(i+14);
                //cout<<"end stiff_system"<<endl;
};

void stiff_system_jacobi(const vector_type& x, matrix_type& J, const double& /* t */, vector_type& dfdt)
	{
                //cout<<"stiff jacobian"<<endl;
		MatrixXd J_t = MatrixXd::Zero(28, 28);
		VectorXd x2 = VectorXd::Zero(28);

                KUKA_INVKIN ik;


		for (int i = 0; i < 28; i++) {
			x2(i) = x[i];
                        q_out_vec(my_flag,i) = x2(i);
                        
		}
		J_t = ik.num_J(x2);
		//cout << "J: " << J_t << endl;
		for (int i = 0; i < 28; i++) {
			for (int j = 0; j < 28; j++) {
				J(i, j) = J_t(i, j);
			}
		}
		
		for (int i = 0; i < 28; i++) {
			dfdt[i] = 0.0;
		}
                my_flag = my_flag + 1;
};



  



VectorXd KUKA_INVKIN::estimated_dynamic(VectorXd q_in){

        VectorXd q_out = VectorXd::Zero(28);     
        vector_type x(28, 0.0); //Initial condition for the solver
        vector<state_type> x_vec;
        vector<double> times;
        

        //position
        for (int i = 0; i < 28; i++) x[i] = q_in(i);
        //cout<<"inside estimated"<<endl;
	size_t num_of_steps = integrate_adaptive(make_dense_output< rosenbrock4< double > >(1.0e-3, 1.0e-6),
		make_pair(stiff_system,stiff_system_jacobi),
		x, 0.0, T_sim, 0.001);
                //cout <<"ext: " << phoenix::arg_names::arg1[1] << "\n" );
        
        //cout<<"outside estimated"<<endl;
        my_flag = 0;
        step_dyn = num_of_steps;
        //cout<<"num_steps:"<<num_of_steps<<endl;
        for (int i = 0; i < 28; i++) q_out(i) = x[i];

        return q_out;
        
}


VectorXd KUKA_INVKIN::NF_fcn(VectorXd qr,VectorXd qv) {

int cmd = 0;
//cout<<qr.transpose()<<endl;
//cout<<qv.transpose()<<endl;

VectorXd qr_r = VectorXd::Zero(7);
VectorXd qr_c = VectorXd::Zero(7);

for(int i = 0;i<7;i++) qr_r(i) = qr(i);
qr_c(0) = qr(7);
qr_c(3) = qr(8);

VectorXd qv_r = VectorXd::Zero(7);
VectorXd qv_c = VectorXd::Zero(7);

for(int i = 0;i<7;i++) qv_r(i) = qv(i);
qv_c(0) = qv(7);
qv_c(3) = qv(8);

VectorXd ro = VectorXd::Zero(9);

VectorXd ro_att = VectorXd::Zero(9);
VectorXd ro_att_den = VectorXd::Zero(2);


VectorXd ro_rep_pos = VectorXd::Zero(9);
VectorXd ro_rep_pos1 = VectorXd::Zero(2);
VectorXd ro_rep_pos2 = VectorXd::Zero(9);


VectorXd ro_rep_trq = VectorXd::Zero(9);
VectorXd ro_rep_trq1 = VectorXd::Zero(2);
VectorXd ro_rep_trq2 = VectorXd::Zero(2);
VectorXd tao_r = VectorXd::Zero(7);
VectorXd tao_c = VectorXd::Zero(2);
MatrixXd Kpr = MatrixXd::Identity(7, 7);
MatrixXd Kdr = MatrixXd::Identity(7, 7);
MatrixXd Kpc = MatrixXd::Identity(2, 2);
MatrixXd Kdc = MatrixXd::Identity(2, 2);
VectorXd tao = VectorXd::Zero(9);

MatrixXd Gr = my_matrix_G(qr_r);
MatrixXd Gc = G_crane(qr_c);

double eta = 1E-9;
double zeta = 0.8;
double delta = 0.005;


ro_att = qr-qv;
ro_att_den(0) = ro_att.norm();
ro_att_den(1) = eta;

ro_att = ro_att/ro_att_den.maxCoeff(); 

//cout<<ro_att.transpose()<<endl;

for(int i=0;i<9;i++){

ro_rep_pos1(0) = (zeta - abs(qv(i)+qlim[i]))/(zeta-delta);

ro_rep_pos2(0) = (zeta - abs(qv(i)-qlim[i]))/(zeta-delta);

ro_rep_pos(i) = ro_rep_pos1.maxCoeff()-ro_rep_pos2.maxCoeff();
}

//cout<<ro_rep_pos.transpose()<<endl;

//control law 
Kpr = 200*Kpr;
Kdr = 50*Kdr;
Kpc = 100000*Kpc;
Kdc= 5000*Kdc;

tao_r = Kpr * (qr_r - qv_r) + Gr;
tao_c(0) = Kpc(0,0) * (qr_c(0) - qv_c(0)) + Gc(0);
tao_c(1) = Kpc(1,1) * (qr_c(3) - qv_c(3)) + Gc(3); //TO CHECKED 


for(int i = 0;i<7;i++)tao(i) = tao_r(i);
tao(7) = tao_c(0);
tao(8) = tao_c(1);




for(int i=0;i<9;i++){

ro_rep_trq1(0) = (zeta - abs(tao(i)+ulim[i]))/(zeta-delta);

ro_rep_trq2(0) = (zeta - abs(tao(i)-ulim[i]))/(zeta-delta);

ro_rep_trq(i) = ro_rep_trq1.maxCoeff()-ro_rep_trq2.maxCoeff();
}

//cout<<ro_rep_trq.transpose()<<endl;

ro = ro_att+ro_rep_pos+ro_rep_trq;

//cout<<"RA: "<<ro_att.transpose()<<endl;
//cout<<"RP: "<<ro_rep_pos.transpose()<<endl;
//cout<<"RT: "<<ro_rep_trq.transpose()<<endl;

return ro;
}



double KUKA_INVKIN::Delta_fcn(VectorXd qv,VectorXd q){

double Delta = 0;
double my_time = 0;
VectorXd q_out = VectorXd::Zero(28);
VectorXd Delta_vec = VectorXd::Zero(3);
VectorXd Delta_vec2 = VectorXd::Zero(2);

VectorXd Delta_tao_vec = VectorXd::Zero(2);
VectorXd Delta_tao_vec2 = VectorXd::Zero(9);
double Delta_tao = 0;
double Delta_tao_prev = 10E15;

VectorXd pos_out = VectorXd::Zero(9);
VectorXd Delta_pos_vec = VectorXd::Zero(2);
VectorXd Delta_pos_vec2 = VectorXd::Zero(9);
double Delta_pos = 0;
double Delta_pos_prev = 10E15;

VectorXd vel_out = VectorXd::Zero(9);
VectorXd Delta_vel_vec = VectorXd::Zero(2);
VectorXd Delta_vel_vec2 = VectorXd::Zero(9);
double Delta_vel = 0;
double Delta_vel_prev = 10E15;


// parameter to tune //
double k_q = 0.8;
double k_qd = 0.05;
double k_tao = 0.001;
double k_E = 85;
double E_term = 1E5;

VectorXd qv_r = VectorXd::Zero(7);
VectorXd qv_c = VectorXd::Zero(7);
for(int i = 0;i<7;i++) qv_r(i) = qv(i);
qv_c(0) = qv(7);
qv_c(3) = qv(8);

VectorXd qr = VectorXd::Zero(7);
VectorXd qc = VectorXd::Zero(7);
VectorXd dqr = VectorXd::Zero(7);
VectorXd dqc = VectorXd::Zero(7);

VectorXd tao_r = VectorXd::Zero(7);
VectorXd tao_c = VectorXd::Zero(2);
VectorXd tao = VectorXd::Zero(9);
MatrixXd Kpr = MatrixXd::Identity(7, 7);
MatrixXd Kdr = MatrixXd::Identity(7, 7);
MatrixXd Kpc = MatrixXd::Identity(2, 2);
MatrixXd Kdc = MatrixXd::Identity(2, 2);
Kpr = 200*Kpr;
Kdr = 50*Kdr;
Kpc = 100000*Kpc;
Kdc= 5000*Kdc;


int cmd = 0;


//auto start = high_resolution_clock::now();
q_out = estimated_dynamic(q);
//auto stop = high_resolution_clock::now();        
//auto duration = duration_cast<microseconds>(stop - start);
//my_time = duration.count()*0.000001;
//cout<<"time_dyn: "<<my_time<<endl;

//cout<<"pre estimated"<<endl;
//q_out = estimated_dynamic(q);
//cout<<"post estimated"<<endl;

for(int i=0;i<step_dyn;i++){

q_out = q_out_vec.row(i);



for(int i = 0;i<7;i++) pos_out(i) = q_out(i);
pos_out(7) = q_out(7);
pos_out(8) = q_out(10);

//cout<<pos_out.transpose()<<endl;
//cin>>cmd;

for(int i = 0;i<7;i++) vel_out(i) = q_out(i+14);
vel_out(7) = q_out(21);
vel_out(8) = q_out(24);

//cout<<vel_out.transpose()<<endl;
//cin>>cmd;


//unpack position
for (int i = 0; i < 7; i++) qr(i) = q_out(i);
for (int i = 0; i < 7; i++) qc(i) = q_out(i+7);

//unpack velocity
for (int i = 0; i < 7; i++) dqr(i) = q_out(i+14);
for (int i = 0; i < 7; i++) dqc(i) = q_out(i+21);


MatrixXd Gr = my_matrix_G(qr);
MatrixXd Gc = G_crane(qc);

//control law 
tao_r = Kpr * (qv_r-qr) - Kdr * dqr + Gr;
tao_c(0) = Kpc(0,0) * (qv_c(0) - qc(0)) - Kdc(0,0) * dqc(0) + Gc(0);
tao_c(1) = Kpc(1,1) * (qv_c(3) - qc(3)) - Kdc(1,1) * dqc(3) + Gc(3);


for(int i = 0;i<7;i++)tao(i) = tao_r(i);
tao(7) = tao_c(0);
tao(8) = tao_c(1);



for(int i=0;i<9;i++){

Delta_tao_vec(0) = tao(i) + ulim[i];
Delta_tao_vec(1) = - tao(i) + ulim[i];
Delta_tao_vec2(i) = Delta_tao_vec.minCoeff();

}

Delta_tao = Delta_tao_vec2.minCoeff();

//cout<<"Delta Tao: "<<Delta_tao_vec2.transpose()<<endl;

if(Delta_tao < Delta_tao_prev) Delta_tao = Delta_tao;
else Delta_tao = Delta_tao_prev;

Delta_tao_prev = Delta_tao;


for(int i=0;i<9;i++){

Delta_pos_vec(0) = pos_out(i) + qlim[i];
Delta_pos_vec(1) = - pos_out(i) + qlim[i];
Delta_pos_vec2(i) = Delta_pos_vec.minCoeff();

}

Delta_pos = Delta_pos_vec2.minCoeff();

if(Delta_pos < Delta_pos_prev) Delta_pos = Delta_pos;
else Delta_pos = Delta_pos_prev;

Delta_pos_prev = Delta_pos;


for(int i=0;i<9;i++){

Delta_vel_vec(0) = vel_out(i) + qdlim[i];
Delta_vel_vec(1) = - vel_out(i) + qdlim[i];
Delta_vel_vec2(i) = Delta_vel_vec.minCoeff();

}

Delta_vel = Delta_vel_vec2.minCoeff();

if(Delta_vel < Delta_vel_prev) Delta_vel = Delta_vel;
else Delta_pos = Delta_pos_prev;

Delta_vel_prev = Delta_vel;

}

//cout<<"DT "<<Delta_tao<<" DP "<<Delta_pos<<" DV"<<Delta_vel<<endl;
//cin>>cmd;

Delta_vec(0) = 5*k_q*Delta_pos;
Delta_vec(1) = 50*k_qd*Delta_vel;
Delta_vec(2) = 50*k_tao*Delta_tao;

//cout<<"Delta_vec: "<<Delta_vec.transpose()<<endl;

Delta = Delta_vec.minCoeff();

Delta_vec2(0) = Delta;
Delta_vec2(1) = 0;

Delta = Delta_vec2.maxCoeff();

return Delta;
}



void KUKA_INVKIN::ctrl_loop() {


        
        int cmd = 0;
        double my_time = 0;
        double time_delta = 0;
        double time_ro = 0;
        double time_ERG = 0;
        double Ts = 0.001;
        int flag_exit = 0;
        int iter = 0;
        int first_reading = 0;

        //PUBLISHERS 
        std_msgs::Float64MultiArray cmd_pub;
        cmd_pub.data.resize(7);
        std_msgs::Float64MultiArray cmd_crane;
        cmd_crane.data.resize(2);
        cmd_crane.data[0] = 2;
        std_msgs::Float64MultiArray cmd_ERG;
        cmd_ERG.data.resize(3);


        VectorXd q_sub = VectorXd::Zero(14);
        VectorXd q_sub_c = VectorXd::Zero(14);
        VectorXd q_out = VectorXd::Zero(28); //q_out for the extim_dyn
        VectorXd ref_qr = VectorXd::Zero(7); //reference
        VectorXd ref_qc = VectorXd::Zero(2); //reference
        VectorXd x0 = VectorXd::Zero(28); //initial condition
        VectorXd x0r = VectorXd::Zero(7); //initial condition
        VectorXd qr = VectorXd::Zero(9);
        VectorXd exit = VectorXd::Zero(9);
        VectorXd qv = VectorXd::Zero(9);
        VectorXd qv_dot = VectorXd::Zero(9);
        
        //ERG
        VectorXd ro = VectorXd::Zero(9);
        double delta = 0;
        int stop_flag = 0;
        float x_off = 0;
        int my_k = 0;
        

        //x0 is the initial condition, qr_des and qc_des are the applied reference (output of the ERG), ref_qr and ref_qc the desired reference 


        //desired reference to move the system of 10cm 
        ref_qr << 0, 0, 0, 1.5708, -1.5708, -1.5708,-1.5708; // inverse kinematics solved
        ref_qc << 0.2,1.48;

        for(int i = 0;i<7;i++) qr(i) = ref_qr(i);
        for(int i = 0;i<2;i++) qr(i+7) = ref_qc(i);


        x0 << 0, 0, 0, 1.5708, -1.5708, -1.5708,-1.5708,0.2, 0, 0, 1.18, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0; //initial condition of the system 
        x0r << 0, 0, 0, 1.5708, -1.5708, -1.5708,-1.5708; //initial condition of the robot
        


        while(1) {
        
        //read for the first time to initialize the states
        if(first_reading == 0){
        

        //COMMENT TILL * IF YOU DONT WANT TO READ THE CURRENT POSITION
        //wait for a message on the robot topic 
        msg_r = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/iiwa_q"); //to know the robot status
        for(int i=0; i<14; i++ ) q_sub(i) = msg_r->data[i];
        //wait for a message on the camera topic 
        msg_c = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/block_state"); //to know the crane status
        for(int i=0; i<14; i++ ) q_sub_c(i) = msg_c->data[i];

        //initial position robot
        //for(int i=0; i<7; i++ ) x0(i) = q_sub(i);
        //x0(0) = x0(0) - 0.7854; //the offset is nedeed to align the model

        //initial vel robot
        //for(int i=14; i<21; i++ ) x0(i) = q_sub(i-7);

        //initial position crane
        //for(int i=7; i<14; i++ ) x0(i) = q_sub_c(i-7);  
        //x0(7) = x0(7);


        //initial vel crane
        //for(int i=21; i<28; i++ ) x0(i) = q_sub_c(i-14);  
        

        //x_off = x0(7);
        //ref_qc(0) = x0(7)-0.1;
        //ref_qc(1) = x0(10);
        //for(int i = 0;i<2;i++) qr(i+7) = ref_qc(i);
        //**********************
        
        //initialize the applied reference 
        for(int i=0; i<7; i++ ) qr_des(i) = x0(i);
        qc_des(0) = x0(7);
        qc_des(1) = x0(10);
 
        first_reading = 1;

        }

        if(first_reading == 1){

        
        //COMMENT TILL * IF YOU DONT WANT TO READ THE CURRENT POSITION
        //wait for a message on the robot topic 
        msg_r = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/iiwa_q");
        for(int i=0; i<14; i++ ) q_sub(i) = msg_r->data[i];
        //wait for a message on the camera topic 
        msg_c = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/block_state");
        for(int i=0; i<14; i++ ) q_sub_c(i) = msg_c->data[i];
       

        //read initial condition robot
        //for(int i=0; i<7; i++ ) x0(i) = q_sub(i);
        //x0(0) = x0(0) - 0.7854; //the offset is nedeed to align the model


        //initial vel robot
        //for(int i=14; i<21; i++ ) x0(i) = q_sub(i-7);

        //read_initial condition crane
        //for(int i=7; i<14; i++ ) x0(i) = q_sub_c(i-7); 
        //x0(7) = x0(7);

        //initial vel crane
        //for(int i=21; i<28; i++ ) x0(i) = q_sub_c(i-14); 
        
        //cout<<"POSITION READING COMPLEATE"<<endl;
        
        // *********************************************************************

        //evaluate the applied reference
        for(int i = 0;i<7;i++) qv(i) = qr_des(i);
        for(int i = 0;i<2;i++) qv(i+7) = qc_des(i);
        //cout<<"q_v:" <<qv.transpose()<<endl;
        //cout<<"x0: "<<x0.transpose()<<endl;
        //cin>>stop_flag;
        //cout<<"cmd: "<<qc_des(0)<<" "<<x_off<<endl;            
        //if(-qc_des(0)*100+x_off*100>= 10) qv = qr; //THIS IS NEEDED FOR THE MOMENT TO AVOID OVERSHOOT IN THE REFERENCE 
        
        if(qv(8) > 1.48) qv = qr;
        exit = qr-qv;
        //cout<<"norm: "<<exit.norm()<<endl;
        //ERG from here
        if (exit.norm()>1E-4 && flag_exit == 0) {
        auto start_ERG = high_resolution_clock::now();
        cout<<"ERG RUNNING..."<<endl;
        //auto start = high_resolution_clock::now();
        //cout<<"norm: "<<exit.norm()<<endl;
        //cout<<"ro"<<endl;
        auto start_ro = high_resolution_clock::now();
        ro = NF_fcn(qr,qv);
        auto stop_ro = high_resolution_clock::now();        
        auto duration_ro = duration_cast<microseconds>(stop_ro - start_ro);
        time_ro = duration_ro.count()*0.000001;



        T_sim = 10.0; //TIME PREDICTION
        auto start_delta = high_resolution_clock::now();
        delta = Delta_fcn(qv,x0);
        auto stop_delta = high_resolution_clock::now();             
        auto duration_delta = duration_cast<microseconds>(stop_delta - start_delta);
        time_delta = duration_delta.count()*0.000001;
        

        //cout<<"ro: "<<ro.transpose()<<endl;
        //cout<<"Delta: "<<delta<<endl;
        //cin>>stop_flag;
        
        //cin>>stop_flag;
        

        if(cmd_crane.data[1] == 0) my_k = 1;
        if(cmd_crane.data[1] > 0 && cmd_crane.data[1] <= 2) my_k = 3;
        if(cmd_crane.data[1] > 2 && cmd_crane.data[1] <= 3 ) my_k = 4;
        if(cmd_crane.data[1] > 3 && cmd_crane.data[1] <= 4 ) my_k = 5;
        if(cmd_crane.data[1] > 4 && cmd_crane.data[1] <= 5 ) my_k = 6;
        if(cmd_crane.data[1] > 5 && cmd_crane.data[1] <= 6 ) my_k = 7;
        if(cmd_crane.data[1] > 6 && cmd_crane.data[1] <= 7 ) my_k = 8;
        if(cmd_crane.data[1] > 8 && cmd_crane.data[1] <= 9 ) my_k = 10;
        if(cmd_crane.data[1] > 9 && cmd_crane.data[1] <= 9.5) my_k = 11;
        if(cmd_crane.data[1] > 9.5) my_k = 15;

        cout<<"Delta: "<<delta<<" my_k: "<<my_k<<endl; 
        qv_dot = my_k*ro*delta; //10* TO TUNE


        qv = qv + qv_dot*Ts;
        //cout<<"applied ref: "<<qv.transpose()<<endl;
        for(int i = 0;i<7;i++) qr_des(i) = qv(i);
        for(int i = 0;i<2;i++) qc_des(i) = qv(i+7);
        cout<<"q_v: "<<(qv(8)-1.18)*100<<endl;
        //if(qv(7) < 0.1) qv = qr;
        exit = qr-qv;
        auto stop_ERG = high_resolution_clock::now();        
        auto duration_ERG = duration_cast<microseconds>(stop_ERG - start_ERG);
        time_ERG = duration_ERG.count()*0.000001;

        cmd_ERG.data[0] = time_ro;
        cmd_ERG.data[1] = time_delta;
        cmd_ERG.data[2] = time_ERG;
        _my_pub_ERG.publish(cmd_ERG);
        }


        else {
        flag_exit = 1;
        }
        
        cout<<"ERG COMPLEATE"<<endl;

        //UNCOMMENT THE NEXT 3 LINES IF YOU NEED TO SIMULATE YOUR MODEL INSTEAD OF USING THE REAL ONE
        T_sim = 0.001;
        q_out = estimated_dynamic(x0);
        x0 = q_out;
        
        //iter = iter + 1;
        //cout<<"iter: "<<iter<<endl;


        //SENDIG COMMANDS TO THE ROBOT AND THE CRANE

        //IF YOU ARE USING THE SCRIPT ros_PD_TS.cpp or iiwa_cntrl_TS.cpp uncomment the next 3 lines 
        cmd_crane.data[1] = (qv(8)-1.18)*100;
        //cout<<"cmd: "<<qc_des(0)<<" "<<x_off<<endl;
        cout<<"des_cmd: "<<cmd_crane.data[1]<<endl;
        //cin>>stop_flag;
        _my_pub2.publish(cmd_crane);

        //IF YOU ARE USING THE SCRIPT ros_PD_JS.cpp uncomment the next 4 lines 
        //for(int i = 0;i<7;i++) cmd_pub.data[i] = qr_des(i)-x0r(i);
        //_my_pub.publish(cmd_pub);
        //cmd_crane.data[1] = qc_des(0)*100;
        //_my_pub2.publish(cmd_crane);
        //cout<<cmd_crane.data[1]<<endl;


        //IF YOU ARE USING THE SCRIPT iiwa_cntrl_JS.cpp uncomment the next 3 lines

        //publish the applied reference
        //for(int i = 0;i<7;i++) cmd_pub.data[i] = qr_des(i)-x0r(i);
        //_my_pub.publish(cmd_pub);
        //cmd_crane.data[1] = -qc_des(0)*100+20;
        //cout<<"des_cmd: "<<cmd_crane.data[1]<<endl;
        //_my_pub2.publish(cmd_crane);


        //_my_pub.publish(cmd_pub);
        //_my_pub2.publish(cmd_crane);
        }

        }
        cout<<"FINISH"<<endl;

       
        
     

        
}


void KUKA_INVKIN::run() {

	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	

}




int main(int argc, char** argv) {
         
        cout<<"START_Z"<<endl;
	ros::init(argc, argv, "my_script_cpp_z");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
