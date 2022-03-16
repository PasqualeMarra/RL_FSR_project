#include "mav_control.h"
/*
        (FRONT)    NED frame
         0 cw           
          |x
1 ccw ____|___y 3 ccw  
          |
          |
         2 cw

eta=[ phi theta psi]'
rotazione positiva ccw: tau_phi = l*(T1 - T3)           
rotazione positiva ccw: tau_theta = l*(T0 - T2)         
rotazione positiva ccw: tau_psi = - T0 + T1 - T2 + T3   

*/
//-- Control Loop
const int fs_loop = 100;
const double dt_loop = double(1.0)/fs_loop; 
//--
//--MomentumBasedEstimator
const int fs_mbe  = 300; 
const double dt_mbe = double(1.0)/fs_mbe;
//--

//--Butterworth LPF (vedi in FilterAndDerivative)
//cut-off frequency = 100 Hz
const double b0 = 0.06014;
const double b1 = 0.1203;
const double b2 = b0;
const double a1 = - 1.019;
const double a2 = 0.2596;
//--

//-- utilizzati da VelocityCommand()
std_msgs::Float32MultiArray cmd_msg;
Eigen::Vector4d inputs;    
Eigen::Vector4d omega_sq;
//--

//--Utilizzato per grafici
std_msgs::Float32MultiArray inputs_msg;
std_msgs::Float32MultiArray est_msg;
std_msgs::Float32MultiArray err_msg;
//--

MAVCTRL::MAVCTRL() {  

    _first_odom = false;
    _first_plan = false;
    _filter = 0;

    _ctrl_act=0;
    
    _curr_p = Eigen::MatrixXd::Zero(3,1);
    _curr_eta = Eigen::MatrixXd::Zero(3,1);
    _curr_dp = Eigen::MatrixXd::Zero(3,1);
    _curr_deta = Eigen::MatrixXd::Zero(3,1);
    _des_p = Eigen::MatrixXd::Zero(3,1);
    _des_dp = Eigen::MatrixXd::Zero(3,1);
    _des_ddp = Eigen::MatrixXd::Zero(3,1);
    _des_yaw=0;
    _des_dyaw=0;
    _des_ddyaw=0;
    _ut=0;
    _tau = Eigen::MatrixXd::Zero(3,1);
    _des_eta = Eigen::MatrixXd::Zero(3,1);
    _des_deta = Eigen::MatrixXd::Zero(3,1);
    _des_ddeta = Eigen::MatrixXd::Zero(3,1);
    _des_eta_old = Eigen::MatrixXd::Zero(3,1);
    _des_deta_old = Eigen::MatrixXd::Zero(3,1);
    _des_deta_1 = Eigen::MatrixXd::Zero(3,1);
    _des_ddeta_1 = Eigen::MatrixXd::Zero(3,1);
    _des_deta_2 = Eigen::MatrixXd::Zero(3,1);
    _des_ddeta_2 = Eigen::MatrixXd::Zero(3,1);
    _des_deta_nf = Eigen::MatrixXd::Zero(3,1);
    _des_deta_1_nf = Eigen::MatrixXd::Zero(3,1);
    _des_deta_2_nf = Eigen::MatrixXd::Zero(3,1);
    _des_ddeta_nf = Eigen::MatrixXd::Zero(3,1);
    _des_ddeta_1_nf = Eigen::MatrixXd::Zero(3,1);
    _des_ddeta_2_nf = Eigen::MatrixXd::Zero(3,1);
    _est_fe = Eigen::MatrixXd::Zero(3,1);
    _est_taue = Eigen::MatrixXd::Zero(3,1); 
    _Q = Eigen::MatrixXd::Zero(3,3); 
    _Q_t = Eigen::MatrixXd::Zero(3,3);
    _dQ = Eigen::MatrixXd::Zero(3,3);
    _wbb = Eigen::MatrixXd::Zero(3,1);
    _S = Eigen::MatrixXd::Zero(3,3);
    _M = Eigen::MatrixXd::Zero(3,3);
    _C = Eigen::MatrixXd::Zero(3,3);

    if(_nh.hasParam("mass")){_nh.getParam("mass",_mass);ROS_INFO_STREAM("_mass\t"<<_mass);}
    if(_nh.hasParam("Kp_ppc_1")){_nh.getParam("Kp_ppc_1",_Kp_ppc_1);ROS_INFO_STREAM("_Kp_ppc_1\t"<<_Kp_ppc_1);}
    if(_nh.hasParam("Kd_ppc_1")){_nh.getParam("Kd_ppc_1",_Kd_ppc_1);ROS_INFO_STREAM("_Kd_ppc_1\t"<<_Kd_ppc_1);}
    if(_nh.hasParam("Kp_ppc_2")){_nh.getParam("Kp_ppc_2",_Kp_ppc_2);ROS_INFO_STREAM("_Kp_ppc_2\t"<<_Kp_ppc_2);}
    if(_nh.hasParam("Kd_ppc_2")){_nh.getParam("Kd_ppc_2",_Kd_ppc_2);ROS_INFO_STREAM("_Kd_ppc_2\t"<<_Kd_ppc_2);}
    if(_nh.hasParam("Kp_ppc_3")){_nh.getParam("Kp_ppc_3",_Kp_ppc_3);ROS_INFO_STREAM("_Kp_ppc_3\t"<<_Kp_ppc_3);}
    if(_nh.hasParam("Kd_ppc_3")){_nh.getParam("Kd_ppc_3",_Kd_ppc_3);ROS_INFO_STREAM("_Kd_ppc_3\t"<<_Kd_ppc_3);}
    if(_nh.hasParam("Ko_mbe_1")){_nh.getParam("Ko_mbe_1",_Ko_mbe_1);ROS_INFO_STREAM("_Ko_mbe_1\t"<<_Ko_mbe_1);}
    if(_nh.hasParam("Ko_mbe_2")){_nh.getParam("Ko_mbe_2",_Ko_mbe_2);ROS_INFO_STREAM("_Ko_mbe_2\t"<<_Ko_mbe_2);}
    if(_nh.hasParam("Ko_mbe_3")){_nh.getParam("Ko_mbe_3",_Ko_mbe_3);ROS_INFO_STREAM("_Ko_mbe_3\t"<<_Ko_mbe_3);}
    if(_nh.hasParam("Ko_mbe_4")){_nh.getParam("Ko_mbe_4",_Ko_mbe_4);ROS_INFO_STREAM("_Ko_mbe_4\t"<<_Ko_mbe_4);}
    if(_nh.hasParam("Ko_mbe_5")){_nh.getParam("Ko_mbe_5",_Ko_mbe_5);ROS_INFO_STREAM("_Ko_mbe_5\t"<<_Ko_mbe_5);}
    if(_nh.hasParam("Ko_mbe_6")){_nh.getParam("Ko_mbe_6",_Ko_mbe_6);ROS_INFO_STREAM("_Ko_mbe_6\t"<<_Ko_mbe_6);}
    if(_nh.hasParam("Co_mbe_1")){_nh.getParam("Co_mbe_1",_Co_mbe_1);ROS_INFO_STREAM("_Co_mbe_1\t"<<_Co_mbe_1);}
    if(_nh.hasParam("Co_mbe_2")){_nh.getParam("Co_mbe_2",_Co_mbe_2);ROS_INFO_STREAM("_Co_mbe_2\t"<<_Co_mbe_2);}
    if(_nh.hasParam("Co_mbe_3")){_nh.getParam("Co_mbe_3",_Co_mbe_3);ROS_INFO_STREAM("_Co_mbe_3\t"<<_Co_mbe_3);}
    if(_nh.hasParam("Co_mbe_4")){_nh.getParam("Co_mbe_4",_Co_mbe_4);ROS_INFO_STREAM("_Co_mbe_4\t"<<_Co_mbe_4);}
    if(_nh.hasParam("Co_mbe_5")){_nh.getParam("Co_mbe_5",_Co_mbe_5);ROS_INFO_STREAM("_Co_mbe_5\t"<<_Co_mbe_5);}
    if(_nh.hasParam("Co_mbe_6")){_nh.getParam("Co_mbe_6",_Co_mbe_6);ROS_INFO_STREAM("_Co_mbe_6\t"<<_Co_mbe_6);}
    if(_nh.hasParam("ni")){_nh.getParam("ni",_ni);ROS_INFO_STREAM("_ni\t"<<_ni);}
    if(_nh.hasParam("ko_1")){_nh.getParam("ko_1",_ko_1);ROS_INFO_STREAM("_ko_1\t"<<_ko_1);}
    if(_nh.hasParam("do_1")){_nh.getParam("do_1",_do_1);ROS_INFO_STREAM("_do_1\t"<<_do_1);}
    if(_nh.hasParam("ko_2")){_nh.getParam("ko_2",_ko_2);ROS_INFO_STREAM("_ko_2\t"<<_ko_2);}
    if(_nh.hasParam("do_2")){_nh.getParam("do_2",_do_2);ROS_INFO_STREAM("_do_2\t"<<_do_2);}
    if(_nh.hasParam("ko_3")){_nh.getParam("ko_3",_ko_3);ROS_INFO_STREAM("_ko_3\t"<<_ko_3);}
    if(_nh.hasParam("do_3")){_nh.getParam("do_3",_do_3);ROS_INFO_STREAM("_do_3\t"<<_do_3);}
    double body_height, body_width;
    if(_nh.hasParam("body_height")){_nh.getParam("body_height",body_height);ROS_INFO_STREAM("body_height\t"<<body_height);}
    if(_nh.hasParam("body_width")){_nh.getParam("body_width",body_width);ROS_INFO_STREAM("body_width\t"<<body_width);}
    double ixx = double(_mass * (pow(body_height,2) + pow(body_width,2)))/12;
    double iyy = ixx;
    double izz = double(_mass * (pow(body_width,2) + pow(body_width,2)))/12;    
    double ixy = 0.0;
    double ixz = 0.0;
    double iyz = 0.0;
    _Ib.row(0) <<  ixx, ixy, ixz;
    _Ib.row(1) <<  ixy, iyy, iyz;
    _Ib.row(2) <<  ixz, iyz, izz;
    if(_nh.hasParam("rotor_drag_coefficient")){_nh.getParam("rotor_drag_coefficient",_c_q);ROS_INFO_STREAM("rotor_drag_coefficient\t"<<_c_q);}
    if(_nh.hasParam("motor_constant")){_nh.getParam("motor_constant",_c_t);ROS_INFO_STREAM("motor_constant\t"<<_c_t);}
    if(_nh.hasParam("arm_length")){_nh.getParam("arm_length",_al);ROS_INFO_STREAM("arm_length\t"<<_al);}
    Eigen::Matrix4d All_mat = Eigen::MatrixXd::Zero(4,4); 
    All_mat.row(0) <<  1,  1,  1,  1;   //incremento angolo ccw:  1,  1,  1,  1;  
    All_mat.row(1) <<  0,  1,  0, -1;   //incremento angolo ccw:  0,  1,  0, -1; 
    All_mat.row(2) <<  1,  0, -1,  0;   //incremento angolo ccw:  1,  0, -1,  0;  
    All_mat.row(3) << -1,  1, -1,  1;   //incremento angolo ccw: -1,  1, -1,  1;  
    _All_mat_inv=All_mat.inverse();
    //cout<<"_Ib"<<endl<<_Ib<<endl;
    //cout<<"_All_mat_inv"<<endl<<_All_mat_inv<<endl;

    _odom_sub  = _nh.subscribe("/hummingbird/odometry", 1, &MAVCTRL::odometry_cb, this);
    _plan_sub  = _nh.subscribe("/planner_des", 1, &MAVCTRL::planner_cb, this);
    
    _cmd_vel_pub = _nh.advertise<std_msgs::Float32MultiArray>("/hummingbird/cmd/motor_vel", 1);

    //-- Utilizzati per grafici
        _inputs_pub = _nh.advertise<std_msgs::Float32MultiArray>("/hummingbird/inputs", 1);
        _est_pub = _nh.advertise<std_msgs::Float32MultiArray>("/hummingbird/est", 1);
        _err_pub = _nh.advertise<std_msgs::Float32MultiArray>("/hummingbird/error", 1);
    //--

}

void MAVCTRL::odometry_cb( const nav_msgs::Odometry& odom ) {
    //frame_id: "worldNED"

    _curr_p(0) = odom.pose.pose.position.x;
    _curr_p(1) = odom.pose.pose.position.y;
    _curr_p(2) = odom.pose.pose.position.z;

    _curr_dp(0) = odom.twist.twist.linear.x;
    _curr_dp(1) = odom.twist.twist.linear.y;
    _curr_dp(2) = odom.twist.twist.linear.z;

    _wbb(0) = odom.twist.twist.angular.x;
    _wbb(1) = odom.twist.twist.angular.y;
    _wbb(2) = odom.twist.twist.angular.z;

    tf::Quaternion q( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y , odom.pose.pose.orientation.z ,  odom.pose.pose.orientation.w );
    tf::Matrix3x3 Rb_tf(q);
    Rb_tf.getRPY(_curr_eta(0),_curr_eta(1),_curr_eta(2));
    tf::matrixTFToEigen(Rb_tf, _Rb);

    _Q.row(0) << 1,                  0,                  -sin(_curr_eta(1));
    _Q.row(1) << 0,  cos(_curr_eta(0)), cos(_curr_eta(1))*sin(_curr_eta(0));
    _Q.row(2) << 0, -sin(_curr_eta(0)), cos(_curr_eta(1))*cos(_curr_eta(0));

    _curr_deta = _Q.inverse()*_wbb;

    _dQ.row(0) << 0,                                0,                                                                       -cos(_curr_eta(1))*_curr_deta(1);
    _dQ.row(1) << 0, -sin(_curr_eta(0))*_curr_deta(0), -sin(_curr_eta(1))*sin(_curr_eta(0))*_curr_deta(1) + cos(_curr_eta(1))*cos(_curr_eta(0))*_curr_deta(0);
    _dQ.row(2) << 0, -cos(_curr_eta(0))*_curr_deta(0), -sin(_curr_eta(1))*cos(_curr_eta(0))*_curr_deta(1) - cos(_curr_eta(1))*sin(_curr_eta(0))*_curr_deta(0);  
        
    _S.row(0) <<       0, -_wbb(2),  _wbb(1);
    _S.row(1) <<  _wbb(2),       0, -_wbb(0);
    _S.row(2) << -_wbb(1),  _wbb(0),       0;

    _Q_t=_Q.transpose();
    _M= _Q_t*_Ib*_Q;
    _C= _Q_t*_S*_Ib*_Q+_Q_t*_Ib*_dQ;

    _first_odom = true;   

}

void MAVCTRL::planner_cb( const custom_msg_pkg::planner_msg& des_msg) {

 /*custom_msg_pkg::planner_msg
    int ctrl_action
    geometry_msgs/Vector3 pos
    geometry_msgs/Vector3 vel
    geometry_msgs/Vector3 acc
    float64 yaw
    float64 dyaw
    float64 ddyaw
    */

    _ctrl_act = des_msg.ctrl_action;

    _des_p(0)=des_msg.pos.x;
    _des_p(1)=des_msg.pos.y;
    _des_p(2)=des_msg.pos.z;

    _des_dp(0)=des_msg.vel.x;
    _des_dp(1)=des_msg.vel.y;
    _des_dp(2)=des_msg.vel.z;

    _des_ddp(0)=des_msg.acc.x;
    _des_ddp(1)=des_msg.acc.y;
    _des_ddp(2)=des_msg.acc.z;

    _des_yaw=des_msg.yaw;
    _des_dyaw=des_msg.dyaw;
    _des_ddyaw=des_msg.ddyaw;

    _first_plan = true;

}

Eigen::Vector3d MAVCTRL::PositionPassiveController(){
    //formula 17.4
    //cout<<"PositionPassiveController"<<endl;

    Eigen::Matrix3d Kp;
    Kp.row(0) <<  _Kp_ppc_1,  0,  0;
    Kp.row(1) <<   0, _Kp_ppc_2,  0;
    Kp.row(2) <<   0,  0, _Kp_ppc_3;
    Eigen::Matrix3d Kd;
    Kd.row(0) <<  _Kd_ppc_1,  0,  0; 
    Kd.row(1) <<   0, _Kd_ppc_2,  0; 
    Kd.row(2) <<   0,  0, _Kd_ppc_3;

    Eigen::Vector3d e_p;
    Eigen::Vector3d de_p;
    Eigen::Vector3d mu_d;

    e_p= _curr_p - _des_p;
    de_p= _curr_dp - _des_dp;
    mu_d = _des_ddp - (double(1)/_mass)*( Kd*de_p + Kp*e_p);
        
    //ROS_INFO("\nmu_d:\t[%f %f %f ]", mu_d(0), mu_d(1), mu_d(2));
    //ROS_INFO("\ne_p:\t[%f %f %f ]", e_p(0), e_p(1), e_p(2));

    err_msg.data.resize(6);
    err_msg.data[0]=e_p(0);
    err_msg.data[1]=e_p(1);
    err_msg.data[2]=e_p(2);

    return mu_d;
}

void MAVCTRL::ThrustAndAttitude(const Eigen::Vector3d& mu_d){
    //formula 17.4bis
    //cout<<"ThrustAndAttitude"<<endl; 

    double temp = 0;
    double des_roll = 0;
    double des_pitch = 0;

    Eigen::Vector3d mu_bar = mu_d - _est_fe*(double(1)/_mass);
    double mu_x = mu_bar(0);
    double mu_y = mu_bar(1);
    double mu_z = mu_bar(2);

    if(mu_z == G) mu_z = G+0.0001;

    _ut = _mass*sqrt(pow(mu_x,2)+pow(mu_y,2)+pow(mu_z-G,2)); 

    temp=mu_y*cos(_des_yaw) - mu_x*sin(_des_yaw);
    temp*=_mass;
    temp/=_ut;
    if(temp>=1) temp=1.0;
    if (temp<=-1) temp=-1.0;
      
    
    des_roll = asin( temp );
    des_pitch = atan( double((mu_x*cos(_des_yaw) + mu_y*sin(_des_yaw))) / (mu_z-G) ); 
    
    _des_eta(0)=des_roll;
    _des_eta(1)=des_pitch;
    _des_eta(2)=_des_yaw;

    //ROS_INFO("\nmu_bar ut des_eta:\t[%f %f %f %f %f %f %f]", mu_bar(0),mu_bar(1),mu_bar(2),_ut, _des_eta(0), _des_eta(1), _des_eta(2));
    
}

void MAVCTRL::FilterAndDerivative(){

    //cout<<"FilterAndDerivative"<<endl;

    /*  ### Second-order Butterworth LPF ###

        T=sampling time;
        fcut = cut-off frequency 3 dB; wcut=2*pi*fcut
        w = (2/T)*tan(wcut*T/2)

        F(s)= w^2 / (s^2 + 2*zita*w*s + w^2); zita=sqrt(2)/2

        s = (2/T)*((1-z^-1)/(1+z^-1))
        
        F(z) = (b0 + b1*z^-1 + b2*z^-2)/(1 + a1*z^-1 + a2*z^-2)

        y(k)= b0*u(k) + b1*u(k-1) + b2*u(k-2) - a1*y(k-1) - a2*y(k-2)
    
    */

    
    // FILTRO CON VARIABILE
    //cerco di evitare il salto nella derivata perchè inizialmente
    //_des_eta_old=_des_deta_old=0
    
    if(_filter>=2){
        //Derivation and filtering for _des_deta
        _des_deta_nf = double(fs_loop)*(_des_eta - _des_eta_old);
        _des_deta = b0*_des_deta_nf + b1*_des_deta_1_nf + b2*_des_deta_2_nf - a1*_des_deta_1 - a2*_des_deta_2;
        
        //Derivation and filtering for _des_ddeta
        _des_ddeta_nf= double(fs_loop)*(_des_deta - _des_deta_old);   
        _des_ddeta = b0*_des_ddeta_nf + b1*_des_ddeta_1_nf + b2*_des_ddeta_2_nf - a1*_des_ddeta_1 - a2*_des_ddeta_2;
   
        //Update
        _des_eta_old = _des_eta;
        _des_deta_old = _des_deta;

        _des_deta_2 = _des_deta_1;
        _des_deta_1 = _des_deta;
        _des_deta_2_nf = _des_deta_1_nf;
        _des_deta_1_nf = _des_deta_nf;
        
        _des_ddeta_2 = _des_ddeta_1;
        _des_ddeta_1 = _des_ddeta;
        _des_ddeta_2_nf = _des_ddeta_1_nf;
        _des_ddeta_1_nf = _des_ddeta_nf;
   
    }
    else if (_filter ==0){

        _filter++;
        _des_eta_old = _des_eta;
    }
    else if (_filter ==1){

        _filter++;

        _des_deta_nf = double(fs_loop)*(_des_eta - _des_eta_old);
        _des_deta = b0*_des_deta_nf + b1*_des_deta_1_nf + b2*_des_deta_2_nf - a1*_des_deta_1 - a2*_des_deta_2;
        
        _des_eta_old = _des_eta;
        _des_deta_old = _des_deta;

        _des_deta_2 = _des_deta_1;
        _des_deta_1 = _des_deta;
        _des_deta_2_nf = _des_deta_1_nf;
        _des_deta_1_nf = _des_deta_nf;

    }
    
}

void MAVCTRL::AttitudePassiveController(){
    //formula 17.2bis
    //cout<<"AttitudePassiveController"<<endl;


    Eigen::Matrix3d Ko;
    Ko.row(0) << _ko_1, 0, 0;
    Ko.row(1) << 0, _ko_2, 0;
    Ko.row(2) << 0, 0, _ko_3;
    Eigen::Matrix3d Do;
    Do.row(0) << _do_1, 0, 0;
    Do.row(1) << 0, _do_2, 0;
    Do.row(2) << 0, 0, _do_3;
  
    Eigen::Vector3d err_eta = _curr_eta - _des_eta;
    Eigen::Vector3d derr_eta = _curr_deta - _des_deta;
    Eigen::Vector3d ref_deta = _des_deta - _ni * err_eta;
    Eigen::Vector3d ref_ddeta = _des_ddeta - _ni * derr_eta;
    Eigen::Vector3d ni_eta = derr_eta + _ni * err_eta;


    Eigen::Vector3d _tau_1= _M*ref_ddeta;
    Eigen::Vector3d _tau_2= _C*ref_deta;
    Eigen::Vector3d _tau_3= _est_taue;
    Eigen::Vector3d _tau_4= Do*ni_eta;
    Eigen::Vector3d _tau_5= Ko*err_eta;
        
    _tau = _Q_t.inverse()*(_tau_1 + _tau_2 - _tau_3 - _tau_4 - _tau_5);

   //ROS_INFO("\ntau:\t[%f %f %f]", _tau(0), _tau(1), _tau(2));

    err_msg.data[3]=err_eta(0);
    err_msg.data[4]=err_eta(1);
    err_msg.data[5]=err_eta(2);
    _err_pub.publish(err_msg);


}

void MAVCTRL::VelocityCommand(){
    
    //cout<<"VelocityCommand"<<endl;

    cmd_msg.data.resize(4);

    inputs_msg.data.resize(4);
    inputs_msg.data[0]=_ut;
    inputs_msg.data[1]=_tau(0);
    inputs_msg.data[2]=_tau(1);
    inputs_msg.data[3]=_tau(2);
    _inputs_pub.publish(inputs_msg);

    inputs={double(_ut)/_c_t,double(_tau(0))/(_al*_c_t),double(_tau(1))/(_al*_c_t),double(_tau(2))/_c_q};  
    //inputs={double(_ut)/_c_t,0,0,0};   //test solo controllore posizione
    omega_sq = _All_mat_inv * inputs;

    if (_ctrl_act == 3) omega_sq = Eigen::MatrixXd::Zero(4,1);

    for (int i=0;i<4;i++){

        if( omega_sq(i)<0 ) {
            ROS_WARN("omega_sq(%i)=\t%f", i, omega_sq(i));
            omega_sq(i)=0;
        }

        cmd_msg.data[i]=sqrt(omega_sq(i));
    }


    _cmd_vel_pub.publish(cmd_msg);
    
    //ROS_INFO("\nCMD_OMEGA:\t[%f %f %f %f]", sqrt(omega_sq(0)), sqrt(omega_sq(1)), sqrt(omega_sq(2)), sqrt(omega_sq(3)) );

}

void MAVCTRL::ctrl_loop(){

    sleep(1);   //in modo da lasciare qualche secondo allo stimatore prima di decollare

    while (!(_first_odom)) sleep(0.1);

    //test_all();
    //test_inputs(); 
    
    _des_p=_curr_p;
    _des_yaw  = _curr_eta(2);


    ros::Rate r(fs_loop);

    ROS_INFO("CONTROL LOOP");
        
    while ( ros::ok()) {
        

        ThrustAndAttitude(PositionPassiveController());
        FilterAndDerivative();
        AttitudePassiveController();
        VelocityCommand();

        r.sleep();

    }



}

void MAVCTRL::MomentumBasedEstimator(){

    //First Order Estimator
    
    Eigen::Vector3d mge3={0,0,_mass*G};
    Eigen::Vector3d q_1 = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d q_2 = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d sub_int_1 = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d sub_int_2 = Eigen::MatrixXd::Zero(3,1);

    Eigen::Matrix<double, 6, 1> est_Fe = Eigen::MatrixXd::Zero(6,1);
    Eigen::Matrix<double, 6, 1> q = Eigen::MatrixXd::Zero(6,1);            //generalized momentum
    Eigen::Matrix<double, 6, 1> integral = Eigen::MatrixXd::Zero(6,1);
    Eigen::Matrix<double, 6, 1> sub_integral = Eigen::MatrixXd::Zero(6,1);
    
     Eigen::Matrix<double, 6, 6> Ko_mbe;
    Ko_mbe.row(0) <<  _Ko_mbe_1,  0,  0,  0,  0,  0;
    Ko_mbe.row(1) <<   0, _Ko_mbe_2,  0,  0,  0,  0;
    Ko_mbe.row(2) <<   0,  0, _Ko_mbe_3,  0,  0,  0;
    Ko_mbe.row(3) <<   0,  0,  0, _Ko_mbe_4,  0,  0;
    Ko_mbe.row(4) <<   0,  0,  0,  0, _Ko_mbe_5,  0;
    Ko_mbe.row(5) <<   0,  0,  0,  0,  0, _Ko_mbe_6;
    Eigen::Matrix<double, 6, 6> Co_mbe;
    Co_mbe.row(0) <<  _Co_mbe_1,  0,  0,  0,  0,  0;
    Co_mbe.row(1) <<   0, _Co_mbe_2,  0,  0,  0,  0;
    Co_mbe.row(2) <<   0,  0, _Co_mbe_3,  0,  0,  0;
    Co_mbe.row(3) <<   0,  0,  0, _Co_mbe_4,  0,  0;
    Co_mbe.row(4) <<   0,  0,  0,  0, _Co_mbe_5,  0;
    Co_mbe.row(5) <<   0,  0,  0,  0,  0, _Co_mbe_6;


    while (!_first_odom) sleep(0.1);

    ros::Rate r(fs_mbe);


    while ( ros::ok() ) {

    
        sub_int_1 = - _ut*_Rb.col(2) + mge3;
        sub_int_2 = _Q_t*_tau + _C.transpose()*_curr_deta;
        sub_integral(0)=sub_int_1(0);
        sub_integral(1)=sub_int_1(1);
        sub_integral(2)=sub_int_1(2);
        sub_integral(3)=sub_int_2(0);
        sub_integral(4)=sub_int_2(1);
        sub_integral(5)=sub_int_2(2);

        integral+= ( Ko_mbe*sub_integral + Co_mbe*est_Fe ) * dt_mbe;
    
        q_1 = _mass*_curr_dp;
        q_2= _M*_curr_deta;
        q(0)=q_1(0);
        q(1)=q_1(1);
        q(2)=q_1(2);
        q(3)=q_2(0);
        q(4)=q_2(1);
        q(5)=q_2(2);


        est_Fe    = Ko_mbe*q - integral;
        _est_fe   = est_Fe.topRows(3);
        _est_taue = est_Fe.bottomRows(3);

        //cout<<"MomentumBasedEstimator"<<endl;
        //ROS_INFO("\nEST_F:\t[%f %f %f]\tEST_T:\t[%f %f %f]", est_Fe(0), est_Fe(1), est_Fe(2), est_Fe(3), est_Fe(4), est_Fe(5));
        est_msg.data.resize(6);
	    est_msg.data[0] = _est_fe(0);
	    est_msg.data[1] = _est_fe(1);
	    est_msg.data[2] = _est_fe(2);
	    est_msg.data[3] = _est_taue(0);
	    est_msg.data[4] = _est_taue(1);
	    est_msg.data[5] = _est_taue(2);
	    _est_pub.publish(est_msg);


        r.sleep();
    }

}

void MAVCTRL::run() {
  
    
    boost::thread mbe_t( &MAVCTRL::MomentumBasedEstimator, this );
    boost::thread ctrl_loop_t( &MAVCTRL::ctrl_loop, this );

    ros::spin();
}


int main(int argc, char** argv) {
    ros::init( argc, argv, "MAVCTRL_node");

    MAVCTRL mc;
    mc.run();

    return 0;
}

//--FUNZIONI PER IL TESTING

void MAVCTRL::test_all(){
    /*
    UTILIZZATA PER TESTARE LA CORRETTEZZA DELLA MATRICE DI ALLOCAZIONE
    */


    cmd_msg.data.resize(4);

    ros::Rate r(fs_loop);

    int w0=1100;
    int w1=1100;
    int w2=1100;
    int w3=1100;

    float tc =0;    

    while(ros::ok() && tc<= 1.5){ //decollo
 
        cmd_msg.data[0]=w0;
        cmd_msg.data[1]=w1;
        cmd_msg.data[2]=w2;
        cmd_msg.data[3]=w3;

        _cmd_vel_pub.publish(cmd_msg);

        tc+=0.001;
        cout<<"decollo"<<endl;
        r.sleep();
    }

    w0=1100;    // rot ccw x: w0=1100; rot ccw y: w0=1100; rot ccw z: w0=1000;
    w1=1100;    // rot ccw x: w1=1100; rot ccw y: w1=1100; rot ccw z: w1=1100;
    w2=1000;    // rot ccw x: w2=1100; rot ccw y: w2=1000; rot ccw z: w2=1000;
    w3=1100;    // rot ccw x: w3=1000; rot ccw y: w3=1100; rot ccw z: w3=1100;

    while(ros::ok()){ //rotazione
 
        //inputs={_ut/_c_t,0,0,0};   //test solo controllore posizione
        //omega_sq = _All_mat_inv * inputs;
        //for (int i=0;i<4;i++)cmd_msg.data[i]=sqrt(abs(omega_sq(i)));

        cmd_msg.data[0]=w0;
        cmd_msg.data[1]=w1;
        cmd_msg.data[2]=w2;
        cmd_msg.data[3]=w3;

        _cmd_vel_pub.publish(cmd_msg);
        //cout<<"ROTAZIONE"<<endl;

        r.sleep();
    }
}

void MAVCTRL::test_inputs(){
    /*
    UTILIZZATA PER TESTARE LA CORRETTEZZA DELLE ROTAZIONI
    DANDO IN INGRESSO DIRETTAMENTE UN VALORE DI COPPIA DESIDERATA
    */


    cmd_msg.data.resize(4);

    ros::Rate r(fs_loop);

    float tc =0;    
    _ut=36; // per _ut=35 inizia a decollare

    while(ros::ok()){

        //--test rotazione intorno x imponendo taux
        if(tc>=1.5){
            _tau(0)=_ut*_al*0.5;
            cout<<"_tau(0)"<<endl<<_tau(0)<<endl;
        }
        //--
        //--test rotazione intorno y imponendo tauy
        //if(tc>=1.5){
        //    _tau(1)=_ut*_al*0.5;
        //    cout<<"_tau(1)"<<endl<<_tau(1)<<endl;
        //}
        //--
        //--test rotazione intorno z imponendo tauz
        //if(tc>=1.5){
        //  _tau(2)=(_ut*_c_q)/_c_t;
        //  cout<<"_tau(2)"<<endl<<_tau(2)<<endl;
        //}
        //--




        VelocityCommand();
        tc+=0.001;        
        r.sleep();
    }

}
