#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" // Per pubblicare velocità desiderata. Vedi rostopic info /iris/cmd/motor_vel
#include "boost/thread.hpp" //Usato in run() per dichiarare un thread che esegue ctrl_loop()
#include "nav_msgs/Odometry.h"  // Per leggere la posa del robot. Vedi rostopic info /iris/odometry
#include "geometry_msgs/Pose.h"  // Per la posa desiderata del robot
#include "custom_msg_pkg/planner_msg.h" //include header del messaggio che ho creato
#include "tf/tf.h"  // Usato in odometry_cb() per passare da quaternione a angoli Eulero
#include <Eigen/Eigen>  //Usato per calcoli matriciali
#include "tf_conversions/tf_eigen.h"//Usato per passare da tf a Eigen
#include <math.h>//Usato per atan()
#include <ros/console.h> //per stampare messaggi di logging (tipo ROSWARN)

using namespace std;

const double G=9.81;
const double PI=3.1415926535897931;

class MAVCTRL {

  public:

    MAVCTRL();
    void run();

    void odometry_cb( const nav_msgs::Odometry& odom );
    void planner_cb( const custom_msg_pkg::planner_msg& des_msg);

    
    void ctrl_loop();
      Eigen::Vector3d PositionPassiveController();
      void ThrustAndAttitude(const Eigen::Vector3d& mu_d);
      void FilterAndDerivative();
      void AttitudePassiveController();
      void VelocityCommand();
    void MomentumBasedEstimator();
    
    //--TEST
      void test_all();
      void test_inputs();
    //--

  private:

    ros::NodeHandle _nh;
 
    ros::Subscriber _odom_sub; 
    ros::Subscriber _plan_sub; 

    ros::Publisher  _cmd_vel_pub; 
    ros::Publisher  _inputs_pub;
    ros::Publisher  _est_pub;
    ros::Publisher  _err_pub;

    Eigen::Vector3d _curr_p;          //current position
    Eigen::Vector3d _curr_eta;        //current orientation
    Eigen::Vector3d _curr_dp;         //current linear velocity
    Eigen::Vector3d _curr_deta;       //current angular velocity

    Eigen::Vector3d _des_p;           //desired position
    Eigen::Vector3d _des_dp;          //desired linear velocity
    Eigen::Vector3d _des_ddp;         //desired linear acceleration
    double _des_yaw;                  //desired yaw
    double _des_dyaw;                 //desired dyaw
    double _des_ddyaw;                //desired ddyaw
    
    Eigen::Matrix3d _Q;
    Eigen::Matrix3d _Q_t;
    Eigen::Matrix3d _dQ;
    Eigen::Vector3d _wbb;
    Eigen::Matrix3d _S;
    Eigen::Matrix3d _M; 
    Eigen::Matrix3d _C; 
    Eigen::Matrix3d _Rb; 

    double _c_t,_c_q,_al;

    double _ut;
    Eigen::Vector3d _tau;
    Eigen::Vector3d _des_eta;         //desired orientation
    Eigen::Vector3d _des_deta;        //desired angular velocity
    Eigen::Vector3d _des_ddeta;       //desired angular aceleration
    
    Eigen::Vector3d _des_eta_old;     //per derivata
    Eigen::Vector3d _des_deta_old;    //per derivata
    Eigen::Vector3d _des_deta_1;      //per filtraggio  
    Eigen::Vector3d _des_ddeta_1;     //per filtraggio
    Eigen::Vector3d _des_deta_2;      //per filtraggio
    Eigen::Vector3d _des_ddeta_2;     //per filtraggio   
    Eigen::Vector3d _des_deta_nf;     //per filtraggio   
    Eigen::Vector3d _des_deta_1_nf;   //per filtraggio   
    Eigen::Vector3d _des_deta_2_nf;   //per filtraggio   
    Eigen::Vector3d _des_ddeta_nf;    //per filtraggio   
    Eigen::Vector3d _des_ddeta_1_nf;  //per filtraggio   
    Eigen::Vector3d _des_ddeta_2_nf;  //per filtraggio  

    Eigen::Vector3d _est_fe;          //estimated external force
    Eigen::Vector3d _est_taue;        //estimated external torque
    

    //--PositionPassiveController parameters
    double _Kp_ppc_1;
    double _Kd_ppc_1;
    double _Kp_ppc_2;
    double _Kd_ppc_2;
    double _Kp_ppc_3;
    double _Kd_ppc_3;
    //--
    //--Iris parameters
      double _mass;
      Eigen::Matrix3d _Ib;
      Eigen::Matrix4d _All_mat_inv;
    //--
    //--AttitudePassiveController parameters
      double _ni;
      double _ko_1;
      double _do_1;
      double _ko_2;
      double _do_2;
      double _ko_3;
      double _do_3;
    //--
    //--MomentumBasedEstimator parameters
    double _Ko_mbe_1;
    double _Co_mbe_1;
    double _Ko_mbe_2;
    double _Co_mbe_2;
    double _Ko_mbe_3;
    double _Co_mbe_3;    
    double _Ko_mbe_4;
    double _Co_mbe_4;
    double _Ko_mbe_5;
    double _Co_mbe_5;
    double _Ko_mbe_6;
    double _Co_mbe_6;
    //--
    //--
    int _ctrl_act;  // 1 = take_off; 2 = mission; 3 = landing; Nota: viene considerato solo quando vale 3.
    //--
    //--Control flags
    bool _first_odom;
    bool _first_plan;        
    int _filter;        
    //--
};
