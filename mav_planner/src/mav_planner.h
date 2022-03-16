#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" // Per pubblicare velocità desiderata. Vedi rostopic info /iris/cmd/motor_vel
#include "boost/thread.hpp" //Usato in run() per dichiarare un thread che esegue PLN_loop()
#include "nav_msgs/Odometry.h"  // Per leggere la posa del robot. Vedi rostopic info /iris/odometry
#include "geometry_msgs/Pose.h"  // Per la posa desiderata del robot
#include "custom_msg_pkg/planner_msg.h" //include header del messaggio che ho creato
#include "tf/tf.h"  // Usato in odometry_cb() per passare da quaternione a angoli Eulero
#include <Eigen/Eigen>  //Usato per calcoli matriciali
#include "tf_conversions/tf_eigen.h"//Usato per passare da tf a Eigen

#include "sensor_msgs/LaserScan.h" // Per leggere il LIDAR 2D. Vedi rostopic info /laser/scan

#include "aruco_msgs/MarkerArray.h" //Per AR_marker
#include "aruco_msgs/Marker.h"
#include <tf/transform_listener.h>  //Per la posa del marker da camera_link->worldNED
#include "geometry_msgs/PoseStamped.h"

#include<cstdlib>

using namespace std;

#define G 9.81
#define PI 3.1415926535897931

class MAVPLN {

  public:

    MAVPLN();

    void run();

    void laser_cb( sensor_msgs::LaserScan  laser );  
      void SolidObstacle(int& obs_start_ind, int& obs_end_ind,  sensor_msgs::LaserScan  laser); 
    void marker_cb( const geometry_msgs::PoseStamped& marker);
      void EmptyObstacle();
    void odometry_cb( const nav_msgs::Odometry& odom );    



    Eigen::Matrix<double,3,1> RepulsiveForce();
    Eigen::Matrix<double,3,1> AttractiveForce(const Eigen::Vector3d& destination);
    bool TotalPotentialIsNotZero();
    void LocalMinima();


    void TakeOff();
    void pln_loop();
    void Planning (const Eigen::Matrix<double,3,1>& ATT, const Eigen::Matrix<double,3,1>& REP);
    void WpListManager();
    void Landing();


    


    
  private:

    ros::NodeHandle _nh;

    ros::Subscriber _odom_sub;  
    ros::Subscriber _lidar_sub;
    ros::Subscriber _marker_sub; 
        
    ros::Publisher   _plan_pub;
    ros::Publisher   _for_pot_pub;
    double _al;

    double _Kp_p;
    double _Kp_o;
    double _Kd_p;
    double _Kd_o;
    double _K_obs;

    int _wp_index;

    Eigen::Vector3d _curr_p;   
    Eigen::Vector3d _curr_eta;
    Eigen::Vector3d _curr_dp;         
    Eigen::Vector3d _curr_deta; 

    Eigen::Matrix3d _Q;
    Eigen::Vector3d _wbb;
    Eigen::Matrix3d _Rb;

    Eigen::Vector3d _des_p;           //desired position
    Eigen::Vector3d _des_dp;          //desired linear velocity
    Eigen::Vector3d _des_ddp;         //desired linear acceleration
    double _des_yaw;                  //desired yaw
    double _des_dyaw;                 //desired dyaw
    double _des_ddyaw;                //desired ddyaw


    Eigen::Vector3d _emp_pos_wned;           //marker position in worldNED
    Eigen::Vector3d _emp_ori_wned;           //marker orientation in worldNED
    Eigen::Vector3d _emp_pos_ll;             //marker position in base_link
    Eigen::Vector3d _emp_ori_bl;             //marker orientation in base_link

   
    vector< Eigen::Vector3d > _wp_list;
    vector< Eigen::Vector2d > _obs_list;


    double _min_obs_dist;
    double _att_pot;
    double _rep_pot;

    bool _first_odom;
    bool _first_laser;
    bool _found_obstacle;
    bool _stop_laser_update;
    bool _stop_odom_update;
    bool _landing;
    bool _dest_reached;
    bool _emp_obs_distant;
    bool _emp_obs_near;
    bool _emp_obs_approaching;
    bool _emp_obs_crossing;
    bool  _add_distant;
    bool  _add_near;

    bool _takeoff_ok;

    bool _TF_zero;
    bool _dest_free;  // è vero se la distanza tra drone e destinazione è inferiore a quella tra drone e il più vicino ostacolo
    bool _tot_pot_not_zero;

};
