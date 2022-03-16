#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"  // Per leggere la posa del robot. Vedi rostopic info /iris/odometry

using namespace std;

class tfBC {

  public:

    tfBC();
    void run();
    void odometry_cb( const nav_msgs::Odometry& odom );
	
  private:

    ros::NodeHandle _nh;
    ros::Subscriber _odom_sub; 
};


tfBC::tfBC() {  
    _odom_sub  = _nh.subscribe("/hummingbird/odometry", 1, &tfBC::odometry_cb, this);
 
}

void tfBC::odometry_cb( const nav_msgs::Odometry& odom ) {
    //frame_id: "worldNED"

	static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
    tf::Quaternion q( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y , odom.pose.pose.orientation.z ,  odom.pose.pose.orientation.w );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "worldNED", "base_linkNED"));


}
void tfBC::run() {
  
    ros::spin();
}


int main(int argc, char** argv) {
    ros::init( argc, argv, "TFBR_node");

    tfBC mc;
    mc.run();

    return 0;
}