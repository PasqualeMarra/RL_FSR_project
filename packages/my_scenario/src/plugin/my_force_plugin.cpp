#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>



using namespace std;
	
namespace gazebo
{
	class MyForcePlugin : public ModelPlugin
	{
		//PRIVATE MEMBERS

		//---ROS
		private: ros::NodeHandle _node_handle;
		
		//---Gazebo
		private: physics::LinkPtr _link;
		private: event::ConnectionPtr updateConnection;

		//PUBLIC MEMBERS

		public: void OnUpdate()  {
			//_link->AddForce(ignition::math::Vector3d(1, 0, 0));
			_link->AddForceAtRelativePosition(ignition::math::Vector3d(1, 0, 0),ignition::math::Vector3d(0, 0, 0));					
		}
		
		public: void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_) {	
			
			_link = parent_->GetLink("base_link");
			
			this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&MyForcePlugin::OnUpdate, this) );
		}

  	};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MyForcePlugin)
}



