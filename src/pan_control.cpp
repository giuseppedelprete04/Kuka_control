#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "boost/thread.hpp"

using namespace std;


class PAN_MOTION {

	public:
		PAN_MOTION();
		void ctrl_loop();
		void joint_data_sub(std_msgs::Float32ConstPtr );
		void run();

	private:
		ros::NodeHandle _nh;
		ros::Publisher  _j_cmd_pub;
		ros::Subscriber _j_pos_sub;
		
		double _left_lim;
		double _right_lim;
		float  _cj_pos;
		double _p;
		bool _first_jpos_data;


};



PAN_MOTION::PAN_MOTION() {

	_j_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/iiwa/j0/cmd", 0);
	_j_pos_sub = _nh.subscribe("/iiwa/j0/pos", 0, &PAN_MOTION::joint_data_sub, this);

	_nh.param( ros::this_node::getName() + "/left_lim", _left_lim, 1.2 );
	_nh.param( ros::this_node::getName() + "/right_lim", _right_lim, -1.2);

	_nh.param( ros::this_node::getName() + "/p", _p, 0.005);
	_first_jpos_data = false;

}

void PAN_MOTION::joint_data_sub( std_msgs::Float32ConstPtr j_pos ) {
	_cj_pos = j_pos->data;
	_first_jpos_data = true;
}

void PAN_MOTION::ctrl_loop() {

	cout << "Pan motion controller: move your joint from " << _left_lim << " to " << _right_lim << endl;
	while( !_first_jpos_data ) sleep(1);

	ROS_INFO("First Joint position info arrived!");

	double set_point = _left_lim;
	std_msgs::Float32 cmd;
	double ref;
	float threshold = 0.01;
	ros::Rate r(100);
	ref = _cj_pos;

	while( ros::ok() ) { 
		float e = set_point - _cj_pos;
		while (fabs(e) > threshold) {

			e = set_point - _cj_pos;
			ref += e*_p;
			cmd.data = ref;
			_j_cmd_pub.publish( cmd );
			r.sleep();	
		}

		set_point = ( set_point == _left_lim ) ? _right_lim : _left_lim;
	}

}

void PAN_MOTION::run() {

	boost::thread ctrl_loop_t( &PAN_MOTION::ctrl_loop, this);
	ros::spin();
}

int main( int argc, char** argv ) {

	ros::init( argc, argv, "pan_control");
	PAN_MOTION pm;
	pm.run();

	return 0;
}

