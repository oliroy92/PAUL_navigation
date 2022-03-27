#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
 
#define SONAR_NUM 3          
#define MAX_DISTANCE 200     // cm
#define PING_INTERVAL 50     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

// Create sonars object, in order: left, center, right
NewPing sonar[SONAR_NUM] = {
	NewPing(3, 2, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
	NewPing(5, 4, MAX_DISTANCE),
	NewPing(7, 6, MAX_DISTANCE)
};
 
ros::NodeHandle nh; //create an object which represents the ROS node.
 
void sensor_msg_init(sensor_msgs::Range &range_name, const char *frame_id_name)
{
	range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_name.header.frame_id = frame_id_name;
	range_name.field_of_view = 0.261799;
	range_name.min_range = 0.02;
	range_name.max_range = 200.0 * 0.01;
}
 
//Create three instances for range messages.
sensor_msgs::Range range[SONAR_NUM];
 
//Create publisher onjects for all sensors
ros::Publisher pub_range_left("/sonars/left", &range[0]);
ros::Publisher pub_range_center("/sonars/center", &range[1]);
ros::Publisher pub_range_right("/sonars/right", &range[2]);
 
void setup() {
	nh.initNode();
	nh.advertise(pub_range_left);
	nh.advertise(pub_range_center);
	nh.advertise(pub_range_right);
	
	sensor_msg_init(range[0], "/left_sonar_link");
	sensor_msg_init(range[1], "/center_sonar_link");
	sensor_msg_init(range[2], "/right_sonar_link");
}
 
void loop() {
	for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
		delay(PING_INTERVAL); 
		range[i].range = sonar[i].ping_cm() *0.01;
		range[i].header.stamp = nh.now();
	}
		pub_range_left.publish(&range[0]);
		pub_range_center.publish(&range[1]);
		pub_range_right.publish(&range[2]);
	
	nh.spinOnce();//Handle ROS events
}