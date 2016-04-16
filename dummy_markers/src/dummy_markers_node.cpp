/*
 * This is a dummy node that can output a box marker for RVIZ or
 * whatever you need. Its purpose is to allow for the Robot Arm Tracking
 * Team to be able to have an easy tool that can emulate the environment
 * that we specify.
 *
 * Setup for this node in Rviz is as follows
 * rosrun rviz rviz
 *
 * In Rviz, set "Global Options -> fixed_frame" to "my_frame"
 * Add a marker that subscribes to /visualization_marker
 *
 * And then run this node in a new terminal:
 * rosrun dummy_markers dummy_markers_node
 * 
 * Now you can publish a center of mass point for the box marker by
 * 
 * rostopic pub -1 /dummy_marker geometry_msgs/Point -- x y z
 * 
 * Where x, y, and z are floating points values that correspond to the 
 * location of the box relative to the origin point 0.0 0.0 0.0
 * E.G. To display the box at the origin use 
 * rostopic pub -1 /dummy_marker geometry_msgs/Point -- 0.0 0.0 0.0
 * 
 * 
 * You should now see the dummy node appear in Rviz.
*/


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// These macros are used to set the scale
// of the output bounding box
#define X_WIDTH 0.1
#define Y_HEIGHT 0.2
#define Z_DEPTH 0.05

ros::Publisher marker_pub;


void callback(const geometry_msgs::Point centerOfMass);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "dummy_marker_node");
  ros::NodeHandle n;
  uint32_t queue_size = 1;

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  std::string sub_topic = n.resolveName("/dummy_marker");

  ros::Subscriber sub = n.subscribe<geometry_msgs::Point> (sub_topic, queue_size, callback);

  ros::spin();

  return 0;

}

void callback(const geometry_msgs::Point centerOfMass)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = centerOfMass.x;
    marker.pose.position.y = centerOfMass.y;
    marker.pose.position.z = centerOfMass.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = X_WIDTH;
    marker.scale.y = Y_HEIGHT;
    marker.scale.z = Z_DEPTH;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    if(marker_pub.getNumSubscribers() < 1)
    {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    else
    {
        marker_pub.publish(marker);
    }

}
