
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "virtual_pen");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/pen", 1);
  ros::Publisher pen_pose_pub = n.advertise<geometry_msgs::PoseStamped>("pen_pose", 1);

  geometry_msgs::PoseStamped pose_pen;

// %EndTag(INIT)%

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "base";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = 0.8;
    marker.pose.position.y = -0.3;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = -0.7071068;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.7071068;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.2;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    pose_pen.header = marker.header;
    pose_pen.pose = marker.pose;
    // pose_pen.pose.orientation.x = 0.0;
    // pose_pen.pose.orientation.y = 0.0;
    // pose_pen.pose.orientation.z = 0.0;
    // pose_pen.pose.orientation.w = 1;
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    pen_pose_pub.publish(pose_pen);
    // switch (shape)
    // {
    // case visualization_msgs::Marker::CUBE:
    //   shape = visualization_msgs::Marker::SPHERE;
    //   break;
    // case visualization_msgs::Marker::SPHERE:
    //   shape = visualization_msgs::Marker::ARROW;
    //   break;
    // case visualization_msgs::Marker::ARROW:
    //   shape = visualization_msgs::Marker::CYLINDER;
    //   break;
    // case visualization_msgs::Marker::CYLINDER:
    //   shape = visualization_msgs::Marker::CUBE;
    //   break;
    // }

    r.sleep();
  }
}
