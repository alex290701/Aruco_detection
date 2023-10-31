// Include necessary headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Image.h>
#include <marker_detection/MarkerDetectionAction.h>
#include <marker_detection/MarkerDetectionGoal.h>

// Global variables for the action client and image
sensor_msgs::ImageConstPtr received_image;
actionlib::SimpleActionClient<marker_detection::MarkerDetectionAction> *ac; // Declare the action client pointer

// Callback function for processing the image topic
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (!ac->isServerConnected())
    {
        ROS_WARN("Action server is not connected. Waiting for it to start...");
        ac->waitForServer();
        ROS_INFO("Connected to server!");
    }

    // Create a goal and populate it with the received image
    marker_detection::MarkerDetectionGoal goal;
    goal.image = *msg;

    // Send the goal to the action server
    ac->sendGoal(goal);

    bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        ac->cancelGoal();
    }
}
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "marker_detection_client");
    ros::NodeHandle nh;

    // Create an action client after initializing the node handle
    ac = new actionlib::SimpleActionClient<marker_detection::MarkerDetectionAction>("marker_detection", true);
    // Create a subscriber for the /image_raw topic
    ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw", 1, imageCallback);

    // Continue your main loop or processing here

    ros::spin(); // Keep the node running

    delete ac; // Clean up the action client

    return 0;
}
