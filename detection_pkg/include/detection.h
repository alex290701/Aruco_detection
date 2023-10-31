#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/server/simple_action_server.h>
#include <marker_detection/MarkerDetectionAction.h>
#include <marker_detection/MarkerDetectionGoal.h>
#include <marker_detection/MarkerDetectionResult.h>
#include <marker_detection/MarkerDetectionFeedback.h>

#ifndef DETECTION_H
#define DETECTION_H

namespace marker_detection
{
    class DetectionAction
    {
    public:
        DetectionAction(std::string name);
        void executeCallback(const MarkerDetectionGoal &goal);

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<MarkerDetectionAction> action_server_;
    };

} // namespace marker_detection

#endif
