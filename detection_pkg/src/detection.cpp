#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/server/simple_action_server.h>
#include <marker_detection/MarkerDetectionAction.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionServer<marker_detection::MarkerDetectionAction> Server;
cv::Mat camera_matrix;
cv::Mat distortion_coeffs;

class MarkerDetectionServer
{
public:
    MarkerDetectionServer() : as_(nh_, "marker_detection", boost::bind(&MarkerDetectionServer::execute, this, _1), false)
    {
        ROS_INFO("Initializing Server...");
        as_.start();
        ROS_INFO("Server Running...");
    }

    void execute(const marker_detection::MarkerDetectionGoalConstPtr &goal)
    {
        marker_detection::MarkerDetectionResult result;
        cv::Mat frame;
        // Marker Detection
        try
        {
            // Convert ROS image message to OpenCV image

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(goal->image);
            frame = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        // Initialize the ArUco dictionary (you can use a different dictionary if needed)
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Detect ArUco markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dictionary, corners, ids);

        if (!ids.empty())
        {
            // Draw markers on the image
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            // Process detected markers

            for (size_t i = 0; i < ids.size(); ++i)
            {
                // ROS_INFO("Detected marker ID: %d", ids[i]);

                std::vector<cv::Vec3d> rvec, tvec;
                // Estimate the pose of the detected marker
                try
                {

                    cv::aruco::estimatePoseSingleMarkers(corners, 0.092, camera_matrix, distortion_coeffs, rvec, tvec);
                }
                catch (cv::Exception &ex)
                {
                    ROS_ERROR("OpenCV exception: %s", ex.what());
                }
                // Extract 2D position (x, y) and orientation (theta) from tvec and rvec
                double x = tvec[0][0];     // X position
                double y = tvec[0][1];     // Y position
                double theta = rvec[0][2]; // Z rotation (around the camera's optical axis)

                // Convert theta from radians to degrees
                theta = theta * 180.0 / CV_PI;

                result.marker_id = ids[i];
                result.x = x;
                result.y = y;
                result.theta = theta;
                // Print the 2D position and orientation
                // ROS_INFO("Marker 2D Position (X, Y, Theta): %.3f, %.3f, %.3f degrees", result.x, result.y, result.theta);
            }
            as_.setSucceeded(result);
        }
        else
        {
            as_.setAborted();
        }
        // Display or publish the image with detected markers
        cv::imshow("ArUco Detection", frame);
        cv::waitKey(1);
    }

private:
    ros::NodeHandle nh_;
    Server as_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detection_server");
    camera_matrix = (cv::Mat_<double>(3, 3) << 910.7556762695312, 0.0, 634.064453125, 0.0, 909.9591064453125, 375.6187438964844, 0.0, 0.0, 1.0);
    distortion_coeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    MarkerDetectionServer server;

    ros::spin();
    return 0;
}
