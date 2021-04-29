#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CameraPositioner {
private:
    ros::Subscriber sub;

    // TF communication channels
    tf2_ros::TransformBroadcaster dynamic_broadcaster;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2_ros::Buffer tfBuffer1;
    tf2_ros::Buffer tfBuffer2;
    tf2_ros::TransformListener tfListener1{tfBuffer1};
    tf2_ros::TransformListener tfListener2{tfBuffer2};

    // constant transforms
    geometry_msgs::TransformStamped optical_transform_geo;
    geometry_msgs::TransformStamped world_bundle_transform_geo;
    tf2::Transform optical_transform;
    tf2::Transform world_bundle_transform;
    // latest measured position of the camera
    tf2::Transform world_camera_transform;
    tf2::Transform last_bundle_transform;
    ros::Time latest_detection_time;

    std::vector<int> bundle_tags;

    // Fraction used for transform interpolation
    float filter_weight;

    // for successful initialization the apriltag has to be detected _once_
    bool initialized;

    // name of the frames of the camera
    std::string camera_link;
    std::string camera_rgb_optical_frame;
    std::string world_frame;
    std::string shared_frame;
    bool static_camera;
    int transform_count=0;
    geometry_msgs::TransformStamped transformStamped;

public:
    CameraPositioner() : initialized(false) {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");
        private_node.param<float>("transform_filter_weight", filter_weight, 0.25);
        if (!private_node.getParam("bundle_tags", bundle_tags) || bundle_tags.size() == 0) {
            ROS_ERROR("CameraPositioner was launched without any defined apriltag bundle ids!"
                      " Please check your launch file for the rosparam bundle_tags.");
            return;
        }
        private_node.param<std::string>("camera_rgb_optical_frame", camera_rgb_optical_frame,
                                        "camera_rgb_optical_frame");
        private_node.param<std::string>("camera_link", camera_link, "camera_link");
        private_node.param<std::string>("world_frame", world_frame, "world");
        private_node.param<std::string>("shared_frame", shared_frame, "ur5_mount_plate");
        private_node.param<bool>("static_camera", static_camera, false);
        getConstantTransforms();
        sub = node.subscribe("tag_detections", 1, &CameraPositioner::callback, this);
    }

    void getConstantTransforms() {
        while (ros::ok()) {
            try {
                world_bundle_transform_geo = tfBuffer1.lookupTransform(world_frame, shared_frame, ros::Time(0));
                tf2::fromMsg(world_bundle_transform_geo.transform, world_bundle_transform);
                break;
            }
            catch (...) {}
            ROS_WARN_STREAM_THROTTLE(10, "Waiting for " << world_frame << "->" << shared_frame <<
            " transform. Please make sure the frame_id did not begin with '/'");
        }

        while (ros::ok()) {
            try {
                optical_transform_geo = tfBuffer2.lookupTransform(camera_rgb_optical_frame, camera_link, ros::Time(0));
                tf2::fromMsg(optical_transform_geo.transform, optical_transform);
                break;
            }
            catch (...) {}
            ROS_WARN_STREAM_THROTTLE(10, "Waiting for " << camera_rgb_optical_frame << "->"
            << camera_link << " transform. Please make sure the frame_id did not begin with '/'");
        }
    }

    void callback(const apriltag_ros::AprilTagDetectionArray &msg) {
        // if we got a valid tag detection, update world_camera_transform
        for (int i = 0; i < msg.detections.size(); i++) {
            if (msg.detections[i].id.size() > 0) {
                if (std::find(bundle_tags.begin(), bundle_tags.end(), msg.detections[i].id[0]) != bundle_tags.end()) {
                    tf2::Transform bundle_transform;
                    tf2::fromMsg(msg.detections[i].pose.pose.pose, bundle_transform);
                    if (!initialized) {
                        ROS_INFO("Camera positioner is running!");
                        initialized = true;
                    } else {
                        interpolateTransforms(last_bundle_transform, bundle_transform, filter_weight, bundle_transform);
                    }
                    last_bundle_transform = bundle_transform;
                    world_camera_transform = world_bundle_transform * bundle_transform.inverse() * optical_transform;
                    latest_detection_time = msg.detections[i].pose.header.stamp;
                }
            } else {
                ROS_WARN_THROTTLE(5, "Found empty AprilTagDetection message!");
            }
        }

        if (ros::Time::now() - latest_detection_time > ros::Duration(20.0)) {
            ROS_WARN_THROTTLE(5, "Didn't detect apriltag bundle for camera position update in 20 seconds."
                                 " The camera might have moved in the meanwhile.");
        }

        // if we measured the camera's position successfully, publish it
        if (initialized) {
            transformStamped.header.frame_id = world_frame;
            transformStamped.child_frame_id = camera_link;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform = tf2::toMsg(world_camera_transform);
            if (static_camera) {
                static_broadcaster.sendTransform(transformStamped);
                transform_count += 1;
                // run 100 loops and then shut down this code
                if (transform_count > 100) {
                    ROS_WARN("Camera positioner is going to unsubscribe tag_detection msg as a static transform has been published.");
                    sub.shutdown();  // unsubscribe the tag_detection node to let it not detect tag anymore.
                }
            }
            else {
                dynamic_broadcaster.sendTransform(transformStamped);
            }

        }
    }

    void interpolateTransforms(const tf2::Transform &t1, const tf2::Transform &t2, double fraction,
                               tf2::Transform &t_out) {
        t_out.setOrigin(t1.getOrigin() * (1 - fraction) + t2.getOrigin() * fraction);
        t_out.setRotation(t1.getRotation().slerp(t2.getRotation(), fraction));
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_position_node");
    CameraPositioner cam_pos;
    ros::spin();
    return 0;
};
