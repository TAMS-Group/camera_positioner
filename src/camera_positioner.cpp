#include <algorithm>
#include <iterator>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>

class CameraPositioner {
private:
   ros::Subscriber sub;

   // TF communication channels
   tf::TransformListener listener;
   tf::TransformBroadcaster br;

   // constant transforms
   tf::StampedTransform optical_transform;
   tf::StampedTransform world_bundle_transform;

   // latest measured position of the camera
   tf::Transform world_camera_transform;
   ros::Time latest_detection_time;
   tf::Transform last_bundle_transform;

   std::vector<int> bundle_tags;

   // Fraction used for transform interpolation
   float filter_weight;

   // for successful initialization the apriltag has to be detected _once_
   bool initialized;

   // name of the frames of the camera
   std::string camera_link;
   std::string camera_rgb_optical_frame;

public:
   CameraPositioner() : initialized(false)
   {
      ros::NodeHandle node;
      ros::NodeHandle private_node("~");
      private_node.param<float>("transform_filter_weight", filter_weight, 0.25);
      private_node.getParam("bundle_tags", bundle_tags);
      private_node.param<std::string>("camera_rgb_optical_frame", camera_rgb_optical_frame, "/camera_rgb_optical_frame");
      private_node.param<std::string>("camera_link", camera_link, "/camera_link");
      getConstantTransforms();
      sub = node.subscribe("tag_detections", 1, &CameraPositioner::callback, this);
  }

   void getConstantTransforms(){
      while(true){
         try {
            listener.waitForTransform("/world", "/ur5_mount_plate", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/world", "/ur5_mount_plate", ros::Time(0), world_bundle_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for world->ur5_mount_plate transform");
      }

      while(true){
         try {
            listener.waitForTransform(camera_rgb_optical_frame, camera_link, ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform(camera_rgb_optical_frame, camera_link,  ros::Time(0), optical_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for camera_rgb_optical_frame->camera_link transform");
      }
   }

   void callback(const apriltags2_ros::AprilTagDetectionArray& msg){
     // if we got a valid tag detection, update world_camera_transform
     for (int i=0; i< msg.detections.size(); i++) {
       int a[] = {128, 154, 162, 164};
       if (std::find(bundle_tags.begin(), bundle_tags.end(), msg.detections[i].id[0]) != bundle_tags.end()) {
         tf::Transform bundle_transform;
         tf::poseMsgToTF(msg.detections[i].pose.pose.pose, bundle_transform);
         if(!initialized){
           ROS_INFO("camera positioner is running");
           initialized = true;
         } else {
           interpolateTransforms(last_bundle_transform, bundle_transform, filter_weight, bundle_transform);
         }
         last_bundle_transform = bundle_transform;
         world_camera_transform= world_bundle_transform * bundle_transform.inverse() * optical_transform;
         latest_detection_time = msg.detections[i].pose.header.stamp;
       }
     }

     if(ros::Time::now() - latest_detection_time > ros::Duration(20.0)){
       ROS_WARN_THROTTLE(5, "Didn't detect apriltag bundle for camera position update in 20 seconds. The camera might have moved in the meanwhile.");
     }

     // if we measured the camera's position successfully, publish it
     if(initialized){
       br.sendTransform(tf::StampedTransform(world_camera_transform, ros::Time::now(), "/world", camera_link));
     }
   }

   void interpolateTransforms(const tf::Transform& t1, const tf::Transform& t2, double fraction, tf::Transform& t_out){
      t_out.setOrigin( t1.getOrigin()*(1-fraction) + t2.getOrigin()*fraction );
      t_out.setRotation( t1.getRotation().slerp(t2.getRotation(), fraction) );
   }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_position_node");
  CameraPositioner cam_pos;
  ros::spin();
  return 0;
};
