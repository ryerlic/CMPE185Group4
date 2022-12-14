#include <ros/ros.h>
#include <box_msg/BoundingBox.h>
#include <box_msg/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// Includes if visualization with OpenCV becomes a priority
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
ros::Publisher pub;
  

geometry_msgs::Point px2mm(geometry_msgs::Point& centerPixel)
{

  // Intrinsic values recorded from /camera/color/topic on RS435i
  double fx, fy, cx, cy;
  geometry_msgs::Point outPt;
  fx = 615.3138427734375;
  cx = 315.85211181640625;
  fy = 615.692138671875;
  cy = 248.609130859375;

  outPt.x = (centerPixel.x-cx)/fx*centerPixel.z;
  outPt.y = (centerPixel.y-cy)/fy*centerPixel.z;
  outPt.z = centerPixel.z;

  return outPt;
}


void callback(const sensor_msgs::ImageConstPtr& depthImg, const box_msg::BoundingBoxesConstPtr& boxImges)
  {
    ROS_INFO("lOGGED");

    
    geometry_msgs::Point objectCenter;
      for(int i=0; i < boxImges->bounding_boxes.size(); i++)
      {
        box_msg::BoundingBox box = boxImges->bounding_boxes[i];
        // For AlexDarknet Implementation 

        // double dlikelihood = box.prob;
        double dlikelihood = box.probability;


        std::string dclass = box.Class;
        // if(dlikelihood >= 0.9 && dclass.compare("Trash") == 0)
        if(dlikelihood >= 0.4)
        {
          ROS_INFO("OBJECT OUTPUT FROM DEPTH");
          if ((i == 0))
          {
            // For AlexDarknet Implementation 
            objectCenter.x = (box.x + box.w)/2;
            objectCenter.y = (box.y + box.h)/2;
            objectCenter.z = depthImg->data[int(objectCenter.x),int(objectCenter.y)];
            
            //For leggedRobotics Implementation
            // objectCenter.x = (box.xmax - box.xmin)/2;
            // objectCenter.y = (box.ymax - box.ymin)/2;
            // objectCenter.z = depthImg->data[int(objectCenter.x),int(objectCenter.y)];
          }
          else
          {
            geometry_msgs::Point tmpPt;
            // For AlexDarknet Implementation 
            tmpPt.x= (box.x + box.w)/2;
            tmpPt.y= (box.y + box.h)/2;


            // tmpPt.x= (box.xmax - box.xmin)/2;
            // tmpPt.y= (box.ymax - box.ymin)/2;


            if(objectCenter.z > depthImg->data[int(tmpPt.x),int(tmpPt.y)] )
            {

              objectCenter.x = tmpPt.x;
              objectCenter.y = tmpPt.y;
              objectCenter.z = depthImg->data[int(objectCenter.x),int(objectCenter.y)];

            }
          }

        }

      }

      if(!(objectCenter.x==0 && objectCenter.y == 0 && objectCenter.z ==0))
        
        {pub.publish(px2mm(objectCenter));}

  }
 
using namespace sensor_msgs;
using namespace message_filters;

int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "yolo_depth_detect");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Point>("goalpt",100);

  message_filters::Subscriber<sensor_msgs::Image> sub(nh,"/camera/aligned_depth_to_color/image_raw",1);
  message_filters::Subscriber<box_msg::BoundingBoxes> darknetImg(nh,"/darknet_boxes/boxdetections",1);

  typedef sync_policies::ApproximateTime<Image, box_msg::BoundingBoxes> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), sub, darknetImg);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
   

}
