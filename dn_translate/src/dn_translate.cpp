#include <DarkHelp.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <box_msg/BoundingBox.h>
#include <box_msg/BoundingBoxes.h>
#include <image_transport/image_transport.h>



using namespace cv;
using namespace std;

class Detector
{
  private:
    image_transport::Subscriber sub;
    image_transport::Publisher pub_frame;
    ros::Publisher box_pub;
    std::string camera;
    std::string config_file =  "/home/ryan/catkin_ws/src/boris/dn_translate/model/litter.cfg";
    std::string weights_file = "/home/ryan/catkin_ws/src/boris/dn_translate/model/litter_best.weights";
    std::string names_file = "/home/ryan/catkin_ws/src/boris/dn_translate/model/train.names";
    DarkHelp::NN nn;


  public:
    Detector()
    {
      ros::NodeHandle nh;

      ROS_INFO("The node has initiated");
      // nh.getParam("camera",camera);
      // nh.getParam("dn_translate/config_file",config_file);
      // nh.getParam("dn_translate/weights_file",weights_file);
      // nh.getParam("dn_translate/names_file",names_file);
      image_transport::ImageTransport it(nh);
      sub = it.subscribe("/camera/color/image_raw" , 1 , &Detector::imageCallback,this);
      pub_frame = it.advertise("darknet_boxes/camloc", 1);
      
      box_pub = nh.advertise<box_msg::BoundingBoxes>("darknet_boxes/boxdetections",1);


    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      ROS_INFO("Successfully subscribed to the camera");
      cv_bridge::CvImagePtr cv_ptr;

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        cv::Mat current_frame = cv_ptr->image;
        // Check if grabbed frame has content
        if (current_frame.empty()) {
          ROS_ERROR_STREAM("Failed to capture image!");
          ros::shutdown();
        }
        


        nn.init(config_file,weights_file,names_file);
      //Run the Neural Network against the captured frame
        const auto result = nn.predict(current_frame);
        uint8_t i = 0;
        box_msg::BoundingBoxes outDetect;

        //Visualizer for the bounding box
        for (const auto & det : result)
        {

          ROS_INFO("BOX AHOY");
            box_msg::BoundingBox curDet;
            Rect detect = det.rect;
            curDet.x = detect.x;
            curDet.y = detect.y;
            curDet.h = detect.height;
            curDet.w = detect.width;
            curDet.probability = det.best_probability;
            curDet.Class = det.best_class;

            Point topLeft = Point(detect.x,detect.y);
            Point bottomRight = Point(detect.x+ detect.width,detect.y+detect.height);

            outDetect.bounding_boxes.push_back(curDet);
            rectangle(current_frame,topLeft,bottomRight,Scalar(255,0,0),5,LINE_8);
            i++;
        }
        
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",current_frame).toImageMsg();
        pub_frame.publish(msg);


        outDetect.header.stamp = ros::Time::now();
        outDetect.header.frame_id = "detection";
        box_pub.publish(outDetect);



      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8' ",msg->encoding.c_str());

      }
    }
};





int main(int argc, char** argv)
{
    ros::init(argc, argv, "dn_translate");

    Detector deteObj;
    ROS_INFO("post initilization");
 
    ros::spin();
    return 0;

}



