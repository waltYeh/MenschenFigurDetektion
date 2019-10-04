#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::VideoCapture cap;
double fps; 
cv::Mat frame;
sensor_msgs::ImagePtr msg;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_node_a");
  ros::NodeHandle na;
  image_transport::ImageTransport it(na);
  image_transport::Publisher pub = it.advertise("node_a", 1);

  if (argc < 2)
  {
  	cap.open(0);
  	fps =  20;
  }
  else 
  {
  	cap.open(argv[1]);
  	fps =  cap.get(CV_CAP_PROP_FPS);
  }	
  if(!cap.isOpened())   
  {  
      ROS_INFO("can not opencv video device\n");  
      return 1;  
  }

 
  ros::Rate loop_rate(fps); //ros frequency same as the video clip fps

  while (na.ok()) {  
    cap >> frame;
    //Check if grabbed frame is actually full with some content  
    if(!frame.empty()) {  
      //publish the frame
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
      pub.publish(msg);   
    }  
    ros::spinOnce();  
    loop_rate.sleep();  
  }    

  return 0;
}