
#include "ros/ros.h"
#include "image_transport/image_transport.h"  
#include "sensor_msgs/image_encodings.h" 
#include <struck/roi_input.h>
#include <struck/track_output.h>
#include <cv_bridge/cv_bridge.h>

#include "Tracker.h"
#include "Config.h"

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
void myrectangle(Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(), r.YMax()), rColour);
}
class StruckTrack
{
private:
	static const int kLiveBoxWidth = 100;
	static const int kLiveBoxHeight = 100;
	FloatRect initBB;
	FloatRect inputBB;
	bool statusRunning;
	bool doInitialise;
	bool paused;
	string configPath;
	Config conf;
	Mat result;
	Tracker tracker;
	ros::Publisher output_pub;
	ros::Subscriber input_sub;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;
public:
	StruckTrack(ros::NodeHandle& nh, const string& configPath);
	~StruckTrack();
	void process(cv::Mat frameOrig);
	void inputCallback(const struck::roi_input msg)
	{
		static int cnt=0;
		cnt++;
		if(!statusRunning&&cnt>20){
			initBB = IntRect(msg.pose1.x-160, msg.pose1.y+30, (msg.pose2.x - msg.pose1.x)*0.7, (msg.pose2.y - msg.pose1.y)*0.7);
			inputBB = IntRect(msg.pose1.x-160, msg.pose1.y+30, (msg.pose2.x - msg.pose1.x)*0.7, (msg.pose2.y - msg.pose1.y)*0.7);
			doInitialise = true;
			statusRunning = true;
		}
		else{
			inputBB = IntRect(msg.pose1.x, msg.pose1.y, msg.pose2.x - msg.pose1.x, msg.pose2.y - msg.pose1.y);
		}
	}
	void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg) 
	{ 
		cout<<"frame"<<endl; 
		cv_bridge::CvImagePtr cv_ptr;
		try{       
		     cv_ptr = cv_bridge::toCvCopy(tem_msg, enc::BGR8); 
		     cv::waitKey(1);
		}    
		catch (cv_bridge::Exception& e)    {
			ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());        
		}
	//	frameOrig = cv_ptr->image.clone();
		cout<<"frame"<<endl;
		process(cv_ptr->image);
	} 

};
StruckTrack::StruckTrack(ros::NodeHandle& nh, const string& configPath)
:conf(configPath)
,tracker(conf)
,result(conf.frameHeight, conf.frameWidth, CV_8UC3)
,statusRunning(false)
,doInitialise(false)
,paused(false)
,it(nh)
{
	srand(conf.seed);
	output_pub = nh.advertise<struck::track_output>("track_output",5);
	input_sub = nh.subscribe("figure_roi",5,&StruckTrack::inputCallback,this);
	sub = it.subscribe("node_a",1,&StruckTrack::imageCallback,this);

//	initBB = IntRect(290, 140, 40, 60);
//	doInitialise = true;
//	statusRunning = true;
}
StruckTrack::~StruckTrack()
{

}
void StruckTrack::process(cv::Mat frameOrig)
{
	float scaleW = 1.f;
	float scaleH = 1.f;
	Mat frame;
	resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
	flip(frame, frame, 1);
	frame.copyTo(result);
	if (doInitialise){
		if (tracker.IsInitialised()){
			tracker.Reset();
		}
		else{
			tracker.Initialise(frame, initBB);
		}
		doInitialise = false;
	}
	else if (!tracker.IsInitialised()){
		myrectangle(result, initBB, CV_RGB(255, 255, 255));
	}
	if (tracker.IsInitialised()){
		tracker.Track(frame);
		if (!conf.quietMode && conf.debugMode)
		{
			tracker.Debug();
		}	
		myrectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
	}
	if (!conf.quietMode){
		imshow("result", result);
		int key = waitKey(paused ? 0 : 1);
	}
	struck::track_output output_msg;
	const FloatRect& bb = tracker.GetBB();
	output_msg.pose1.x = bb.XMin()/scaleW;
	output_msg.pose1.y = bb.YMin()/scaleH;
	output_msg.pose2.x = bb.XMin()/scaleW+bb.Width()/scaleW;
	output_msg.pose2.y = bb.YMin()/scaleH+bb.Height()/scaleH;
	output_msg.vel.x = 0;
	output_msg.vel.y = 0;
	output_pub.publish(output_msg);
}


int main(int argc, char* argv[])
{
	string configPath = argv[1];
	cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
	ros::init(argc, argv, "simcontroller");
	ros::NodeHandle nh;
	StruckTrack st(nh,configPath);
	ros::Rate loop_rate(20);
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return EXIT_SUCCESS;
}

