#include "ros/ros.h"
#include <controls/pose.h>

#include <iostream>
#include <fstream>
double get(const ros::NodeHandle& n,const std::string& name) 
{
    double value;
    n.getParam(name, value);
    return value;
}
class PID
{
private:
	
public:
	float m_kp;
	float m_kd;
	float m_ki;
	float m_kpp;
	float m_ff;
	float m_minOutput;
	float m_maxOutput;
	float m_integratorMin;
	float m_integratorMax;
	float m_integral;
	float m_previousError;
	ros::Time m_previousTime;
	PID(
		float kp,
		float kd,
		float ki,
		float kpp,
		float ff,
		float minOutput,
		float maxOutput,
		float integratorMin,
		float integratorMax)
		: m_kp(kp)
		, m_kd(kd)
		, m_ki(ki)
		, m_kpp(kpp)
		, m_ff(ff)
		, m_minOutput(minOutput)
		, m_maxOutput(maxOutput)
		, m_integratorMin(integratorMin)
		, m_integratorMax(integratorMax)
		, m_integral(0)
		, m_previousError(0)
		, m_previousTime(ros::Time::now())
	{
	}

	void reset()
	{
		m_integral = 0;
		m_previousError = 0;
		m_previousTime = ros::Time::now();
	}

	void setIntegral(float integral)
	{
		m_integral = integral;
	}

	float ki() const
	{
		return m_ki;
	}
	float ff() const
	{
		return m_ff;
	}
	float pid_update(float est, float setpt)
	{
		ros::Time time = ros::Time::now();
		float dt = time.toSec() - m_previousTime.toSec();
		float error = setpt - est;
		m_integral += error * dt;
		m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
		float p = m_kp * error;
		float d = 0;
		if (dt > 0){
			d = m_kd * (error - m_previousError) / dt;
		}
		float i = m_ki * m_integral;
		float output = p + d + i;
		m_previousError = error;
		m_previousTime = time;
		return std::max(std::min(output, m_maxOutput), m_minOutput);
	}
	float pp_update(float est, float setpt)
	{
		float error = setpt - est;
		float output = m_kpp * error;
		return output;
	}
};
class Controller
{
private:
	ros::Subscriber pose_sub;
	float FigurHeight;
	float FigurOffset;
	float FaceAngle;
	PID pidX;
	PID pidY;
	PID pidYaw;
public:
	Controller(ros::NodeHandle& nh);
	~Controller();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void poseCallback(const controls::pose msg)
	{
		FigurHeight = msg.FigurHeight;
		FigurOffset = msg.FigurOffset;
		FaceAngle = msg.FaceAngle;
	}
};
Controller::Controller(ros::NodeHandle& n)
:pidX(
		get(n, "PIDs/X/kp"),
		get(n, "PIDs/X/kd"),
		get(n, "PIDs/X/ki"),
		get(n, "PIDs/X/kpp"),
		get(n, "PIDs/X/ff"),
		get(n, "PIDs/X/minOutput"),
		get(n, "PIDs/X/maxOutput"),
		get(n, "PIDs/X/integratorMin"),
		get(n, "PIDs/X/integratorMax"))
,pidY(
		get(n, "PIDs/Y/kp"),
		get(n, "PIDs/Y/kd"),
		get(n, "PIDs/Y/ki"),
		get(n, "PIDs/Y/kpp"),
		get(n, "PIDs/Y/ff"),
		get(n, "PIDs/Y/minOutput"),
		get(n, "PIDs/Y/maxOutput"),
		get(n, "PIDs/Y/integratorMin"),
		get(n, "PIDs/Y/integratorMax"))
,pidYaw(
		get(n, "PIDs/Z/kp"),
		get(n, "PIDs/Z/kd"),
		get(n, "PIDs/Z/ki"),
		get(n, "PIDs/Z/kpp"),
		get(n, "PIDs/Z/ff"),
		get(n, "PIDs/Z/minOutput"),
		get(n, "PIDs/Z/maxOutput"),
		get(n, "PIDs/Z/integratorMin"),
		get(n, "PIDs/Z/integratorMax"))
{
	pose_sub = n.subscribe("pose",5,&Controller::poseCallback,this);

	std::cout<<pidY.m_kp;
	std::cout<<"hello"<<std::endl;
}
Controller::~Controller()
{}
void Controller::run(double freq)
{
	ros::NodeHandle node;
	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Controller::iteration, this);
	ros::spin();
}
void Controller::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;

}
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n("~");
	Controller controller(n);
	controller.run(20);
	
}