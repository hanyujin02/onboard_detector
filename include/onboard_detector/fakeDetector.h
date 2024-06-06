/*
	FILE: fakeDetector.h
	---------------------
	fake dynamic obtacle detector for gazebo simulation
*/
#ifndef FAKEDETECTOR_H
#define FAKEDETECTOR_H
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <onboard_detector/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <mutex>
#include <deque>
#include <onboard_detector/regression.h>
#include <onboard_detector/gaussian_process_regression.h>

using std::cout; using std::endl;

namespace onboardDetector{
	class fakeDetector{
	private:
		ros::NodeHandle nh_;
		ros::Timer obstaclePubTimer_;
		ros::Timer visTimer_;
		ros::Timer histTimer_;
		ros::Timer predTimer_;
		ros::Subscriber gazeboSub_;
		ros::Publisher visPub_; // publish bounding box
		ros::Publisher historyTrajPub_; //publish obstacle history
		ros::Publisher predTrajPub_; //publish obstacle prediction
		ros::Subscriber odomSub_;
		
		int histSize_;
		int predSize_;
		std::vector<std::string> targetObstacle_;
		std::vector<int> targetIndex_;
		bool firstTime_;
		std::vector<onboardDetector::box3D> obstacleMsg_;
		std::vector<onboardDetector::box3D> lastObVec_;
		std::vector<ros::Time> lastTimeVec_;
		std::vector<std::vector<double>> lastTimeVel_;
		std::vector<std::deque<onboardDetector::box3D>> obstacleHist_;
		std::vector<std::deque<onboardDetector::box3D>> obstaclePred_;

		// visualization:
		nav_msgs::Odometry odom_;
		double colorDistance_;
		visualization_msgs::MarkerArray visMsg_;

	public:
		fakeDetector(const ros::NodeHandle& nh);

		void visCB(const ros::TimerEvent&);
		void histCB(const ros::TimerEvent&);
		void predCB(const ros::TimerEvent&);
		std::vector<Eigen::Vector3d> getTraj(const std::deque<onboardDetector::box3D> &boxes);
		std::vector<std::deque<onboardDetector::box3D>> evaluate(const std::vector<std::vector<std::deque<onboardDetector::box3D>>> &pred);
		// void evaluate();
		std::vector<std::deque<onboardDetector::box3D>> linearPred();
		std::vector<std::deque<onboardDetector::box3D>> kalmanPred();
		std::vector<std::deque<onboardDetector::box3D>> llsPred(const int &order);
		std::vector<std::deque<onboardDetector::box3D>> gprPred();
		void stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates);
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		std::vector<int>& findTargetIndex(const std::vector<std::string>& modelNames);
		void updateVisMsg();
		void publishObstacles();
		void publishHistoryTraj();
		void publishPredTraj();
		void publishVisualization();
		bool isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov);
		void getObstacles(std::vector<onboardDetector::box3D>& obstacles);
		void getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles);
		void kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox, Eigen::MatrixXd& states, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& H, Eigen::MatrixXd& P, Eigen::MatrixXd& Q, Eigen::MatrixXd& R);
		void getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, const onboardDetector::box3D& prevMatchBBox, Eigen::MatrixXd& Z);
		// void getPredObstacles(std::vector<std::vector<Eigen::Vector3d>> &pos, std::vector<std::vector<Eigen::Vector3d>> &vel, std::vector<std::vector<Eigen::Vector3d>> &size);
	};
}

#endif