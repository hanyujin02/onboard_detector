/*
	FILE: fakeDetector.cpp
	-----------------------
	Function definition of fake detector
*/

#include <onboard_detector/fakeDetector.h>
#include <onboard_detector/kalmanFilter.h>


namespace onboardDetector{
	fakeDetector::fakeDetector(const ros::NodeHandle& nh) : nh_(nh){
		// load ros parameter:
		if (not this->nh_.getParam("target_obstacle", this->targetObstacle_)){
			this->targetObstacle_ = std::vector<std::string> {"person", "obstacle"};
			cout << "[Fake Detector]: No target obstacle param. Use default value." << endl;
		}

		if (not this->nh_.getParam("color_distance", this->colorDistance_)){
			this->colorDistance_ = 5.0; // change color for obstacles in meter distance
			cout << "[Fake Detector]: No color distance param. Use default value: 5.0m." << endl;
		}

		if (not this->nh_.getParam("color_distance", this->colorDistance_)){
			this->colorDistance_ = 5.0; // change color for obstacles in meter distance
			cout << "[Fake Detector]: No color distance param. Use default value: 5.0m." << endl;
		}

		std::string odomTopicName;
		if (not this->nh_.getParam("odom_topic", odomTopicName)){
			odomTopicName = "/CERLAB/quadcopter/odom";
			cout << "[Fake Detector]: No odom topic param. Use default: /CERLAB/quadcopter/odom" << endl;
		}

		// tracking history size
        if (not this->nh_.getParam("history_size", this->histSize_)){
            this->histSize_ = 5;
            std::cout << "[Fake Detector]: No tracking history isze parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << "[Fake Detector]: The history for tracking is set to: " << this->histSize_ << std::endl;
        }  

		// prediction size
        if (not this->nh_.getParam("prediction_size", this->predSize_)){
            this->predSize_ = 5;
            std::cout << "[Fake Detector]: No prediction isze parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << "[Fake Detector]: Prediction size is set to: " << this->predSize_ << std::endl;
        }  

		this->firstTime_ = true;
		this->gazeboSub_ = this->nh_.subscribe("/gazebo/model_states", 10, &fakeDetector::stateCB, this);
		this->odomSub_ = this->nh_.subscribe(odomTopicName, 10, &fakeDetector::odomCB, this);
		// this->odomSub_ = this->nh_.subscribe("/mavros/local_position/odom", 10, &fakeDetector::odomCB, this);
		this->visPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/GT_obstacle_bbox", 10);
		// history trajectory pub
        this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/history_trajectories", 10);
		this->predTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/prediction_trajectories", 10);
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &fakeDetector::visCB, this);
		this->histTimer_ = this->nh_.createTimer(ros::Duration(0.033), &fakeDetector::histCB, this);
		this->predTimer_ = this->nh_.createTimer(ros::Duration(0.1), &fakeDetector::predCB, this);
	}


	void fakeDetector::visCB(const ros::TimerEvent&){
		this->publishHistoryTraj();
		this->publishPredTraj();
		this->publishVisualization();
	}

	void fakeDetector::stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates){
		bool update = false;
		if (this->firstTime_){
			this->targetIndex_ = this->findTargetIndex(allStates->name);
			this->firstTime_ = false;
		}
		
		std::vector<onboardDetector::box3D> obVec;
		onboardDetector::box3D ob;
		geometry_msgs::Pose p;
		geometry_msgs::Twist tw;
		for (int i : this->targetIndex_){
			std::string name = allStates->name[i];

			// 1. get position and velocity
			p = allStates->pose[i];
			tw = allStates->twist[i];
			ob.x = p.position.x;
			ob.y = p.position.y;
			ob.z = p.position.z;

			if (this->lastObVec_.size() == 0){
				ob.Vx = 0.0;
				ob.Vy = 0.0;
				ros::Time lastTime = ros::Time::now();
				this->lastTimeVec_.push_back(lastTime);
				this->lastTimeVel_.push_back(std::vector<double> {0, 0, 0});
				update = true;
			}
	
			else{
				ros::Time currTime = ros::Time::now();
				double dT = (currTime.toSec() - this->lastTimeVec_[i].toSec());
				if (dT >= 0.1){
					double vx = (ob.x - this->lastObVec_[i].x)/dT;
					double vy = (ob.y - this->lastObVec_[i].y)/dT;
					double vz = (ob.z - this->lastObVec_[i].z)/dT;
					ob.Vx = vx;
					ob.Vy = vy;
					this->lastTimeVel_[i][0] = vx;
					this->lastTimeVel_[i][1] = vy;
					this->lastTimeVel_[i][2] = vz;
					this->lastTimeVec_[i] = ros::Time::now();
					update = true;
				}
				else{
					ob.Vx = this->lastTimeVel_[i][0];
					ob.Vy = this->lastTimeVel_[i][1];
				}
			}

			// 2. get size (gazebo name contains size):
			double xsize, ysize, zsize;
			int xsizeStartIdx = name.size() - 1 - 1 - 3 * 3;
			std::string xsizeStr = name.substr(xsizeStartIdx, 3);
			xsize = std::stod(xsizeStr);

			int ysizeStartIdx = name.size() - 1 - 3 * 2;
			std::string ysizeStr = name.substr(ysizeStartIdx, 3);
			ysize = std::stod(ysizeStr);

			int zsizeStartIdx = name.size() - 3;
			std::string zsizeStr = name.substr(zsizeStartIdx, 3);
			zsize = std::stod(zsizeStr);
			
			ob.x_width = xsize;
			ob.y_width = ysize;
			ob.z_width = zsize;
			obVec.push_back(ob);
		}
		if (update){
			this->lastObVec_ = obVec;
		}
		this->obstacleMsg_ = obVec;
		// ros::Rate r (60);
		// r.sleep();
	}

	void fakeDetector::histCB(const ros::TimerEvent&){
		if (this->obstacleHist_.size() == 0){
			this->obstacleHist_.resize(this->obstacleMsg_.size());
		}
		for (int i=0; i<this->obstacleMsg_.size();i++){
			if (this->obstacleHist_[i].size() >= this->histSize_){
				this->obstacleHist_[i].pop_front();
			}
			this->obstacleHist_[i].push_back(this->obstacleMsg_[i]);
		}
	}

	void fakeDetector::predCB(const ros::TimerEvent&){
		// cout<<"pred"<<endl;
		// std::vector<std::deque<onboardDetector::box3D>> pred1 = this->llsPred(1);
		// std::vector<std::deque<onboardDetector::box3D>> pred2 = this->llsPred(3);
		// std::vector<std::deque<onboardDetector::box3D>> pred3 = this->llsPred(4);

		// // this->obstaclePred_.clear();
		// // for (int i=0;i<pred1.size();i++){
		// // 	this->obstaclePred_.push_back(pred1[i]);
		// // }
		// // for (int i=0;i<pred2.size();i++){
		// // 	this->obstaclePred_.push_back(pred2[i]);
		// // }
		// // for (int i=0;i<pred3.size();i++){
		// // 	this->obstaclePred_.push_back(pred3[i]);
		// // }
		// this->obstaclePred_ = this->gprPred();
// 
		// std::vector<std::vector<std::deque<onboardDetector::box3D>>> pred(pred1.size());
		// for (int i=0;i<pred.size();i++){
		// 	pred[i].resize(3);
		// 	for (int j=0;j<pred1[i].size();j++){
		// 		pred[i][0].push_back(pred1[i][j]);
		// 	}
		// 	for (int j=0;j<pred2[i].size();j++){
		// 		pred[i][1].push_back(pred2[i][j]);
		// 	}
		// 	for (int j=0;j<pred3[i].size();j++){
		// 		pred[i][2].push_back(pred3[i][j]);
		// 	}
			
		// }

		// this->obstaclePred_ = this->evaluate(pred);

		// this->obstaclePred_ = this->regressionPred(3);

	}
	std::vector<Eigen::Vector3d> fakeDetector::getTraj(const std::deque<onboardDetector::box3D> &boxes){
		std::vector<Eigen::Vector3d> traj;
		for (int i=0;i<boxes.size();i++){
			Eigen::Vector3d pose;
			pose<<boxes[i].x, boxes[i].y, boxes[i].z;
			traj.push_back(pose);
		}
		return traj;
	}

	std::vector<std::deque<onboardDetector::box3D>> fakeDetector::evaluate(const std::vector<std::vector<std::deque<onboardDetector::box3D>>> &pred){
		std::vector<std::deque<onboardDetector::box3D>> bestPred;
		bestPred.resize(pred.size());
		for (int i=0; i<pred.size();i++){
			for (int j=0; j<pred[i][0].size();j++){
				bestPred[i].push_back(pred[i][0][j]);
			}
		}
		return bestPred;
	}

	std::vector<std::deque<onboardDetector::box3D>> fakeDetector::linearPred(){
		std::vector<std::deque<onboardDetector::box3D>> obstaclePred;
		obstaclePred.clear();
		obstaclePred.resize(this->obstacleMsg_.size());
		// linear
		double dt = 0.1;
		for(int i=0; i<obstaclePred.size();i++){
			onboardDetector::box3D ob = this->obstacleMsg_[i];
			for (int j=0; j<this->predSize_;j++){
				onboardDetector::box3D predOb;
				predOb = ob;
				predOb.x = ob.x+ob.Vx*dt;
				// +ob.Ax*1/2*pow(dt,2);
				predOb.y = ob.y+ob.Vy*dt;
				// +ob.Ay*1/2*pow(dt,2);
				// predOb.Vx = ob.Vx+dt*ob.Ax;
				// predOb.Vy = ob.Vy+dt*ob.Ay;
				obstaclePred[i].push_back(predOb);
				ob = predOb;
			}
		}
		
		return obstaclePred;
	}

	std::vector<std::deque<onboardDetector::box3D>> fakeDetector::llsPred(const int &order){
		std::vector<std::deque<onboardDetector::box3D>> obstaclePred;
		// int order = 3;
		obstaclePred.clear();
		obstaclePred.resize(this->obstacleMsg_.size());
		double dt = 0.1;
		double lambda = 1;
		for (int i=0; i<this->obstacleHist_.size();i++){
			onboardDetector::box3D ob = this->obstacleMsg_[i];
			int numberPoints = this->obstacleHist_[i].size();
			Eigen::MatrixXd x,y,t;
			
			x.resize(numberPoints,1);
			y.resize(numberPoints,1);
			t.resize(numberPoints,1);
			for (int j = 0; j < numberPoints; j++)
			{
				x(j,0) = this->obstacleHist_[i][j].x;
				y(j,0) = this->obstacleHist_[i][j].y;
				t(j,0) = j*dt;
			}
			Eigen::MatrixXd X;
			X.resize(this->predSize_, 1);
			for (int j=0;j<this->predSize_;j++){
				X(j,0) = (j+numberPoints)*dt;
			}

			LLS regressor;
			Eigen::MatrixXd predx = regressor.pred(t,x,X,order);
			Eigen::MatrixXd predy = regressor.pred(t,y,X,order);

			for (int j=0; j<predx.rows();j++){
				onboardDetector::box3D predOb;
				predOb = ob;
				if (predOb.x<=40 and predOb.y <=40){
					predOb.x = predx(j,0);
					// +ob.Ax*1/2*pow(dt,2);
					predOb.y = predy(j,0);
				}
				// +ob.Ay*1/2*pow(dt,2);
				// predOb.Vx = ob.Vx+dt*ob.Ax;
				// predOb.Vy = ob.Vy+dt*ob.Ay;
				obstaclePred[i].push_back(predOb);
			}
			
		}	
		return obstaclePred;
	}

	std::vector<std::deque<onboardDetector::box3D>> fakeDetector::gprPred(){
		std::vector<std::deque<onboardDetector::box3D>> obstaclePred;

		// int order = 3;
		obstaclePred.clear();
		obstaclePred.resize(this->obstacleMsg_.size());
		double dt = 0.1;
		double lambda = 1;
		int inputDim = 1;
		int outputDim = 1;
		for (int i=0; i<this->obstacleHist_.size();i++){
			onboardDetector::box3D ob = this->obstacleMsg_[i];
			int numberPoints = this->obstacleHist_[i].size();
			GaussianProcessRegression<double> gprx(inputDim,outputDim);
			GaussianProcessRegression<double> gpry(inputDim,outputDim);
			gprx.ClearTrainingData();
			gpry.ClearTrainingData();
			gprx.SetHyperParams(1.1,1.0,0.4);
			gpry.SetHyperParams(1.1,1.0,0.4);
			Eigen::MatrixXd input, outputx, outputy, testInput;
			input.resize(inputDim, 1);
			outputx.resize(outputDim, 1);
			outputy.resize(outputDim, 1);

			for (int j=0;j<this->obstacleHist_[i].size();j++){
				input(0,0) = j*dt;
				outputx(0,0) = this->obstacleHist_[i][j].x;
				outputy(0,0) = this->obstacleHist_[i][j].y;
				gprx.AddTrainingData(input,outputx);
				gpry.AddTrainingData(input,outputy);
			}

			// gpr.AddTrainingData(input, output);
			// cout<<"train data added"<<endl;
			testInput.resize(inputDim, 1);
			for (int j=0; j<numberPoints+this->predSize_;j++){
				onboardDetector::box3D predOb;
				predOb = ob;
				testInput(0,0) = j*dt;
				auto testOutputx = gprx.DoRegression(testInput);
				auto testOutputy = gpry.DoRegression(testInput);
				// cout<<"output size:"<<testOutput.rows()<<","<<testOutput.cols()<<endl;
				predOb.x = testOutputx(0,0);
				predOb.y = testOutputy(0,0);
				obstaclePred[i].push_back(predOb);
			}
			// auto testOutput = gpr.DoRegression(testInput);
			// cout<<"output size:"<<testOutput.rows()<<","<<testOutput.cols()<<endl;
			// ASSERT_EQ(1, gpr.get_n_data());
			// assert_matrix_equal<float>(gpr.get_input_data(),input,1e-5);
			// assert_matrix_equal<float>(gpr.get_output_data(),output,1e-5);


			// for (int j=0; j<predx.rows();j++){
			// 	onboardDetector::box3D predOb;
			// 	predOb = ob;
			// 	if (predOb.x<=40 and predOb.y <=40){
			// 		predOb.x = predx(j,0);
			// 		// +ob.Ax*1/2*pow(dt,2);
			// 		predOb.y = predy(j,0);
			// 	}
			// 	// +ob.Ay*1/2*pow(dt,2);
			// 	// predOb.Vx = ob.Vx+dt*ob.Ax;
			// 	// predOb.Vy = ob.Vy+dt*ob.Ay;
			// 	obstaclePred[i].push_back(predOb);
			// }
			
		}	
		return obstaclePred;
	}


	std::vector<std::deque<onboardDetector::box3D>> fakeDetector::kalmanPred(){
		std::vector<std::deque<onboardDetector::box3D>> obstaclePred;
		obstaclePred.clear();
		obstaclePred.resize(this->obstacleMsg_.size());
		onboardDetector::kalman_filter filter;
		for (int i=0;i<obstaclePred.size();i++){
			onboardDetector::box3D ob = this->obstacleMsg_[i];
			onboardDetector::box3D prevOb;
			if (this->obstacleHist_[i].size() != 0){
				prevOb = this->obstacleHist_[i].back();
			}
			else{
				prevOb = this->obstacleMsg_[i];
			}
			for(int j=0;j<this->predSize_;j++){
				Eigen::MatrixXd states, A, B, H, P, Q, R, Z;
				this->kalmanFilterMatrixAcc(ob,states, A, B, H, P, Q, R);
				filter.setup(states, A, B, H, P, Q, R);
				this->getKalmanObservationAcc(ob, prevOb, Z);
				filter.estimate(Z,MatrixXd::Zero(6,1));
				onboardDetector::box3D predOb;
				predOb.x = filter.output(0);
                predOb.y = filter.output(1);
                predOb.z = ob.z;
                predOb.Vx = filter.output(2);
                predOb.Vy = filter.output(3);
                predOb.Ax = filter.output(4);
                predOb.Ay = filter.output(5);    
				predOb.x_width = ob.x_width;
                predOb.y_width = ob.y_width;
                predOb.z_width = ob.z_width;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
				obstaclePred[i].push_back(predOb);
				prevOb = ob;
				ob = predOb;
			}
		}
		return obstaclePred;
	}


	void fakeDetector::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
	}

	std::vector<int>& fakeDetector::findTargetIndex(const std::vector<std::string>& modelNames){
		static std::vector<int> targetIndex;
		int countID = 0;
		for (std::string name : modelNames){
			for (std::string targetName : this->targetObstacle_){
				if (name.compare(0, targetName.size(), targetName) == 0){
					targetIndex.push_back(countID);
				}
			}
			++countID;
		}
		return targetIndex;
	}

	void fakeDetector::updateVisMsg(){
		std::vector<visualization_msgs::Marker> bboxVec;
		int obIdx = 0;
		for (onboardDetector:: box3D obstacle : this->obstacleMsg_){

			// 12 lines for each obstacle
			geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
			// upper four points
			p1.x = obstacle.x+obstacle.x_width/2; p1.y = obstacle.y+obstacle.y_width/2; p1.z = obstacle.z+obstacle.z_width;
			p2.x = obstacle.x-obstacle.x_width/2; p2.y = obstacle.y+obstacle.y_width/2; p2.z = obstacle.z+obstacle.z_width;
			p3.x = obstacle.x+obstacle.x_width/2; p3.y = obstacle.y-obstacle.y_width/2; p3.z = obstacle.z+obstacle.z_width;
			p4.x = obstacle.x-obstacle.x_width/2; p4.y = obstacle.y-obstacle.y_width/2; p4.z = obstacle.z+obstacle.z_width;

			p5.x = obstacle.x+obstacle.x_width/2; p5.y = obstacle.y+obstacle.y_width/2; p5.z = obstacle.z;
			p6.x = obstacle.x-obstacle.x_width/2; p6.y = obstacle.y+obstacle.y_width/2; p6.z = obstacle.z;
			p7.x = obstacle.x+obstacle.x_width/2; p7.y = obstacle.y-obstacle.y_width/2; p7.z = obstacle.z;
			p8.x = obstacle.x-obstacle.x_width/2; p8.y = obstacle.y-obstacle.y_width/2; p8.z = obstacle.z;

			std::vector<geometry_msgs::Point> line1Vec {p1, p2};
			std::vector<geometry_msgs::Point> line2Vec {p1, p3};
			std::vector<geometry_msgs::Point> line3Vec {p2, p4};
			std::vector<geometry_msgs::Point> line4Vec {p3, p4};
			std::vector<geometry_msgs::Point> line5Vec {p1, p5};
			std::vector<geometry_msgs::Point> line6Vec {p2, p6};
			std::vector<geometry_msgs::Point> line7Vec {p3, p7};
			std::vector<geometry_msgs::Point> line8Vec {p4, p8};
			std::vector<geometry_msgs::Point> line9Vec {p5, p6};
			std::vector<geometry_msgs::Point> line10Vec {p5, p7};
			std::vector<geometry_msgs::Point> line11Vec {p6, p8};
			std::vector<geometry_msgs::Point> line12Vec {p7, p8};

			std::vector<std::vector<geometry_msgs::Point>> allLines{
				line1Vec,
				line2Vec,
				line3Vec,
				line4Vec,
				line5Vec,
				line6Vec,
				line7Vec,
				line8Vec,
				line9Vec,
				line10Vec,
				line11Vec,
				line12Vec
			};

			int count = 0;
			std::string name = "GT osbtacles" + std::to_string(obIdx);
			for (std::vector<geometry_msgs::Point> lineVec: allLines){
				visualization_msgs::Marker line;

				line.header.frame_id = "map";
				line.ns = name;
				line.points = lineVec;
				line.id = count;
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.lifetime = ros::Duration(0.5);
				line.scale.x = 0.05;
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.a = 1.0;
				if (this->isObstacleInSensorRange(obstacle, PI_const)){
					line.color.r = 1;
					line.color.g = 0;
					line.color.b = 0;
				}
				else{
					line.color.r = 0;
					line.color.g = 1;
					line.color.b = 0;
				}
				++count;
				bboxVec.push_back(line);
			}
			++obIdx;
		}
		this->visMsg_.markers = bboxVec;
	}

	void fakeDetector::publishHistoryTraj(){
		if (this->obstacleHist_.size() != 0){
			visualization_msgs::MarkerArray trajMsg;
			int countMarker = 0;
			for (size_t i=0; i<this->obstacleHist_.size(); ++i){
				visualization_msgs::Marker traj;
				traj.header.frame_id = "map";
				traj.header.stamp = ros::Time::now();
				traj.ns = "fake_detector";
				traj.id = countMarker;
				traj.type = visualization_msgs::Marker::LINE_STRIP;
				traj.scale.x = 0.1;
				traj.scale.y = 0.1;
				traj.scale.z = 0.1;
				traj.color.a = 1.0; 
				traj.color.r = 0.0;
				traj.color.g = 1.0;
				traj.color.b = 0.0;
				for (size_t j=0; j<this->obstacleHist_[i].size(); ++j){
					geometry_msgs::Point p1;
					onboardDetector::box3D box1 = this->obstacleHist_[i][j];
					p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
					traj.points.push_back(p1);
				}

				++countMarker;
				trajMsg.markers.push_back(traj);
			}
			this->historyTrajPub_.publish(trajMsg);
		}
	}

	void fakeDetector::publishPredTraj(){
		if (this->obstaclePred_.size() != 0){
			visualization_msgs::MarkerArray trajMsg;
			int countMarker = 0;
			for (size_t i=0; i<this->obstaclePred_.size(); ++i){
				visualization_msgs::Marker traj;
				traj.header.frame_id = "map";
				traj.header.stamp = ros::Time::now();
				traj.ns = "fake_detector";
				traj.id = countMarker;
				traj.type = visualization_msgs::Marker::LINE_STRIP;
				traj.scale.x = 0.1;
				traj.scale.y = 0.1;
				traj.scale.z = 0.1;
				traj.color.a = 1.0;
				traj.color.r = 1.0;
				traj.color.g = 0.0;
				traj.color.b = 0.0;
				for (size_t j=0; j<this->obstaclePred_[i].size(); ++j){
					geometry_msgs::Point p1, p2;
					onboardDetector::box3D box1 = this->obstaclePred_[i][j];
					p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
					traj.points.push_back(p1);
				}

				++countMarker;
				trajMsg.markers.push_back(traj);
			}
			this->predTrajPub_.publish(trajMsg);
		}
	}

	void fakeDetector::publishVisualization(){
		this->updateVisMsg();
		this->visPub_.publish(this->visMsg_);
	}

	bool fakeDetector::isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov){
		Eigen::Vector3d pRobot (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d pObstacle (ob.x, ob.y, ob.z);	
		
		Eigen::Vector3d diff = pObstacle - pRobot;
		diff(2) = 0.0;
		double distance = diff.norm();
		double yaw = rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);

		double angle = angleBetweenVectors(direction, diff);
		if (angle <= fov/2 and distance <= this->colorDistance_){
			return true;
		}
		else{
			return false;
		}
	
	}

	void fakeDetector::getObstacles(std::vector<onboardDetector::box3D>& obstacles){
		obstacles = this->obstacleMsg_;
	}

	void fakeDetector::getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles){
		obstacles.clear();
		for (onboardDetector::box3D obstacle : this->obstacleMsg_){
			if (this->isObstacleInSensorRange(obstacle, fov)){
				obstacles.push_back(obstacle);
			}
		}
	}

	// Test: kalman filter state estimation, constant acceleration
    void fakeDetector::kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox, Eigen::MatrixXd& states, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& H, Eigen::MatrixXd& P, Eigen::MatrixXd& Q, Eigen::MatrixXd& R){
		double eQPos = 0.5;
		double eQVel = 0.5;
		double eQAcc = 0.5;
		double eRPos = 0.5;
		double eRVel = 0.5;
		double eRAcc = 0.5;
		double eP = 0.5;
		double dt = 0.1;

		states.resize(6,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        // init vel and acc to zeros
        states(2) = currDetectedBBox.Vx;
        states(3) = currDetectedBBox.Vy;
        states(4) = currDetectedBBox.Ax;
        states(5) = currDetectedBBox.Ay;

        MatrixXd ATemp;
        ATemp.resize(6, 6);

        ATemp <<  1, 0, dt, 0, 0.5*pow(dt, 2), 0,
                  0, 1, 0, dt, 0, 0.5*pow(dt, 2),
                  0, 0, 1, 0, dt, 0,
                  0 ,0, 0, 1, 0, dt,
                  0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 1;
        A = ATemp;
        B = Eigen::MatrixXd::Zero(6, 6);
        H = Eigen::MatrixXd::Identity(6, 6);
        P = Eigen::MatrixXd::Identity(6, 6) * eP;
        Q = Eigen::MatrixXd::Identity(6, 6);
        Q(0,0) *= eQPos; Q(1,1) *= eQPos; Q(2,2) *= eQVel; Q(3,3) *= eQVel; Q(4,4) *= eQAcc; Q(5,5) *= eQAcc;
        R = Eigen::MatrixXd::Identity(6, 6);
        R(0,0) *= eRPos; R(1,1) *= eRPos; R(2,2) *= eRVel; R(3,3) *= eRVel; R(4,4) *= eRAcc; R(5,5) *= eRAcc;
    }

    void fakeDetector::getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, const onboardDetector::box3D& prevMatchBBox, Eigen::MatrixXd& Z){
        Z.resize(6, 1);
        Z(0) = currDetectedBBox.x;
        Z(1) = currDetectedBBox.y;
        // // use previous k frame for velocity estimation
        // int k = this->kfAvgFrames_;
        // int historySize = this->boxHist_[bestMatchIdx].size();
        // if (historySize < k){
        //     k = historySize;
        // }
        // onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];
        double dt = 0.1;
        // Z(2) = (currDetectedBBox.x - prevMatchBBox.x)/dt;
        // Z(3) = (currDetectedBBox.y - prevMatchBBox.y)/dt;
		Z(2) = currDetectedBBox.Vx;
		Z(3) = currDetectedBBox.Vy;
        Z(4) = (Z(2) - prevMatchBBox.Vx)/dt;
        Z(5) = (Z(3) - prevMatchBBox.Vy)/dt;
    }
	// void fakeDetector::getPredObstacles(std::vector<std::vector<Eigen::Vector3d>> &pos, std::vector<std::vector<Eigen::Vector3d>> &vel, std::vector<std::vector<Eigen::Vector3d>> &size){
	// 	pos.resize(this->obstaclePred_.size());
	// 	vel.resize(this->obstaclePred_.size());
	// 	size.resize(this->obstaclePred_.size());
	// 	for (int i=0; i<this->obstaclePred_.size();i++){
	// 		for (int j=0; j<this->obstaclePred_[i].size();j++){
	// 			Eigen::Vector3d obPos, obVel, obSize;
	// 			obPos << this->obstaclePred_[i][j].x, this->obstaclePred_[i][j].y, this->obstaclePred_[i][j].z+this->obstaclePred_[i][j].z_width/2;
	// 			obVel << this->obstaclePred_[i][j].Vx, this->obstaclePred_[i][j].Vy, 0.0;
	// 			obSize << this->obstaclePred_[i][j].x_width, this->obstaclePred_[i][j].y_width, this->obstaclePred_[i][j].z_width;
	// 			pos[i].push_back(obPos);
	// 			vel[i].push_back(obVel);
	// 			size[i].push_back(obSize);
	// 		}
	// 	}
		
	// }
}