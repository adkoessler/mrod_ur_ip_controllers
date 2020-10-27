/*
 *  A ROS package to....
 *  Adrien
 *   
 *   
 *   
 * 
*/

#include <vel_based_shape_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

#define TRACE_ACTIVATED 1


namespace mrod_ur_ip_controllers 
{
    VelBasedShapeController::VelBasedShapeController() {}
    VelBasedShapeController::~VelBasedShapeController() {}
    
    bool test = true;
    
    bool VelBasedShapeController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
    {
		 
		my_log_file_.open("/home/test/my_controller_log.txt", std::ios::out | std::ios::binary);
		
		my_log_file_ << "Before KinematicChainControllerBase !\n";
		
		ROS_INFO("***** START VelBasedShapeController::init ************");

        if( !(KinematicChainControllerBase<hardware_interface::VelocityJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize VelBasedShapeController controller.");
            return false;
        }
        
        my_log_file_ << "After KinematicChainControllerBase !\n"; 
        
        
        
        // setup the kinematic chain and solvers
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        q_dot_cmd_.resize(kdl_chain_.getNrOfJoints());
       
        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

		// create publishers and subscribers
        //sub_master_pose_ = nh_.subscribe("x_master", 1, &VelBasedShapeController::masterPoseCB, this);
        //sub_master_velocity_ = nh_.subscribe("x_dot_master", 1, &VelBasedShapeController::masterVelocityCB, this);
        sub_desired_shape_ = nh_.subscribe("desired_shape", 1, &VelBasedShapeController::desiredShapeCB, this);
        sub_vision_shape_ = nh_.subscribe("vision_shape", 1, &VelBasedShapeController::visionShapeCB, this);
        realtime_x_dot_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(n, "x_dot_slave", 4));
        realtime_x_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(n, "x_slave", 4));
        
        // set model parameters
        A_ = .001; //section_area_;
		E_ = 2100000.0; //youngs_modulus_;
		L0_ = ((double).8)/((double)element_count_); //default_length_;
		connectivity_matrix_.resize(element_count_,2);
		connectivity_matrix_.col(0)  << 0,1,2,3,4,5,6,7,8,9; 
		connectivity_matrix_.col(1)  << 1,2,3,4,5,6,7,8,9,10; 

		// fill indexes
		vector_master_.resize(dof_count_*index_master_.size());
		vector_slave_.resize(dof_count_*index_slave_.size());
		vector_target_.resize(dof_count_*index_target_.size());
		vector_free_.resize(dof_count_*index_free_.size());
		vector_perceived_.resize(dof_count_*index_perceived_.size());
		vector_estimated_.resize(dof_count_*index_estimated_.size());
		indexToCoords(index_master_,vector_master_);
		indexToCoords(index_slave_,vector_slave_);
		indexToCoords(index_target_,vector_target_);
		indexToCoords(index_free_,vector_free_);
		indexToCoords(index_perceived_,vector_perceived_);
		indexToCoords(index_estimated_,vector_estimated_);
		
		//testStructure();
		//testConversions();
		//testSubmatrix();
		
		// Transforms
		/*cam_in_base_frame_, slave_xy_in_base_frame_, master_xy_in_base_frame_, master_base_in_base_frame_;
		// coords of "camera_frame" in "base_link" - the camera postion is not known beforehand
		cam_in_base_frame_.setOrigin( tf::Vector3(1.0, 1.0, 1.0) );
		tf::Quaternion q;
		q.setRPY(1.2, 0, 0);
		cam_in_base_frame_.setRotation(q);
		br_.sendTransform(tf::StampedTransform(cam_in_base_frame_, ros::Time::now(), "base_link", "camera_frame"));
		// coords of "slave_xy_frame" in "base_link" - just pi/2 rotation so that yOz becomes xOy
		slave_xy_in_base_frame_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		q.setRPY(1.57076, 0, 1.57076);
		slave_xy_in_base_frame_.setRotation(q);
		br_.sendTransform(tf::StampedTransform(slave_xy_in_base_frame_, ros::Time::now(), "base_link", "slave_xy_frame"));
		// coords of "master_xy_frame" in "base_link" - aligned with "slave_xy_frame" but origin located at master's base origin
		master_xy_in_base_frame_.setOrigin( tf::Vector3(0.0, 1.6, 0.0) );
		q.setRPY(1.57076, 0, 1.57076);
		master_xy_in_base_frame_.setRotation(q);
		br_.sendTransform(tf::StampedTransform(master_xy_in_base_frame_, ros::Time::now(), "base_link", "master_xy_frame"));
		// coords of "master_base_link" in "base_link" - every coord is possibly non-zero
		master_base_in_base_frame_.setOrigin( tf::Vector3(0.0, 1.6, 0.0) );
		q.setRPY(0, 0, 0.1);
		master_base_in_base_frame_.setRotation(q);
		br_.sendTransform(tf::StampedTransform(master_base_in_base_frame_, ros::Time::now(), "base_link", "master_base_link"));*/
		
		//testFrameTransform();
		//testListOperations();
		//testTwistTransform();
		
		// time values for motion generation
		ti_ = 10;
		tf_ = 20;
		
		// x_dot_master_ setup - initial guess is null velocity
		x_dot_master_.resize(1);
		tf::TwistKDLToMsg(KDL::Twist::Zero(), x_dot_master_[0]);
		
		// x_dot_des_ setup - initial command is null velocity
		x_dot_des_.resize(1);
		tf::TwistKDLToMsg(KDL::Twist::Zero(), x_dot_des_[0]);
		
		// shapes setup - all according to initial_shape_
		fillInitialShape(initial_shape_);
		// desired_shape_ = elemnts of initial_shape_
		desired_shape_.resize(index_target_.size());
		reduceList(initial_shape_, desired_shape_, index_target_);
		target_shape_.resize(index_target_.size());
		target_shape_ = desired_shape_;
		initial_target_shape_.resize(index_target_.size());
		initial_target_shape_ = target_shape_;
		
		// x_master_ setup from shape
		x_master_.resize(1);
		x_master_[0].position.x = initial_shape_[0].position.x;
		x_master_[0].position.y = initial_shape_[0].position.y;
		
		// perception = elemnts of initial_shape_
		whole_shape_.resize(node_count_);
		whole_shape_ = initial_shape_;
		estimated_shape_.resize(index_estimated_.size());
		reduceList(initial_shape_, estimated_shape_, index_estimated_);
		perceived_shape_.resize(index_perceived_.size());
		reduceList(initial_shape_, perceived_shape_, index_perceived_);
		vision_shape_.resize(index_vision_.size());
		reduceList(initial_shape_, vision_shape_, index_vision_);
		vision_previous_shape_.resize(index_vision_.size());
		reduceList(initial_shape_, vision_previous_shape_, index_vision_);
		// perceived_velocity_ = fill with zeros;
		geometry_msgs::Twist tz;
		tf::TwistKDLToMsg(KDL::Twist::Zero(), tz);
		for(int i=0; i<index_perceived_.size(); i++) perceived_velocity_.push_back(tz);
		for(int i=0; i<index_vision_.size(); i++) vision_velocity_.push_back(tz);
		for(int i=0; i<index_vision_.size(); i++) vision_previous_velocity_.push_back(tz);
		
		// structural rigidity matrix - according to initial_shape_
		K_struct_.resize(node_count_*dof_count_,node_count_*dof_count_);
		structuralRigidity(K_struct_, initial_shape_, connectivity_matrix_);
		
		// proportional gain matrix as unit matrix
		Gp_.resize(vector_target_.rows(),vector_target_.rows());
		Gp_.setIdentity();
		Gp_ *= 0.01; // set proportional gain to 0.01
		
		//testElemental();
		//testStructural();
		
		/*x_ = KDL::Frame::Identity();		//current pose
		x_frame_dot_current_ = KDL::FrameVel::Identity();
		tf::twistKDLToMsg(x_frame_dot_current_.GetTwist(), x_dot_msg_);
		tf::poseKDLToMsg(x_, x_msg_);*/
		
		x_msg_ = whole_shape_[node_count_-1];
		x_dot_msg_ = tz;
		
		//testControlLaw();
		
		for (int i = 0; i < index_target_.size(); i++){
			ROS_INFO_STREAM("At init, target node " << i << ", x=" << target_shape_[i].position.x << ", y=" << target_shape_[i].position.y);
		}
		ROS_INFO_STREAM("At init, vector target: " << vector_target_);
        ROS_INFO("***** FINISH VelBasedShapeController::init ************");
	    
        my_log_file_.close();

        return true;
    }

    void VelBasedShapeController::starting(const ros::Time& time)
    {
		ROS_INFO("***** VelBasedShapeController::starting ************");

		// Initialising the current time (for debug)
		current_time_ = ros::Time::now();
		last_time_ = ros::Time::now();
		previous_time_ = ros::Time::now();
		starting_time_ = ros::Time::now();
		
		// Run solvers to initialize values
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        q_dot_current_.q.resize(kdl_chain_.getNrOfJoints());
        q_dot_current_.qdot.resize(kdl_chain_.getNrOfJoints());
        q_dot_current_.q = joint_msr_states_.q;
        q_dot_current_.qdot = joint_msr_states_.qdot;
        fk_vel_solver_->JntToCart(q_dot_current_, x_frame_dot_current_);
        
        // Apply null articular velocity command
        KDL::SetToZero(q_dot_cmd_);

		// Setting flag command to 0 to not run the update method
        target_set_ = 0;
        
        ROS_INFO("***** FINISH VelBasedShapeController::starting ************");
    }

	void VelBasedShapeController::stopping(const ros::Time& time)
	{
		ROS_INFO("***** VelBasedShapeController::stopping ************");
		
		target_set_ = 0;

	}

    void VelBasedShapeController::update(const ros::Time& time, const ros::Duration& period)
    {
		if(target_set_){
			// ******** 1. Get values from subscribed topics *********
			// set time
			previous_time_ = current_time_;
			current_time_ = ros::Time::now();
			// read topics
			//x_master_ptr_ = x_master_buffer_.readFromRT();
			//x_dot_master_ptr_ = x_dot_master_buffer_.readFromRT();
			vision_shape_ptr_ = vision_shape_buffer_.readFromRT();
			//x_master_ = *x_master_ptr_;
			//x_dot_master_ = *x_dot_master_ptr_;
			vision_previous_shape_ = vision_shape_;
			vision_previous_velocity_ = vision_velocity_;
			vision_shape_ = *vision_shape_ptr_;
			desired_shape_ptr_ = desired_shape_buffer_.readFromRT();
			desired_shape_ = *desired_shape_ptr_;
			
			// ******** 2. Set current shape target *********
			setTarget(initial_target_shape_, desired_shape_, target_shape_, x_master_, ti_, tf_);
			
			// ******** 3. Perceive and estimate all node coordinates *********
			// emulate values for testing purposes
			//updateAll();
			// get velocity from vision_shape_
			visionVelocity(vision_velocity_, vision_previous_velocity_, vision_shape_, vision_previous_shape_);
			// set proprioception values in "slave_xy_frame" and merge to get values of perceived_shape_ and perceived_velocity_
			shapePerception(perceived_shape_, perceived_velocity_);
			shapeEstimation(estimated_shape_, perceived_shape_, perceived_velocity_, K_struct_);
			shapeMerger(estimated_shape_, perceived_shape_, whole_shape_);
			
			// ******** 4. Update tangent stiffness matrix *********
			structuralRigidity(K_struct_, whole_shape_, connectivity_matrix_);
			
			// ******** 5. Compute control output *********
			controlLaw(whole_shape_, target_shape_, K_struct_, x_dot_master_, x_dot_des_);
			
			// ******** 6. Send commands *********
			
			// Get current joint positions
			for(int i=0; i < joint_handles_.size(); i++){
				joint_msr_states_.q(i) = joint_handles_[i].getPosition();
				joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			}
			// Reading realtime buffer without to be blocked.
			KDL::Twist slave_twist;
			geometry_msgs::Twist x_dot_slave_base;
			linearTwistFrameTransform(x_dot_des_[0], x_dot_slave_base, "slave_xy_frame", "base_link");
			tf::TwistMsgToKDL(x_dot_slave_base, slave_twist);
			// computing forward kinematics
			fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
			q_dot_current_.q = joint_msr_states_.q;
			q_dot_current_.qdot = joint_msr_states_.qdot;
			fk_vel_solver_->JntToCart(q_dot_current_, x_frame_dot_current_);
			ik_vel_solver_->CartToJnt(q_dot_current_.q, slave_twist, q_dot_cmd_);
			// set command to joints
			for (int i = 0; i < joint_handles_.size(); i++){
				joint_handles_[i].setCommand(q_dot_cmd_.data(i));
			}
			if (realtime_x_dot_pub_->trylock()){
				tf::twistKDLToMsg(x_frame_dot_current_.GetTwist(), x_dot_msg_);
				realtime_x_dot_pub_->msg_ = x_dot_msg_;
				realtime_x_dot_pub_->unlockAndPublish();
			}
			if (realtime_x_pub_->trylock()){
				tf::poseKDLToMsg(x_, x_msg_);
				realtime_x_pub_->msg_ = x_msg_; 
				realtime_x_pub_->unlockAndPublish();
			}
			
		}else{
			/*#if TRACE_ACTIVATED
				current_time_ = ros::Time::now();
				ros::Duration elapsed_time = current_time_ - last_time_;
				if (elapsed_time.toSec() >= 1.0){
					ROS_INFO("****** VelBasedShapeController::update ******");
					ROS_INFO("*** No target has been set for the control law ***");
					last_time_ = current_time_;
				}
			#endif*/
		}
		// prints
		#if TRACE_ACTIVATED
			current_time_ = ros::Time::now();
			ros::Duration elapsed_time = current_time_ - last_time_;
			if (elapsed_time.toSec() >= 1.0){
				
				if(!target_set_){
					ROS_INFO("****** VelBasedShapeController::update ******");
					ROS_INFO("*** No target has been set for the control law ***");
				}
				
				ROS_INFO_STREAM("At time t=" << (current_time_-starting_time_).toSec());
				
				// for testing step 2
				for (int i = 0; i < index_target_.size(); i++){
				  ROS_INFO_STREAM("Node " << index_target_[i] << ", x=" << initial_shape_[index_target_[i]].position.x << ", y=" << initial_shape_[index_target_[i]].position.y);
				}
				for (int i = 0; i < index_target_.size(); i++){
				  ROS_INFO_STREAM("Desired node " << i << ", x=" << desired_shape_[i].position.x << ", y=" << desired_shape_[i].position.y);
				}
				for (int i = 0; i < index_target_.size(); i++){
				  ROS_INFO_STREAM("Target node " << i << ", x=" << target_shape_[i].position.x << ", y=" << target_shape_[i].position.y);
				}
				ROS_INFO_STREAM("Master pose x=" << x_master_[0].position.x << ", y=" << x_master_[0].position.y);
				
				// for testing step 3 and 4
				for (int i = 0; i < index_perceived_.size(); i++){
				  ROS_INFO_STREAM("perceived node " << i << ", x=" << perceived_shape_[i].position.x << ", y=" << perceived_shape_[i].position.y);
				  ROS_INFO_STREAM("perceived node " << i << ", vx=" << perceived_velocity_[i].linear.x << ", vy=" << perceived_velocity_[i].linear.y);
				}
				for (int i = 0; i < index_estimated_.size(); i++){
				  ROS_INFO_STREAM("estimated node " << i << ", x=" << estimated_shape_[i].position.x << ", y=" << estimated_shape_[i].position.y);
				}
				for (int i = 0; i < node_count_; i++){
				  ROS_INFO_STREAM("whole shape node " << i << ", x=" << whole_shape_[i].position.x << ", y=" << whole_shape_[i].position.y);
				}
				//ROS_INFO_STREAM("(Sub)Matrix: " << K_struct_.block(0,0,8,8));
				
				// for testing step 5
				ROS_INFO_STREAM("Master velocity vx=" << x_dot_master_[0].linear.x << ", vy=" << x_dot_master_[0].linear.y);
				for (int i = 0; i < index_target_.size(); i++){
				  ROS_INFO_STREAM("Real estimated node " << index_target_[i] << ", x=" << whole_shape_[index_target_[i]].position.x << ", y=" << whole_shape_[index_target_[i]].position.y);
				}
				for (int i = 0; i < index_target_.size(); i++){
				  ROS_INFO_STREAM("Target node " << i << ", x=" << target_shape_[i].position.x << ", y=" << target_shape_[i].position.y);
				}
				ROS_INFO_STREAM("Computed velocity vx=" << x_dot_des_[0].linear.x << ", vy=" << x_dot_des_[0].linear.y);
				
				// show velocity command
				for (int i = 0; i < joint_handles_.size(); i++){
				  ROS_INFO_STREAM("Joint name = " << kdl_chain_.getSegment(i).getJoint().getName() << ", command = " << q_dot_cmd_.data(i));
				}
				ROS_INFO("=========================");
				ROS_INFO("=========================");
				
				// time step
				last_time_ = current_time_;
			}
			#endif
	//ROS_INFO("***** CartesianVelocityControl::update fin ************");
    }
	
	// Callback for master pose
    /*void VelBasedShapeController::masterPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
    {
		std::vector<geometry_msgs::Pose> pose_list_master_frame, pose_list_slave_xy_frame;
		pose_list_master_frame.resize(1);
		pose_list_slave_xy_frame.resize(1);
		pose_list_master_frame[0] = *msg;
		poseListFrameTransform(pose_list_master_frame, pose_list_slave_xy_frame, "master_base_link", "slave_xy_frame");
		x_master_buffer_.writeFromNonRT(pose_list_slave_xy_frame);
        if(test){
			ROS_INFO("***** VelBasedShapeController::masterPoseCB ************");
		}
    }*/
	
	// Callback for master velocity
    /*void VelBasedShapeController::masterVelocityCB(const geometry_msgs::Twist::ConstPtr& msg)
    {
		ROS_INFO("***** START VelBasedShapeController::masterVelocityCB ************");
		std::vector<geometry_msgs::Twist> twist_list_master_frame, twist_list_slave_xy_frame;
		twist_list_master_frame.resize(1);
		twist_list_slave_xy_frame.resize(1);
		ROS_INFO("1");
		twist_list_master_frame[0] = *msg;
		ROS_INFO("2");
		linearTwistListFrameTransform(twist_list_master_frame, twist_list_slave_xy_frame, "master_base_link", "slave_xy_frame");
		ROS_INFO("3");
		x_dot_master_buffer_.writeFromNonRT(twist_list_slave_xy_frame);   
        ROS_INFO("***** FINISH VelBasedShapeController::masterVelocityCB ************");
    }*/
    
    // Callback for desired shape
    void VelBasedShapeController::desiredShapeCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		ROS_INFO("***** VelBasedShapeController::desiredShapeCB ************");
		std::vector<geometry_msgs::Pose> shape_list;
		shape_list.resize(index_target_.size());
		for(int i=0; i<index_target_.size(); i++){
			shape_list[i].position.x = (*msg).data[i*dof_count_];
			shape_list[i].position.y = (*msg).data[i*dof_count_+1];
		}
		desired_shape_buffer_.writeFromNonRT(shape_list);
		target_set_ = true;
        ROS_INFO("***** Desired shape has been set, you can start the controller ************");
    }
    
    // Callback for perceived shape
    void VelBasedShapeController::visionShapeCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		//ROS_INFO("***** START VelBasedShapeController::visionShapeCB ************");
		std::vector<geometry_msgs::Pose> shape_list_camera_frame, shape_list_slave_xy_frame;
		shape_list_camera_frame.resize(index_vision_.size());
		shape_list_slave_xy_frame.resize(index_vision_.size());
		for(int i=0; i<index_vision_.size(); i++){
			shape_list_camera_frame[i].position.x = (*msg).data[i*3];
			shape_list_camera_frame[i].position.y = (*msg).data[i*3+1];
			shape_list_camera_frame[i].position.z = (*msg).data[i*3+2];
		}
		poseListFrameTransform(shape_list_camera_frame, shape_list_slave_xy_frame, "camera_frame", "slave_xy_frame");
		for(int i=0; i<index_vision_.size(); i++){
			shape_list_slave_xy_frame[i].position.x -= 0.1;
		}
		vision_shape_buffer_.writeFromNonRT(shape_list_slave_xy_frame);
        //ROS_INFO("***** FINISH VelBasedShapeController::visionShapeCB ************");
        //for(int i=0; i<shape_list_slave_xy_frame.size(); i++) ROS_INFO_STREAM("Point #" << i << ", x=" << shape_list_slave_xy_frame[i].position.x << ", y=" << shape_list_slave_xy_frame[i].position.y << ", z=" << shape_list_slave_xy_frame[i].position.z);
	}
    
    // method for elemental tangent stiffness matrix construction
	void VelBasedShapeController::elemRigidity(Eigen::MatrixXd& K_elem, const double& x1, const double& y1, const double& x2, const double& y2, const double& E, const double& A, const double& L0)
	{
		double L=sqrt(pow(x2-x1,2)+pow(y2-y1,2));
		//if(L>L0){
			double k=E*A/L0;
			double k11=k*(pow(L,3)-L0*pow(y2-y1,2))/pow(L,3);
			double k12=k*(x2-x1)*(y2-y1)*L0/pow(L,3);
			double k22=k*(pow(L,3)-L0*pow(x2-x1,2))/pow(L,3);
			Eigen::MatrixXd K_node(2,2);
			K_node << k11, k12, k12, k22;
			K_elem.resize(4,4);
			K_elem.block<2,2>(0,0) = K_node;
			K_elem.block<2,2>(2,0) = -K_node;
			K_elem.block<2,2>(0,2) = -K_node;
			K_elem.block<2,2>(2,2) = K_node;
		//}
		//else{
		//	K_elem = Eigen::MatrixXd::Zero(dof_count_*dof_count_,dof_count_*dof_count_);
		//	ROS_INFO_STREAM("At time t=" << current_time_ << " element with negative elongation.");
		//}
	}
	
	// method for structural tangent stiffness matrix construction
	void VelBasedShapeController::structuralRigidity(Eigen::MatrixXd& K_struct, const std::vector<geometry_msgs::Pose>& whole_shape, const Eigen::MatrixXi& connectivity_matrix)
	{
		// fill with zeros
		K_struct = Eigen::MatrixXd::Zero(node_count_*dof_count_,node_count_*dof_count_);
		
		for(int i=0; i<element_count_; i++){
			// nodes numbering
			int n1_index, n2_index;
			n1_index = connectivity_matrix(i,0);
			n2_index = connectivity_matrix(i,1);
			
			// elemental matrix
			double x1 = whole_shape[n1_index].position.x;
			double y1 = whole_shape[n1_index].position.y;
			double x2 = whole_shape[n2_index].position.x;
			double y2 = whole_shape[n2_index].position.y;
			//ROS_INFO_STREAM("x1: " << x1 << "y1: " << y1 << "x2: " << x2 << "y2: " << y2 << "\n");
			Eigen::MatrixXd K_elem(dof_count_*dof_count_,dof_count_*dof_count_);
			elemRigidity(K_elem, x1, y1, x2, y2, E_, A_, L0_);
			//ROS_INFO_STREAM("elem: " << K_elem << "\n");
			
			// structural matrix assembly
			K_struct.resize(dof_count_*node_count_,dof_count_*node_count_);
			K_struct.block<2,2>(dof_count_*n1_index,dof_count_*n1_index) += K_elem.block(0,0,2,2);
			K_struct.block<2,2>(dof_count_*n2_index,dof_count_*n1_index) += K_elem.block(2,0,2,2);
			K_struct.block<2,2>(dof_count_*n1_index,dof_count_*n2_index) += K_elem.block(0,2,2,2);
			K_struct.block<2,2>(dof_count_*n2_index,dof_count_*n2_index) += K_elem.block(2,2,2,2);
		}
	}
	
	// extracting submatrices based on vectors for colums and rows
	void VelBasedShapeController::extractSubmatrix(const Eigen::MatrixXd& fullmat, Eigen::MatrixXd& submat, const Eigen::VectorXi& rows, const Eigen::VectorXi& cols)
	{
		submat.resize(rows.size(),cols.size()); // TODO ???
		for(int i=0; i<rows.size(); i++){
			for(int j=0; j<cols.size(); j++){
				submat(i,j) = fullmat(rows(i),cols(j));
			}
		}
	}
	
	// creation of coordinate vectors based on indexes
	void VelBasedShapeController::indexToCoords(const std::vector<int>& index, Eigen::VectorXi& coords)
	{
		for(int i=0; i<index.size(); i++){
			coords(dof_count_*i) = dof_count_*index[i];
			coords(dof_count_*i+1) = dof_count_*index[i]+1;
		}
	}
	
	// reduction of a list based on Eigen vector of indices
	void VelBasedShapeController::reduceList(const std::vector<geometry_msgs::Pose>& in, std::vector<geometry_msgs::Pose>& out, const std::vector<int>& indexes)
	{
		for(int i=0; i<indexes.size(); i++){
			out[i] = in[indexes[i]];
		}
	}
	
	// transform of a list of poses in other frames
	void VelBasedShapeController::poseListFrameTransform(const std::vector<geometry_msgs::Pose>& in, std::vector<geometry_msgs::Pose>& out, const std::string from_frame, const std::string to_frame)
	{	
		for(int i=0; i<in.size(); i++){
			tf::Stamped<tf::Point> point_from, point_to;
			point_from.frame_id_ = from_frame;
			point_from.stamp_ = ros::Time();
			point_from.setX(in[i].position.x);
			point_from.setY(in[i].position.y);
			point_from.setZ(in[i].position.z);
			
			listener_.transformPoint(to_frame, point_from, point_to);
			out[i].position.x = point_to.x();
			out[i].position.y = point_to.y();
			out[i].position.z = point_to.z();
		}
	}
	
	// transform a pose in other frames
	void VelBasedShapeController::poseFrameTransform(const geometry_msgs::Pose& in, geometry_msgs::Pose& out, const std::string from_frame, const std::string to_frame)
	{
		tf::Stamped<tf::Point> point_from, point_to;
		point_from.frame_id_ = from_frame;
		point_from.stamp_ = ros::Time();
		point_from.setX(in.position.x);
		point_from.setY(in.position.y);
		point_from.setZ(in.position.z);
		
		listener_.transformPoint(to_frame, point_from, point_to);
		out.position.x = point_to.x();
		out.position.y = point_to.y();
		out.position.z = point_to.z();
	}
	
	// transform of a list of twists in other frames
	void VelBasedShapeController::linearTwistListFrameTransform(const std::vector<geometry_msgs::Twist>& in, std::vector<geometry_msgs::Twist>& out, const std::string from_frame, const std::string to_frame)
	{
		tf::StampedTransform tr, tr_twist; // create a new transform just for twists
		ros::Time time = ros::Time::now();  // or better msg->header.stamp, if available; never ros::Time(0)
		bool success = false;
		while (!success) {
		  try {
			listener_.lookupTransform(to_frame, from_frame, time, tr);
			success = true;
		  } catch (tf::ExtrapolationException e) {
		  }
		  sleep(0.1);
		}
		tr_twist.setRotation(tr.getRotation()); // set identical rotation
		tr_twist.setOrigin( tf::Vector3(0.0, 0.0, 0.0) ); // set null translation since it is not used in twist computation (frames are fixed)
		for(int i=0; i<in.size(); i++){
			tf::Vector3 v = tr_twist(tf::Vector3(in[i].linear.x, in[i].linear.y, in[i].linear.z));
			out[i].linear.x = v.getX();
			out[i].linear.y = v.getY();
			out[i].linear.z = v.getZ();
		}
	} 
	
	// transform of a twist in other frames
	void VelBasedShapeController::linearTwistFrameTransform(const geometry_msgs::Twist& in, geometry_msgs::Twist& out, const std::string from_frame, const std::string to_frame)
	{
		tf::StampedTransform tr, tr_twist; // create a new transform just for twists
		ros::Time time = ros::Time::now();  // or better msg->header.stamp, if available; never ros::Time(0)
		bool success = false;
		while (!success) {
		  try {
			listener_.lookupTransform(to_frame, from_frame, time, tr);
			success = true;
		  } catch (tf::ExtrapolationException e) {
		  }
		  sleep(0.1);
		}
		tr_twist.setRotation(tr.getRotation()); // set identical rotation
		tr_twist.setOrigin( tf::Vector3(0.0, 0.0, 0.0) ); // set null translation since it is not used in twist computation (frames are fixed)
		tf::Vector3 v = tr_twist(tf::Vector3(in.linear.x, in.linear.y, in.linear.z));
		out.linear.x = v.getX();
		out.linear.y = v.getY();
		out.linear.z = v.getZ();
	} 
	
	
	// conversion of a list of poses to Eigen vector
	void VelBasedShapeController::poseListXYToEigen(const std::vector<geometry_msgs::Pose>& shape, Eigen::VectorXd& node_coords, const int node_count)
	{
		for(int i=0; i<node_count; i++){
			if(node_coords.rows()==dof_count_*node_count){
				node_coords(dof_count_*i) = shape[i].position.x;
				node_coords(dof_count_*i+1) = shape[i].position.y;
			}
			else{
				ROS_INFO("***** ERROR in VelBasedShapeController::poseListXYToEigen ************");
				ROS_INFO("*** List has more elements than number of nodes ***");
			}
		}
	}
	
	// conversion of Eigen vector to a list of poses
	void VelBasedShapeController::eigenToPoseListXY(std::vector<geometry_msgs::Pose>& shape, const Eigen::VectorXd& node_coords, const int node_count)
    {
		for(int i=0; i<node_count; i++){
			if(shape.size()==node_count){
				shape[i].position.x = node_coords(dof_count_*i) ;
				shape[i].position.y = node_coords(dof_count_*i+1);
			}
			else{
				ROS_INFO("***** ERROR in VelBasedShapeController::EigenToPoseListXY ************");
				ROS_INFO("*** Vector of coords has more elements than number of nodes ***");
				ROS_INFO_STREAM("Vector size=" << shape.size() << ", node number= " << node_count);
			}
		}
	}
	
	// conversion of a list of twists to Eigen vector
	void VelBasedShapeController::twistListXYToEigen(const std::vector<geometry_msgs::Twist>& velocity, Eigen::VectorXd& node_coords, const int node_count)
	{
		for(int i=0; i<node_count; i++){
			if(node_coords.rows()==dof_count_*node_count){
				node_coords(dof_count_*i) = velocity[i].linear.x;
				node_coords(dof_count_*i+1) = velocity[i].linear.y;
			}
			else{
				ROS_INFO("***** ERROR in VelBasedShapeController::twistListXYToEigen ************");
				ROS_INFO("*** List of twists has more elements than number of nodes ***");
			}
		}
	}
	
	// conversion of Eigen vector to a list of twists
	void VelBasedShapeController::eigenToTwistListXY(std::vector<geometry_msgs::Twist>& velocity, const Eigen::VectorXd& node_coords, const int node_count)
    {
		for(int i=0; i<node_count; i++){
			if(velocity.size()==node_count){
				velocity[i].linear.x = node_coords(dof_count_*i) ;
				velocity[i].linear.y = node_coords(dof_count_*i+1);
			}
			else{
				ROS_INFO("***** ERROR in VelBasedShapeController::EigenToTwistListXY ************");
				ROS_INFO("*** Vector of coords has more elements than number of nodes ***");
			}
		}
	}
	
	void VelBasedShapeController::visionVelocity(std::vector<geometry_msgs::Twist>& velocity, const std::vector<geometry_msgs::Twist>& prev_velocity, const std::vector<geometry_msgs::Pose>& shape, const std::vector<geometry_msgs::Pose>& prev_shape)
	{
		//ROS_INFO_STREAM("Velocity size=" << velocity.size() << ", prev vel size=" << prev_velocity.size() );
		//ROS_INFO_STREAM("Shape size=" << shape.size() << ", prev shape size=" << prev_shape.size() );
		for(int i=0; i<velocity.size(); i++){
			velocity[i].linear.x = .2*(prev_velocity[i].linear.x) + .8*(shape[i].position.x - prev_shape[i].position.x)/(current_time_ - previous_time_).toSec();
			velocity[i].linear.y = .2*(prev_velocity[i].linear.y) + .8*(shape[i].position.y - prev_shape[i].position.y)/(current_time_ - previous_time_).toSec();
			velocity[i].linear.z = .2*(prev_velocity[i].linear.z) + .8*(shape[i].position.z - prev_shape[i].position.z)/(current_time_ - previous_time_).toSec();
		}
	}
	
	void VelBasedShapeController::shapePerception(std::vector<geometry_msgs::Pose>& perceived_shape, std::vector<geometry_msgs::Twist>& perceived_velocity)
	{
		// set slave values to adequate frame
		//geometry_msgs::Pose x_slave;
		//geometry_msgs::Twist x_dot_slave;
		//poseFrameTransform(x_msg_, x_slave, "base_frame", "slave_xy_frame");
		//linearTwistFrameTransform(x_dot_msg_, x_dot_slave,  "base_frame", "slave_xy_frame");
		// merge all values in lists
		perceived_shape[0] = x_master_[0];
		perceived_shape[1] = vision_shape_[0];
		perceived_shape[2] = vision_shape_[1];
		perceived_shape[3] = x_msg_;
		perceived_velocity[0] = x_dot_master_[0];
		perceived_velocity[1] = vision_velocity_[0];
		perceived_velocity[2] = vision_velocity_[1];
		perceived_velocity[3] = x_dot_msg_;
	}
	
	// method for regrouping estimated and perceived nodes in a same list
	void VelBasedShapeController::shapeMerger(const std::vector<geometry_msgs::Pose>& estimated_shape, const std::vector<geometry_msgs::Pose>& perceived_shape, std::vector<geometry_msgs::Pose>& whole_shape)
	{
		// loop on all nodes
		for(int i=0; i<node_count_; i++){
			// search if node is perceived or estimated
			std::vector<int>::iterator itp = std::find(index_perceived_.begin(), index_perceived_.end(), i);
			if (itp != index_perceived_.end()){
				int ip = std::distance(index_perceived_.begin(), itp);
				whole_shape[i] = perceived_shape[ip];
			}else{
				std::vector<int>::iterator ite = std::find(index_estimated_.begin(), index_estimated_.end(), i);
				if (ite != index_estimated_.end()){
					int ie = std::distance(index_estimated_.begin(), ite);
					whole_shape[i] = estimated_shape[ie];
				}else{
					ROS_INFO("***** ERROR in VelBasedShapeController::shapeMerger ************");
					ROS_INFO("*** Node is neither perceived nor estimated ***");
				}
			}
		}
	}
	
	// method for estimation of node coordinates based on perception
	void VelBasedShapeController::shapeEstimation(std::vector<geometry_msgs::Pose>& estimated_shape, const std::vector<geometry_msgs::Pose>& perceived_shape, const std::vector<geometry_msgs::Twist>& perceived_twist, const Eigen::MatrixXd& K_struct)
	{
		// populate submatrices
		Eigen::MatrixXd K_ep(vector_estimated_.rows(),vector_perceived_.rows()), K_ee(vector_estimated_.rows(),vector_estimated_.rows());
		//K_ep = K_struct(vector_estimated_, vector_perceived_);
		extractSubmatrix(K_struct, K_ep, vector_estimated_, vector_perceived_);
		//K_ee = K_struct(vector_estimated_, vector_estimated_);
		extractSubmatrix(K_struct, K_ee, vector_estimated_, vector_estimated_);
		
		// create vectors
		Eigen::VectorXd estimated_node_pos(vector_estimated_.rows());
		poseListXYToEigen(estimated_shape_, estimated_node_pos, index_estimated_.size());
		Eigen::VectorXd perceived_node_vel(vector_perceived_.rows());
		twistListXYToEigen(perceived_twist, perceived_node_vel, index_perceived_.size());
		Eigen::VectorXd estimated_node_vel(vector_estimated_.rows());
		
		// solve for estimated node velocity
		Eigen::VectorXd b = -K_ep*perceived_node_vel;
		estimated_node_vel = K_ee.colPivHouseholderQr().solve(b);
		
		// numerical integration
		ros::Duration dt = current_time_ - previous_time_;
		estimated_node_pos += dt.toSec()*estimated_node_vel;
		
		// final output
		eigenToPoseListXY(estimated_shape, estimated_node_pos, index_estimated_.size());
	}
	
	// method for computation of slave robot twist to control shape
	void VelBasedShapeController::controlLaw(const std::vector<geometry_msgs::Pose>& whole_shape, const std::vector<geometry_msgs::Pose>& target_shape, const Eigen::MatrixXd& K_struct, const std::vector<geometry_msgs::Twist>& master_twist, std::vector<geometry_msgs::Twist>& slave_twist)
	{
		// compute error vector on target nodes
		std::vector<geometry_msgs::Pose> estimated_target_shape;
		estimated_target_shape.resize(index_target_.size());
		// TODO turn whole shape into estimated target shape
		Eigen::VectorXd estimated_target_pos(vector_target_.rows());
		Eigen::VectorXd specified_target_pos(vector_target_.rows());
		Eigen::VectorXd target_error(vector_target_.rows());
		poseListXYToEigen(target_shape, specified_target_pos, index_target_.size());
		//ROS_INFO_STREAM("specified_target_pos size=" << specified_target_pos.rows());
		reduceList(whole_shape, estimated_target_shape, index_target_);
		poseListXYToEigen(estimated_target_shape, estimated_target_pos, index_target_.size());
		//ROS_INFO_STREAM("specified_target_pos size=" << estimated_target_pos.rows());
		target_error = specified_target_pos - estimated_target_pos;
		
		// populate submatrices
		Eigen::MatrixXd K_tt(vector_target_.rows(),vector_target_.rows());
		Eigen::MatrixXd K_tm(vector_target_.rows(),vector_master_.rows());
		Eigen::MatrixXd K_ts(vector_target_.rows(),vector_slave_.rows());
		Eigen::MatrixXd K_tf(vector_target_.rows(),vector_free_.rows());
		Eigen::MatrixXd K_ft(vector_free_.rows(),vector_target_.rows());
		Eigen::MatrixXd K_fm(vector_free_.rows(),vector_master_.rows());
		Eigen::MatrixXd K_fs(vector_free_.rows(),vector_slave_.rows());
		Eigen::MatrixXd K_ff(vector_free_.rows(),vector_free_.rows());
		
		//K_tt = K_struct(vector_target_, vector_target_);
		extractSubmatrix(K_struct, K_tt, vector_target_, vector_target_);
		//K_tm = K_struct(vector_target_, vector_master_);
		extractSubmatrix(K_struct, K_tm, vector_target_, vector_master_);
		//K_ts = K_struct(vector_target_, vector_slave_);
		extractSubmatrix(K_struct, K_ts, vector_target_, vector_slave_);
		//K_tf = K_struct(vector_target_, vector_free_);
		extractSubmatrix(K_struct, K_tf, vector_target_, vector_free_);
		//K_ft = K_struct(vector_free_, vector_target_);
		extractSubmatrix(K_struct, K_ft, vector_free_, vector_target_);
		//K_fm = K_struct(vector_free_, vector_master_);
		extractSubmatrix(K_struct, K_fm, vector_free_, vector_master_);
		//K_fs = K_struct(vector_free_, vector_slave_);
		extractSubmatrix(K_struct, K_fs, vector_free_, vector_slave_);
		//K_ff = K_struct(vector_free_, vector_free_);
		extractSubmatrix(K_struct, K_ff, vector_free_, vector_free_);
		
		// compute control matrices
		Eigen::MatrixXd K_cc(vector_target_.rows(),vector_target_.rows());
		Eigen::MatrixXd K_cm(vector_target_.rows(),vector_master_.rows());
		Eigen::MatrixXd K_cs(vector_target_.rows(),vector_slave_.rows());
		K_cc = K_tt - K_tf*K_ff.inverse()*K_ft; //linking target disp to target known forces
		K_cs = K_ts - K_tf*K_ff.inverse()*K_fs; //linking slave disp to target known forces
		K_cm = K_tm - K_tf*K_ff.inverse()*K_fm; //linking master disp to target known forces
		
		// master twist to vector
		Eigen::VectorXd master_node_vel(vector_master_.rows());
		twistListXYToEigen(master_twist, master_node_vel, index_master_.size());
		
		// least square control output
		// dxps = pinv(Kcs)*(- Kcm*dxpm - Kcc*Gp*ec);
		Eigen::VectorXd b = - K_cm*master_node_vel - K_cc*Gp_*target_error;
		Eigen::VectorXd slave_node_vel = K_cs.colPivHouseholderQr().solve(b);
		eigenToTwistListXY(slave_twist, slave_node_vel, index_slave_.size());
	}
    
    // method for computation of target point trajectory (5th-order polynomial interpolation)
    void VelBasedShapeController::setTarget(const std::vector<geometry_msgs::Pose>& initial_shape, const std::vector<geometry_msgs::Pose>& desired_shape, std::vector<geometry_msgs::Pose>& target_shape, const std::vector<geometry_msgs::Pose>& x_master, const double &ti, const double &tf)
    {
		// get desired shape as a Eigen vector
		Eigen::VectorXd desired_node_pos(vector_target_.rows());
		poseListXYToEigen(desired_shape, desired_node_pos, index_target_.size());
		Eigen::VectorXd initial_node_pos(vector_target_.rows());
		poseListXYToEigen(initial_shape, initial_node_pos, index_target_.size());
		//ROS_INFO_STREAM("Init=" << initial_node_pos);
		
		// set matrix for polynomial generation
		Eigen::MatrixXd M(6,6);
		M.row(0) << 1, ti, pow(ti,2), pow(ti,3), pow(ti,4), pow(ti,5);
		M.row(1) << 1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5);
		M.row(2) << 0, 1, 2*ti, 3*pow(ti,2), 4*pow(ti,3), 5*pow(ti,4);
		M.row(3) << 0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4);
		M.row(4) << 0, 0, 2*1, 3*2*ti, 4*3*pow(ti,2), 5*4*pow(ti,3);
		M.row(5) << 0, 0, 2*1, 3*2*tf, 4*3*pow(tf,2), 5*4*pow(tf,3);
		Eigen::VectorXd vx(6);
		Eigen::VectorXd ax(6);
		
		// set time with constraints
		ros::Duration dt = ros::Time::now() - starting_time_;
		double tc = dt.toSec();
		if(tc<ti) tc = ti;
		if(tc>tf) tc = tf;
		Eigen::RowVectorXd vt(6);
		vt << 1, tc, pow(tc,2), pow(tc,3), pow(tc,4), pow(tc,5);
		
		// loop to compute target node coordinates
		Eigen::VectorXd target_node_pos(vector_target_.rows());
		for(int i=0; i<vector_target_.rows(); i++){
			// compute coefficients for polynomial trajectory
			vx << initial_node_pos(i), desired_node_pos(i), 0, 0, 0, 0;
			ax = M.colPivHouseholderQr().solve(vx);
			//ROS_INFO_STREAM("Val=" << vx);
			//ROS_INFO_STREAM("Mat=" << M);
			//ROS_INFO_STREAM("Coeff=" << ax);
			// apply interpolation between initial and desired value
			target_node_pos(i) = ax.dot(vt);
		}
		
		// account for master robot pose
		/*for(int i=0; i<vector_target_.rows(); i++){
			if(i%2==0) target_node_pos(i) += x_master[0].position.x;
			else target_node_pos(i) += x_master[0].position.y;
		}*/
		
		// output as a shape
		eigenToPoseListXY(target_shape, target_node_pos, index_target_.size());
	}
	
	/*void VelBasedShapeController::fillInitialShape(std::vector<geometry_msgs::Pose>& shape)
	{
		geometry_msgs::Pose pose1;
		pose1.position.x = 0;
		pose1.position.y = 0;
		shape.push_back(pose1);
		pose1.position.x = .116;
		pose1.position.y = -.1809;
		shape.push_back(pose1);
		pose1.position.x = .2513;
		pose1.position.y = -.3448;
		shape.push_back(pose1);
		pose1.position.x = .4105;
		pose1.position.y = -.4824;
		shape.push_back(pose1);
		pose1.position.x = .5957;
		pose1.position.y = -.5783;
		shape.push_back(pose1);
		pose1.position.x = .8;
		pose1.position.y = -.6135;
		shape.push_back(pose1);
		pose1.position.x = 1.0043;
		pose1.position.y = -.5783;
		shape.push_back(pose1);
		pose1.position.x = 1.1895;
		pose1.position.y = -.4824;
		shape.push_back(pose1);
		pose1.position.x = 1.3487;
		pose1.position.y = -.3448;
		shape.push_back(pose1);
		pose1.position.x = 1.484;
		pose1.position.y = -.1809;
		shape.push_back(pose1);
		pose1.position.x = 1.6;
		pose1.position.y = 0;
		shape.push_back(pose1);
		ROS_INFO_STREAM("VelBasedShapeController::fillInitialShape executed");
	}*/
	
	void VelBasedShapeController::fillInitialShape(std::vector<geometry_msgs::Pose>& shape)
	{
		geometry_msgs::Pose pose1;
		pose1.position.x = -1.1225;
		pose1.position.y = .0871;
		shape.push_back(pose1);
		pose1.position.x = -1.0722;
		pose1.position.y = .0348;
		shape.push_back(pose1);
		pose1.position.x = -1.0104;
		pose1.position.y = -.0195;
		shape.push_back(pose1);
		pose1.position.x = -.9422;
		pose1.position.y = -.0561;
		shape.push_back(pose1);
		pose1.position.x = -.8666;
		pose1.position.y = -.0727;
		shape.push_back(pose1);
		pose1.position.x = -.7872;
		pose1.position.y = -.0653;
		shape.push_back(pose1);
		pose1.position.x = -.7130;
		pose1.position.y = -.0368;
		shape.push_back(pose1);
		pose1.position.x = -.6497;
		pose1.position.y = .0097;
		shape.push_back(pose1);
		pose1.position.x = -.5962;
		pose1.position.y = .0693;
		shape.push_back(pose1);
		pose1.position.x = -.5508;
		pose1.position.y = .1405;
		shape.push_back(pose1);
		pose1.position.x = -.5;
		pose1.position.y = .2;
		shape.push_back(pose1);
		ROS_INFO_STREAM("VelBasedShapeController::fillInitialShape executed");
	}
	
	void VelBasedShapeController::testTopics(void)
	{
		// ******** 1. Get values from subscribed topics *********
		// set time
		previous_time_ = current_time_;
		current_time_ = ros::Time::now();
		// read topics
		x_master_ptr_ = x_master_buffer_.readFromRT();
		x_dot_master_ptr_ = x_dot_master_buffer_.readFromRT();
		desired_shape_ptr_ = desired_shape_buffer_.readFromRT();
		x_master_ = *x_master_ptr_;
		x_dot_master_ = *x_dot_master_ptr_;
		desired_shape_ = *desired_shape_ptr_;
		
		ROS_INFO_STREAM("x_master size: " << x_master_.size() << ", value = " << x_master_[0]);
		ROS_INFO_STREAM("x_master size: " << x_dot_master_.size() << ", value = " << x_dot_master_[0]);
		ROS_INFO_STREAM("desired_shape size: " << desired_shape_.size() << ", value = " << desired_shape_[0]);
	}
	
	void VelBasedShapeController::testStructure(void)
	{
		// all operations executed in init() method
		ROS_INFO_STREAM("connectivity matrix: " << connectivity_matrix_);
		
		ROS_INFO_STREAM("target index: ");
		for (const auto& val: index_target_)
			ROS_INFO_STREAM( val << ' ');
			
		ROS_INFO_STREAM("target vector: " << vector_target_);
		ROS_INFO_STREAM("free vector: " << vector_free_);
		ROS_INFO_STREAM("perceived vector: " << vector_perceived_);
	}
	
	void VelBasedShapeController::testConversions(void)
	{
		std::vector<geometry_msgs::Pose> shape;
		Eigen::VectorXd node_pos(4);
		std::vector<geometry_msgs::Twist> velocity;
		Eigen::VectorXd node_vel(4);
		int node_count = 2;
		
		ROS_INFO_STREAM("Shape definition");
		geometry_msgs::Pose pose1;
		pose1.position.x = 2;
		pose1.position.y = -4;		
		geometry_msgs::Pose pose2;
		pose2.position.x = 3;
		pose2.position.y = -5;
		shape.push_back(pose1);
		shape.push_back(pose2);
		
		ROS_INFO_STREAM("Shape defined");
		poseListXYToEigen(shape, node_pos, node_count);
		ROS_INFO_STREAM("Eigen pose vector: " << node_pos);
		
		node_pos(1) = 20;
		node_pos(3) = 40;
		
		eigenToPoseListXY(shape, node_pos, node_count);
		ROS_INFO_STREAM("List of poses 0: " << shape[0] << " and 1: " << shape[1]);
		
		geometry_msgs::Twist tw1;
		tw1.linear.x = .2;
		tw1.linear.y = -.4;		
		geometry_msgs::Twist tw2;
		tw2.linear.x = .3;
		tw2.linear.y = -.5;	
		velocity.push_back(tw1);
		velocity.push_back(tw2);
		
		twistListXYToEigen(velocity, node_vel, node_count);
		ROS_INFO_STREAM("Eigen vel vector: " << node_vel);
		
		node_vel(1) = 80;
		node_vel(3) = 90;
		
		eigenToTwistListXY(velocity, node_vel, node_count);
		ROS_INFO_STREAM("List of twists 0: " << velocity[0] << " and 1: " << velocity[1]);
	}
	
	void VelBasedShapeController::testSubmatrix(void)
	{
		Eigen::MatrixXd fullmat(4,4);
		Eigen::MatrixXd submat;
		Eigen::VectorXi rows(3);
		Eigen::VectorXi cols(2);
		fullmat << 0,1,2,3,10,11,12,13,20,21,22,23,30,31,32,33;
		rows << 0,1,3;
		cols << 0,2;
		extractSubmatrix(fullmat, submat, rows, cols);
		ROS_INFO_STREAM("Submatrix: " << submat);
	}
	
	void VelBasedShapeController::testElemental(void)
	{
		Eigen::MatrixXd K1(dof_count_*dof_count_,dof_count_*dof_count_);
		Eigen::MatrixXd K2(dof_count_*dof_count_,dof_count_*dof_count_);
		Eigen::MatrixXd K3(dof_count_*dof_count_,dof_count_*dof_count_);
		elemRigidity(K1, 0, 0, 1, 1, 1000, 1, 1);
		elemRigidity(K2, 0, 0, 0, .9, E_, A_, L0_);
		elemRigidity(K3, 0, 0, 0, .9, 210000.0, .001, .2);
		ROS_INFO_STREAM("Matrix with sqrt(2) elongation at 0: " << K1);
		ROS_INFO_STREAM("Matrix with sqrt(2) elongation non-zero: " << K2);
		ROS_INFO_STREAM("Matrix with <0 elongation: " << K3);
	}
	
	void VelBasedShapeController::testStructural(void)
	{
		std::vector<geometry_msgs::Pose> shape;
		Eigen::MatrixXd K1(node_count_*dof_count_,node_count_*dof_count_);
		geometry_msgs::Pose pose1;
		pose1.position.x = 0;
		pose1.position.y = 0;
		shape.push_back(pose1);
		pose1.position.x = .116;
		pose1.position.y = -.1809;
		shape.push_back(pose1);
		pose1.position.x = .2513;
		pose1.position.y = -.3448;
		shape.push_back(pose1);
		pose1.position.x = .4105;
		pose1.position.y = -.4824;
		shape.push_back(pose1);
		pose1.position.x = .5957;
		pose1.position.y = -.5783;
		shape.push_back(pose1);
		pose1.position.x = .8;
		pose1.position.y = -.6135;
		shape.push_back(pose1);
		pose1.position.x = 1.0043;
		pose1.position.y = -.5783;
		shape.push_back(pose1);
		pose1.position.x = 1.1895;
		pose1.position.y = -.4824;
		shape.push_back(pose1);
		pose1.position.x = 1.3487;
		pose1.position.y = -.3448;
		shape.push_back(pose1);
		pose1.position.x = 1.484;
		pose1.position.y = -.1809;
		shape.push_back(pose1);
		pose1.position.x = 1.6;
		pose1.position.y = 0;
		shape.push_back(pose1);
		structuralRigidity(K1, shape, connectivity_matrix_);
		ROS_INFO_STREAM("Matrix: " << K1);
	}
	
	void VelBasedShapeController::testFrameTransform(void)
	{
		std::vector<geometry_msgs::Pose> shape_in_camera, shape_in_master;
		geometry_msgs::Pose pose1;
		pose1.position.x = 1;
		pose1.position.y = 0;
		pose1.position.z = 0;
		shape_in_camera.push_back(pose1);
		pose1.position.x = 0;
		pose1.position.y = 1;
		pose1.position.z = 0;
		shape_in_camera.push_back(pose1);
		pose1.position.x = 0;
		pose1.position.y = 0;
		pose1.position.z = 1;
		shape_in_camera.push_back(pose1);
		
		shape_in_master.push_back(pose1);
		shape_in_master.push_back(pose1);
		shape_in_master.push_back(pose1); // only to initialize vector size
		
		poseListFrameTransform(shape_in_camera, shape_in_master, "slave_xy_frame", "base_link");
		
		for(int i=0; i<shape_in_master.size(); i++) ROS_INFO_STREAM("Point #" << i << ", x=" << shape_in_master[i].position.x << ", y=" << shape_in_master[i].position.y << ", z=" << shape_in_master[i].position.z);
	}
	
	void VelBasedShapeController::testTwistTransform(void)
	{
		std::vector<geometry_msgs::Twist> twist_in_base, twist_in_slave;
		geometry_msgs::Twist tw1;
		tw1.linear.x = 1;
		tw1.linear.y = 0;
		tw1.linear.z = 0;
		twist_in_base.push_back(tw1);
		tw1.linear.x = 0;
		tw1.linear.y = 1;
		tw1.linear.z = 0;
		twist_in_base.push_back(tw1);
		tw1.linear.x = 0;
		tw1.linear.y = 0;
		tw1.linear.z = 1;
		twist_in_base.push_back(tw1);
		twist_in_slave.push_back(tw1);
		twist_in_slave.push_back(tw1);
		twist_in_slave.push_back(tw1); // only to initialize vector size
		
		linearTwistListFrameTransform(twist_in_base, twist_in_slave, "base_link", "slave_xy_frame");
		
		for(int i=0; i<twist_in_slave.size(); i++) ROS_INFO_STREAM("Point #" << i << ", x=" << twist_in_slave[i].linear.x << ", y=" << twist_in_slave[i].linear.y << ", z=" << twist_in_slave[i].linear.z);
	}
	
	void VelBasedShapeController::testListOperations(void)
	{
		std::vector<geometry_msgs::Pose> shape, shapeper, shapeest, shape2;
		geometry_msgs::Pose pose1;
		pose1.position.x = 0;
		pose1.position.y = 0;
		shape.push_back(pose1);
		pose1.position.x = .1;
		pose1.position.y = -.1;
		shape.push_back(pose1);
		pose1.position.x = .2;
		pose1.position.y = -.2;
		shape.push_back(pose1);
		pose1.position.x = .3;
		pose1.position.y = -.3;
		shape.push_back(pose1);
		pose1.position.x = .4;
		pose1.position.y = -.4;
		shape.push_back(pose1);
		pose1.position.x = .5;
		pose1.position.y = -.5;
		shape.push_back(pose1);
		pose1.position.x = .6;
		pose1.position.y = -.6;
		shape.push_back(pose1);
		pose1.position.x = .7;
		pose1.position.y = -.7;
		shape.push_back(pose1);
		pose1.position.x = .8;
		pose1.position.y = -.8;
		shape.push_back(pose1);
		pose1.position.x = .9;
		pose1.position.y = -.9;
		shape.push_back(pose1);
		pose1.position.x = 1;
		pose1.position.y = -1;
		shape.push_back(pose1);
		
		shapeper.resize(4);
		shapeest.resize(7);
		shape2.resize(11);
		
		reduceList(shape, shapeper, index_perceived_);
		for(int i=0; i<shapeper.size(); i++) ROS_INFO_STREAM("Point #" << i << ", x=" << shapeper[i].position.x << ", y=" << shapeper[i].position.y);
		reduceList(shape, shapeest, index_estimated_);
		for(int i=0; i<shapeest.size(); i++) ROS_INFO_STREAM("Point #" << i << ", x=" << shapeest[i].position.x << ", y=" << shapeest[i].position.y);
		
		shapeest[2].position.x = 999;
		shapeMerger(shapeest, shapeper, shape2);
		for(int i=0; i<shape2.size(); i++) ROS_INFO_STREAM("Point #" << i << ", x=" << shape2[i].position.x << ", y=" << shape2[i].position.y);
	}
	
	void VelBasedShapeController::updateAll(void)
	{
		// time step
		vision_previous_shape_ = vision_shape_;
		vision_previous_velocity_ = vision_velocity_;
		// slave driving
		x_dot_msg_.linear.x = .0;
		x_dot_msg_.linear.y = .0;
		x_msg_.position.x += x_dot_msg_.linear.x*(current_time_ - previous_time_).toSec();
		x_msg_.position.y += x_dot_msg_.linear.y*(current_time_ - previous_time_).toSec();
		// master driving
		x_dot_master_[0].linear.x = .0;
		x_dot_master_[0].linear.y = .0;
		x_master_[0].position.x += x_dot_master_[0].linear.x*(current_time_ - previous_time_).toSec();
		x_master_[0].position.y += x_dot_master_[0].linear.y*(current_time_ - previous_time_).toSec();
		// vision evolution
		vision_shape_[0].position.x += x_dot_master_[0].linear.x*(current_time_ - previous_time_).toSec();
		vision_shape_[0].position.y += x_dot_master_[0].linear.y*(current_time_ - previous_time_).toSec();
		vision_shape_[1].position.x += x_dot_master_[0].linear.x*(current_time_ - previous_time_).toSec();
		vision_shape_[1].position.y += x_dot_master_[0].linear.y*(current_time_ - previous_time_).toSec();
	}
	
	void VelBasedShapeController::testControlLaw(void)
	{
		// master velocity
		x_dot_master_[0].linear.x = 0;
		x_dot_master_[0].linear.y = 0;
		
		// object shape
		for(int i=0; i<node_count_; i++){
			target_shape_[i].position.y += .02;
		}
		
		// target shape
		
		controlLaw(whole_shape_, target_shape_, K_struct_, x_dot_master_, x_dot_des_);
		ROS_INFO_STREAM("Master velocity vx=" << x_dot_master_[0].linear.x << ", vy=" << x_dot_master_[0].linear.y);
		for (int i = 0; i < index_target_.size(); i++){
		  ROS_INFO_STREAM("Real estimated node " << index_target_[i] << ", x=" << whole_shape_[index_target_[i]].position.x << ", y=" << whole_shape_[index_target_[i]].position.y);
		}
		for (int i = 0; i < index_target_.size(); i++){
		  ROS_INFO_STREAM("Target node " << i << ", x=" << target_shape_[i].position.x << ", y=" << target_shape_[i].position.y);
		}
		ROS_INFO_STREAM("Computed velocity vx=" << x_dot_des_[0].linear.x << ", vy=" << x_dot_des_[0].linear.y);
	}
}

PLUGINLIB_EXPORT_CLASS(mrod_ur_ip_controllers::VelBasedShapeController, controller_interface::ControllerBase)
