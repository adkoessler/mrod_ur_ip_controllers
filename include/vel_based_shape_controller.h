/*
 *  A ROS package to control deformation of linear textured materials with an UR5 robot
 *  Adrien KOESSLER
 * 	Postdoc fellow, Université Clermont Auvergne (France)
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  adrien.koessler@sigma-clermont.fr
 *  laurent.lequievre@uca.fr
 * 
*/


#ifndef MROD_CONTROLLERS__H
#define MROD_CONTROLLERS__H

// Controller base
#include "kinematic_chain_controller_base.h"

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_conversions/kdl_msg.h>

// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Float 64 MultiArray message
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

// time
#include <time.h>

// file stream
#include <fstream>

// unique ptr
#include <memory>

namespace mrod_ur_ip_controllers
{
	class VelBasedShapeController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::VelocityJointInterface>
	{
		public:
			VelBasedShapeController();
			~VelBasedShapeController();

			bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			void masterPoseCB(const geometry_msgs::Pose::ConstPtr &msg);
			void masterVelocityCB(const geometry_msgs::Twist::ConstPtr &msg);
			void desiredShapeCB(const std_msgs::Float64MultiArrayConstPtr& msg);
			void visionShapeCB(const std_msgs::Float64MultiArrayConstPtr& msg);
			
			// K=K_elem_NL(x1,y1,x2,y2,E,A,L0);
			bool elemRigidity(Eigen::MatrixXd& K_elem, const double& x1, const double& y1, const double& x2, const double& y2, const double& E, const double& A, const double& L0);
			// Ks=K_struct(Elem_List,Node_Coord,E)
			void structuralRigidity(Eigen::MatrixXd& K_struct, const std::vector<geometry_msgs::Pose>& whole_shape, const Eigen::MatrixXi& connectivity_matrix);
			
			void extractSubmatrix(const Eigen::MatrixXd& fullmat, Eigen::MatrixXd& submat, const Eigen::VectorXi& rows, const Eigen::VectorXi& cols);
			void indexToCoords(const std::vector<int>& index, Eigen::VectorXi& coords);
			void reduceList(const std::vector<geometry_msgs::Pose>& in, std::vector<geometry_msgs::Pose>& out, const std::vector<int>& indexes);
			void poseListFrameTransform(const std::vector<geometry_msgs::Pose>& in, std::vector<geometry_msgs::Pose>& out, const std::string from_frame, const std::string to_frame);
			void poseFrameTransform(const geometry_msgs::Pose& in, geometry_msgs::Pose& out, const std::string from_frame, const std::string to_frame);
			void linearTwistListFrameTransform(const std::vector<geometry_msgs::Twist>& in, std::vector<geometry_msgs::Twist>& out, const std::string from_frame, const std::string to_frame);
			void linearTwistFrameTransform(const geometry_msgs::Twist& in, geometry_msgs::Twist& out, const std::string from_frame, const std::string to_frame);
			
			void poseListXYToArray(const std::vector<geometry_msgs::Pose>& shape, std_msgs::Float64MultiArray& node_array, const int node_count);
			void poseListXYToEigen(const std::vector<geometry_msgs::Pose>& shape, Eigen::VectorXd& node_coords, const int node_count);
			void eigenToPoseListXY(std::vector<geometry_msgs::Pose>& shape, const Eigen::VectorXd& node_coords, const int node_count);
			void twistListXYToEigen(const std::vector<geometry_msgs::Twist>& velocity, Eigen::VectorXd& node_coords, const int node_count);
			void eigenToTwistListXY(std::vector<geometry_msgs::Twist>& velocity, const Eigen::VectorXd& node_coords, const int node_count);
			void eigenToTwistXY(geometry_msgs::Twist& velocity, const Eigen::VectorXd& node_coords);
			
			void visionVelocity(std::vector<geometry_msgs::Twist>& velocity, const std::vector<geometry_msgs::Twist>& prev_velocity, const std::vector<geometry_msgs::Pose>& shape, const std::vector<geometry_msgs::Pose>& prev_shape);
			void shapePerception(std::vector<geometry_msgs::Pose>& perceived_shape, std::vector<geometry_msgs::Twist>& perceived_velocity);
			
			void shapeMerger(const std::vector<geometry_msgs::Pose>& estimated_shape, const std::vector<geometry_msgs::Pose>& perceived_shape, std::vector<geometry_msgs::Pose>& whole_shape);
			// Est_out = free_node_estimation(Measured_Node_Coord, Prev_Node_Coord)
			void shapeEstimation(std::vector<geometry_msgs::Pose>& estimated_shape, const std::vector<geometry_msgs::Pose>& perceived_shape, const std::vector<geometry_msgs::Twist>& perceived_twist, const Eigen::MatrixXd& K_struct);
			// dxps = controlLaw_Dyn(Shape_Target,Mp,dxpm,Node_Coord_t)
			void controlLaw(const std::vector<geometry_msgs::Pose>& estimated_shape, const std::vector<geometry_msgs::Pose>& target_shape, const Eigen::MatrixXd& K_struct, const std::vector<geometry_msgs::Twist>& master_twist, std::vector<geometry_msgs::Twist>& slave_twist, std::vector<geometry_msgs::Pose>& target_error);
			void setTarget(const std::vector<geometry_msgs::Pose>& initial_shape, const std::vector<geometry_msgs::Pose>& desired_shape, std::vector<geometry_msgs::Pose>& target_shape, const std::vector<geometry_msgs::Pose>& x_master, const double &ti, const double &tf);
			
			void twistSaturation(KDL::Twist tw, double lin, double ang);
			
			void fillInitialShape(std::vector<geometry_msgs::Pose>& shape);
			
			// tests
			void testTopics(void);
			void testStructure(void);
			void testConversions(void);
			void testSubmatrix(void);
			void testElemental(void);
			void testStructural(void);
			void testFrameTransform(void);
			void testTwistTransform(void);
			void testListOperations(void);
			void updateAll(void);
			void testControlLaw(void);

		private:
			// model parameters
			std::vector<double> A_; //section_area_;
			std::vector<double> E_; //youngs_modulus_;
			std::vector<double> L0_; //default_length_;
			Eigen::MatrixXi connectivity_matrix_;
			
			std::vector<int> index_master_{0}; // list of master nodes indices
			std::vector<int> index_slave_{10}; // list of slave nodes indices
			std::vector<int> index_target_{5, 6}; // list of target nodes indices
			std::vector<int> index_free_{1, 2, 3, 4, 7, 8, 9}; // list of free nodes indices
			std::vector<int> index_perceived_{0, 1, 2, 3, 4, 10}; // list of perceived nodes indices
			std::vector<int> index_vision_{1, 2, 3, 4}; // list of vision nodes indices
			std::vector<int> index_estimated_{5, 6, 7, 8, 9}; // list of estimated nodes indices
			Eigen::VectorXi vector_master_; // vector of master coords indices
			Eigen::VectorXi vector_slave_; // vector of slave coords indices
			Eigen::VectorXi vector_target_; // vector of target coords indices
			Eigen::VectorXi vector_free_; // vector of free coords indices
			Eigen::VectorXi vector_perceived_; // vector of perceived coords indices
			Eigen::VectorXi vector_estimated_; // vector of estimated coords indices
			
			
			int node_count_ = 11; // number of nodes
			int element_count_ = 10; // number of elements
			int dof_count_ = 2; // number of dofs in each nodes
			
			// trajectory generation
			double ti_, tf_;
			std::vector<geometry_msgs::Pose> x_master_;
			std::vector<geometry_msgs::Pose>* x_master_ptr_;
			std::vector<geometry_msgs::Pose> initial_shape_;
			std::vector<geometry_msgs::Pose> initial_target_shape_;
			std::vector<geometry_msgs::Pose> desired_shape_;
			std::vector<geometry_msgs::Pose>* desired_shape_ptr_;
			std::vector<geometry_msgs::Pose> target_shape_;
			unsigned char target_set_ = false;
			
			// perception
			std::vector<geometry_msgs::Pose> perceived_shape_;
			std::vector<geometry_msgs::Pose> vision_shape_;
			std::vector<geometry_msgs::Pose> vision_previous_shape_;
			std::vector<geometry_msgs::Pose>* vision_shape_ptr_;
			std::vector<geometry_msgs::Twist> vision_velocity_;
			std::vector<geometry_msgs::Twist> vision_previous_velocity_;
			std::vector<geometry_msgs::Twist> perceived_velocity_;
			std::vector<geometry_msgs::Pose> estimated_shape_;
			std::vector<geometry_msgs::Pose> whole_shape_;
			unsigned char new_vision_ = false;
			
			// control
			Eigen::MatrixXd K_struct_;
			Eigen::MatrixXd Gp_;
			std::vector<geometry_msgs::Pose> target_error_;
			std::vector<geometry_msgs::Twist> x_dot_master_;
			std::vector<geometry_msgs::Twist>* x_dot_master_ptr_;
			std::vector<geometry_msgs::Twist> x_dot_des_;	//desired velocity
			
			// robot driving
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			KDL::JntArray q_dot_cmd_; // computed set points
			ros::Time  last_time_, previous_time_, current_vision_time_, previous_vision_time_, current_time_, starting_time_;
			KDL::Frame x_;		//current pose
			KDL::JntArrayVel q_dot_current_;
			KDL::FrameVel x_frame_dot_current_;
			geometry_msgs::Pose x_msg_;
			geometry_msgs::Twist x_dot_msg_;
			KDL::Twist slave_twist_, previous_slave_twist_;
			
			// transforms
			tf::Transform cam_in_base_frame_, slave_xy_in_base_frame_, master_xy_in_base_frame_, master_base_in_base_frame_;
			tf::TransformListener listener_;
			tf::TransformBroadcaster br_;
			
			// buffers
			realtime_tools::RealtimeBuffer< std::vector<geometry_msgs::Pose> > x_master_buffer_;
			realtime_tools::RealtimeBuffer< std::vector<geometry_msgs::Twist> > x_dot_master_buffer_;
			realtime_tools::RealtimeBuffer< std::vector<geometry_msgs::Pose> > desired_shape_buffer_;
			realtime_tools::RealtimeBuffer< std::vector<geometry_msgs::Pose> > vision_shape_buffer_;
			
			// callbacks
			ros::Subscriber sub_desired_shape_;
			ros::Subscriber sub_vision_shape_;
			ros::Subscriber sub_master_pose_;
			ros::Subscriber sub_master_velocity_;
			int cmd_flag_;
			
			// publishers
			std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > realtime_x_dot_pub_;
			std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > realtime_x_pub_;
			//std::shared_ptr<realtime_tools::RealtimePublisher< std::vector<geometry_msgs::Pose> > > realtime_error_pub_;
			//std::shared_ptr<realtime_tools::RealtimePublisher< std::vector<geometry_msgs::Pose> > > realtime_whole_shape_pub_;
			//std::shared_ptr<realtime_tools::RealtimePublisher< std::vector<geometry_msgs::Pose> > > realtime_target_shape_pub_;
			ros::Publisher error_pub_;
			ros::Publisher whole_shape_pub_;
			ros::Publisher target_shape_pub_;
			
			std::ofstream my_log_file_;
			
			// functions
			inline void twistSaturation_(KDL::Twist& twist, const double limit_lin, const double limit_rot){
				for(int i=0; i<3; i++){
					if(twist.vel(i)>limit_lin) twist.vel(i) = limit_lin;
					if(twist.vel(i)<(-limit_lin)) twist.vel(i) = -limit_lin;
					if(twist.rot(i)>limit_rot) twist.rot(i) = limit_rot;
					if(twist.rot(i)<(-limit_rot)) twist.rot(i) = -limit_rot;
				}
			}
		
	};

}

#endif
