#include <wb_state_publisher/WholeBodyStatePublisherController.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(wb_state_publisher::WholeBodyStatePublisherController,
		controller_interface::ControllerBase)

namespace wb_state_publisher
{

WholeBodyStatePublisherController::WholeBodyStatePublisherController() :
		num_meas_base_joints_(0), num_joints_(0), num_end_effectors_(0)
{

}


WholeBodyStatePublisherController::~WholeBodyStatePublisherController()
{

}


bool WholeBodyStatePublisherController::init(hardware_interface::JointStateInterface* hw,
											 ros::NodeHandle& controller_nh)
{
	assert(hw);

	// Getting the robot namespace. Note that this node is deploy in robot_ns/controller_ns
	std::string robot_namespace = ros::names::parentNamespace(controller_nh.getNamespace());
	ros::NodeHandle robot_node(robot_namespace);

	// Reading the robot config file
	std::string robot_config;
	robot_node.getParam("robot_config", robot_config);

	// Reading the robot model path in the ROS parameter server
	std::string model_path;
	if (!controller_nh.getParam("robot_model", model_path)) {
		ROS_WARN("Setting the default model path as %s/robot_model", robot_node.getNamespace().c_str());
		model_path = robot_node.getNamespace() + "/robot_model";
	}

	// Reading the urdf model
	XmlRpc::XmlRpcValue robot_model;
	if (!node_.getParam(model_path, robot_model)) {
		ROS_ERROR("Could not find the robot model");
		return false;
	}

	// Initializing the kinematic and dynamic models
	fbs_.resetFromURDFFile(robot_model, robot_config);
	wdyn_.reset(fbs_, wkin_);
	wkin_.reset(fbs_);

	// Reading floating system properties
	num_joints_ = fbs_.getJointDoF();
	num_end_effectors_ = fbs_.getNumberOfEndEffectors();
	joint_names_ = fbs_.getJointList();
	end_effector_names_ = fbs_.getEndEffectorList();

    // get all joint names from the hardware interface
    const std::vector<std::string>& hw_joint_names = hw->getNames();
	unsigned num_hw_joints = hw_joint_names.size();

	// Reading and setting up the actuated joint information
	joint_states_.resize(num_joints_);
	for (dwl::model::ElementId::const_iterator jnt_it = fbs_.getJoints().begin();
			jnt_it != fbs_.getJoints().end(); ++jnt_it) {
		std::string joint_name = jnt_it->first;
		unsigned int joint_id = jnt_it->second;

		// Getting the joint handle
		joint_states_[joint_id] = hw->getHandle(joint_name);
	}

	// Initializing the real-time whole-body state publisher
	controller_commons_.initWholeBodyStatePublisher(controller_nh, fbs_);

	// Create the real-time subscriber of state estimation
	if (num_meas_base_joints_ == 0) { // a full floating-base robot
		controller_commons_.initStateEstimationSubscriber(robot_node);
	}

	// Initializing the global variables
	robot_state_.setJointDoF(num_joints_);

	return true;
}


void WholeBodyStatePublisherController::starting(const ros::Time& time)
{
	// Initializing the current time
	robot_state_.time = time.toSec();

	// Initializing the current base state
   	controller_commons_.updateStateEstimationSubscription(robot_state_.base_pos,
   														  robot_state_.base_vel);


    // Initializing the current joint state
	for (unsigned int joint_idx = 0; joint_idx < num_joints_; joint_idx++) {
		robot_state_.joint_pos(joint_idx) = joint_states_[joint_idx].getPosition();
		robot_state_.joint_vel(joint_idx) = joint_states_[joint_idx].getVelocity();
    	robot_state_.joint_acc(joint_idx) = 0.;
		robot_state_.joint_eff(joint_idx) = joint_states_[joint_idx].getEffort();
	}

	// Computing the initial contact information
	computeContactState();
}


void WholeBodyStatePublisherController::update(const ros::Time& time,
											   const ros::Duration& period)
{
	// Updating the current time
	robot_state_.time = time.toSec();

	// Updating the current base state
   	controller_commons_.updateStateEstimationSubscription(robot_state_.base_pos,
   														  robot_state_.base_vel);

    // Updating the current joint state
	for (unsigned int joint_idx = 0; joint_idx < num_joints_; joint_idx++) {
		robot_state_.joint_pos(joint_idx) = joint_states_[joint_idx].getPosition();
    	robot_state_.joint_acc(joint_idx) = (joint_states_[joint_idx].getVelocity() -
    			robot_state_.joint_vel(joint_idx)) / period.toSec();
		robot_state_.joint_vel(joint_idx) = joint_states_[joint_idx].getVelocity();
		robot_state_.joint_eff(joint_idx) = joint_states_[joint_idx].getEffort();
	}

	// Computing the contact information
	computeContactState();

	// Initializing the real-time whole-body state publisher
	controller_commons_.publishWholeBodyState(time, robot_state_);
}


void WholeBodyStatePublisherController::stopping(const ros::Time& time)
{

}


void WholeBodyStatePublisherController::computeContactState()
{
	// Computing the contact positions w.r.t the base reference system
	robot_state_.setContactSE3_W(
			wkin_.computePosition(robot_state_.getBaseSE3(),
								  robot_state_.getJointPosition(),
						  	  	  end_effector_names_));

	// Computing the contact velocities
	robot_state_.setContactVelocity_W(
			wkin_.computeVelocity(robot_state_.getBaseSE3(),
								  robot_state_.getJointPosition(),
								  robot_state_.getBaseVelocity_W(),
								  robot_state_.getJointVelocity(),
								  end_effector_names_));

	// Computing the contact accelerations
	robot_state_.setContactAcceleration_W(
			wkin_.computeAcceleration(robot_state_.getBaseSE3(),
					  	  	  	  	  robot_state_.getJointPosition(),
									  robot_state_.getBaseVelocity_W(),
									  robot_state_.getJointVelocity(),
									  robot_state_.getBaseAcceleration_W(),
									  robot_state_.getJointAcceleration(),
									  end_effector_names_));

	// Computing the contact forces
	wdyn_.estimateContactForces(robot_state_.contact_eff,
								dwl::SE3(),
								robot_state_.getJointPosition(),
								robot_state_.getBaseVelocity_W(),
								robot_state_.getJointVelocity(),
								robot_state_.getBaseAcceleration_W(),
								robot_state_.getJointAcceleration(),
								robot_state_.getJointEffort(),
								end_effector_names_);
}

} //@namespace wb_state_publisher
