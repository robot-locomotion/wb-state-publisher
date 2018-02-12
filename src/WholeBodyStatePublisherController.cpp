#include <wb_state_publisher/WholeBodyStatePublisherController.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(wb_state_publisher::WholeBodyStatePublisherController,
		controller_interface::ControllerBase)

namespace wb_state_publisher
{

WholeBodyStatePublisherController::WholeBodyStatePublisherController() :
		num_base_joints_(0), num_meas_base_joints_(0), num_joints_(0),
		num_end_effectors_(0)
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
	fbs_.resetFromURDFModel(robot_model, robot_config);
	wkin_.reset(fbs_);
	wdyn_.reset(fbs_, wkin_);

	// Reading floating system properties
	num_base_joints_ = fbs_.getFloatingBaseDoF();
	num_joints_ = fbs_.getJointDoF();
	num_end_effectors_ = fbs_.getNumberOfEndEffectors();
	joint_names_ = fbs_.getJointNames();
	end_effector_names_ = fbs_.getEndEffectorNames();

    // get all joint names from the hardware interface
    const std::vector<std::string>& hw_joint_names = hw->getNames();
	unsigned num_hw_joints = hw_joint_names.size();


	// In a partially floating-base system, we model the base with different joints. In this case,
	// it suppose that we measure the joint states in the base joints (simulation and
	// robot). For simplicity, we take advantage of this condition in simulation by applied a
	// virtual PID controllers in the base joints, this allows us to keep the floating-base on air
	// in simulation
	// Reading and setting up the base information
	for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
		dwl::rbd::Coords6d base_id = dwl::rbd::Coords6d(base_idx);
		dwl::model::FloatingBaseJoint base_joint = fbs_.getFloatingBaseJoint(base_id);

		if (base_joint.active) {
			// Getting the base joint handle in cases that of non-full floating-base robot
			for (unsigned int joint_idx = 0; joint_idx < num_hw_joints; joint_idx++) {
				if (base_joint.name == hw_joint_names[joint_idx]) {
					// Getting the base joint handle in cases that of non-full floating-base robot
					base_joint_states_.push_back(hw->getHandle(base_joint.name));
					++num_meas_base_joints_;
				}
			}
		}
	}

	// Reading and setting up the actuated joint information
	joint_states_.resize(num_joints_);
	dwl::urdf_model::JointID joints = fbs_.getJoints();
	for (dwl::urdf_model::JointID::const_iterator joint_it = joints.begin();
			joint_it != joints.end(); joint_it++) {
		std::string joint_name = joint_it->first;
		unsigned int joint_id = joint_it->second;

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
    for (unsigned int base_idx = 0; base_idx < num_meas_base_joints_; base_idx++) {
    	unsigned int base_coord = fbs_.getFloatingBaseJointCoordinate(base_idx);
    	robot_state_.base_pos(base_coord) = base_joint_states_[base_idx].getPosition();
    	robot_state_.base_vel(base_coord) = base_joint_states_[base_idx].getVelocity();
    	robot_state_.base_acc(base_coord) = 0.;
    }
    if (num_meas_base_joints_ == 0) { // full floating-base robot
    	controller_commons_.updateStateEstimationSubscription(robot_state_.base_pos,
    														  robot_state_.base_vel);
    }

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
    for (unsigned int base_idx = 0; base_idx < num_meas_base_joints_; base_idx++) {
    	unsigned int base_coord = fbs_.getFloatingBaseJointCoordinate(base_idx);
    	robot_state_.base_pos(base_coord) = base_joint_states_[base_idx].getPosition();
    	robot_state_.base_acc(base_idx) = (base_joint_states_[base_idx].getVelocity() -
    			robot_state_.base_vel(base_idx)) / period.toSec();
    	robot_state_.base_vel(base_coord) = base_joint_states_[base_idx].getVelocity();
    }
    if (num_meas_base_joints_ == 0) { // full floating-base robot
    	controller_commons_.updateStateEstimationSubscription(robot_state_.base_pos,
    														  robot_state_.base_vel);
    }

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
	wkin_.computeForwardKinematics(robot_state_.contact_pos,
								   dwl::rbd::Vector6d::Zero(), robot_state_.joint_pos,
								   end_effector_names_, dwl::rbd::Linear);

	// Computing the contact velocities
	wkin_.computeVelocity(robot_state_.contact_vel,
						  dwl::rbd::Vector6d::Zero(), robot_state_.joint_pos,
						  dwl::rbd::Vector6d::Zero(), robot_state_.joint_vel,
						  end_effector_names_, dwl::rbd::Linear);

	// Computing the contact accelerations
	wkin_.computeAcceleration(robot_state_.contact_acc,
							  dwl::rbd::Vector6d::Zero(), robot_state_.joint_pos,
							  dwl::rbd::Vector6d::Zero(), robot_state_.joint_vel,
							  dwl::rbd::Vector6d::Zero(), robot_state_.joint_acc,
							  end_effector_names_, dwl::rbd::Linear);

	// Computing the contact forces
	wdyn_.estimateContactForces(robot_state_.contact_eff,
								dwl::rbd::Vector6d::Zero(), robot_state_.joint_pos,
								robot_state_.base_vel, robot_state_.joint_vel,
								robot_state_.base_acc, robot_state_.joint_acc,
								robot_state_.joint_eff, end_effector_names_);
}

} //@namespace wb_state_publisher
