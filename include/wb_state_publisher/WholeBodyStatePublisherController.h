#ifndef WB_STATE_PUBLISHER__WHOLE_BODY_STATE_PUBLISHER_CONTROLLER__H
#define WB_STATE_PUBLISHER__WHOLE_BODY_STATE_PUBLISHER_CONTROLLER__H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

#include <dwl_msgs/ControllerCommons.h>

#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/WholeBodyDynamics.h>



namespace wb_state_publisher
{

class WholeBodyStatePublisherController :
		public controller_interface::Controller<hardware_interface::JointStateInterface>
{
	public:
		/** @brief Constructor function */
		WholeBodyStatePublisherController();

		/** @brief Destructor function */
		~WholeBodyStatePublisherController();

		/** @brief The init function is called to initialize the controller from a
		 * non-realtime thread with a pointer to the hardware interface, itself,
		 * instead of a pointer to a RobotHW.
		 * @param hardware_interface::JointStateInterface* The specific hardware interface
		 * used by this controller.
		 * @param ros::NodeHandle& A NodeHandle in the namespace from which the controller
		 * should read its configuration, and where it should set up its ROS
		 * interface.
		 * @returns True if initialization was successful and the controller
		 * is ready to be started.
		 */
		bool init(hardware_interface::JointStateInterface* hw,
				  ros::NodeHandle& controller_nh);

		/** @brief This is called from within the realtime thread just before the
		 * first call to @ref update
		 * @param const ros::Time& The current time
		 */
		void starting(const ros::Time& time);

		/** @brief This is called periodically by the realtime thread when the controller is running
		 * @param const ros::Time&  The current time
		 * @param const ros::Duration& The time passed since the last call to @ref update
		 */
		void update(const ros::Time& time,
					const ros::Duration& period);

		/** @brief This is called from within the realtime thread just after the last
		 * update call before the controller is stopped
		 * @param const ros::Time& The current time
		 */
		void stopping(const ros::Time& time);


	private:
		void computeContactState();

		/** @brief Ros node handle */
		ros::NodeHandle node_;

		/** @brief Controller common utilities */
		dwl_msgs::ControllerCommons controller_commons_;

		/** @brief Joint names */
		std::vector<std::string> joint_names_;

		/** @brief End-effector names */
		std::vector<std::string> end_effector_names_;

		/** @brief Number of measurable base joints */
		unsigned int num_meas_base_joints_;

		/** @brief Number of joints */
		unsigned int num_joints_;

		/** @brief Number of end-effectors */
		unsigned int num_end_effectors_;

		/** @brief Whole-body state of the robot */
		dwl::WholeBodyState robot_state_;

		/** @brief Floating-base system information */
		dwl::model::FloatingBaseSystem fbs_;

		/** @brief Whole-body kinematics model */
		dwl::model::WholeBodyKinematics wkin_;

		/** @brief Whole-body dynamics */
		dwl::model::WholeBodyDynamics wdyn_;

		/** @brief Base joint handles */
		std::vector<hardware_interface::JointStateHandle> base_joint_states_;

		/** @brief Joint handles */
		std::vector<hardware_interface::JointStateHandle> joint_states_;
};

} //@namespace wb_state_publisher

#endif
