#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import GetModelState

def wait_for_model(model_name):
    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.loginfo("Waiting for model [%s] to spawn in Gazebo..." % model_name)
    while not rospy.is_shutdown():
        try:
            resp = get_state(model_name, "")
            if resp.success:
                rospy.loginfo("Model [%s] found in Gazebo." % model_name)
                return
        except:
            pass
        rospy.sleep(0.5)

def set_joints():
    rospy.init_node("set_initial_joints")

    model_name = "jamal"   # same as in spawn_urdf

    # Wait until robot is spawned
    wait_for_model(model_name)

    rospy.wait_for_service("/gazebo/set_model_configuration")
    try:
        set_config = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        resp = set_config(
            model_name=model_name,
            urdf_param_name="legged_robot_description",
            joint_names=[
                "RF_HAA", "RF_HFE", "RF_KFE",
                "LF_HAA", "LF_HFE", "LF_KFE",
                "RH_HAA", "RH_HFE", "RH_KFE",
                "LH_HAA", "LH_HFE", "LH_KFE"
            ],
            joint_positions=[
                0.0, 1.396, -2.670,
                0.0, 1.396, -2.670,
                0.0, -1.396, 2.670,
                0.0, -1.396, 2.670
            ]
        )
        if resp.success:
            rospy.loginfo("Initial joint positions set successfully ✅")
        else:
            rospy.logwarn("Failed to set initial joint positions: " + resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    set_joints()
