#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("guided_test")

    ns = "uav0"
    state_sub = rospy.Subscriber(f"/{ns}/mavros/state", State, callback = state_cb)

    ## Publishers
    local_pos_pub = rospy.Publisher(f"/{ns}/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    ## Service Clients
    rospy.wait_for_service(f"/{ns}/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy(f"/{ns}/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service(f"/{ns}/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(f"/{ns}/mavros/set_mode", SetMode)

    rospy.wait_for_service(f"/{ns}/mavros/cmd/takeoff")
    takeoff_client = rospy.ServiceProxy(f"/{ns}/mavros/cmd/takeoff", CommandTOL)

    ## Setpoint Publishing
    rate = rospy.Rate(30)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rospy.loginfo("Waiting for FCU connection")
        rate.sleep()


    ## Messages to be sent
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1

    ## SET MODE
    guided_mode = SetModeRequest()
    guided_mode.custom_mode = 'GUIDED'

    ## ARM
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    ## TAKEOFF
    takeoff_cmd = CommandTOLRequest()
    takeoff_cmd.altitude = 0.2
    takeoff_status = False

    last_req = rospy.Time.now()
    while(not rospy.is_shutdown()):
        if(current_state.mode != "GUIDED" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(guided_mode).mode_sent == True):
                rospy.loginfo("GUIDED enabled")
            last_req = rospy.Time.now()
        
        elif(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")
            last_req = rospy.Time.now()

        elif(not takeoff_status and current_state.armed and current_state.mode == "GUIDED" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            takeoff_resp = takeoff_client.call(takeoff_cmd)
            if (takeoff_resp.success or takeoff_resp.result == 4):
                takeoff_status = True
                rospy.loginfo("Vehicle in air")
            last_req = rospy.Time.now()
                    
        else:
            if (takeoff_status and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                rospy.loginfo_once("Publishing setpoint")
                local_pos_pub.publish(pose)

        rate.sleep()
