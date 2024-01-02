#! /usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import Float64
from mavros_msgs.msg import State, Trajectory
from geometry_msgs.msg import PoseStamped, Transform, Twist

class UAVController:
    def __init__(self, ns="uav0"):
        rospy.init_node(f"{ns}_controller")

        ## Subscribers
        rospy.Subscriber(f"/{ns}/mavros/state", State, callback = self.state_cb)
        rospy.Subscriber(f"/{ns}/mavros/global_position/rel_alt", Float64, callback = self.alt_cb)
        
        ## Publishers
        self.pos_pub = rospy.Publisher(f"/{ns}/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        ## roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.scriptapi.ROSLaunch()

        ## Control Loop
        self.rate = rospy.Rate(30)

        # Variables
        self.ns = ns
        self.takeoff_alt   = 1.0
        self.current_state = State()
        self.current_alt   = None
        self.target_pose   = PoseStamped()
        self.target_pose.header.frame_id = "map"

    def state_cb(self,msg):
        self.current_state = msg

    def alt_cb(self,msg):
        self.current_alt = msg.data

    def init_system(self):
        rospy.loginfo_once("Initializing system")
        self.launch.start()
        ns = self.ns

        ## setmode GUIDED
        ## rosrun mavros mavsys -n 'uav0/mavros' mode -c 4 # "GUIDED",4 ; "STABILIZE",0
        set_mode_node = roslaunch.core.Node('mavros', 'mavsys', name="mavsys", namespace=f"{ns}", args=f"-n '{ns}/mavros' mode -c 4")
        self.launch.launch(set_mode_node)

        ## ARM
        ## rosrun mavros mavsafety -n 'uav0/mavros' arm
        arm_node = roslaunch.core.Node('mavros', 'mavsafety', name="mavsafety", namespace=f"{ns}", args=f"-n '{ns}/mavros' arm")
        self.launch.launch(arm_node)
        
        ## TAKEOFF
        ## rosrun mavros mavcmd -n 'uav0/mavros' takeoffcur {min_pitch yaw altitude}
        cmd_node = roslaunch.core.Node('mavros', 'mavcmd', name="mavcmd", namespace=f"{ns}", args=f"-n '{ns}/mavros' takeoffcur 0 0 {self.takeoff_alt}")
        self.launch.launch(cmd_node)
        rospy.loginfo("System initialized")

    def control_cycle(self):
        ## Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rospy.loginfo("Waiting for FCU connection")
            self.rate.sleep()
        
        ## Takeoff
        if not self.current_state.armed:
            self.init_system()
        else:
            rospy.loginfo("Vehicle already armed, skipping initialization")

        ## Main Loop
        rospy.loginfo("UAV ready")
        while(not rospy.is_shutdown()):
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position.z = self.takeoff_alt
            self.pos_pub.publish(self.target_pose)
            self.rate.sleep()
    

if __name__ == "__main__":
    uav0 = UAVController()
    try:
        uav0.control_cycle()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down UAV Controller")
        pass

