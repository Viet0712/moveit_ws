#!/usr/bin/env python3
from moveit_configs_utils import MoveItConfigsBuilder
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import tf_transformations
from std_msgs.msg import Bool , Float64MultiArray
ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config")\
                                    .robot_description_semantic("config/my_robot.srdf", {"name": "my_robot"})\
                                    .to_dict()

ROBOT_CONFIG = { 
    **ROBOT_CONFIG,
    "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1
        }
}
class CommanderNode(Node):
    def __init__(self):
        super().__init__("commander_node")
        self.robot_ = MoveItPy(node_name="moveit_py",config_dict=ROBOT_CONFIG)
        self.arm_ : PlanningComponent = self.robot_.get_planning_component("arm")
        self.gripper_:PlanningComponent = self.robot_.get_planning_component("gripper")
        self.open_gripper_sub_ = self.create_subscription(Bool,"open_gripper",self.call_back_open_gripper,10)
        self.joint_cmd_sub_ = self.create_subscription(Float64MultiArray,"joint_cmd",self.call_back_joint_cmd,10)
        self.pose_cmd_sub_ = self.create_subscription(Float64MultiArray,"pose_cmd",self.call_back_pose_cmd,10)
    def go_to_named_target(self,name):
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(configuration_name=name)
        self.plan_and_execute(self.arm_)

    def go_to_joint(self,joint_goal_list):
        if len(joint_goal_list) != 6:
            return
        robot_state = RobotState(self.robot_.get_robot_model())
        joint_goal = [
            joint_goal_list[0],  # joint1
            joint_goal_list[1],  # joint2
            joint_goal_list[2],  # joint3
            joint_goal_list[3],  # joint4
            joint_goal_list[4],  # joint5
            joint_goal_list[5],  # joint6
        ]
        robot_state.set_joint_group_positions("arm", joint_goal)
        robot_state.update()
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(robot_state=robot_state)
        self.plan_and_execute(self.arm_)

    def go_to_pose_target(self,x,y,z,roll,pitch,yaw):
        q_x ,q_y,q_z ,q_w = tf_transformations.quaternion_from_euler(roll,pitch,yaw)
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.x = q_x
        pose_goal.pose.orientation.y = q_y
        pose_goal.pose.orientation.z = q_z
        pose_goal.pose.orientation.w = q_w
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool_link")
        self.plan_and_execute(self.arm_)

    def open_gripper(self):
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_open")
        self.plan_and_execute(self.gripper_)

    def closed_gripper(self):
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_closed")
        self.plan_and_execute(self.gripper_)  

    def plan_and_execute(self,inteface):
        plan_result = inteface.plan()
        if plan_result:
            self.robot_.execute(plan_result.trajectory,controllers=[])

    def call_back_open_gripper(self,msg:Bool):
        if msg.data:
            self.open_gripper()
        else:
            self.closed_gripper()
    def call_back_joint_cmd(self,msg:Float64MultiArray):
        # data = msg.data
        # joint_goal_list = [data[0],data[1],data[2],data[3],data[4],data[5]]
        self.go_to_joint(msg.data)
    def call_back_pose_cmd(self,msg:Float64MultiArray):
        data = msg.data
        if len(data) != 6:
            return
        x = data[0]
        y = data[1]
        z = data[2]
        roll = data[3]
        pitch = data[4]
        yaw = data[5]
        self.go_to_pose_target(x,y,z,roll,pitch,yaw)              
def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    rclpy.spin(node)
    rclpy.shutdown()    