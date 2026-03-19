#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dino_interfaces.action import Skill
from rclpy.action import ActionClient, ActionServer

# the action control stuff comes from: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html
class TaskExecutor(Node):
    def __init__(self):
        super().__init__('task_executor')

        self._action_client = ActionClient(self, Skill, 'execute_skill')

        # find these tasks in /workspace/data/* folder.
        # <HOME> is special skill which will move robot joints to home position
        self.tasks = {
                # "stash_fruit": ["robot_pickup_strawberry", "<HOME>", "item_drop_box", "<HOME>", "robot_pickup_kiwi", "<HOME>", "item_drop_box"]
                "stash_fruit": ["robot_pickup_kiwi", "<HOME>", "item_drop_box", "<HOME>", "robot_pickup_strawberry", "<HOME>", "item_drop_box"]
                }

        self.curr_task = "stash_fruit" # this should be a param but im lazy
        self.idx = 0

    def send_next_skill(self):
        next_skill = self.tasks[self.curr_task][self.idx]
        self.idx += 1

        self.get_logger().info(f"executing skill {next_skill}")
        
        self._action_client.wait_for_server()

        goal_msg = Skill.Goal()
        goal_msg.name = next_skill

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"{feedback.state}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.send_next_skill()
        else:
            self.get_logger().error(f"failed skill")
                

def main(args=None):
    rclpy.init(args=args)

    task_executor = TaskExecutor()

    try:
        # send the first skill to start the task
        task_executor.send_next_skill()
        rclpy.spin(task_executor)
    except KeyboardInterrupt:
        pass
    finally:
        task_executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


