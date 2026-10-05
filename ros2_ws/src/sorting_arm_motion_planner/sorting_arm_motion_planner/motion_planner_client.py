import threading

from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sorting_arm_motion_planner.srv import MoveToTarget


class MotionPlannerClient:
  """
  Client for the /move_to_pose service, attached to a caller-owned node.

  The client never calls rclpy.init() or spins anything itself: the owning
  node must be spun by an executor running in another thread (e.g. a
  MultiThreadedExecutor), which delivers the service response while
  move_to_pose() blocks waiting for it.
  """

  def __init__(self, node: Node, service_name: str = '/move_to_pose',
               service_timeout_sec: float = 10.0):
    self._node = node
    self._client = node.create_client(
      MoveToTarget, service_name, callback_group=ReentrantCallbackGroup())

    if not self._client.wait_for_service(timeout_sec=service_timeout_sec):
      raise RuntimeError(f"Motion planner service {service_name} not available")

  def move_to_pose(self, pose: Pose, timeout_sec: float = 120.0) -> bool:
    """
    Move the robot to the target pose.

    Args:
      pose: geometry_msgs.msg.Pose target pose for the arm group's tip link
      timeout_sec: how long to wait for planning + execution to finish

    Returns:
      bool: True if movement succeeded, False otherwise
    """
    request = MoveToTarget.Request()
    request.target_pose = pose

    done = threading.Event()
    future = self._client.call_async(request)
    future.add_done_callback(lambda _: done.set())

    if not done.wait(timeout_sec):
      self._node.get_logger().error(f"/move_to_pose did not respond within {timeout_sec}s")
      future.cancel()
      return False

    try:
      return future.result().success
    except Exception as e:
      self._node.get_logger().error(f"/move_to_pose call failed: {e}")
      return False
