import rclpy
from geometry_msgs.msg import Pose
from sorting_arm_motion_planner.srv import MoveToTarget

_client = None
_node = None


def _initialize_client():
  global _client, _node
  if _client is not None:
    return

  if not rclpy.ok():
    rclpy.init()

  if _node is None:
    _node = rclpy.create_node('motion_planner_client')

  _client = _node.create_client(MoveToTarget, '/move_to_pose')

  if not _client.wait_for_service(timeout_sec=10.0):
    raise RuntimeError("Motion planner service /move_to_pose not available")


def move_to_pose(pose: Pose) -> bool:
  """
  Move the robot to the target pose.

  Args:
    pose: geometry_msgs.msg.Pose target pose

  Returns:
    bool: True if movement succeeded, False otherwise
  """
  _initialize_client()

  request = MoveToTarget.Request()
  request.target_pose = pose

  future = _client.call_async(request)

  while rclpy.ok() and not future.done():
    rclpy.spin_once(_node, timeout_sec=0.1)

  if not future.done():
    return False

  try:
    response = future.result()
    return response.success
  except Exception:
    return False
