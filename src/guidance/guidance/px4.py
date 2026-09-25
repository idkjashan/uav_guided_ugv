"""PX4 uXRCE-DDS plumbing shared by the guidance nodes."""

from __future__ import annotations

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    import px4_msgs.msg as px4_msgs
except ImportError as exc:  # pragma: no cover - environment issue
    raise SystemExit('px4_msgs not found: source ~/px4_ros_ws/install/setup.bash') from exc

# PX4's DDS client publishes best effort; a reliable subscriber gets nothing.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST,
                     depth=10)


def subscribe_versioned(node, type_name, topic, callback, group=None):
    """Subscribe to ``topic`` and ``topic_v1``.

    PX4 1.16 appends the message version to versioned topics (for example
    /fmu/out/vehicle_status_v1), and which ones are versioned depends on the
    exact PX4 commit. Only one of the two will ever publish.
    """
    msg_type = getattr(px4_msgs, type_name)
    for name in (topic, topic + '_v1'):
        node.create_subscription(msg_type, name, callback, PX4_QOS, callback_group=group)

