"""Topic names and QoS shared by agents, scheduler and CLIs."""
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TOPIC_INFO = 'cluster/node_info'
TOPIC_ASSIGN = 'cluster/assignments'
TOPIC_STATUS = 'cluster/status'

# heartbeats: reliable, not latched (a stale heartbeat must never be replayed to a new subscriber)
INFO_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
# desired state / status: latched so a late joiner or restarted agent gets the current picture
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
