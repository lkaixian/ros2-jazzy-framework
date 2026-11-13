# The complete desired placement. Published transient_local and re-published ~1 Hz.
std_msgs/Header header
string scheduler_id
uint64 epoch             # increases whenever the placement changes
string catalog_hash      # agents with a different catalog refuse this set
Assignment[] assignments
