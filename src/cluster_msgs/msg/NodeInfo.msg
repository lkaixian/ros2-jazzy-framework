# Heartbeat of one machine (~2 Hz). The scheduler uses its OWN receive time, not header.stamp.
std_msgs/Header header
string host_id
string[] roles           # e.g. core, compute
string[] tags            # e.g. camera, audio, gpu
uint32 cpu_cores
float32 cpu_load         # whole machine, 0..1
float32 mem_total_mb
float32 mem_free_mb      # MemAvailable
float32 gpu_mem_total_mb # -1: no GPU / unknown
float32 gpu_mem_free_mb  # -1: no GPU / unknown
float32 gpu_util         # 0..1, -1 unknown
float32 temp_c           # hottest thermal zone, -1 unknown
float32 battery_pct      # laptop battery 0..100, -1 no battery / unknown
int8 charging            # 1 charging or full on AC, 0 discharging, -1 unknown
string catalog_hash      # hash of this agent's local job catalog
uint64 seq
uint64 assignment_epoch  # epoch of the last AssignmentSet this agent accepted (0 = none)
JobStatus[] jobs
ServiceEndpoint[] endpoints   # services of RUNNING jobs that declare `endpoint:` in the catalog
