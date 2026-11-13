# One row of ClusterStatus.
string job_id
string host_id           # "" if not placed
string state             # placed | pending (no capacity) | unplaceable (constraints / pinned host down)
string reason
string job_state         # running | backoff | failed | stopping | stopped | unknown (host heartbeat)
