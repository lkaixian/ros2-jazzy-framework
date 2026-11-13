# State of one job on one host, as seen by that host's agent.
uint8 STOPPED=0
uint8 RUNNING=1
uint8 BACKOFF=2      # crashed, waiting to restart
uint8 FAILED=3       # too many restarts; stays FAILED until the job is unassigned
uint8 STOPPING=4

string job_id
uint8 state
uint32 restarts
int32 pid            # first process of the job, -1 if none
string last_error
