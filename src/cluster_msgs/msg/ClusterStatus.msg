std_msgs/Header header
string scheduler_id
uint64 epoch
string[] hosts_alive
string[] hosts_dead
string[] hosts_excluded  # alive, but catalog hash differs from the scheduler's
JobPlacement[] jobs
string[] events          # recent decisions (failover, migration, quarantine)
