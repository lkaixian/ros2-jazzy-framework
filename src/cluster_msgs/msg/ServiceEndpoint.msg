# A network service one of this host's RUNNING jobs offers (advertised in the heartbeat).
# Consumers (e.g. llm_bridge) treat the host going silent as the endpoint leaving.
string job_id
string kind        # e.g. llm
string url         # http://<advertise_host>:<port>
int32 priority     # lower = preferred
