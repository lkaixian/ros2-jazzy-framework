std_msgs/Header header
string task
string task_spec           # JSON produced by the llama.cpp stage
string xml                 # BehaviorTree.CPP v4 XML (last attempt)
bool valid
string[] errors            # validator errors of the last attempt (empty if valid)
uint8 attempts
