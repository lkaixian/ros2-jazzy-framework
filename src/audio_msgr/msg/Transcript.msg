std_msgs/Header header     # stamp = end of the utterance
string text
string language            # "" if the engine does not report it
float32 confidence         # [0,1]; -1 if unavailable
bool is_final
float32 duration_sec
string engine
