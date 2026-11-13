uint8 IDLE=0
uint8 SPEAKING=1
uint8 ERROR=2
std_msgs/Header header
uint8 state
string text                # utterance being spoken (SPEAKING) or that failed (ERROR)
string error
uint32 queued
