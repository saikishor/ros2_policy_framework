# Fault injected through a policy_framework/MockSafetyCheck, to see how the robot reacts to a
# fault at a chosen moment. The latest message holds until the next one.

# true: the check fails on every cycle until a message with false.
bool fault
# Reaction asked for: neutral_commands, hold, damp, safe or error; it must be listed in the
# check's `reactions` (else the host applies error). Empty: the check's on_fault.
string reaction
# Why, added to the logged reason.
string reason
# Seconds after which the fault ends by itself (e.g. a brief fault while walking, for a
# neutral_commands reaction that recovers); 0: until the next message.
float64 duration
