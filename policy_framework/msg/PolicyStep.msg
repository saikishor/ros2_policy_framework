# One step of a policy: what the model was given, what it answered and what was commanded.
# Published by a policy_framework runner (node or controller) with `publish_steps`, to record
# and to run the same inputs through a model again (policy_replay).

uint8 STATUS_OK = 0                # commands were written
uint8 STATUS_WAITING_FOR_DATA = 1  # an input is missing: the model did not run, the tensors are
                                   # those of the previous step
uint8 STATUS_ERROR = 2             # a stage failed

# stamp: when the step finished (node clock).
std_msgs/Header header
# Active policy.
string policy
# Successful steps of the policy since it was activated, this one included.
uint64 step
uint8 status
# Duration of the step (preprocessor, inference, postprocessor), microseconds.
float64 step_time_us
# Stamp of the observation in seconds: its oldest input, those fed back from the previous
# outputs excluded. 0: unknown.
float64 observation_stamp
# Stamp in seconds of every input of the preprocessor (per observation term or state entry).
string[] input_names
float64[] input_stamps
# Model inputs and outputs.
Tensor[] inputs
Tensor[] outputs
# Every command entry after the step.
string[] command_names
float64[] commands
