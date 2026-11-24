# A policy loaded by a policy_framework runner (node or controller), as configured.

# Name, as listed in the `policies` parameter.
string name
# Plugin classes of its stages.
string preprocessor
string inference
string postprocessor
# > 0: the runner returns to the default policy after this many steps; 0: never.
int32 max_steps
# false: switch requests are rejected while this policy is active.
bool interruptible
