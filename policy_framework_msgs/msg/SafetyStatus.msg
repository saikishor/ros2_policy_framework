# One failing safety check of a policy_framework runner (node or controller), an entry of
# SafetyStatusArray.

# What failed: a safety_checks name, a source name, tilt, joint_position_limits, max_input_age,
# policy_input (a stale input of the policy) or policy_failure (the policy step failed).
string name
# Its reaction: neutral_commands, hold, damp, safe or error.
string reaction
# Why, as logged. Taken when the entry appears or its reaction changes: a value in it (e.g. the
# age of a stale input) is not updated while the entry stays.
string reason
