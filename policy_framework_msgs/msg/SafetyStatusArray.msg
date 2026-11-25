# Safety state of a policy_framework runner (node or controller), published latched on
# `~/safety_status` when it changes: when a check starts or stops failing, or its reaction
# changes. Empty statuses: every check passes.
#
# A neutral_commands entry leaves once its check passes again. Any other reaction stops the
# policy until the host is reactivated or its safety is reset (`~/reset_safety`): its entry
# stays until then, also once its check passes again, so that the cause of the stop stays
# visible.

# When it changed.
builtin_interfaces/Time stamp
# Policy active when it changed.
string active_policy
# Reaction in effect, the most severe one; empty when every check passes.
string reaction
# Every failing check.
SafetyStatus[] statuses
