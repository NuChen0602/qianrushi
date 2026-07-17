# Candidate root causes

The reported behavior is consistent with PMON treating a non-timer input during countdown as a permanent delay/menu condition. It is not confirmed for this firmware because exact source and a board serial capture have not yet been obtained. Other possibilities include a boot menu policy, serial noise, or a configuration that has not survived a true cold start. The read-only serial capture and audit are required before a configuration change.
