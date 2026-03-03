# Decider Notes

## cmd_vel Safety Rules

Decider no longer exposes dedicated `cmd_vel` clipping entries in `config.yaml`.
Command scaling/shaping is handled by control logic and state machines.
