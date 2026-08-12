# V062: target confirmation and automatic safe task stop

This version keeps the existing three-terminal startup sequence unchanged.

## Success rule

The metrics logger evaluates only `target_candidate` objects belonging to the
current `query_version`. The first object satisfying one of the following
conditions latches trial success:

1. `target_state == CONFIRMED`;
2. `target_state == LIKELY` and `target_confidence >= 0.75`;
3. `target_confidence >= 0.80` (compatibility fallback).

A low-confidence `POSSIBLE` marker is therefore displayed on the semantic map
without prematurely terminating the experiment.

The thresholds are launch parameters of `search_metrics_logger.py`:

```xml
<param name="likely_success_confidence" value="0.75"/>
<param name="success_confidence" value="0.80"/>
```

## Automatic stop signal

At first success, `search_metrics_logger.py` publishes a latched JSON message:

```text
/experiment/task_finished
```

The signal causes:

- both UAV executors to clear active/pending routes and hold the current pose;
- the UGV executor to clear its route and continuously command zero velocity;
- the trigger scheduler to stop emitting new VLM events;
- the synchronous coordinator to clear queued events and leave Gazebo running;
- the central planner to reject new plan requests;
- the metrics logger to save `summary.json`, `summary.csv`, and
  `time_series.csv`.

ROS, Gazebo, PX4, and the three terminals remain alive for post-trial
inspection. Stop them manually before starting the next trial.

Calling:

```bash
rosservice call /experiment_metrics/finish
```

also publishes the terminal stop signal, but records `success=0` unless a
target was already confirmed automatically.

## Runtime checks

Inspect the terminal signal:

```bash
rostopic echo -n 1 /experiment/task_finished
```

Inspect the evidence used by the logger:

```bash
rostopic echo -n 1 /semantic_overlay/summary
```

Expected terminal statuses:

```bash
rostopic echo -n 1 /uav0/mission/status
rostopic echo -n 1 /uav1/mission/status
rostopic echo -n 1 /ugv0/mission/status
rostopic echo -n 1 /vlm/trigger_status
rostopic echo -n 1 /vlm/sync_status
```

They should report `task_finished_hover`, `task_finished_hold`, or
`TASK_FINISHED` as applicable.
