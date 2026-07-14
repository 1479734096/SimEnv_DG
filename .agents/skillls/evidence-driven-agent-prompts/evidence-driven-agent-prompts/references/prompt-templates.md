# Prompt Templates

## Contents

1. Main executor
2. Read-only reviewer
3. Reviewed-plan implementation
4. Stage checkpoint and handoff
5. Efficiency optimization
6. Existing-prompt audit

Use these as adaptable structures. Remove irrelevant sections instead of filling them with generic text.

---

## 1. Main executor

```text
You are the primary execution agent for this stage.

## Current objective
[Define one observable outcome.]

## Current checkpoint
- Repository/path:
- Current HEAD:
- Relevant run directories:
- Known good baseline:
- Current blocking issue:

## Confirmed facts
- [Fact supported by code/log/test.]
- [Fact supported by measured behavior.]

## Working hypotheses
- [Plausible cause not yet proven.]
- [Alternative explanation that must be distinguished.]

The hypotheses above are not mandatory implementations. Independently determine the root cause from production-path evidence.

## Validated behavior that must not regress
- [Existing correct mechanism.]
- [Existing test or successful behavior.]

## Hard constraints
- Do not modify [protected/upstream/competition paths].
- Do not use [truth or prohibited metadata] at runtime.
- Do not execute [destructive Git or system actions].
- Do not modify unrelated subsystems.
- Do not use environment edits, teleportation, or hidden state to mask a robot-side failure.

## Soft defaults
- Start with a short diagnostic before a long regression.
- Change one primary mechanism per round.
- Reuse existing interfaces and diagnostics where practical.

You may deviate when code evidence justifies it; explain the deviation.

## Open decisions
Choose the diagnostic method, test seed, timeout, helper structure, and exact implementation based on evidence.

## Required investigation
Align:
- task phase;
- goal and path;
- command output;
- odometry and actual displacement;
- sensor data;
- service responses;
- collision/contact data;
- map writes;
- timing/thread behavior.

Separate environment, sensor, service, planner, controller, physics, and diagnostic failures.

## Execution loop
1. Inspect current Git and code state using read-only commands.
2. Reproduce or replay the failure.
3. Build a code-log-behavior evidence chain.
4. Identify the earliest invalid transition or interface mismatch.
5. Select the smallest defensible repair.
6. Add a production-path regression test.
7. Run a short controlled validation.
8. Run the lowest-cost complete-loop regression needed for confidence.
9. Stop and checkpoint when evidence contradicts the hypothesis or scope expands.

## Acceptance criteria
- [Failure-specific measurable condition.]
- [No regression condition.]
- [Complete task continuation condition.]

## Immediate stop conditions
- [Fall/flip/collision trap.]
- [Sustained command with no displacement.]
- [Wrong state or wrong data routing.]
- [High-frequency loop.]
- [Gazebo/clock/odom/sensor failure.]

After two or three unsuccessful iterations of the same hypothesis, stop patching and produce a checkpoint.

## Final report
1. Conclusion
2. Confirmed facts
3. Root-cause chain
4. Changed files and behavior
5. Explicitly unchanged modules
6. Tests and commands
7. Run directories
8. Metrics versus baseline
9. Residual risks and new issues
10. Git status
11. Seal/no-seal decision
12. Next minimal experiment
```

---

## 2. Read-only reviewer

```text
You are an independent reviewer. Work read-only. Do not modify code or run destructive commands.

## Review object
- Main-agent checkpoint:
- Current diff:
- Relevant source files:
- Tests:
- Run directories/logs:

## Confirmed facts
- [Evidence already established.]

## Review questions
1. Does the root-cause chain follow from code and runtime evidence?
2. Is the repair located at the first responsible interface?
3. Does the patch preserve the documented behavior of lower layers?
4. Is a watchdog, retry, cooldown, timeout, or special motion hiding a source-contract problem?
5. What normal path could be misclassified or blocked?
6. What failure, timeout, task-clear, or stale-data path is missing?
7. Do tests call the production interface and reproduce the real failure chain?
8. Does the simulation prove the complete task stage, rather than only a service call?
9. Are environment failures and algorithm failures classified correctly?
10. Is the patch small enough to proceed, or should it be reduced/replanned?

Compare candidate solutions when relevant. Do not declare one “the only correct solution” without checking its assumptions.

## Output
### Root cause: supported / partly supported / unsupported
### Confirmed correct behavior
### Deterministic defects
### High-risk unproven edges
### Test gaps
### Required changes before approval
### Acceptable residual risk
### Proceed / revise / roll back
### Next minimal validation
```

---

## 3. Reviewed-plan implementation

```text
The following plan has passed independent review:
[Insert plan and review findings.]

Before implementation, verify that the current HEAD, diff, interfaces, and assumptions still match the reviewed checkpoint. Do not apply the plan mechanically when they differ.

## Implement only
[One mechanism and its required supporting changes.]

## Preserve
- [Validated behavior.]
- [Public interfaces and state timing.]

## Do not modify
- [Protected paths and unrelated modules.]

## Required boundary handling
- [Failure path.]
- [Timeout/task-clear path.]
- [Stale data or asynchronous path.]
- [Recovery and latch-release path.]

## Validation
1. Production-interface unit test
2. Historical failure replay when available
3. Short controlled simulation
4. Lowest-cost complete loop
5. Full regression only after the above pass

If the implementation requires broader architecture changes, a public-interface change, or a prohibited file modification, stop and report the evidence instead of expanding scope.

Report the final diff, behavior change, tests, runs, residual risk, Git state, and seal decision.
```

---

## 4. Stage checkpoint and handoff

```text
# Stage checkpoint

## 1. Project objective
[Competition or system-level objective.]

## 2. Current stage objective
[One bounded mechanism.]

## 3. Stable checkpoint
- HEAD:
- Branch:
- Known-good configuration:
- Known-good runs:

## 4. Confirmed facts
- [Code/log/test evidence.]

## 5. Root-cause chain
[First invalid transition and downstream effects.]

## 6. Implemented changes
- File/function:
- Behavioral change:
- Why this is the narrowest responsible layer:

## 7. Explicitly unchanged
- [Modules/interfaces not changed.]

## 8. Validation
- Unit/interface tests:
- Short simulations:
- Complete-loop regressions:
- Metrics:

## 9. Failed or invalid runs
- Run:
- Classification:
- Why it must not be counted as algorithm success/failure:

## 10. Residual risks
- [Unverified edge.]

## 11. Current Git state
- Tracked diff:
- Untracked files:
- External Git changes observed:

## 12. Protected behavior
- [Must not regress.]

## 13. Current blocker
[Single highest-priority issue.]

## 14. Next minimal experiment
[Action, evidence to collect, acceptance and stop conditions.]
```

---

## 5. Efficiency optimization

```text
You are optimizing task efficiency after the relevant safety and completeness blockers have been stabilized.

## Goal
Increase useful coverage, hazard approach rate, and successful return probability per unit time. Do not optimize visual smoothness unless it materially affects task metrics.

## Baseline
- Total simulation time:
- Useful translation time:
- In-place rotation time:
- Complete idle time:
- Planning time:
- Door/elevator wait:
- Recovery time:
- Floor-transition time:
- Return time:
- Coverage/hazard/false-positive results:

## Candidate directions to assess independently
- turn toward the elevator door while it opens;
- enter or exit when measured clearance is sufficient instead of waiting for full opening;
- leave the door zone and continue while door closing is asynchronous;
- combine linear and angular velocity for safe curves;
- distinguish A* geometry from path-follower stop-turn-go behavior;
- prepare future plans asynchronously while safely executing the current path;
- discover or infer floor count through a legal interface;
- allocate time dynamically across floors;
- balance unexplored regions against conservative return margin.

These are hypotheses, not requirements.

## Required analysis
1. Quantify where time is actually spent.
2. Determine whether each delay is necessary, avoidable, or a symptom of another failure.
3. Rank candidates by expected score impact, implementation risk, and regression surface.
4. Select one mechanism for the first implementation round.
5. Preserve narrow-door, large-heading-error, obstacle-proximity, elevator, precise-arrival, and recovery safety behavior.
6. Do not add concurrency unless blocking computation is measured and shared-state ownership is defined.
7. Do not probe invalid floors unless rules and service semantics make it safe, reliable, and worthwhile.
8. Preserve a conservative return budget while allowing high-value exploration near the return path.

## Scheduling principles
Use dynamic inputs rather than fixed per-floor seconds:
- remaining total time;
- estimated return and floor-transition cost;
- safety margin;
- unvisited floors;
- map-growth rate;
- frontier quality;
- repeated-path ratio;
- recent displacement;
- new-detection rate;
- recovery cost.

Model three phases:
1. normal exploration;
2. opportunity exploration near the return route;
3. mandatory return.

## Output
- measured time breakdown;
- confirmed bottleneck;
- rejected ideas and reasons;
- selected first optimization;
- expected gain and risk;
- minimal implementation boundary;
- test plan;
- stop and rollback conditions;
- next checkpoint trigger.
```

---

## 6. Existing-prompt audit

```text
Audit the following engineering-agent prompt:
[Insert prompt.]

Evaluate:
1. Is there one primary objective?
2. Are facts separated from hypotheses and user ideas?
3. Are hard constraints, soft defaults, and open decisions distinct?
4. Does the prompt protect stable behavior and upstream files?
5. Does it demand production-path evidence?
6. Does it classify environment failures separately?
7. Are acceptance conditions measurable?
8. Are stop and replan conditions explicit?
9. Is the agent overconstrained by exact patches, constants, seeds, or line numbers?
10. Is the agent underconstrained by vague goals or uncontrolled subsystem scope?
11. Does the report format support model handoff?

Return:
- strengths;
- missing information;
- overconstraints;
- uncontrolled freedoms;
- revised ready-to-paste prompt.
```
