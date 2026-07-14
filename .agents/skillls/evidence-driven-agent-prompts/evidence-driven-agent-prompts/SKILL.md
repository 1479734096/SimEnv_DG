---
name: evidence-driven-agent-prompts
description: design and revise evidence-driven prompts for coding agents, debugging agents, reviewers, and model handoffs in complex engineering projects. use when the user needs a ready-to-paste prompt for diagnosing failures, modifying code, protecting upstream or competition files, running staged tests, assigning roles across codex/deepseek/glm/kimi/gpt, reviewing another agent's plan, or creating a checkpoint/handoff. separate confirmed facts from hypotheses, preserve agent autonomy, require production-path evidence, define validation and stop conditions, and prevent multi-module patch accumulation.
---

# Evidence-Driven Agent Prompts

## Purpose

Create prompts that tell an engineering agent:

- what outcome to achieve;
- what is already confirmed;
- what remains uncertain;
- what must not be changed;
- how to prove the result;
- when to stop, roll back, or request review.

Do not prescribe an implementation before the evidence supports it. Constrain scope and risk while preserving reasonable investigative autonomy.

## Core rule

Write prompts around this formula:

> Define the outcome, evidence, protected boundaries, proof standard, and stop conditions. Do not dictate the patch unless an independent review has already established the implementation boundary.

## Workflow

### 1. Determine the prompt type

Choose one primary type:

1. **Main executor** — diagnose, implement, test, and report one bounded mechanism.
2. **Read-only reviewer** — challenge the root cause, patch boundary, tests, and regression risk without modifying code.
3. **Implementation handoff** — implement an already reviewed plan while checking that its assumptions still hold.
4. **Checkpoint or model handoff** — preserve state, evidence, Git status, validated behavior, unresolved risks, and the next minimal task.
5. **Prompt audit** — review an existing prompt for overconstraint, missing evidence, unclear acceptance criteria, or uncontrolled scope.

Do not combine these roles unless the user explicitly asks for a single-agent workflow.

### 2. Reconstruct the current checkpoint

Extract only material facts from the conversation, files, logs, screenshots, or reports. Organize them into:

- current objective;
- confirmed facts;
- failed and successful runs;
- current Git state;
- modified and protected files;
- confirmed root cause;
- unresolved hypotheses;
- validated behavior that must not regress;
- current blocking issue.

Do not turn a prior model's confident wording into a fact unless supported by code, logs, measurements, or reproducible behavior.

### 3. Separate facts, hypotheses, and user ideas

Use explicit labels:

- **Confirmed fact** — supported by code, logs, measurements, or repeatable tests.
- **Working hypothesis** — plausible but not yet distinguished from alternatives.
- **User-proposed direction** — must be assessed independently; not a required implementation.

Convert statements such as “A* is wrong” into a diagnostic question when the evidence does not yet distinguish A*, path compression, path following, control, or physics.

### 4. Select one main mechanism

Limit each implementation round to one primary mechanism, such as:

- elevator exit traversal;
- floor-transition map gating;
- negative-obstacle detection;
- path-following turn behavior;
- door/elevator wait parallelization;
- frontier selection;
- time-budget scheduling.

Place other issues in a backlog. Permit small supporting changes only when they are required to validate or complete the same mechanism.

Do not ask the agent to simultaneously change planning, control, mapping, perception, state-machine timing, and scoring.

### 5. Classify constraints

Write constraints in three levels.

#### Hard constraints

Treat these as non-negotiable. Examples:

- do not modify competition or upstream files;
- do not use truth data at runtime;
- do not add prohibited metadata dependencies;
- do not use teleportation or environment edits to hide a robot-side problem;
- do not execute destructive Git operations;
- do not modify unrelated modules.

#### Soft defaults

Treat these as preferred unless code evidence justifies deviation. Examples:

- start with a short diagnostic before a long simulation;
- modify one module first;
- reuse the current test harness;
- use an existing stable seed for reproduction.

Require the agent to explain deviations.

#### Open decisions

Leave these to the agent. Examples:

- exact diagnostic fields;
- test seed and timeout;
- helper-function structure;
- whether to use replay, unit tests, or a short GUI test first;
- local implementation details that do not change public semantics.

### 6. Require an evidence chain

For a failure, ask the agent to align the relevant layers:

- task state and phase;
- planner goal and path;
- controller command;
- odometry and actual displacement;
- sensor observations;
- service responses;
- collision or contact evidence;
- map updates;
- timing and thread behavior.

Require explicit separation of:

- environment failure;
- sensor/startup failure;
- service failure;
- planner failure;
- controller or physical-execution failure;
- diagnostic interruption;
- expected budget-driven exit.

Do not allow a successful lower-level service call to be reported as success of the full task.

### 7. Preserve agent autonomy

Use language such as:

- “independently assess these directions; they are not mandatory implementations”;
- “choose the smallest defensible change based on production-path evidence”;
- “adjust seed, timeout, and diagnostic method when justified”;
- “do not mechanically follow the proposed plan if its assumptions do not hold.”

Avoid unnecessary restrictions on line numbers, exact constants, fixed seed counts, or exact file lists before the root cause is established.

### 8. Define a staged validation ladder

Use the minimum ladder needed for the risk:

1. production-path unit or interface test;
2. historical-log replay when available;
3. short controlled simulation that reaches the failing phase;
4. lower-cost complete loop, such as one- or two-floor regression;
5. full multi-floor or competition-style regression;
6. broader seed expansion only after the mechanism is stable.

Do not use “all unit tests pass” as the sole acceptance condition. Require behavioral evidence.

Prefer short diagnostics before hour-long simulations. Stop a simulation as soon as a decisive failure is observed.

### 9. Define acceptance and stop conditions

Acceptance conditions must be measurable and tied to the actual failure. Examples:

- wrong-floor map writes equal zero during the ambiguity window;
- the robot crosses the elevator threshold under its own motion;
- a skipped cloud is not replayed after state recovery;
- command velocity produces measurable displacement;
- no new planner loop, collision, fall, or false completion occurs;
- the complete task continues after the repaired phase.

Stop conditions must include:

- clear fall, flip, collision trap, or no-displacement command loop;
- wrong state transition or wrong-floor data write;
- high-frequency replanning loop;
- Gazebo, clock, odometry, or sensor failure;
- two or three unsuccessful iterations of the same hypothesis;
- patch scope expanding into unrelated modules;
- evidence contradicting the current root-cause hypothesis.

On stop, require a checkpoint instead of another speculative patch.

### 10. Require a structured report

Ask for:

1. conclusion;
2. confirmed facts;
3. root-cause chain;
4. changed files and behavior;
5. explicitly unchanged modules;
6. tests and commands;
7. simulation run directories;
8. metric comparison with baseline;
9. new problems and residual risk;
10. Git state;
11. whether the stage is approved for sealing;
12. the next minimal experiment.

Do not let the agent describe an interrupted diagnostic run as a full regression result.

## Prompt construction rules

- Put the ready-to-paste prompt in one fenced code block.
- Use the user's language.
- Begin with the current objective and checkpoint, not general background.
- Include only facts relevant to the current mechanism.
- Refer to exact files, functions, runs, and timestamps when available.
- State that line numbers are navigation aids, not mandatory edit locations.
- Ask the agent to inspect current Git state before assuming an earlier checkpoint still applies.
- Protect already validated mechanisms from unrelated edits.
- Distinguish runtime inputs from offline evaluation data.
- Require production-interface tests instead of tests that only exercise a mock state machine.
- Require a rationale when the agent selects a compensating watchdog instead of repairing a source contract.
- Avoid broad commands such as “optimize everything,” “make it competition-ready,” or “run many seeds” without a decision framework.

## Multi-model role allocation

Use the fewest models necessary.

### Default orchestration

- Use one primary executor for two or three evidence-driven iterations.
- Insert one independent review at a meaningful checkpoint, not after every small change.
- Add a second reviewer only when the first review and executor disagree, the patch crosses several modules, or the failure remains ambiguous.

### Practical role defaults

Treat these as user-specific defaults, not universal capability claims:

- **Codex** — primary repository executor, implementation, tests, simulations, and checkpoint production.
- **DeepSeek** — adversarial root-cause and patch-boundary review.
- **GLM** — engineering-completeness review, call-site coverage, fallback branches, and implementation handoff.
- **Kimi** — video, trajectory, and behavior interpretation when logs alone do not explain motion.
- **GPT** — strategy design, task decomposition, rule interpretation, and multi-objective scheduling.

Respect the user's observed tool reliability. Do not assign two implementation models to modify the same working tree concurrently.

## Common failure patterns to prevent

### Overconstrained prompt

Symptoms:

- exact patch specified before diagnosis;
- many fixed thresholds and seed counts;
- only one permitted function or line range;
- no room to reject the user's hypothesis.

Fix: retain hard safety boundaries but reopen diagnostic and implementation decisions.

### Underconstrained prompt

Symptoms:

- “improve navigation” or “increase coverage” with no baseline;
- no protected files;
- no acceptance criteria;
- no stop condition;
- several subsystems mixed together.

Fix: choose one mechanism and supply a checkpoint, evidence, boundaries, validation ladder, and report format.

### Compensation instead of source repair

Symptoms:

- watchdog, cooldown, retry, or timeout added before checking interface contracts;
- repeated parameter tuning hides a deterministic mismatch;
- a lower layer is changed even though its behavior matches its documented contract.

Fix: trace the first point where a valid state becomes invalid and repair the narrowest responsible interface.

### False success

Symptoms:

- service success reported as task success;
- physical transport reported as complete floor change;
- an interrupted run used as a recognition benchmark;
- environment failure counted as algorithm failure.

Fix: define success at each layer and report them separately.

## Output modes

### Creating a new prompt

Return:

1. a short assignment recommendation when model selection matters;
2. one ready-to-paste prompt;
3. at most one brief note explaining sequencing or review timing.

### Auditing an existing prompt

Return:

- what is already strong;
- missing facts or boundaries;
- overconstraints;
- uncontrolled freedoms;
- revised ready-to-paste prompt.

### Creating a checkpoint or handoff

Use the checkpoint template in [references/prompt-templates.md](references/prompt-templates.md).

### Building specialized prompts

Read [references/prompt-templates.md](references/prompt-templates.md) for executor, reviewer, implementation-handoff, checkpoint, and efficiency-optimization templates.
