# Working boundaries

Preserve the intention stack, train-track examples and semantic source maps.
Every CNC waypoint and emitted block must retain object and operation identity.

Read `docs/cnc-prototype.md` before changing kinematics, constraints, planning,
or controller output. Use C11 here; do not add C++ or Python runtime dependencies.
Read `algebraic-geometry/findings.md` before promoting an algebraic experiment
into the production planner. An analogy or a rank warning is not acceptance.

Before writing Idriç source, resolve and read the user's canonical Idriç style
guide and reference examples in the current language repository. Do not use the
exploratory `intent.adric` text as a claim of current compilable syntax or copy a
language style guide into this repository.

Run `make all test examples`. Validate the exact printed G-code coordinates,
initial-state preconditions and source identities. Keep source-equation parity,
independent C tests, controller simulation, external ai-ci acceptance and physical
machining evidence separate. The latter three currently have no passing receipt.
Never claim a twelve-joint trace is twelve-axis controller output.

No merge, release, physical-machine execution or branch deletion is authorized
by implementation or review alone.
