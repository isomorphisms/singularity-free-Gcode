# CNC evidence generator: types and execution contract

Values: millimetre positions; unit tool directions; radians for internal joints;
positive seconds; named machining objects and operations. A `Machine` owns bounded
joint coordinates, one spindle, geometry and collision envelopes. A `Toolpath`
owns intended geometry, process mode and source identity. A `Plan` supplies one
configuration per waypoint and a reference duration per segment.

Central operations:

- `forward: Machine × Configuration → Pose` (pure).
- `plan: Machine × Toolpath → Plan or Diagnostic` (allocation, otherwise pure).
- `validate: Machine × Toolpath × Plan → Evidence or Diagnostic`.
- `write: Evidence × Plan → files` (explicit filesystem effects).

Invariants: preserve every waypoint and its semantic identity; no invented
positioning/retract moves; initial joints are a precondition; finite values;
bounded joints; tool-direction norm one; reference velocities, accelerations and
jerks within declared limits; output refusal on uncertified connections.

The user explicitly selected C, D or an existing compatible language and excluded
C++/Python runtime dependencies. This attempt therefore uses C11 for both the
production implementation and this small reproducible fixture generator. It does
not claim an Idriç compiler run or invent compilable Idriç syntax. No Idriç,
Grease, Ick or Cat Food entrypoint was installed in this disposable host's PATH.

The generator is a fixture author, not an independent acceptance owner. It
produces input text first, re-reads it with the public parser, then runs the
planner and emits traces, metrics and SVG plots. `tests/check.c` independently
reconstructs poses and parses emitted G-code. Flexible Pipes and ai-ci acceptance
remain separately labeled; local C tests cannot attest either service's execution.

The language-shaped requirement is a typed source-preserving CNC lowering:
distinguish directions from unrestricted vectors, radians from controller
degrees, poses from configurations, and a modeled-trajectory certificate from
physical-machine acceptance. Acceptance would require the same wrap/pole fixtures
and independent forward checks through the canonical Idriç compiler/runtime.
