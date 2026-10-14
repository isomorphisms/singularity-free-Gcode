# Configuration-space-aware CNC postprocessor

The prototype solves a particular reproducible failure: independently choosing
principal rotary angles gives correct endpoint directions but an incorrect
connecting tool trajectory. Continuation instead retains signed tilt and rotary
winding through the whole path. It preserves the requested geometric corridor
and every input waypoint, adding no retract or dummy positioning moves.

The implementation has no C++ or Python runtime dependency. It links C11, libc
and libm. The earlier drawing/intention documents and train-track work remain
intact; CNC blocks retain their high-level object and operation identifiers.

## Build and run

Requirements on a Linux build host: a C11 compiler, make, libc and libm. The tests
also exercise POSIX temporary files and child-process exits. From an arbitrary
launch directory, invoke make with `-C` and the absolute verified checkout path:
`make -C /absolute/verified/checkout all test examples`.

The native program interface is:

```text
/absolute/verified/checkout/build/gcode-plan linuxcnc MACHINE TOOLPATH OUTPUT
/absolute/verified/checkout/build/gcode-plan trace MACHINE TOOLPATH OUTPUT
```

All arguments are file paths. Use verified absolute paths for an arbitrary launch
directory. `linuxcnc` supports `head5`; `trace` supports both modeled machines.
`make examples` regenerates inputs, G-code, TSV traces, measured comparisons,
reference time simulations, configuration-space projections and algebraic
results. `make test` runs independent scalar forward checks and G-code parsing.
`make sanitize` rebuilds tests with address and undefined-behavior sanitizers.
On this host LeakSanitizer cannot inspect `/proc/.../task`; address/undefined
checks were run with `ASAN_OPTIONS=detect_leaks=0`. Leak checking is NOT_RUN.

Ick and Grease were not installed on this disposable build host. This is an
explicit native-C prototype build receipt, not an Ick or managed-target build.
The request's selected C language is honored; no alternate shell implementation
or mobile compilation procedure is introduced.

## Input contract

Inputs are bounded, whitespace-delimited text. `#` begins a comment. Units are
explicit. Unknown, repeated or malformed records, nonfinite numbers, wrong axis
kinds, invalid limits, zero durations and non-unit directions are rejected.

Machine records:

```text
machine head5|hybrid12 mm-radians-v1
axis NAME linear|rotary LOWER UPPER VELOCITY ACCELERATION JERK INITIAL
tool TIP_TO_NOSE_LENGTH CUTTER_RADIUS HEAD_RADIUS ASYMMETRIC_HOUSING_OFFSET
tolerance POSITION_MM UNIT_DIRECTION_CHORD
spindle RPM
sphere WORLD_X WORLD_Y WORLD_Z RADIUS
fitting HALF_X HALF_Y HALF_Z THROUGH_BORE_RADIUS BRANCH_BORE_RADIUS BRANCH_TILT
```

Exactly one record for every modeled axis is required. The tool, tolerance and
spindle records are mandatory. Spheres and the fitting clearance solid are
optional. The kinematic chain is selected by the header, not arbitrary executable
machine input. Linear quantities use millimetres, rotary quantities radians,
velocity units per second, acceleration units per second squared, and jerk units
per second cubed. Rotary ranges may span at most three turns. Machine geometry
is idealized: perpendicular joints, coincident rotary pivots, stated tool length.

Toolpath records:

```text
toolpath mm-unit-vector-v1
OBJECT OPERATION cut|travel SECONDS X Y Z NX NY NZ
```

`SECONDS` is nominal duration from the previous waypoint; the first value is
ignored. Directions point from tip toward holder. Between points, intended tip
position is linear and intended direction is the normalized linear interpolation
of the endpoint unit vectors. Steps exceeding 120 degrees require additional
explicit orientation detail. Object/operation identifiers permit letters, digits,
hyphen, underscore and slash, preventing injected G-code comments or words.

The first pose must exactly match the machine's feasible declared initial joints.
The post does not invent an approach to an unknown starting state. Every output
motion names its source waypoint, object, operation and cutting/travel role.
An infeasible request fails before creating or truncating its output program.

The process flags and feature names preserve CAM intent; they do not automatically
derive feeds, tooling, material allowances or a stock-removal process. Tool roll
is deliberately unspecified for axisymmetric cutters. A full frame is returned
by forward kinematics, but the toolpath target space is position × tool direction,
not arbitrary six-coordinate frame orientation. A roll-constrained toolpath
needs a different explicit input contract and inverse solver.

## Five-axis reference machine

Three linear joints X,Y,Z position the spindle nose. An outer C joint turns
about machine z; inner B tilts about the rotated local y. One spindle points
along the head frame's z direction. Let H = Rz(C)Ry(B), length L = 80 mm, and
linear nose position h = (X,Y,Z). Then the full tool frame and tip are

```text
F(q) = (h − L H(0,0,1), H) in SE(3)
n(q) = (sin(B)cos(C), sin(B)sin(C), cos(B))
```

The configuration space has three bounded linear intervals and two rotary
coordinates with bounded lifted winding. Angular positions agreeing modulo
2π reach the same orientation but incur different travel and cable/limit costs.

For a regular direction, the two inverse families have `(B,C)` and
`(−B,C+π)`, with all allowed winding lifts. Multiple inverse solutions do not
establish a singularity. The full frame map has rank five, including at B=0.
For axisymmetric machining its projection to position × S² has generic rank
five and rank four at B=0 or π. This **task singularity** is caused by C no
longer changing the tool direction; it is not full-frame rank loss.

A path whose direction crosses the pole along a fixed meridian can continue
through signed B without moving C. Arbitrary outgoing directions need a
different C at the pole. A pole's C continuum is sampled using the initial
longitude and the limiting longitudes on both sides. Tests include consecutive
pole waypoints and a case where nearest-point greedy choice prevents continuation.

## Twelve controllable joints, one toolhead

The extension has eight linear and four rotary joints, in this order:

| Joints | Mechanical role |
|---|---|
| X,Y,Z | Primary head gantry |
| B,C | The same head tilt and outer yaw |
| U,V,W | Secondary XYZ head carriage |
| P,Q | Movable workpiece's XY carriage |
| A,E | Workpiece table tilt about x and outer yaw about z |

With table frame W = Rz(E)Rx(A), work origin w = (P,Q,0), and head nose
h = (X+U,Y+V,Z+W), where the last W in that vector names the linear joint,

```text
F(q) = (transpose(table_frame)(head_nose − work_origin − L head_frame ez),
        transpose(table_frame) head_frame)
```

The code uses descriptive names to keep the linear W and table frame distinct.
Generic full-frame rank is six, so twelve joints leave six ordinary redundant
degrees of freedom. The direction-only task has rank five and seven redundant
degrees of freedom. A free linear carriage split is ordinary redundancy, not a
singularity or non-reduced fiber. The table's transverse tilt also supplies a
direction velocity missing from the five-axis head at its pole.

This extension also has genuine full-frame singularities: at A=B=E=0 and
C=90°, the head tilt axis aligns with the table tilt axis. The rotary angular
span falls to two, full F rank falls to five, and direction-task rank falls to
four. Tests cover that rank loss and its nearby conditioning; adding joints
does not automatically remove singularities.

The inverse solver samples table A/E at −30°,0°,30°, enumerates the head's
branches/winding and projects linear carriage allocation onto bounded affine
fibers. It minimizes squared displacement from the declared initial joints with
an active-set limit projection. This is an implemented twelve-joint model,
not a complete IK solver for every twelve-axis architecture. Unsampled redundant
configurations can exist when this bounded solver reports no continuation.

The worked trace reaches tool X=40 mm with X limited to ±10 mm using X=10,
U=15 and P=−15. It retains all twelve coordinates. LinuxCNC has a finite documented
axis-word set; the post refuses to invent P/Q/E motion words or twelve-axis output.

## Jacobians, planning and constraints

Forward derivatives use central differences with independently checked scalar
equations. Translation rows and linear-joint columns scale by L, while rotary
columns use radians. Task analysis uses two orthonormal tangent directions on S²,
so it does not mistake the unit-vector constraint for an extra zero singular
value. A one-sided Jacobi SVD avoids squaring the condition number in J Jᵀ.
Rank uses a relative 1e-8 threshold. Six-row full-frame analysis is separate;
a five-joint full-frame matrix necessarily has one structural zero singular value.

`sg_velocity_residual` provides box-constrained least squares with inward-only
velocity at active limits and an explicit residual. The frozen-column condition
probe is a conservative two-sided subspace probe, not a replacement for that
one-sided cone. Production feasibility checks the actual chosen joint path and
its bounded reference profile, rather than inferring it from a determinant.

The planner searches the entire layered path, retaining up to 32 histories and
160 IK candidates per point. Costs include scaled joint travel, average velocity,
changes in average velocity/acceleration, limit margin and task conditioning.
Those smoothness terms are selection heuristics. This is bounded beam search,
not proof of a global optimum or completeness over continuous redundant fibers.
An exhausted search returns a diagnostic identifying the first failed waypoint.

Every connecting joint-line is checked against the specified geometric corridor.
Analytic first/second derivative bounds plus recursive interval subdivision
bound tool-position/direction errors and swept clearance. Depth exhaustion rejects
an inconclusive edge. Calculations use double precision with a positive clearance
margin; this is not a formally verified interval-arithmetic proof.

Fixtures are machine-frame spheres. Collision bodies are a full-length cutter
capsule, nose sphere and an offset housing sphere. The optional finished fitting
solid is a box minus an X cylinder and a tilted branch ray. Its max-composed
1-Lipschitz free-space field is a conservative clearance bound. Tool-centerline
samples include a spacing margin, so whole capsules are bounded rather than
declared clear from point samples alone. Unmodeled gantry members, clamps, tool
changers, motors, screws, compliance and stock-removal states remain outside it.

The reference time profile is rest-to-rest quintic per block:
`s(u)=10u³−15u⁴+6u⁵`. Segment duration is stretched to respect exact peak factors
1.875 for velocity, 10√3/3 for acceleration and 60 for jerk. Time simulations and
reported peaks refer to this profile. They are not measured controller behavior.

## LinuxCNC dialect

The adapter emits direct XYZBC coordinates for
`trivkins coordinates=XYZBC`, with XYZ as spindle-nose coordinates and B/C in
degrees. Do not feed this output to a TCP kinematics module: compensation would
be applied twice. See the pinned LinuxCNC source-equation comparison in the tests.
Use unwrapped B/C semantics (`WRAPPED_ROTARY=0`), with matching joint and axis
limits. The [LinuxCNC INI reference](https://linuxcnc.org/docs/stable/html/config/ini-config.html)
documents how wrapped rotary commands use different direction/position semantics.

The preamble explicitly sets millimetres, absolute distance, XY plane, canceled
cutter and tool-length compensation, canceled canned cycles, G54, exact stop
and inverse-time feed. Each G1 has its own `F=60/seconds`. No rapid, canned drilling,
tool-change, retract or controller-specific compensation command is invented.
The specified spindle speed is emitted; the spindle and coolant stop at the end.

Assumptions: installed tool matches the modeled geometry; G54 is the modeled
origin; G52/G92 offsets are zero; rotary mapping/signs and limits match the machine;
initial joints are verified; spindle direction/speed and workholding are valid.
Those premises are declared, not observed. Printed coordinates and feed words
are revalidated after quantization before emitting any bytes.

G61 avoids controller blending away from the certified joint line. G93 asks for
a block duration; it does not prescribe a quintic or bound a real controller's
jerk. The controller can slow a move. A controller-specific simulation and machine
validation must establish actual timing and dynamics before equipment use.

## What completion means here

The prototype automatically produces continuous feasible modeled trajectories in
two cases where the defined baseline needs repair. It supplies actual joints,
G-code, reference traces and quantitative evidence. It does not claim universal
industrial superiority, finished fitting geometry, improved finish, cycle-time
savings, controller runtime qualification, independent ai-ci acceptance or a real
cut. The fixture's chamfer label describes an illustrative center path in a
pre-cleared access volume, not a validated tool-contact or material-removal model.
Those process and controller acceptance boundaries remain explicit work.
