# Industrial capability review

Reviewed on 2026-10-13 UTC. Primary documentation/source is preferred; the shop
threads below are primary reports of their authors' experience, not controlled
benchmarks. A capability not established in this bounded source review is marked
`not established`, rather than asserted absent. Configured machine/controller
versions and optional software modules materially affect capabilities.

| Capability | LinuxCNC | Machinekit | FreeCAD CAM/Path | Autodesk Fusion post API | SINUMERIK example |
|---|---|---|---|---|---|
| Forward/inverse kinematics | Machine-specific modules | Serial and machine modules | Current machine/rotation chains | Machine frame conversions | TRAORI transformations |
| Jacobian | genserkins; current frame/Jacobian interfaces | genserkins computes forward/inverse Jacobians | Not established here | Internal implementation not established | Internal implementation not established |
| Multiple IK solutions | Machine-dependent; current orientation inverse | Seeded iterative serial inverse | Not established for simultaneous paths here | Secondary ABC solution documented | Controller-dependent swivel solutions |
| Rotary winding/selection | Joint/coordinate conventions and limits | Module/controller dependent | Rotation strategy is machine/post dependent | Shortest rotation and rewind strategies | Axis/plane strategy dependent |
| Singularity handling/feed | Error/limit behavior and Jacobian interface; not every module avoids every pole | Jacobian inversion can fail | Not established here | Tool-axis adjustment, linearization and rewind settings | Transformation behavior documented; full avoidance not established |
| Redundant allocation | A machine/module-specific concern | Pseudoinverse code includes non-square cases | Not established here | General twelve-joint optimization not established | General twelve-joint optimization not established |
| Collision/limits | Limits configured; full collision planning not established | Limits/motion planning; full collision planning not established | Model and machine strategy dependent | Rewind stock/clearance settings; full collision planner not established by this API | Machine/options dependent |

## Primary sources and bounded findings

**LinuxCNC.** [Five-axis kinematics](https://linuxcnc.org/docs/stable/html/motion/5-axis-kinematics.html)
derives table/head transformations and supplies simulation models. The
[G-code reference](https://linuxcnc.org/docs/stable/html/gcode/g-code.html) defines
G93 minutes-based inverse time and per-block F words. The pinned
[5axiskins source](https://github.com/LinuxCNC/linuxcnc/blob/b8ccac8f835c0610233fdf799255c63f2b9738ac/src/emc/kinematics/5axiskins.c)
has explicit spherical head kinematics. The tests check equation parity under
its reversed B convention and Z datum shift; the LinuxCNC binary is not executed.

Current master [kinematics conventions](https://linuxcnc.org/docs/master/html/uk/motion/kinematics-conventions.html)
also describes frame pairs, multiple orientation solutions, continuous fibers
and Jacobians. Those current interfaces must not be retroactively assumed in
v2.9.4. [Switchable kinematics](https://linuxcnc.org/docs/master/html/en/motion/switchkins.html)
describes singularity errors and alternate kinematics modes. Source support for
an interface does not prove a particular configured machine's avoidance behavior.

**Machinekit.** [Kinematics documentation](https://www.machinekit.io/docs/motion/kinematics/)
describes forward/inverse mappings and iterative inverse methods. The pinned
[genserkins source](https://github.com/machinekit/machinekit/blob/ef46939b490bfa029c9d25474cc2e35543c9ac8e/src/emc/kinematics/genserkins.c)
implements `compute_jfwd`, `compute_jinv` and iterative inverse updates. Its
non-square pseudoinverse cases are source-level machinery, not evidence of a
collision-aware redundant twelve-joint milling deployment.

**FreeCAD.** Current
[postprocessor source](https://github.com/FreeCAD/FreeCAD/blob/15bd466ccbe19403ccd948e6ae405741999e83dd/src/Mod/CAM/Path/Post/Processor.py)
uses machine definitions, builds kinematic chains and selects dynamic-work-offset
or tilted-work-plane strategies. Treating FreeCAD as permanently three-axis would
be wrong. This review did not demonstrate its simultaneous twelve-joint branch
optimizer; its current machine-aware work is not used as the deficient baseline.

**Autodesk.** The
[MachineConfiguration reference](https://cam.autodesk.com/posts/reference/classMachineConfiguration.html)
documents secondary ABC solutions, conversions, shortest rotations, minimizing
rewinds, retract/plunge settings and singularity adjustment. Singularity settings
can alter tool direction and add linearization points within tolerances. Thus
these capabilities already exist; the API review is not a commercial performance
benchmark or a claim about internal Jacobian algorithms.

**Commercial controllers.** Siemens'
[five-axis manual](https://cache.industry.siemens.com/dl/files/454/37335454/att_110322/v1/SIN_WF5_0509_en.pdf)
documents CYCLE800, TRAORI, tool/workpiece orientation modes and oriented
retraction. This historical 2009 manual establishes existing transformations,
not current option availability, exhaustive collision avoidance or modern
twelve-joint optimization. No commercial commands are copied into our adapter.

## Documented shop complaints

1. [Tool Orientation Flip Axis Frustration 3+2](https://forums.autodesk.com/t5/fusion-manufacture-forum/tool-orientation-flip-axis-frustration-3-2/m-p/10074103),
   February 2021: equivalent A90/C180 was output when A−90/C0 was needed because
   part swing clearance prevented C180. The reported workaround changed post
   axis limits; the author confirmed the desired output afterward. This is a
   branch choice/clearance complaint, not proof that Fusion lacks branch handling.
2. [Unexpected A-Axis Flip in Rotary Toolpath](https://forums.autodesk.com/t5/fusion-manufacture-forum/unexpected-a-axis-flip-in-rotary-toolpath-need-help/td-p/13717929),
   July 2025: an unexpected orientation flip with A limited to 0–180 degrees.
   The discussion points to kinematic awareness of toolpath generation and fixture
   selection. It is a reported case, not an independently reproduced product defect.

No primary dummy-positioning-move report was established in this review. Earlier
assistant claims about dummy moves are not promoted to sourced industrial facts.
Likewise no measured unnecessary-retract reduction is claimed: our planner adds
zero moves, and both inputs already contain their requested approach/retract moves.

## Demonstrable deficiency selected for implementation

The reference baseline computes each point with B≥0, C in [−π,π], C=0 at poles.
It applies exact inverse equations and checks endpoints, but does not retain
branches/winding or validate the connecting joint trajectory. The fixture exposes
that failure independently of any vendor: correct endpoints can produce 0.269 mm
tip deviation against a 0.03 mm corridor. Our paired plan reduces that deviation
to about 0.00225 mm and certifies every modeled connection.

This establishes an improvement over that specified ordinary stateless baseline.
It does not establish a new industrial capability or superiority to properly
configured LinuxCNC, FreeCAD, Fusion, Siemens or other commercial posts.
