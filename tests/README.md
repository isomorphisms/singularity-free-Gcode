# Independent checks and intentional refusals

`check.c` independently implements scalar position, direction and full-frame
equations, using no production rotation helpers. It compares both machines and
Jacobian translation columns, checks full/task rank separately, and checks the
pinned LinuxCNC head equations under an explicit B/Z convention conversion.

Path regressions cover signed-tilt pole crossing, disconnected regular branches,
rotary winding, active limits, redundant carriage families, true twelve-joint
full-map rank loss, near-singular numerical conditioning, look-ahead versus a
greedy dead end, an asymmetric housing collision, an interior swept collision,
unreachable poses, source identifiers, bad units, NaN and invalid directions.
An additional fixed-obstruction case is unreachable for head5 but feasible for
the same relative toolpath with hybrid12's moving workpiece and secondary carriage.

G-code checks independently parse every emitted motion for both supplied example
paths, reconstruct its physical tip/direction from printed axis coordinates,
check time words and count/source association. This establishes the emitted
subset's numeric consistency; it does not replace LinuxCNC's interpreter or a
configured machine simulator.

One child-process regression deliberately sends an unreachable pose to the real
CLI. Its `FAIL code=2` diagnostic is expected: the test requires that exit code
and verifies the old output file remains intact. The final `PASS ... checks,
0 failures` is the suite result. A passing negative case is not machining failure
silently promoted to success.

The algebraic executable checks a nonzero nilpotent, fold root decisions and
annulus-area exponents. It is separate from production kinematics.
