# Five-axis kinematics, schemes, and gantry compliance — research notes

## Purpose
Preserve the research direction from the CNC discussion alongside the existing intention-preserving Adriç drawing work. Target: faster, collision-free, machine-aware G-code, including a tall workpiece, AC rotary axes, branch changes, and height-dependent gantry vibration. This is a hypothesis-driven laboratory, not a claim of proven safety.

## Architecture
Use ordinary Unix shell to orchestrate small, inspectable programs. C handles numerical kinematics, collision calculations, and machine-specific geometry where appropriate; awk/sed and text files handle textual assembly. Keep Macaulay2 running as a separate algebraic process if repeated symbolic queries warrant it; communicate via framed stdin/stdout pipes or sockets, with request IDs, completion markers, errors and timeouts. Do not parse interactive prompts as a protocol. Do not issue motion or publish G-code while required verification is pending; timeout means unknown, not safe. Sage/Singular are alternatives, not excluded on language grounds. Retain named human-readable motion primitives rather than exposing trigonometric matrices everywhere.

## Mathematical objects
For joint coordinates q and tool pose p, define forward kinematics F(q)=p. Inverse kinematics is a fiber F⁻¹(p); multiple branches may need continuous selection. Jacobian dF determines local first-order sensitivity and rank, but rank alone neither proves clearance nor provides a global path. Singular values and joint-speed constraints are needed for near-singular conditioning. Tool and holder geometry, fixtures, joint limits and swept-volume checks remain explicit.

For algebraic calculations, encode rotation with polynomial variables s_a,c_a,s_c,c_c and relations s_a²+c_a²−1 and s_c²+c_c²−1; do not put sin(a) or cos(a) directly into a polynomial ring over QQ. The AC *head* example used previously is not automatically an AC *table* model: establish actual machine frame order and tool/workpiece ownership before computing F. For the head model Rz(c)Rx(a)(0,0,−L)+(x,y,z), the tip is (x−L sin(a)sin(c), y+L sin(a)cos(c), z−L cos(a)). The prior thread had inconsistent signs and swapped trigonometric factors: do not treat them as validated.

## Exact toy algebra (Macaulay2)
Start with a small fiber where a nonreduced structure really occurs:
```m2
R = QQ[u,v];
I = ideal(u^2, v);
isRadical I
radical I
```
The underlying real point is u=v=0 but u remains a nonzero nilpotent class in R/I. Compare tangent dimension and Jacobian rank with the reduced ideal ideal(u,v). Next derive such fibers from an actual linkage or 5-axis kinematic polynomial map, rather than inserting nilpotents arbitrarily. Consult Daniel Litt's nonreduced linkage examples.

## Compliance and clearance
A gantry raised in Z can exhibit increased dynamic tool displacement, depending on geometry, stiffness, loading, acceleration and resonant frequency. This must be measured or bounded, not inferred from the symbol ε. Model nominal tool pose F(q), disturbance d(t), and clearance g(F(q),d(t),workpiece). A first-order sensitivity calculation is δp ≈ J(q)δq + D_dF δd. For independent bounded joint errors |δq_j|≤b_j, a conservative first-order coordinate bound is Σ_j |J_ij|b_j, NOT the signed sum Jδq. Use measured height-dependent compliance/transfer functions and acceleration/jerk for vibration; nilpotent ε with ε²=0 is a formal derivative bookkeeping device, not a positive physical displacement or finite uncertainty interval.

A collision boundary g=0 can be algebraic in suitable polynomial models, but unsafe penetration g<0 is a semialgebraic inequality, not simply the zero locus of an ideal. Mesh collision also requires inequalities, intersection predicates, or suitable algebraic encodings. An ideal ideal(f−ε z) alone does not prove danger when nominal f>0; if ε is nilpotent, it cannot numerically dominate positive clearance. For a legitimate thickened model use a quotient ring R[ε]/(ε²), then distinguish scheme-theoretic support, associated primes and multiplicity from physical clearance margins.

## Experiments and acceptance
The [Fulton bridge](fulton-constraint-fibers.md) supplies exact frame and
Jacobian identities for the stated idealized AC head; finite-difference and
physical-convention acceptance in experiment 1 remain pending. Experiment 2's
genuine nonreduced inverse fiber is established by the two-link arm. The head
pole is reduced excess freedom. The earlier toy ideal is still an illustration,
not evidence that the head has nilpotents. Experiments 3–6 remain unimplemented.

1. Verify an AC-head FK model and its symbolic Jacobian against finite differences, including axis convention and tool length.
2. Construct a genuine nonreduced inverse-kinematic fiber; compare radical, tangent space and multiplicity with ordinary point-set IK.
3. Introduce a tall workpiece and collision/holder swept volumes; test alternate IK branches and detours.
4. Calibrate a height-dependent flex/vibration model using synthetic data initially, then measured machine data if available.
5. Compare baseline retract/dummy moves against algebra-informed branch/path selection on total time, clearance margin, acceleration, and tool-tip error.
6. Keep simulation and offline verification separate from controller execution; no safety claims until independent verification.

## Source thread principles
- C, shell, and Macaulay2 have separate jobs; no monolithic C++ framework is required.
- Human-readable motion names should hide trigonometric implementation without concealing coordinate conventions.
- Scheme theory must yield a testable advantage over numerical geometry, not merely rename familiar objects.
- Keep the original semantic-intent stack and source mapping intact when adding machine execution constraints.
