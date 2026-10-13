# Offline pole-continuation diagnostic

This experiment checks a small decision needed before a realization can choose
an inverse-kinematic branch at the lower pole of the idealized AC head.  It
does not alter the existing Adriç semantic object, geometric realization,
traversal, source map, or emitted G-code.

## Declared model

Use a right-handed world frame, active rotations, column vectors and

```text
direction(A,C) = (−sinA × sinC, sinA × cosC, −cosA).
```

The commanded pose consists of tool tip plus oriented tool direction.  Roll is
unspecified.  A is signed and unrestricted near the lower pole, C is modulo
2π, translations are unrestricted and the idealized rotary pivots coincide.
This is the `Rz(C)Rx(A)` **head** model from the Fulton research.  It is not an
AC-table model and has no measured tool length, pivot offset, joint range,
holder, fixture, compliance, velocity, acceleration or controller convention.

At the lower pole, the original inverse fiber has the reduced ideal

```text
(sinA, cosA − 1, sinC² + cosC² − 1)
```

and dimension 1.  Its real support is the full C-circle.  A meridian
approaching the pole in nonzero horizontal direction v=(vₓ,v_y) has the
two-point limiting C-setting scheme

```text
(sinC² + cosC² − 1, vₓ × cosC + v_y × sinC).
```

For incoming u and outgoing v, the two limiting schemes meet exactly when
uₓv_y − u_yvₓ = 0.  Independent lines force both sinC and cosC to vanish,
which contradicts the unit-circle relation.  Dependent nonzero lines give the
same two-point scheme.  The check computes these ideals directly in
Macaulay2; it does not replace the special fiber by one of its path limits.

## Input and output

Run `pole-continuation.m2` with one TSV whose first row is exactly

```text
case_id	incoming_x	incoming_y	outgoing_x	outgoing_y	pose_kind
```

Coordinates are exact rationals written as `integer` or `integer/positive-integer`.
Each vector must be nonzero.  The only accepted `pose_kind` is
`tip-plus-direction`.  The program rejects unsupported full-frame input rather
than pretending it has the same pole freedom.

For every case it prints both original limiting ideals, their scheme-theoretic
intersection, the determinant and either `LIMIT_SETS_INTERSECT` or
`LIMIT_SETS_DISJOINT`.  It also prints that the full pole fiber has reduced
dimension 1.  These statuses say only whether limiting C settings agree.  They
do not establish a continuous machine motion, a dwell, finite joint reach,
clearance, gouge avoidance, safe cutting or collision freedom.

## Cases

`cases.tsv` covers equal and opposite Y meridians, orthogonal Y/X meridians,
parallel integer vectors, a nonparallel Pythagorean pair, and a rational
rescaling, plus a meridian whose two real settings are irrational over QQ.
`invalid-zero.tsv` and `invalid-full-frame.tsv` exercise the two
input boundaries.

## Execution boundary

Macaulay2 owns the exact ideal, radical, dimension and degree calculations.
No compiled helper is introduced. The Flexible Pipes pipeline invokes its
Grease orchestration stage, which captures producer exits and outputs before
calling the independent ai-ci oracles. Compiled Grease runs without Python 2;
the development launcher is a different entrypoint. A run receipt must
identify the Macaulay2 executable, source hashes, input hash, argv and output
hash.  Independent ai-ci acceptance is required before a result can inform a
later realization policy.

The scope of the status remains limiting-setting compatibility. A successful
exact run is neither a physical machine test nor platform-delivery attestation.
