# Scheme-theoretic experiments: useful structure, no demonstrated production gain

`fibers.c` is a separate C research executable. It is not linked into the
postprocessor. `results.tsv` records quotient arithmetic, real-root decisions and
numerical clearance-region scaling. The conclusion is **NOT_DEMONSTRATED** for
an additional production planning benefit in these examples, not a theorem that
scheme theory can never help CNC.

## Recovering the linkage example

[Daniel Litt's original thread](https://twitter.com/littmath/status/1291142806807941131)
and its [readable preserved copy](https://threadreaderapp.com/thread/1291142806807941131.html)
describe two unit rods joining fixed endpoints to one moving point. Their circle
constraints meet in two configurations, a tangent configuration, or no real
configuration. Litt also motivates tolerances by replacing circles with annuli.

Here is the calculation independently used in this experiment. Place fixed
endpoints at (0,0) and (d,0), with d>0. The moving point satisfies

```text
x²+y² = 1
(x−d)²+y² = 1
```

Subtracting gives x=d/2. The coordinate algebra becomes
`ℝ[y]/(y²−(1−d²/4))`. Thus t=1−d²/4 identifies this linkage with the fold family
y²=t. At d=0 the elimination is invalid and the fiber is a circle; the code does
not apply the d>0 formula there.

## The family x²=t

Over ℝ[t], the algebra ℝ[t,x]/(x²−t) has basis 1,x and is free of rank two.
Consequently the family is finite and flat of degree two. Multiplication in that
basis, implemented directly in C, is

```text
(a+b x)(c+d x) = (ac+bd t) + (ad+bc)x
```

For t>0, evaluation at ±√t separates the two simple real points. For t=0, x is
nonzero in the quotient but x²=0: a length-two non-reduced point. For t<0, no real
configurations exist, although the algebraic fiber is not empty; it still has
degree two and two distinct points after complexification. Multiplicity in the
TSV is geometric point multiplicity over an algebraic closure, not a count of
physical configurations.

At t=0, the local ring is ℝ[x]/x². Its maximal ideal is (x), so the Zariski tangent
space has dimension one. The associated graded ring remains ℝ[x]/x²: the tangent
cone is a non-reduced zero-dimensional scheme, not a real line of available
motion. Setting x=εv satisfies the constraint modulo ε² for any v. Lifting to
second order with fixed t=0 forces v²=0 and hence v=0 over ℝ. An infinitesimal
tangent direction therefore need not extend to an actual finite configuration.

Varying t instead splits that double point into two real branches or removes
real configurations. Branch slopes dx/dt=1/(2x) become unbounded near the fold.
The ordinary Jacobian 2x also vanishes there. The polynomial discriminant 4t,
direct elimination and conventional continuation with event detection already
give the same existence and branch-merger decisions.

The executable compares those decisions at t=−1,−1e−8,0,1e−8,0.01,1. They agree.
The quotient retains algebraic length, but it supplies no different feasible path
or earlier threshold in this test. No computational-cost improvement is measured
or claimed. A local multiplicity by itself also cannot decide collision clearance,
actuator capacity, tool contact, material removal or compliance.

## Annulus tolerance experiment

Allow each rod length to lie in [1−ε,1+ε]. At a given x, each annulus gives an
interval for y². Intersect those intervals and integrate the resulting positive
and negative y lengths. The C executable uses 40,000 midpoint cells over a bounded
interval derived from the two radial constraints; it does not use a CAS or an
unattributed numerical library.

For d=1 the two circle intersections are transverse. Their Jacobian determinant
is 4dy, with y=±√(1−d²/4), so the available area scales as ε². At d=2, writing
x=1+u gives to leading order y²≤2(ε−|u|). Integrating gives
`(8√2/3) ε^(3/2)`, so the tangent configuration has the slower 3/2 area exponent.

The final measured log slopes at ε=0.00125 are about 2.0000015 and 1.5006303.
This reproduces a concrete consequence of tangency. Classical second-order
expansion of the same constraints supplies the same prediction. It does not
demonstrate a new planning decision beyond differential and algebraic methods.

## Algebraizing the production head

Introduce sine/cosine coordinates `sB,cB,sC,cC`, imposing
`sB²+cB²=1`, `sC²+cC²=1`. Tool direction is
`(sB cC,sB sC,cB)`; tip coordinates are polynomial in these and linear joints.
The table/head extension is likewise polynomial in sine/cosine variables.
Rotary winding, limits, inequalities and collision envelopes must still be
retained separately; algebraization must not identify physically different turns.

At the north-pole task fiber, cB=1 and `sB cC=sB sC=0`. Since
`cC²+sC²=1`, these equations imply sB=0. The remaining algebra is
`ℝ[sC,cC]/(sC²+cC²−1)`: a **reduced smooth circle**, not a double point.
The fiber dimension jumps and task rank drops, while the full frame map remains
rank five. This directly refutes equating every machining pole with a non-reduced
fiber. The twelve-joint model's affine carriage fibers are also ordinary reduced
redundant families.

Axis limits and collision inequalities create feasible-set boundaries and
corners. Their constrained velocity cones are different objects from algebraic
fiber singularities. The production code handles those mechanically rather than
deducing them from multiplicity.

## Comparison and admission decision

| Method | Useful information here | Additional demonstrated planning benefit |
|---|---|---|
| Differential Jacobian and SVD | Rank, conditioning, feasible velocity probes | Production baseline machinery |
| Numerical/geometric continuation | Signed tilt and rotary winding through the pole | Demonstrated by both machining fixtures |
| Polynomial elimination/discriminant | Number of real fold configurations and merger event | Same decision as the quotient experiment |
| Scheme fiber/local ring | Multiplicity and obstructed infinitesimal direction | None demonstrated in these fixtures |
| Tangent-cone/second-order analysis | Tolerance-area scaling near tangency | Conventional expansion gives the same result |
| Flat family/deformation | Conserved algebraic degree across real-root changes | No improved trajectory or cost measured |

No scheme-specific method enters production. A future positive admission would
need a paired failing conventional test and passing scheme-assisted test with
independent correctness, completeness, robustness or cost evidence. Real-root
certification or difficult closed-chain higher-order obstructions are possible
research targets, not established benefits or present capabilities.

For the broader linkage theorem, see Kapovich and Millson,
[Universality theorems for configuration spaces of planar linkages](https://www.math.ucdavis.edu/~kapovich/EPR/KM_2002.pdf),
Topology 41 (2002), 1051–1107. Its hypotheses and finite-cover constructions do
not make an arbitrary CNC mechanism complete, nor guarantee an efficient planner.
