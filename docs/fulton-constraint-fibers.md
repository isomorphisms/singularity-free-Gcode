# Fulton: what the worked models establish

Companion [Fulton research revision](https://github.com/walnut-burgundy/fulton/tree/de7d0bf7901f8f595e3518e50a375755c5974ac6):
[source map](https://github.com/walnut-burgundy/fulton/blob/de7d0bf7901f8f595e3518e50a375755c5974ac6/sources/fulton-intersection-theory/cnc-source-map.md),
[two inverse fibers](https://github.com/walnut-burgundy/fulton/blob/de7d0bf7901f8f595e3518e50a375755c5974ac6/notes/cnc-constraint-fibers.md),
[linkage and offsets](https://github.com/walnut-burgundy/fulton/blob/de7d0bf7901f8f595e3518e50a375755c5974ac6/notes/cnc-linkage-and-offsets.md).
These identify unmerged research, not installed postprocessor behavior.

| Existing question | Worked result | Consequence |
| --- | --- | --- |
| Genuine nonreduced IK | Lengths 2,1 and target (distance,0): at distance=1 or 3, ideal=(elbow_x−2,elbow_y²), length 2 | Keep original ideal, reduced support, real feasibility and multiplicity distinct. The vertical tangent is second-order obstructed. |
| AC-head pole | Right-handed Rz(C)Rx(A), initial axis −Z: direction=(−sinA×sinC,sinA×cosC,−cosA), tip=translation+L×direction | For tip-plus-direction semantics, the pole is a **reduced circle**, not a nilpotent fiber. |
| Exceptional freedom | Y meridian limits C=0,π; X meridian limits C=π/2,3π/2 | An isolated pole time cannot join disjoint limits continuously. Dwell/path changes have independent time and collision obligations. |
| Degeneracy | Intrinsic derivative has divisor sinA=0, with zero affine class | Use the actual ideal. The 3×2 ambient derivative fails expected codimension for a fundamental-cycle formula. |
| Litt's chain | Real-rigid extended chains can have reduced positive-dimensional complex schemes | Ordinary radical and Jacobian kernel cannot identify finite real motion. |
| Cutter offsets | Radius 5/4 parabola has regular self-intersection (0,41/32), local factor 9/25 | Local focal and global crossing tests differ. Residual divided differences separate double points from diagonal ramification. |

No measured tool length, pivot offsets or machine identity was recovered.
Fulton uses symbolic L and synthetic L=10. An AC-table or full SO(3) target
is a different model. Exact Macaulay2 experiments and ai-ci's independent
geometric oracle ran on disposable x86_64 Ubuntu, not a machine controller.
Analytic IK and competent continuation recover the same branch information;
Fulton supplies precise multiplicity, excess and limit statements. No speed
or clearance advantage has been established.

The bounded [offline pole limiting-setting compatibility diagnostic](../experiments/pole-continuation/README.md)
is implemented. It returns incoming and outgoing limiting ideals, their
intersection, the full exceptional fiber and scoped status; it emits no motion.
Flexible Pipes runs the Grease orchestration and independent ai-ci acceptance.
Macaulay2 owns algebra; C retains the later numerical-geometry boundary.

Attach results to the requested toolpath's feasible-configuration-lift view,
retaining the [semantic object](intention-stack.md), [overlapping views](semantic-views.md),
scoped [constraints](adverbs.md) and provenance. A singularity diagnostic
cannot erase the named object or authorize a controller action.
