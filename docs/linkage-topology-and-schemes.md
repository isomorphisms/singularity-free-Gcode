# Linkage topology, algebraic families, and CNC motion

Research notes from discussion of Kevin Walker, Rob Ghrist, and Hrones–Nelson. These are **proposed connections and tests**, not implemented singularity avoidance.

## Sources and cautions

- Kevin Walker, *Configuration Spaces of Linkages* (Princeton undergraduate thesis, 23 April 1985), http://canyon23.net/math/1985thesis.pdf . The scanned thesis explicitly warns of a gap in the proof of Proposition 3.3 and refers to J.-C. Hausmann, *Sur la topologie des bras articulés*. Do not rely on that proposition without checking the correction.
- Robert Ghrist, *Elementary Applied Topology*, chapter 1: linkage configuration spaces as a motivating example. Verify exact wording against the book; previous conversational quotations were not authenticated.
- J. A. Hrones and G. L. Nelson, *Analysis of the Four-Bar Linkage* (1951): mechanical coupler-curve atlas and linkage design reference. Verify individual claims against pages before attributing equations or methods to the book.

## Mathematical model

Fix a planar four-cycle with vertices p₀,…,p₃ and squared edge lengths ℓᵢ². Its real configuration locus is cut out by

    ‖pᵢ₊₁ − pᵢ‖² − ℓᵢ² = 0   (indices modulo 4).

Remove global Euclidean rigid motions by fixing a base edge (with suitable nonzero-length assumptions). Over a parameter ring in the squared lengths, these equations define a family of affine schemes. The physical machine uses a *real*, possibly constrained subset of the fibers; a complex algebraic solution need not be physically realizable.

- **Configuration space:** distinguish connected components, paths, and homology of physically feasible poses.
- **Coupler map:** mark a point on a moving bar; map each configuration to its planar location. Its image is an algebraic/semialgebraic coupler curve (possibly requiring elimination and closure).
- **Singularities:** distinguish singular points of the constraint scheme, critical points of the coupler/tool-position map, and mechanical collision or actuator limits. These are not interchangeable.
- **Parameter families:** vary lengths and identify discriminant loci where geometry, topology, or feasible motion changes. Flatness, singular fibers, and homology are research questions, not assumed conclusions.
- **Fiber products:** assemble mechanisms by requiring shared joint coordinates to agree; compare topological pullbacks with scheme-theoretic fiber products carefully.

## CNC question

Given a requested toolpath γ:[0,1]→work space and a forward-kinematics map F:C→work space, does a continuous feasible lift q:[0,1]→C exist with F(q(t))=γ(t), avoiding a specified singular/forbidden locus? A nonsingular lift alone does not establish collision clearance, safe cutting, or tool orientation.

This should inform *realization* without erasing *intention*:

    named semantic object
      → drawing/cutting operation + scoped adverbs
      → desired geometric toolpath
      → feasible machine-configuration lift
      → traversal and process plan
      → PostScript / plotter / G-code
      ↖ provenance back to the semantic object

Do not turn all levels into one mandatory type system. Keep multiple semantic views and source mappings intact. For example, a train track remains a train track even when represented by tool motions.

## Small experiments

1. Choose four concrete bar lengths and solve the planar closure constraints; draw both mechanism poses and the marked coupler locus.
2. Compute connected components and first Betti number of the real configuration space where feasible; test a collinear limiting case. Separate verified computation from conjecture.
3. Eliminate joint coordinates to obtain a coupler polynomial, and compare its singularities with the original configuration scheme and critical points of the projection.
4. Sweep length parameters across a known change in linkage motion class; record how real fibers and accessible branches change.
5. Request a simple drawing path, test whether it lifts continuously to a chosen mechanism, and emit illustrative G-code only after a feasibility check. Maintain per-motion provenance.
6. Cross-check Walker's Proposition 3.3 against Hausmann before using it as a theorem.

## Open design decision

Represent singularity avoidance as a scoped Adriç realization constraint, not as a claim already enforced by the current G-code generator. Determine separately which requirements are geometric, topological, kinematic, manufacturing, or semantic.
