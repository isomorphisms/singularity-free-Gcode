# singularity-free-Gcode

An experimental laboratory for expressing drawings and machine work without collapsing the user's intention into anonymous strokes too early.

The central Adriç question is not merely how to emit G-code. It is how to retain several levels of intention at once:

- a semantic object, such as a train track;
- an operation, such as drawing, engraving, or cutting it;
- adverbial constraints on how the operation is realized;
- a geometric realization in paths and regions;
- a traversal or machining plan;
- a concrete PostScript, plotter, or G-code program;
- a source map back from every emitted motion to the object that caused it.

The same track may therefore be:

```text
draw track once using a pen
```

or:

```text
cut track from plate
using an end mill
back and forth where safe
progressively deeper
make sure every rail reaches the requested depth
```

Those are not two unrelated programs and they are not merely two configurations of a list of lines. They share a semantic object and differ in operation, realization policy, process schedule, and mechanism.

## Running example

The first example follows Danny Calegari's post [Laying train tracks](https://lamington.wordpress.com/2011/12/02/laying-train-tracks/), prompted by laying track with his son Felix. A sequence of left- and right-turning pieces is a train track before it is a collection of pen strokes. Calegari's PostScript is valuable precisely because the program retains named `Rtrack`, `Ltrack`, `R`, and `L` operations rather than presenting only a finished picture.

This repository uses that example to test:

1. semantic identity through lowering;
2. transformations applied to a track as a whole;
3. one-pass, repeated, and back-and-forth traversals;
4. pen, PostScript, plotter, and milling realizations;
5. inspection and replacement of one lowering decision without discarding the rest.

## Relation to `casselman`

[`dilapidated-shed/casselman`](https://github.com/dilapidated-shed/casselman) is the provenance-first corpus of mathematical drawing programs and source references. This repository is the implementation and language-design laboratory. Historical source stays visibly historical; Adriç experiments live here.

## Working meaning of “singularity-free”

For now, the name imposes one definite rule: there must be no irreversible point at which a train track, curve, diagram, or manufactured part becomes *only* G-code. Lowering may add detail, but it must retain the relation between low-level motions and high-level objects.

Geometric and kinematic singularity avoidance may later become explicit realization constraints. The repository does not yet claim to have solved those separate problems.

## Initial documents

- `docs/intention-stack.md` — the proposed levels and their relations;
- `docs/semantic-views.md` — why track pieces, continuous rails, figures, paths, and machine motions form overlapping views rather than one erasing hierarchy;
- `docs/adverbs.md` — Adriç clauses as scoped constraints on valid realizations;
- `examples/train-track/` — one semantic track lowered to PostScript and illustrative G-code.

The syntax is exploratory. This repository deliberately postpones deciding whether every distinction belongs in a type system, an effect system, a module system, a constraint solver, or a family of intermediate representations.

## Check the first example

```sh
sh checks/check-train-track.sh
```

The check verifies ten semantic strokes, one PostScript and G-code source marker for each, exactly one working XY traversal per stroke, and pen-up travel. When Ghostscript is available it also parses the PostScript program. The same check runs in GitHub Actions.