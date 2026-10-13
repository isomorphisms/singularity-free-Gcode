# Intention stack

Adriç should not treat a request as a call to one predetermined function. A request describes a set of acceptable realizations. Further clauses narrow that set only where the user cares.

A provisional intention record is:

```text
Intention ≝
  subject
  operation
  scope
  assumptions
  realization constraints
  required outcomes
```

For example:

```text
subject                  straight track A
operation                draw
scope                    each rail and each sleeper
assumptions              millimetres; planar work surface
realization constraints  using a pen; once per semantic stroke
required outcomes        every requested component is visibly covered
```

This is deliberately not yet declared to be one type. Parts of it may eventually live in types, effects, modules, contracts, constraint solving, or distinct intermediate representations.

## 1. Semantic object

The highest useful object is not `list of strokes`. It is a train track with named parts and relations:

```text
track A
├── rail left
├── rail right
├── sleeper 1
├── sleeper 2
└── …
```

The object may also contain topology, adjacency, orientation, gauge, sockets, legal connections, and other facts that no anonymous path retains.

A track may provide a drawable view, but becoming drawable must not replace its identity with the weak category `figure`.

## 2. Operation

The operation says what is to happen to the object:

```text
draw track A
engrave track A
cut track A from plate
inspect track A
```

`draw`, `engrave`, and `cut` may share geometry while imposing different physical meanings and required evidence.

## 3. Semantic transformation

A transformation can act on the object before geometric realization:

```text
rotate track A by 30 degrees
move track A 20 millimetres right
scale track A uniformly by 2
```

This differs from blindly transforming emitted coordinates. The former preserves statements such as “left rail,” “socket,” and “connected piece”; the latter has only a path unless provenance survived lowering.

## 4. Adverbial realization constraints

Adverbial clauses select among otherwise valid realizations:

```text
draw track A once
draw track A back and forth
cut track A progressively deeper
draw track A using a pen
cut track A without climb milling
```

They should be scoped predicates over candidate plans, not fields in one giant configuration structure.

Unspecified choices remain open. `draw track A` does not require the user to choose a traversal algorithm merely because the implementation eventually needs one.

## 5. Geometry

The geometric realization introduces points, curves, widths, offsets, and regions:

```text
track A
  ↦ two parallel rail curves
  ↦ eight perpendicular sleeper segments
```

This level answers where the marks or cuts belong. It does not yet determine the order of tool movement or number of passes.

## 6. Traversal

Traversal orders and orients the geometric components:

```text
rail left from west to east
rail right from east to west
then sleepers in increasing x
```

A traversal also inserts non-working movement, such as travel with a pen raised or a tool retracted.

The distinction between geometry and traversal matters because the same ten geometric strokes admit many routes.

## 7. Process schedule

A physical process may require repeated realizations of one geometric path:

```text
pen drawing       one deposition pass
ink darkening     several deposition passes
milling           depth passes 1 mm, 2 mm, 3 mm, 4 mm
laser cutting     repeated power/speed schedule
```

`progressively deeper` belongs here. It does not alter the track geometry, and it should not be confused with retracing a pen line.

## 8. Backend program

The backend lowers the plan into a language with its own state:

```text
PostScript  current point, path, transformation matrix, graphics stack
G-code      units, distance mode, work coordinates, feed, tool position
plotter     pen state, current position, velocity limits
```

At this level, one semantic stroke may become many machine instructions. Conversely, one machine instruction must never silently stand for unrelated semantic objects.

## 9. Receipts and source maps

Every lowering step should emit a map in both directions:

```text
machine motion
  → process pass
  → traversal segment
  → geometric component
  → semantic part
  → track A
```

and:

```text
track A / rail left
  → all geometric and machine operations realizing it
```

This is what allows a later instruction such as:

```text
go over the left rail only once
```

to modify the relevant realization without reconstructing the drawing from anonymous coordinates.

## What does “once” count?

`Once` is meaningless without a scope and counting unit. These are different requirements:

```text
draw the whole job once
draw each track once
draw each rail once
draw each geometric segment once
execute one machine pass
```

For the first train-track example, `once per semantic stroke` means each rail and sleeper receives one continuous working traversal. It does **not** mean one G-code instruction: a curve may require many interpolated instructions while still being traversed once.

## Resolution rule

Conceptually, let `Candidates(subject, operation, environment)` be the possible plans. Each adverb or clause contributes a predicate over those plans. Adriç resolves an intention by finding a candidate satisfying all predicates and producing evidence for the required outcomes.

```text
acceptable(plan) ≝
  supports(plan, subject, operation)
  ∧ assumptions_hold(plan)
  ∧ every_realization_constraint_holds(plan)
  ∧ every_required_outcome_has_a_receipt(plan)
  ∧ source_map_is_complete(plan)
```

Failure should report the conflicting intentions. It must not silently reinterpret `once`, ignore `without`, or replace `cut` with `draw` merely because a backend is available.

## Relation to types and modules

A future type or capability system can reject nonsense such as requesting cutting depth from an ordinary ink pen. Modules can supply alternative geometry, traversal, and backend realizations. The adverb system then narrows which otherwise valid module composition is acceptable.

A useful provisional division is:

```text
types/capabilities   determine what statements can make sense
modules              provide possible realizations
adverbs/constraints  select among those realizations
receipts             demonstrate what was actually chosen and done
```

The boundary is intentionally provisional. The train-track examples are meant to reveal which distinctions must become formal rather than assuming the answer in advance.