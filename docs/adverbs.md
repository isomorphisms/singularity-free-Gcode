# Adverbs as realization constraints

Adriç clauses such as `using`, `assuming`, `without`, `forbidding`, `restricted to`, and `make sure` should not be decorative English around an ordinary function call. Each clause changes the set of admissible realizations or the evidence required from them.

The initial rule is:

> Every modifier has a scope, a target, and a checkable effect on resolution.

## `using`

`using X` positively constrains the realization to one that provides or employs `X`.

```text
draw track A using a pen
emit track A using PostScript
cut track A using a 3 millimetre end mill
```

The named thing may be a mechanism, backend, library, algorithm family, material, or intermediate representation. Adriç must retain which meaning was resolved.

`using a pen` should not be erased into a generic `device = pen` field before the consequences are known. It implies capabilities such as a working/non-working state, planar movement, and deposition rather than material removal.

## `assuming`

`assuming P` records a premise under which the request may be realized.

```text
draw track A assuming a flat work surface
cut track A assuming the stock origin has been probed
```

An assumption is not a postcondition. Adriç may verify it when an observation is available, but it must not pretend to have established it merely because the user permitted reliance on it.

Receipts should distinguish:

```text
verified premise
unverified declared premise
inferred premise
```

## `make sure`

`make sure P` introduces an obligation and requires evidence.

```text
make sure every rail is drawn
make sure non-drawing travel occurs with the pen raised
make sure final cut depth reaches 4 millimetres
```

A successful plan must provide a receipt that can be traced to `P`. Choosing an implementation that usually satisfies `P` is weaker than demonstrating that this realization does.

## `without`

`without X` excludes `X` from the scoped realization.

```text
draw the track without retracing a rail
emit the drawing without rasterization
```

The initial interpretation is local to the selected realization path. It forbids the named feature from the plan that realizes this request.

## `forbidding`

`forbidding X` is stronger than a casual preference to avoid `X`. It excludes `X` throughout the scoped dependency and lowering closure.

```text
emit the drawing forbidding raster intermediates
resolve the backend forbidding vendor-specific pen commands
```

A module does not satisfy this merely by hiding `X` behind an intermediate wrapper. The receipt must demonstrate that the transitive realization does not use it.

This distinction is provisional but useful:

```text
without X     X must not occur in the selected plan
forbidding X  no selected component may introduce X transitively
```

## `restricted to`

`restricted to S` bounds the implementation choice set.

```text
emit motion restricted to G0 and G1
approximate curves restricted to an error below 0.02 millimetres
```

This does not itself choose one realization. It removes candidates outside the named set or bound.

## Strategy adverbs

Strategy adverbs constrain how an action is carried out:

```text
once
repeatedly
back and forth
clockwise
continuously
progressively deeper
roughly, then precisely
```

They are not interchangeable global flags.

### `once`

`Once` requires an explicit counting unit:

```text
draw each semantic stroke once
draw each geometric path once
execute the job once
perform one depth pass
```

For the straight-track example, the requested unit is a semantic stroke: two rails and eight sleepers. Travel moves do not count as drawing. Subdividing one rail into several G-code moves does not turn it into several semantic passes.

### `back and forth`

`Back and forth` constrains consecutive traversal orientation. It may reduce non-working travel, but it often implies repeated traversal of the same geometric support.

That means these may conflict:

```text
draw each rail once
back and forth over each rail repeatedly
```

while these do not:

```text
cut each depth pass once
alternate the direction of successive depth passes
```

The resolver should report the first contradiction instead of choosing whichever phrase appears last.

### `progressively deeper`

`Progressively deeper` applies to a material-removal schedule, not directly to geometry. It requires a monotone increase in absolute cut depth until the requested final depth is reached.

```text
cut rail left
progressively deeper
with no pass deeper than 1 millimetre beyond the preceding pass
make sure final depth is 4 millimetres
```

A pen backend normally lacks this capability. A marker could perhaps make progressively darker strokes, but that would be a different adverb and outcome.

## Scope

Natural language permits modifiers to attach at several levels. Adriç must make the resolved attachment inspectable.

```text
draw [track A once]
draw [each rail once]
[draw track A] once
cut [each rail progressively deeper]
```

A provisional explicit form can be used when ordinary English is ambiguous:

```text
once per rail
once per track
progressively deeper per geometric path
using a pen for the complete job
```

The parser may accept several phrasings, but they should normalize to the same scoped constraint rather than to unrelated procedures.

## Expansion and inspection

A request begins at the level where the user currently cares:

```text
draw track A
```

The user can expose one unresolved choice:

```text
show how track A becomes geometry
show how the geometry is traversed
show how rail left becomes G-code
```

and then constrain only that choice:

```text
use the second traversal
but draw each sleeper from south to north
```

Inspection is therefore not only debugging output. It is part of the language's interaction model: descend into one realization boundary, replace or constrain it, and leave the rest abstract.

## Normalized constraint sketch

The parser may eventually normalize varied English into records resembling:

```text
constraint
  kind        strategy
  name        once
  scope       each semantic stroke of track A
  obligation  exactly one working traversal
  excludes    repeated deposition over the same semantic stroke
```

or:

```text
constraint
  kind        assurance
  name        make sure
  proposition every semantic stroke has a realizing path
  evidence    complete source-map coverage
```

This normalized form is an intermediate description, not necessarily the final Adriç type system.