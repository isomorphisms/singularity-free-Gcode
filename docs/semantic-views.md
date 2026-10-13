# Semantic views, not one erasing hierarchy

The levels of intention do not form one simple chain:

```text
train track → figure → strokes → G-code
```

That chain loses too much. An assembled track has several useful decompositions at the same time, and no one of them should erase the others.

## Several views of one track

```text
assembly view
  pieces, sockets, connection order, left/right choices

railway view
  continuous rails, gauge, sleepers, orientation, loops

drawing view
  paths, widths, caps, joins, colours, fill/stroke decisions

manufacturing view
  material regions, centre lines, kerf or tool compensation, depth

traversal view
  ordered working paths, reversals, lifts, travel moves

machine view
  modal state and concrete instructions
```

A train track can supply a drawing view without ceasing to be a train track. In this sense, `figure` is better treated as a capability or projection than as the replacement type of every drawable thing.

```text
view track A as a figure
```

should mean approximately:

```text
provide bounds, transformations, and renderable geometry for track A
while retaining track A and its named parts
```

## Membership is many-to-many

A single geometric segment may simultaneously realize:

```text
piece 04
left rail
continuous rail loop 01
track A
visible figure path 17
```

Conversely, one semantic rail may be approximated by hundreds of short machine segments.

The eventual source map therefore cannot be only a tree with one parent per emitted instruction. It needs typed relations such as:

```text
part_of
realizes
approximates
belongs_to_view
produced_by
transformed_from
pass_of
```

The initial `source-map.tsv` uses one parent because the straight ten-stroke example is deliberately simple. It is not the final provenance model.

## The Calegari track exposes the issue

In Danny Calegari's left/right-piece construction, the assembled object can be described as a sequence over `R` and `L`. Each piece has an entry socket, exit socket, orientation change, and geometric realization. The cumulative sequence determines the placement of later pieces.

The same assembly also has two rails that continue across piece boundaries. Therefore these are different requests:

```text
draw each physical piece once
draw each continuous rail once
draw every PostScript path once
execute the complete drawing job once
```

A backend organized around piece procedures may naturally retrace or interrupt a continuous rail. A backend organized around continuous rail paths may cross many piece boundaries in one working traversal. Neither organization is universally “the” track.

## Adverbs attach to views

An adverb must name or resolve the view whose units it counts.

```text
once per piece              assembly view
once per continuous rail    railway view
once per visible path       drawing view
once per depth level        process-schedule view
once per emitted block      machine view
```

When ordinary English leaves this unresolved, Adriç should preserve the unresolved attachment and expose the alternatives. It should not silently pick whichever internal representation happens to be convenient.

```text
draw the track once
```

may normalize first to:

```text
operation event count = 1
subobject traversal multiplicity = unresolved
```

The user can then descend only if the output is wrong:

```text
show how “once” was realized
```

and refine it:

```text
once per continuous rail and once per sleeper
```

## Transformations also have views

These differ:

```text
rotate the complete track
rotate each physical piece about its own centre
rotate the final page image
rotate the machine coordinate system
```

They may sometimes produce related pictures, but they preserve different invariants. The normalized intention must retain which object or view receives the transformation.

## Changes and dependency

Changing one early `R` piece to `L` in an assembled sequence changes the cumulative transformation of the suffix. A useful derivation graph should therefore record both:

```text
piece 04 geometry depends on piece 04 kind
piece 05 placement depends on the accumulated placement through piece 04
```

That dependency information allows selective recomputation without pretending that every downstream coordinate is an independent primitive.

## Proposed rule

Lowering may forget freedom, but it must not forget identity without an explicit boundary.

A backend can choose one traversal, coordinate system, and instruction sequence. It must still return enough provenance to state which semantic objects and views those decisions realized.