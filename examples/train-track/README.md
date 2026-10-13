# Ten-stroke train track

This example starts with the deliberately simple case from the Adriç discussion: ten pen strokes that the user regards as **one train track**.

```text
straight track A
├── rail left
├── rail right
├── sleeper 01
├── sleeper 02
├── sleeper 03
├── sleeper 04
├── sleeper 05
├── sleeper 06
├── sleeper 07
└── sleeper 08
```

Another arbitrary collection of ten strokes is not thereby a train track. The semantic object records the roles, arrangement, gauge, and common transformation of these components.

## Geometry

The untransformed example uses millimetres:

```text
centreline     (20, 50) → (180, 50)
gauge          20
left rail      (20, 60) → (180, 60)
right rail     (20, 40) → (180, 40)
sleepers       x = 25, 45, 65, 85, 105, 125, 145, 165
sleeper span   y = 35 → 65
```

The labels `left` and `right` follow the orientation of the centreline from west to east. They are semantic roles, not merely aliases for larger and smaller `y` coordinates.

## Intention

`intent.adric` contains two realizations of the same object:

1. draw every semantic stroke once with a pen;
2. cut the same geometry in repeated depth passes.

The second is not “the first program with a different device setting.” Drawing and cutting impose different process schedules, capabilities, and assurances.

## Lowered examples

- `straight-track.ps` — a PostScript program retaining a named `straightTrack` procedure and semantic IDs;
- `straight-track-pen.gcode` — illustrative, machine-neutral pen motion using an abstract Z lift;
- `source-map.tsv` — stable semantic IDs shared by both outputs.

The G-code is for inspection and simulation. `Z5` means pen raised and `Z0` means pen touching only in this abstract example. It must not be sent to an actual machine without a machine-specific adapter, limits, origin, and safety review.

## Required invariants

A correct one-pass pen realization must demonstrate:

```text
each of the ten semantic strokes has one working traversal
all repositioning occurs while the pen is raised
every emitted working move maps to exactly one semantic stroke
the complete output maps back to straight track A
```

A curve may later be approximated by many G-code segments while still satisfying “once per semantic stroke.” Instruction count is not the semantic pass count.

## Why preserve the track object?

Preservation allows later requests such as:

```text
rotate the track by 30 degrees
move the complete track 15 millimetres right
redraw only sleeper 04
go over both rails twice but each sleeper once
replace the sleepers with rounded rectangles
```

Without the object and source map, these become error-prone coordinate surgery over anonymous motions.

## Next example

The next, less trivial object should model Danny Calegari's left- and right-turning track pieces. A sequence such as `R R L R …` carries connectivity, orientation, and endpoint holonomy before it is rendered as PostScript curves or lowered to machine motion.