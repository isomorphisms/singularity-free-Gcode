# One-toolhead twelve-joint simulation

The primary X carriage is limited to ±10 mm while the required tool motion
reaches 40 mm. The additional carriage and movable workpiece share that motion.
`axis-trace.tsv` records all twelve joints. This fixture emits no twelve-axis
G-code: the production adapter currently supports only five direct coordinates.
