# Evidence boundaries

The recovered repository base is c806c89f446093367e5dfc0e2dc4c21688edee24.
The repository was transferred from `isomorphisms` to
[`the-twin-pines/singularity-free-Gcode`](https://github.com/the-twin-pines/singularity-free-Gcode)
during this run. GitHub repository ID 1416283318 was verified unchanged; the
historical source-pin rows retain their original location for provenance.
The full reachable history and all eight source files were inspected. At recovery
the repository had one main branch and no issues or pull requests. Existing
intention, adverb, train-track/PostScript and source-map work was preserved.

`source-pins.tsv` records base and reference revisions. `source.sha256` binds
implementation, tests, input fixtures, generated comparisons and outputs.
`verification.tsv` records actual local commands/outcomes and explicit unknowns.
It does not self-attest an external acceptance service or physical equipment.

New main work through 24659b853ad922d83cb7869812ffcd61e005e262 was merged into
the implementation branch before publication. Its semantic views, executable
train-track checks and both linkage/compliance research documents are retained.
Run `sha256sum --check evidence/source.sha256` in the verified checkout to check
the retained input/source/output bytes. The manifest excludes itself and runtime
receipts, avoiding a self-referential source digest.

| Evidence | Boundary |
|---|---|
| C tests and independent scalar equations | Disposable x86_64 Linux host |
| LinuxCNC equation parity | Pinned 5axiskins source formulas with explicit convention conversion |
| G-code round trip | Independent parser of the emitted XYZBC subset |
| Continuous collision/corridor bounds | Idealized modeled tool/housing/part/fixture, double precision |
| Velocity/acceleration/jerk peaks | Reference rest-to-rest quintic, not controller measurements |
| GitHub workflow | Exact PR source head checkout; inspect its actual run separately |
| Flexible Pipes / independent ai-ci | NOT_RUN |
| LinuxCNC controller simulation | NOT_RUN; executable unavailable |
| Material-removal and finished chamfer verification | NOT_RUN |
| Physical machining | NOT_RUN |

The before/after G-code preserves a failed baseline as commented diagnostic
blocks after M2. It is deliberately not an executable failed trajectory. The
continuous output is a controller-dialect prototype requiring machine validation.
