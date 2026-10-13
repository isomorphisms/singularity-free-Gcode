# Existing orchestration boundary

Leaf CNC code owns kinematics, planning, controller output and local evidence.
Flexible Pipes owns registered execution, stage artifacts and captured receipts;
ai-ci owns independent acceptance. No second scheduler or leaf copy of their
acceptance policy is introduced.

Reviewed owner pins:

- [Flexible Pipes def2a66fe61a2b23440f98031f1defa738a0d1ee](https://github.com/isomorphisms/flexible-pipes/tree/def2a66fe61a2b23440f98031f1defa738a0d1ee)
- [ai-ci 3d174000649f9f828c29e6be981cfd0e60646014](https://github.com/isomorphisms/ai-ci/tree/3d174000649f9f828c29e6be981cfd0e60646014)

`flexible-pipes.pipeline.json` uses the existing schema-version-1 command boundary.
JSON is required by that external interface; inputs, measurements and leaf
receipts otherwise remain ordinary text/TSV. It is a data-only registration
candidate, not a claimed deployed job. A controller must register it with the
pinned owner, supply verified absolute git/make/checkout paths, bind the exact
source commit and retained stage hashes, and arrange independent ai-ci acceptance.
The source-identity stage captures the actual head; its successful process exit
alone cannot establish equality with the controller's requested pin.

No verified deployed Flexible Pipes entrypoint or independent CNC ai-ci gate was
available in this disposable environment. Both receipts therefore remain
NOT_RUN. GitHub CI and independent scalar tests in this repository do not become
external acceptance merely by using the word independent.
