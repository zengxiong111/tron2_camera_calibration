# Standalone extraction review

Baseline: `fd672f3f329e188ceb392201a43faf5fa4052c47`. Initial extraction commit: `069bd60`. Review follows the Matt Pocock two-axis code-review workflow.

## Standards

Two findings were resolved before publication:

- Wheel package-data previously matched all configuration JSON files, including ignored machine profiles. It now matches only `*.example.json`. A disposable source copy with all three local profile files confirmed that none enters the wheel.
- Source provenance initially had pending final hashes. The manifest now records adapter origins, original import hashes and edited destination hashes. The hashes were refreshed after final guide edits.

## Spec

One finding was resolved: the broad wheel JSON glob violated the requirement to exclude local profiles and credentials. The narrowed glob and packaging boundary check resolve it. No outstanding implementation gaps were reported. Publication and collaborator invitations are performed after local verification.

## Validation

Source tests: 19 passed. Fresh wheel installation with declared test/live dependencies outside the checkout: 19 passed, three CLI help checks, packaged resource checks, blocked parent deployment imports, and head/wrist kinematic parity checks passed. Model snapshots omit meshes; camera, robot and ROS capture were not exercised.

Totals: Standards 2 resolved, 0 outstanding; Spec 1 resolved, 0 outstanding.
