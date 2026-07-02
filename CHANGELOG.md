# Changelog

## Unreleased

- Added low-level Bullet bindings for math, collision shapes, rigid dynamics,
  constraints, vehicles, soft bodies, multibodies, collision worlds, and contact
  queries.
- Added `Bullet::Simulation` for local direct/gui/shared-memory-compatible
  modes, primitive body creation, URDF/SDF/MJCF loading, joint helpers,
  ray/contact/AABB queries, camera ray rendering, JSON world snapshots,
  dynamics updates, reset, and cleanup.
- Added native `.bullet` serialization/import wrappers and `btIDebugDraw`
  debug drawing support.
- Added RBS signatures, native/fallback specs, examples, and CI coverage for
  native extension builds.
