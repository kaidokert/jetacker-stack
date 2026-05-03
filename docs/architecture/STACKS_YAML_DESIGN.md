# stacks.yaml Design Rationale

## Purpose

`stacks.yaml` is a declarative manifest that serves as the single source of truth for
stack topology — what components exist, how they compose into stacks, and what
dependencies/constraints govern startup ordering.

It does NOT replace `docker-compose.yml`. Compose owns container-level config (volumes,
network_mode, environment variables, command arguments). `stacks.yaml` owns the logical
layer: what runs where, in what order, with what readiness checks.

## Core Design Principles

### 1. Convention over Configuration

- **`config:` block** centralizes naming pattern and component defaults
- **Naming**: `{robot}-{component}` generates compose service names automatically
- **Defaults**: `kind: node`, `modes: [sim, hw]`, `critical: true`, `timeout: 30`, `gates: []`
- Only deltas from defaults appear in component definitions
- Three run shorthands: `ros:` / `launch:` / `script:` determine launch method

### 2. Separation of Concerns

| Concern | Owner |
|---------|-------|
| What components exist, their metadata | `stacks.yaml` components |
| Which components form a stack | `stacks.yaml` stacks |
| Startup ordering and readiness | `stacks.yaml` after + gates |
| Container config (volumes, network, env) | `docker-compose.yml` |
| Runtime orchestration logic | `stack.py` (reads stacks.yaml) |

### 3. Composition via Inheritance

Stacks use `extends` to inherit from parent stacks, with `overrides` for per-stack
dependency rewiring. Example: `nav2_odom` extends `base` but overrides Nav2 server
dependencies from `ekf_localization` to `static_map_odom`.

### 4. Legacy Quarantine

The `slam_bot` robot uses `service_map` (name translation) and `component_excludes`
(subtraction) to handle its non-standard naming and different component set, keeping
the `base` stack definition clean.

## Schema Reference

### Component Fields

| Field | Default | Description |
|-------|---------|-------------|
| `kind` | `node` | `node` \| `infra` \| `bridge` \| `transient` \| `virtual` |
| `modes` | `[sim, hw]` | Execution modes: `[sim]`, `[hw]`, or `[sim, hw]` |
| `ros` | — | `"package/executable"` → `ros2 run package executable` |
| `launch` | — | `"package/file.launch.py"` → `ros2 launch package file` |
| `script` | — | `"path/to/script.py"` → `python3 /workspace/path/to/script.py` |
| `node` | component name | ROS2 node name if different from component key |
| `compose_profile` | — | Docker compose profile/image group |
| `after` | `[]` | Components that must start before this one |
| `gates` | `[]` | Readiness checks (string DSL) |
| `timeout` | `30` | Seconds to wait for gates |
| `critical` | `true` | Whether failure aborts stack startup |
| `excludes` | `[]` | Mutually exclusive components |

### Run Semantics

Exactly one of `ros:`, `launch:`, `script:`, or none (for `infra`/`virtual`):

- `ros: "pkg/exe"` → stock ROS2 node via `ros2 run`
- `launch: "pkg/file.launch.py"` → ROS2 launch file via `ros2 launch`
- `script: "path.py"` → custom Python node via `python3`
- none → infrastructure (gazebo, x11) or virtual gate

### Kind Enum

| Kind | Description |
|------|-------------|
| `node` | Long-running ROS2 node |
| `infra` | Non-ROS2 infrastructure (Gazebo, x11, debug shell) |
| `bridge` | Gazebo↔ROS2 or Foxglove relay |
| `transient` | Runs and exits (controller_spawner) |
| `virtual` | No container — just a gate validation point in the DAG |

### Gate String DSL

Gates use a colon-separated string format: `type:arg1:arg2`

| Gate | Format | Description |
|------|--------|-------------|
| Clock | `clock_monotonic` | Wait for sim clock to be publishing |
| Topic | `topic_active:/topic_name` | Wait for topic to have publishers |
| TF Static | `tf_static:parent:child` | Wait for static TF to exist |
| TF Transform | `tf_transform:parent:child` | Wait for dynamic TF to exist |

### Stack Composition

```yaml
stacks:
  nav2:
    extends: [base]           # Inherit all components from base
    include: [amcl, ...]      # Add these components
    overrides:                 # Per-component tweaks for this stack
      planner_server: { after: [static_map_odom] }
    lifecycle_managed: [...]   # Components managed by lifecycle_manager
```

### Robot Declaration

```yaml
robots:
  jetacker:
    stacks: [base, nav2, ...]              # Supported stacks
    naming: "{robot}-{component}"          # Service name pattern (default)
    service_map: { old: new }              # Name exceptions
    component_excludes: [comp1, comp2]     # Components to skip from inherited stacks
```

## What stacks.yaml Generates

| Output | How |
|--------|-----|
| Service lists for `stack.py` | Flatten `extends` + `include` - `component_excludes` |
| DAG for reset orchestrator | Component `after` edges + stack `overrides` |
| Lifecycle manager params | Stack `lifecycle_managed` list → `node_names` param |
| Conflict detection | `conflicts` list + component `excludes` |
| Documentation matrix | Iterate robots × stacks × components |

## Changes Applied (v1.1)

Based on external review feedback, four changes were applied:

1. **`env` → `modes`**: Better semantics, extensible to `[replay, ci, bench]` later
2. **`infrastructure` → `infra`**: Shorter, consistent with `kind` being a small enum
3. **`launch:` field added**: Separate from `ros:` — no more "exception" comments for slam_toolbox
4. **`image` → `compose_profile`**: Honest about what the field means (not a Docker image tag)

---

## Deferred Improvements

Suggestions evaluated and deferred for future implementation when the need arises.

### Typed Gate Objects (Priority: Medium)

**Current**: String DSL `"topic_active:/odom"`
**Proposed**: Typed objects with per-gate parameters

```yaml
gates:
  - type: topic_active
    topic: /odom
    min_hz: 5
    timeout: 10
  - type: tf_transform
    parent: odom
    child: base_link
    max_age_ms: 500
```

**Why deferred**: Current 4 gate types work fine as strings. Add typed objects when you
need per-gate timeouts, `min_hz`, `max_age_ms`, or `severity: warn` vs `fail`.

**Migration**: Support both formats in the parser — strings for simple cases, objects
for parameterized ones. Normalize to objects internally.

### Per-Robot Component Overrides (Priority: Low)

**Current**: `service_map` (name translation) + `component_excludes` (subtraction)
**Proposed**: Full `component_overrides` at robot level

```yaml
robots:
  slam_bot:
    component_overrides:
      clock_bridge:
        ros: ros_gz_bridge/parameter_bridge  # different config
        gates: []
```

**Why deferred**: `service_map` + `component_excludes` covers slam_bot's needs.
Add `component_overrides` when a second real robot needs different component behavior
(not just different naming).

### Auto-Derived Conflicts via `provides:` (Priority: Low)

**Current**: Manual `conflicts:` list (4 entries)
**Proposed**: Declare what each component provides, auto-detect conflicts

```yaml
amcl:
  provides_tf: ["map->odom"]
slam_toolbox:
  provides_tf: ["map->odom"]
gazebo:
  provides: ["gazebo_sim"]
```

Then `stack.py` auto-conflicts if two selected components provide the same exclusive
resource. Keep manual `conflicts:` as override/escape hatch.

**Why deferred**: 4 manual conflict entries is manageable. Add `provides:` when you
have 10+ stacks or cross-robot resource conflicts become common.

### Component Classification (Priority: Low)

**Proposed**: Add `class: core | optional | dev | test` field

```yaml
ekf_localization:
  class: core          # failure is fatal
cmd_vel_relay:
  class: optional      # nice to have
test_drive:
  class: test          # only for testing
debug:
  class: dev           # development tooling
```

Could derive `critical` from `class` instead of setting it manually.

**Why deferred**: Current explicit `critical: true/false` works. The 5 non-critical
components are already marked. Add `class` when you want richer categorization for
docs or filtering (e.g. `stack.py start --only core`).

### Lifecycle Manager as Role (Priority: Low)

**Proposed**: Formalize lifecycle management as a component role

```yaml
components:
  lifecycle_manager:
    role: lifecycle_manager

stacks:
  nav2:
    lifecycle:
      manager: lifecycle_manager
      managed: [map_server, amcl, ...]
```

**Why deferred**: Only one lifecycle manager exists. Current `lifecycle_managed` on the
stack is sufficient. Add `role:` when you have multiple lifecycle managers or want to
auto-validate that exactly one exists per stack.

### Sparse `after` + Rich Gates (Priority: Low)

**Observation**: Some `after` edges duplicate what gates already enforce. For example,
`robot_state_publisher after clock_bridge` could be replaced by just gating on TF.

**Counter-argument**: `after` as "don't even attempt to start until X is up" is cheaper
than letting things start and fail/retry. Keep `after` for real startup ordering
dependencies, gates for functional readiness.

**Conclusion**: Keep both. `after` = ordering constraint, `gates` = readiness proof.
They serve different purposes even when they overlap.
