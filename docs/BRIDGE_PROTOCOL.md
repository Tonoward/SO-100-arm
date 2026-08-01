# Blender ⇄ ROS2 data contracts

**Last updated:** 2026-07-28
**Status:** specification — not yet implemented.
**This file is the single source of truth for both formats.** Both
[`ROS2_IMPLEMENTATION_PLAN.md`](ROS2_IMPLEMENTATION_PLAN.md) and
[`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) must conform to it. If you
change a format, change it here first, then both implementations.

## Which part applies to you

| | Contract | Used by |
|---|---|---|
| **Part A** | **The build file** — Blender exports it, ROS2 executes it. Like G-code. | Options **B** and **C** (the recommended slicer architecture — see `BLENDER_ADDON_PLAN.md` §2) |
| **Part B** | **The live socket protocol** — Blender commands the robot in real time. | Option **A**, and any future read-only telemetry link |

✅ **Decided 2026-07-27: Option C.** **Part A is the integration path.** Part B
is **dormant** — `so_arm_100_blender_bridge` is not being built. It is
retained in full because a read-only telemetry subset of it is a likely later
addition, and because it documents why the socket exists at all if one is ever
needed.

`StickSpec` (§6) is shared by both parts.

---

# Part A — The build file

A single JSON document describing an entire job: the sticks, the build order,
the stick lengths, and the validation verdicts. Blender writes it; ROS2 reads,
re-validates and executes it; ROS2 writes status back to a sidecar.

## A.1 Design rules

1. **Self-contained.** Everything needed to build the job is in the file — no
   references back into the `.blend`.
2. **Already in robot coordinates.** Metres, in `base_link`. Blender does the
   transform before writing; ROS2 does no frame conversion.
3. **Ordered.** The `sticks` array *is* the build order. ROS2 executes it
   as-is and does not reorder — one authority, no drift.
4. **Human-readable and diffable.** Pretty-printed, stable key order.
5. **Status lives elsewhere.** The build file is immutable input; progress is
   written to a separate status file so a re-export never destroys progress.

## A.2 Build file format

```json
{
  "format": "so100_build",
  "version": 1,
  "generated": "2026-07-27T14:22:31Z",
  "source": "tower_v3.blend",
  "frame": "base_link",
  "units": "meters",
  "kinematics_version": "1.0.0",
  "stock": { "section_m": [0.00645, 0.00645], "joint_allowance_m": 0.00325,
             "length_range_m": [0.080, 0.150] },
  "build_volume": { "min": [-0.12, -0.45, 0.0], "max": [0.12, -0.29, 0.20] },
  "sticks": [
    {
      "id": "s_001",
      "order": 0,
      "base": [0.020, -0.360, 0.000],
      "tip":  [0.020, -0.360, 0.110],
      "roll_deg": 0.0,
      "length_m": 0.110,
      "shared_ends": 1,
      "supports": [],
      "validation": { "buildable": true, "reason": null },
      "warnings": []
    },
    {
      "id": "s_002",
      "order": 1,
      "base": [0.020, -0.360, 0.11650],
      "tip":  [0.020, -0.360, 0.22650],
      "roll_deg": 0.0,
      "length_m": 0.110,
      "shared_ends": 2,
      "supports": ["s_001"],
      "validation": { "buildable": true, "reason": null },
      "warnings": ["cantilever"]
    }
  ]
}
```

⚠ **`base` and `tip` are the PHYSICAL stick ends, already inset from the ideal
mesh vertices** — ROS2 places exactly what it is given and does no gap
arithmetic. Note the **6.5 mm** gap between `s_001`'s tip (z = 0.110) and
`s_002`'s base (z = 0.1165): that is the glue joint. The ideal mesh vertex
they share sits midway at z = 0.11325, and **both** sticks stop 3.25 mm short
of it — which is why the gap is 2 × 3.25 mm, not 3.25 mm.

*(Corrected 2026-07-28. This example previously read 0.11325 / 0.22325, which
inset only one of the two ends and so contradicted
[`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) §5.2 and the `required_edge`
formula in §5.2.1. The formula is authoritative; these numbers now follow it
and match the addon's output.)*

`s_002` has `shared_ends: 2`, so a further stick meets its tip — that
neighbour would start at z = 0.2330, its own ideal vertex being z = 0.22975.
`shared_ends` describes the stick's **final topology**, independent of
`warnings: ["cantilever"]`, which describes its **temporary support state
during the build sequence** — at the moment `s_002` is placed, only `s_001`
supports it; whatever eventually attaches to its tip hasn't been built yet.

`length_m` is the real stick length — what the operator cuts and loads — so
`‖tip − base‖ == length_m` always, and the 80–150 mm range is checked against
it. The design mesh is expanded so this holds; see
[`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) §5.2.1.

⚠ **Consequence:** the built structure is larger than the mesh the user drew,
by ~3.25 mm per joint along any path. The `build_volume` bounds and every
validation check apply to these expanded coordinates.

| Field | Meaning |
|---|---|
| `kinematics_version` | `so_arm_100_kinematics.__version__` of the copy that generated this file. **ROS2 must compare it against its own and refuse to execute on mismatch** — the two sides sharing a kinematics module is the entire basis for trusting Blender's validation, so a drifted copy is a hard error, not a warning. |
| `order` | Build index. Must be dense and match array position. |
| `length_m` | The **physical stick length** = ‖tip − base‖ = what the human cuts and loads into the feeder |
| `shared_ends` | 0, 1 or 2 — how many of this stick's ends meet another stick. Informational; the gaps are already baked into `base`/`tip`. |
| `supports` | ids of already-placed sticks this one's `base` end attaches to. Empty ⇒ it seats on the base plate. ROS2 uses this to sanity-check the order. |
| `validation` | Blender's verdict. ROS2 **re-validates** and may disagree — if it does, it refuses to start and reports both. |
| `warnings` | `cantilever`, `loop_closure`, `high_valence`, `tight_clearance` (`BLENDER_ADDON_PLAN.md` §6.2) — surfaced to the operator, never fatal. |

Sticks whose `validation.buildable` is `false` **must still appear** in the
file, in order, so the operator sees the complete picture and can decide to
build the rest.

## A.3 Status sidecar

Written by ROS2 (and read back by Blender) next to the build file as
`<name>.status.json`. Rewritten after every state change so a crash loses at
most one stick.

```json
{
  "format": "so100_build_status",
  "version": 1,
  "build_file": "tower_v3.build.json",
  "updated": "2026-07-27T15:03:11Z",
  "current_index": 7,
  "sticks": {
    "s_001": { "status": "placed",  "at": "2026-07-27T14:31:02Z" },
    "s_002": { "status": "placed",  "at": "2026-07-27T14:35:40Z" },
    "s_008": { "status": "failed",  "at": "2026-07-27T15:03:11Z",
               "reason": "grasp verification failed twice" }
  }
}
```

`status` ∈ `pending`, `placed`, `failed`, `skipped`. Anything absent is
`pending`.

## A.4 Execution model

ROS2 loads the build file and, for each stick in order:

1. **Prompt the human**: *"Load stick `s_008`, stick length 110 mm, into the
   feeder. Press Enter."* — this is the Q1 requirement, and it is the reason
   `length_m` is in the file.
2. `PickStick` → runs to completion, ends holding the stick.
3. **Prompt**: *"Press Enter to place."*
4. `PlaceStick` → carries it to the target, holds, motionless.
5. **Prompt**: *"Glue it. Press Enter to release."*
6. `ReleaseStick` → opens, registers the placed stick as an obstacle,
   withdraws, returns to standby.
7. Write the status sidecar; continue.

The prompt machinery already exists and is proven — it is the same
`logger.info` + `input()` gate used by today's demo
(`ROS2_IMPLEMENTATION_PLAN.md` §3 finding #10). Run it with `ros2 run`, not
`ros2 launch`.

## A.5 Resuming

On start, if a status sidecar exists, ROS2 re-adds every `placed` stick to the
planning scene as a permanent collision object and resumes at the first
non-`placed` stick. This makes the scene correct after any restart — see
`ROS2_IMPLEMENTATION_PLAN.md` §10.

---

# Part B — The live socket protocol

*Applies only if the live-bridge architecture (Option A) is chosen, or for a
future read-only telemetry link.*

## 1. Why this protocol

### 1.1 The constraint that forces a socket

Blender ships its own CPython (3.11 for Blender 4.x). Every ROS2 Python
extension on this machine is compiled for **CPython 3.10**. `import rclpy`
inside Blender cannot work — it is an ABI mismatch, not a `PYTHONPATH`
problem. So the addon must speak a transport.

### 1.2 Alternatives considered

| Option | Verdict |
|---|---|
| `rclpy` inside Blender | ✗ Impossible (§1.1). |
| Run a ROS2 node in a subprocess with the system Python, talk over stdin/stdout | ✗ Works, but process lifetime is tied to Blender, hard to debug, awkward when ROS is already running. |
| `rosbridge_suite` (WebSocket + JSON) | ~ Generic and well-known, but needs `ros-humble-rosbridge-suite` installed plus a WebSocket client inside Blender's Python (`websocket-client` is not bundled), and exposes raw ROS topics rather than the small task-level API this project actually wants. Keep as a fallback if a second client ever needs generic ROS access. |
| **Newline-delimited JSON over TCP** | ✅ **Chosen.** Stdlib-only on both sides (`socket`, `json`), trivially debuggable with `nc`/`telnet`, easy to mock without ROS, and the API is task-level (`pick_stick`) rather than transport-level. |

### 1.3 Design rules

1. **Never block Blender's UI.** Long operations return immediately with an
   acknowledgement; completion arrives later as an event.
2. **One frame, one unit system.** Everything on the wire is metres, in
   `base_link`, right-handed Z-up. Blender does its own transform before
   sending (see the addon doc §5). The bridge does *no* frame conversion.
3. **The robot is authoritative.** Blender proposes; ROS validates and may
   refuse. Blender must never assume a placement is reachable.
4. **Additive evolution.** Unknown JSON fields are ignored by both sides;
   unknown commands get a structured error, never a disconnect.

---

## 2. Transport

- **TCP**, default bind `127.0.0.1:5556` (configurable on both sides).
- **Framing:** one JSON object per line, UTF-8, terminated by `\n`.
  No embedded newlines (`json.dumps` default is safe).
- **One client at a time.** A second connection is accepted and immediately
  closed with an `error` frame (`code: "busy"`).
- **Reconnect:** the client may drop and reconnect at any time. The server's
  state machine is *not* reset by a disconnect — a robot holding a stick keeps
  holding it. On reconnect the client should issue `get_status` first.
- **Keepalive:** the client sends `ping` every 5 s. If the server sees no
  traffic for 30 s it logs a warning but does **not** abort a running task.

---

## 3. Frame formats

Three frame types share one socket.

### 3.1 Request (client → server)

```json
{"id": 17, "cmd": "pick_stick", "args": {"stick_id": "s_004"}}
```

- `id` — client-generated, monotonically increasing integer. Echoed back.
- `cmd` — command name (§5).
- `args` — object; may be omitted when empty.

### 3.2 Response (server → client) — exactly one per request

Success:
```json
{"id": 17, "ok": true, "result": {"accepted": true, "task_id": "t_31"}}
```

Failure:
```json
{"id": 17, "ok": false, "error": {"code": "wrong_state", "message": "pick_stick requires state IDLE, currently HOLDING"}}
```

A response means **the command was accepted or rejected** — for long-running
commands it does *not* mean the motion finished.

### 3.3 Event (server → client, unsolicited)

```json
{"event": "task_progress", "data": {"task_id": "t_31", "step": "lift", "step_index": 6, "step_count": 8}}
```

Events have no `id`. The client must tolerate events arriving interleaved with
responses, and must tolerate event types it does not know.

---

## 4. Error codes

| Code | Meaning |
|---|---|
| `bad_request` | Malformed JSON, missing `cmd`, or bad argument types |
| `unknown_command` | Not in §5 |
| `wrong_state` | Command not valid in the current task state |
| `busy` | Another task is running, or a second client connected |
| `unreachable` | IK found no solution for the requested placement |
| `planning_failed` | MoveIt could not find a path |
| `execution_failed` | The controller aborted or the arm did not arrive |
| `grasp_failed` | Grasp verification says the gripper closed on nothing |
| `not_ready` | ROS side up but MoveIt / controllers not yet available |
| `aborted` | Cancelled by an `abort` command |
| `internal` | Anything else; `message` carries detail |

---

## 5. Commands

### 5.1 `ping`
`args`: none → `result`: `{"pong": true, "server_time": <float epoch>}`

### 5.2 `get_status`
`args`: none

`result`:
```json
{
  "state": "IDLE",
  "holding_stick_id": null,
  "moveit_ready": true,
  "controllers_ready": true,
  "joint_positions_deg": {"Shoulder_Rotation": 0.0, "Shoulder_Pitch": -99.98,
                          "Elbow": 85.94, "Wrist_Pitch": 71.62,
                          "Wrist_Roll": 90.0, "Gripper": 30.0},
  "tcp_pose": {"position": [0.13, 0.0, 0.21], "quaternion_xyzw": [0,0,0,1]},
  "active_task": null,
  "protocol_version": 1
}
```

`state` is one of `IDLE`, `PICKING`, `HOLDING`, `PLACING`, `AT_PLACE`,
`ERROR` — see the ROS2 doc §6. **Angles on the wire are degrees**, matching
the project's existing config convention. Positions are metres.

### 5.3 `get_workspace`
`args`: `{"resolution_m": 0.01}` (optional)

`result`: the cached reachability map, for Blender to visualise:
```json
{
  "frame": "base_link",
  "bounds_min": [-0.30, -0.30, 0.00],
  "bounds_max": [ 0.30,  0.30, 0.35],
  "resolution_m": 0.01,
  "encoding": "rle_bitmap_z_major",
  "dims": [61, 61, 36],
  "data": "<base64 run-length-encoded occupancy bitmap>",
  "notes": "vertical-stick placements only; see ROS2 doc §9.3"
}
```

Sending a raw voxel list would be megabytes of JSON; RLE + base64 keeps it
small enough to fetch on demand. The client caches it and re-fetches only when
the user asks.

### 5.4 `validate_placements`
The most important command for UX: check a whole design at once, before any
motion.

`args`:
```json
{"sticks": [ <StickSpec>, ... ]}
```

`result`:
```json
{"results": [
  {"id": "s_001", "reachable": true,  "reason": null},
  {"id": "s_002", "reachable": false, "reason": "out_of_reach",
   "detail": "target 0.412 m from base, max 0.340 m in this direction"},
  {"id": "s_003", "reachable": false, "reason": "orientation_infeasible",
   "detail": "tilt is out of the arm plane; 5-DOF cannot yaw the tool independently"}
]}
```

`reason` ∈ `out_of_reach`, `orientation_infeasible`, `joint_limit`,
`collision_with_placed`, `collision_with_self`, `below_table`.

This is a pure query — it never moves the robot and is valid in any state.

### 5.5 `set_build_plan`
`args`: `{"sticks": [<StickSpec>, ...], "replace": true}`

Stores the ordered list server-side and persists it. `result`:
`{"accepted_count": 42, "rejected": [{"id": "s_002", "reason": "out_of_reach"}]}`

### 5.6 `pick_stick` *(long-running)*
`args`: `{"stick_id": "s_004", "length_m": 0.08, "source": "feeder_0"}`

`result`: `{"accepted": true, "task_id": "t_31"}`
Completion arrives as a `task_done` event. Requires state `IDLE`; ends in
`HOLDING` on success.

### 5.7 `place_stick` *(long-running)*
`args`: `{"stick": <StickSpec>}` — or `{"stick_id": "s_004"}` to use the spec
already stored by `set_build_plan`.

Requires `HOLDING`; ends in `AT_PLACE` **with the gripper still closed**.
Fails fast with `unreachable` *before moving* if IK has no solution.

### 5.8 `release_stick` *(long-running)*
`args`: none. Requires `AT_PLACE`. Opens the gripper, registers the stick as a
permanent obstacle, withdraws, returns to standby, ends in `IDLE`.

### 5.9 `go_to_named` *(long-running)*
`args`: `{"name": "home"}` — `home`, `standby`, `init`.

### 5.10 `abort`
`args`: none. Cancels the active task and stops motion. Always accepted.
Ends in `ERROR` (deliberately — it forces an explicit operator decision about
where the arm and the stick actually are).

### 5.11 `reset`
`args`: `{"assume_gripper_empty": true}`. Clears `ERROR` → `IDLE`. The flag
exists because after an abort only the human knows whether a stick is still in
the jaws.

### 5.12 `set_placed_sticks`
`args`: `{"sticks": [<StickSpec>, ...]}`

Overwrites the planning scene's set of already-placed sticks. Used when
Blender and ROS have to be re-synchronised after a restart on either side.

---

## 6. `StickSpec` object

Identical on the wire and in `so_arm_100_stick_msgs/msg/StickSpec.msg`.

```json
{
  "id": "s_004",
  "base": [0.145, 0.030, 0.000],
  "tip":  [0.145, 0.030, 0.080],
  "roll_deg": 0.0,
  "length_m": 0.080,
  "section_m": [0.006, 0.006]
}
```

| Field | Required | Meaning |
|---|---|---|
| `id` | yes | Stable identifier, generated by Blender, stable across saves |
| `base` | yes | metres, `base_link`. **The end that seats against the table or another stick.** |
| `tip` | yes | metres, `base_link`. The free end. |
| `roll_deg` | no (default 0) | Rotation about the stick's own axis. Matters — the stock is square. |
| `length_m` | no | Defaults to ‖tip − base‖. If given and inconsistent by >1 mm, the server responds `bad_request`. |
| `section_m` | no | Defaults to the configured stock section. |

Rationale for base/tip rather than a pose: a lattice of sticks is naturally
described by its endpoints, and "which end goes down" is exactly the fact the
grasp-offset maths needs (ROS2 doc §8.2). It also removes any quaternion
convention ambiguity between Blender (w,x,y,z) and ROS (x,y,z,w) for the
common case.

---

## 7. Events

| Event | `data` | When |
|---|---|---|
| `state_changed` | `{"from": "IDLE", "to": "PICKING"}` | Every task-state transition |
| `task_started` | `{"task_id", "command", "step_count"}` | A long-running command begins |
| `task_progress` | `{"task_id", "step", "step_index", "step_count"}` | Each sequence step |
| `task_done` | `{"task_id", "ok": true, "result": {...}}` or `{"task_id", "ok": false, "error": {...}}` | A long-running command finishes |
| `grasp_result` | `{"stick_id", "verified": true, "gap_rad": 0.34}` | After grasp verification |
| `stick_placed` | `{"stick_id"}` | After `release_stick` succeeds — Blender marks it done |
| `robot_state` | `{"joint_positions_deg": {...}, "tcp_pose": {...}}` | Throttled to ~10 Hz, **only while `subscribe_state` is on** |
| `log` | `{"level": "warn", "message": "..."}` | Server-side messages worth surfacing in the Blender UI |

`subscribe_state` is toggled by `{"cmd": "subscribe_state", "args": {"enabled": true, "rate_hz": 10}}` — off by default so an idle addon costs nothing.

---

## 8. Worked example — one full cycle

```
C→S  {"id":1,"cmd":"get_status"}
S→C  {"id":1,"ok":true,"result":{"state":"IDLE","holding_stick_id":null, ...}}

C→S  {"id":2,"cmd":"validate_placements","args":{"sticks":[{...s_004...}]}}
S→C  {"id":2,"ok":true,"result":{"results":[{"id":"s_004","reachable":true,"reason":null}]}}

C→S  {"id":3,"cmd":"pick_stick","args":{"stick_id":"s_004","length_m":0.08}}
S→C  {"id":3,"ok":true,"result":{"accepted":true,"task_id":"t_31"}}
S→C  {"event":"state_changed","data":{"from":"IDLE","to":"PICKING"}}
S→C  {"event":"task_progress","data":{"task_id":"t_31","step":"lower","step_index":3,"step_count":8}}
S→C  {"event":"grasp_result","data":{"stick_id":"s_004","verified":true,"gap_rad":0.34}}
S→C  {"event":"task_done","data":{"task_id":"t_31","ok":true,"result":{}}}
S→C  {"event":"state_changed","data":{"from":"PICKING","to":"HOLDING"}}

     ... user clicks "Place" in Blender ...

C→S  {"id":4,"cmd":"place_stick","args":{"stick_id":"s_004"}}
S→C  {"id":4,"ok":true,"result":{"accepted":true,"task_id":"t_32"}}
S→C  {"event":"task_done","data":{"task_id":"t_32","ok":true,"result":{}}}
S→C  {"event":"state_changed","data":{"from":"PLACING","to":"AT_PLACE"}}

     ... user glues the stick, then clicks "Release" ...

C→S  {"id":5,"cmd":"release_stick"}
S→C  {"id":5,"ok":true,"result":{"accepted":true,"task_id":"t_33"}}
S→C  {"event":"stick_placed","data":{"stick_id":"s_004"}}
S→C  {"event":"state_changed","data":{"from":"AT_PLACE","to":"IDLE"}}
```

---

## 9. Testing the protocol without a robot

The bridge node must support `--mock`: it serves this entire protocol with
canned timings and **no ROS dependency at all**, so the Blender addon can be
built and demoed on a laptop with no ROS installed.

Manual smoke test of the real bridge:
```bash
ros2 run so_arm_100_blender_bridge bridge_node
# other terminal:
printf '{"id":1,"cmd":"get_status"}\n' | nc localhost 5556
```

---

## 10. Versioning

`get_status.result.protocol_version` is an integer, currently **1**. Bump it
on any breaking change; the addon warns (and refuses long-running commands)
if the server's version is higher than it knows.
