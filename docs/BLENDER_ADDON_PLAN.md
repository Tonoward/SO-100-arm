# Blender addon — design & implementation plan

**Last updated:** 2026-07-28
**Status:** specification — nothing implemented yet. **Blender is not
installed on this machine.** Target: **Blender 5.2.0 LTS**, Linux + Windows.
**Audience:** a developer or AI agent implementing the addon.
**Companion documents:**
- [`ROS2_IMPLEMENTATION_PLAN.md`](ROS2_IMPLEMENTATION_PLAN.md) — the robot side.
  Its §9 contains the **validated kinematics** this addon depends on.
- [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) — the build-file format and the
  optional live protocol. Implement against that file, not prose here.

---

## 1. What the addon is for

The user models a sculpture in Blender as a **wireframe mesh**: every edge is
one wooden stick (6.45 mm square stock, **80–150 mm** long). The addon turns
that mesh into a buildable job:

1. **Extract** sticks from the mesh edges.
2. **Compute the build order** — which stick goes down first, second, … so the
   robot never has to reach into a place it has already walled off (§6).
3. **Validate** every placement against the robot's real kinematics, in build
   order, and show the user exactly which sticks are impossible and why.
4. **Emit a cut list** — the human needs to cut each stick to length, and
   needs to know which one to load into the feeder next.
5. **Drive or export the build**, one stick at a time, with the human gates.

Blender is the design tool, the *slicer*, and the build console.

---

## 2. ✅ Architecture: Option C — "slicer + shared kinematics"

**Decided 2026-07-27.** The user proposed treating Blender as a **slicer** —
export a build file, let ROS2 execute it, no live connection. That instinct
was right, and **Option C below was chosen**: the slicer model, plus the
kinematics module vendored into the addon so validation and preview work
offline.

**Practical consequences — apply these throughout:**
- ❌ **No `net/` layer.** Delete it from §4's tree. No sockets, no worker
  threads, no reconnect logic, no mock server. Constraint B2 becomes moot.
- ✅ **`kinematics/` is vendored and load-bearing.** It must stay pure Python
  (no `numpy` — see B4) so it runs in Blender's interpreter on Linux and
  Windows alike.
- ✅ **Part A of [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md)** (the build file
  and its status sidecar) is the entire integration surface.
- ✅ The `Build` panel (§10.3) shows the **next stick to load** and syncs
  status from the sidecar; there is no live action button.

The full comparison is retained below as the record of why.

---

Three options were considered:

### Option A — Live bridge (originally proposed)
Blender holds a TCP connection to a ROS2 bridge node and commands the robot
step by step.

- ➕ Instant validation feedback while designing.
- ➕ Live robot mirroring and live build progress in the viewport.
- ➖ Needs a worker thread, a queue, a main-thread pump, reconnect logic,
  protocol versioning, and a mock server — realistically **~40 % of the
  addon's total complexity, and ~80 % of its bug surface**.
- ➖ Blender must be running, connected, and on the network path to ROS2. The
  user has a **Windows** Blender machine; cross-machine sockets mean firewall
  and address configuration.
- ➖ Nothing is reproducible: no artifact to inspect, diff, version, or replay.

### Option B — Pure slicer (user's proposal)
Blender exports a build file; ROS2 loads and executes it. No connection ever.

- ➕ Radically simpler addon: no networking at all.
- ➕ The build file is a real artifact — inspectable, versionable, replayable,
  emailable, and the exact thing to attach to a bug report.
- ➕ Works across machines and offline by construction.
- ➕ Matches a mental model the user already has (3D printer → G-code).
- ➖ Validation becomes a round trip: export → run a checker on the ROS
  machine → read a report → go back to Blender and fix. Slow, and the fix
  loop is where the user will spend most of their time.
- ➖ No robot mirroring, no live progress (the user asked for mirroring, QB3).

### Option C — Slicer **+ shared kinematics module** ✅ recommended
Same as B, **plus** the one insight that removes B's only real drawback:

> **Reachability validation does not need the robot, ROS, or a network — it
> needs the kinematic model.** The ROS2 plan already specifies
> `so_arm_100_kinematics` as **pure Python with zero ROS imports**
> (`ROS2_IMPLEMENTATION_PLAN.md` §9.3, Phase 1). That module can be **vendored
> directly into the addon** and run inside Blender's own interpreter.

So:

| Capability | How it works under Option C |
|---|---|
| Reachability validation | Runs **inside Blender**, instantly, offline, on Windows. Red/green per stick as you model. |
| Reachability volume overlay | Same module, no connection. |
| Robot mirroring (QB3) | Blender runs **FK** from the shared module and poses a rig. Better than mirroring live hardware: it can preview the *whole build* before anything moves, like a printer's preview. |
| Build execution | Export a build file; ROS2 executes it with the human gates in a terminal. |
| Ground truth | ROS2 **re-validates** the file on load with the same module — same code, so no drift, but MoveIt still gets the final say on collisions. |

Option C gets ~95 % of Option A's user experience with ~20 % of its
complexity, and keeps the reproducible artifact. The only thing genuinely lost
is live progress in the Blender viewport during a build — and a terminal on
the robot machine covers that.

**A live link can still be added later** as a strictly *read-only* telemetry
stream (joint states → Blender). Read-only is far simpler than a command
protocol: no state machine, no request/response, no versioning. If it is ever
wanted, it is a small additive feature, not a rewrite.

**Chosen: Option C.** A read-only telemetry link (joint states → Blender) may
be added later; it is additive and needs none of Option A's command
machinery.

---

## 3. Hard constraints (read before writing any code)

| # | Constraint | Consequence |
|---|---|---|
| B1 | **No `rclpy` inside Blender.** Blender bundles its own CPython; ROS2 Humble's extensions are built for CPython 3.10. ABI incompatibility, not a `PYTHONPATH` problem. | Under A/C any robot link is a socket. Under B/C the shared kinematics module works because it is **pure Python with no ROS imports** — that is the whole point of that design rule. |
| B2 | **`bpy` is not thread-safe.** Calling `bpy.*` from a background thread corrupts state or crashes Blender, often not immediately. | *(Option A/live telemetry only)* Network I/O on a worker thread touching **only** a `queue.Queue`; all `bpy` mutation in a `bpy.app.timers` callback on the main thread. |
| B3 | **The UI must never block.** | Long computations (validation of a big mesh) must be chunked across timer ticks or run with a progress modal, not in one call. |
| B4 | **Stdlib only.** Do not require `pip install` into Blender's Python — users will not do it and it breaks on upgrades. | `math`, `json`, `queue`, `threading`, `socket`, `mathutils`, `bpy`, `gpu`. **Note: no `numpy` guarantee** — write the kinematics module in plain Python, or keep a plain-Python fallback path. |
| B5 | Blender quaternions are **(w, x, y, z)**; ROS uses **(x, y, z, w)**. | Convert in exactly one place. The base/tip stick format mostly sidesteps this — deliberately. |
| B6 | Blender's unit scale is configurable (`scene.unit_settings.scale_length`) and objects carry arbitrary parent/scale transforms. | Always read `object.matrix_world` and apply the scene scale. Never read `object.location` directly. |
| B7 | Must be usable with **no robot and no ROS present**, on Windows. | Option C satisfies this by construction. |
| B8 | **Blender 5.x API.** Verify the addon manifest format (`blender_manifest.toml` extensions vs. legacy `bl_info`) against the 5.2 docs before writing the scaffolding — this changed in the 4.2 extensions platform. | Affects packaging and distribution only. |

---

## 4. Addon architecture

```
so100_builder/
├── blender_manifest.toml   (or bl_info — see B8)
├── __init__.py             register()/unregister(), module wiring
├── prefs.py                AddonPreferences
├── properties.py           PropertyGroups on Scene and Object
├── kinematics/             *** VENDORED, unmodified, from so_arm_100_kinematics ***
│   ├── chain.py            FK + closed-form IK (ROS2 doc §9.3)
│   ├── envelope.py         reachability queries
│   └── VERSION             so a mismatch with the robot side is detectable
├── core/
│   ├── transform.py        Blender world <-> base_link metres (B5, B6)
│   ├── sticks.py           mesh edges -> StickSpec; joint allowance; cut list
│   ├── order.py            *** build-order solver (§6) ***
│   ├── validate.py         order-aware validation (§7)
│   └── state.py            build state; .blend + JSON persistence (QB4)
├── io/
│   ├── export_build.py     write the build file (protocol Part A)
│   └── import_status.py    read back per-stick status after a build
├── net/                    (Option A / live telemetry only)
│   ├── client.py           worker thread + queues + main-thread pump
│   └── protocol.py
├── ops/                    operators
├── ui/
│   ├── panels.py           N-panel "SO-100"
│   ├── lists.py            UIList of sticks with status icons
│   └── overlay.py          GPU overlay: envelope, order, status colours
└── tests/                  run via `blender --background --python tests/run.py`
```

**The `kinematics/` folder is a verbatim copy, never a fork.** Add a CI check
(or a `make sync-kinematics`) comparing it against the ROS2 package, and a
`VERSION` file so a mismatch is loud rather than silent.

---

## 5. From wireframe mesh to cut list

This is the step where "a mesh" becomes "wooden sticks", and it is where the
geometry gets real.

### 5.1 Extraction

Each **edge** of the designated mesh becomes one stick. For each edge:

1. Take both vertices' world positions via `matrix_world`.
2. Apply `scene.unit_settings.scale_length` → metres.
3. Transform into the `SO100_Base` empty's frame → this *is* `base_link`.
4. Decide which end is `base` (the end that seats down): the lower Z, unless
   the stick connects to an already-placed stick at the other end — see §6.
   Overridable per stick.

### 5.2 Joint allowance — sticks have volume, mesh vertices do not

⚠ **A wireframe mesh's edges meet at a mathematical point. 6.45 mm square
sticks cannot.** At any shared vertex the stick volumes interpenetrate, which
is physically impossible. Each stick must therefore stop **3.25 mm short** of
the ideal vertex, leaving a gap that a glue blob fills.

✅ **N4 decided: 3.25 mm per shared end.** An end that seats on the base plate
is **not** shared and gets no allowance.

That value is half the 6.45 mm section (6.45/2 = 3.225), i.e. each stick stops
at the *face* of the joint rather than at its centreline — geometrically exact
for a **90°** joint.

### 5.2.1 ⭐ Mesh expansion — preserve stick length, grow the design

**This is the defining behaviour of the addon.** There are two ways to
reconcile a mesh with physical sticks, and the user has chosen the second:

| | Approach | Consequence |
|---|---|---|
| ❌ | **Shorten the sticks** to fit the design. `cut = edge − 3.25×shared_ends` | The design is preserved exactly, but every stick needs a bespoke cut. |
| ✅ | **Grow the design** to fit the sticks. `required_edge = stick_length + 3.25×shared_ends` | Stick lengths stay whatever you drew and cut; the *structure* ends up slightly larger. |

So: **the design mesh's edge lengths are the physical stick lengths** — what
you actually cut. The addon then moves vertices apart so sticks of exactly
those lengths sit with a 3.25 mm gap at every joint.

**The inverted-U example, worked through.** Three sticks, all physically
110 mm:

| Stick | Shared ends | Required edge length |
|---|---|---|
| bottom-left upright | 1 (top only — base sits on the plate) | 110 + 3.25 = **113.25 mm** |
| bottom-right upright | 1 | **113.25 mm** |
| top horizontal | 2 (both ends) | 110 + 2×3.25 = **116.5 mm** |

*(The values 114.25 / 117.5 in the original request are off by 1 mm — the
formula gives 113.25 and 116.5. Worth confirming, since these are the exact
numbers the addon implements.)*

### 5.2.2 The constraint solve

Growing each edge by a *different* amount is **over-constrained in general**.
Two cases:

**Acyclic structures (trees) — exact.** Walk outward from the grounded
vertices and push each vertex along its parent edge's direction by the
required amount. Every edge lands on its target length exactly, and the
design's angles are preserved. No solver needed.

**Structures with closed loops — approximate.** In the inverted U, raising
both top vertices by 3.25 mm fixes the uprights but leaves the top edge at
110 mm; widening it to 116.5 mm then tilts the uprights by ~1.6° and pushes
them to 113.30 mm. Every edge cannot be satisfied simultaneously.

Solve it as **iterative constraint relaxation** (position-based dynamics
style): repeatedly, for each edge, move both endpoints symmetrically along the
edge to correct its length error; pin grounded vertices. A few dozen
iterations converge to sub-0.1 mm residuals for structures of this scale.

**Report the residual per edge.** Any edge that cannot reach its target within
tolerance is a design the sticks will not physically fit — the user must know
which one, not discover it at the glue gun.

**Optional simplification: uniform growth.** Add 3.25 mm at *every* end,
jointed or not — so every edge grows by exactly 6.5 mm and free ends simply
overhang the design vertex by 3.25 mm (harmless: it is an exposed stick end).
A uniform additive growth is far better conditioned than a per-edge variable
one, and removes most of the residual error on looped structures. Offer it as
a toggle.

### 5.2.3 ⚠ The built sculpture is larger than the mesh you drew

This follows unavoidably from fixed-length sticks plus physical joints: the
structure grows by ~3.25 mm **per joint along any path through it**. A
10-layer tower ends up ~65 mm taller than designed.

Three consequences the addon must handle:

1. **Validate the expanded mesh, not the design mesh.** A design that fits the
   240 × 160 × 200 mm build volume may not fit after expansion.
2. **Show both.** Display design vs. expanded overall dimensions so the growth
   is never a surprise.
3. **Non-destructive.** Generate a derived *build mesh* as a separate object
   and never modify the user's design mesh. They will iterate.

The addon must show, per stick,
`design edge (= stick length) → shared ends → required edge → residual`.

⚠ **Shallow angles need more.** For two sticks of width `w` meeting at angle
θ, the interpenetration runs about `w / (2·tan(θ/2))` along each axis:

| joint angle θ | required allowance |
|---:|---:|
| 90° | 3.2 mm ✅ matches the fixed value |
| 60° | 5.6 mm |
| 45° | 7.8 mm |
| 30° | 12.0 mm |

The user glues with a glue gun and has accepted gaps and blobs, so the
**fixed 3.25 mm is the default**. But the addon should compute the angle-based
value per vertex and **warn** (`tight_clearance`) when it exceeds the fixed
one — below ~45° the sticks will physically clash, which no amount of glue
fixes. Expose the allowance as a UI field so it can be raised globally.

This warning is now *more* important, not less: under the §5.2.1 model the
gaps are what the expansion is sized around, so an under-sized allowance at a
shallow joint means the solved mesh puts two sticks in the same place.

### 5.3 Two length modes (QB1/QB2 — both required)

Under the §5.2.1 model the design mesh's **edge lengths are the stick
lengths**, so both modes are pre-passes that run *before* the expansion solve:

| Mode | Behaviour |
|---|---|
| **Design-driven** (default) | Stick length = the edge length as drawn. Cut each stick to that. Maximum design freedom, every stick potentially unique. |
| **Fixed stock lengths** | The user defines the available lengths (e.g. 80 / 100 / 120 / 150 mm). Each edge snaps to the nearest **before** expansion, and the addon reports the per-edge error and can move the vertex to make the design exact. Far easier for testing — and it means the whole build uses a handful of pre-cut lengths instead of a bespoke cut list. |

⚠ Order matters: **snap first, then expand.** Snapping after the solve would
re-break every edge length the solver just satisfied.

### 5.4 Length limits

Enforce **80–150 mm** on the **stick length** — which, under §5.2.1, is the
*design* edge length, not the expanded one. The expanded edge is longer by the
joint gaps and is not what gets cut or gripped.

Two distinct limits (ROS2 doc §8.2):
- **`min_stick_length` = 80 mm** — the user's stock threshold. A UI field, so
  it can be lowered if short sticks turn out to be wanted.
- **Hard floor ≈ 66 mm** — grip height (51 mm) + jaw margin. Below this the
  jaws close at or above the stick's tip. Never allow the field below it.

⚠ **Read the grip height from the shared kinematics module's constants, never
hardcode it here** — Phase 0 confirms the real number with a ruler.

### 5.5 Cut list output

A stick can only be loaded into the feeder by a human who knows which one it
is. Export, and show in the UI, an ordered table:
`# | stick id | stick length (mm) | build order | status`.

Under the fixed-stock-length mode this collapses to a tally
(*"12 × 100 mm, 8 × 120 mm"*), which is what you actually want at a saw.
Provide a printable/CSV export either way.

---

## 6. Build-order generation (the "slicer" algorithm)

The user's instinct — sort by average Z, like a 3D printer's layers — is the
right *starting* point but is not sufficient on its own. Three real
constraints, in priority order:

**C1 — Support.** A stick can only be placed if its base end is either on the
base plate or at a vertex of an already-placed stick. Nothing floats. This is
a **dependency graph** constraint, not a height one — and it is the constraint
that a pure Z sort silently violates.

**C2 — Accessibility.** The robot must be able to reach the placement without
the arm or jaws hitting what is already built. This *is* mostly a height
constraint (hence bottom-up), but not only: a low stick sitting *behind* a
finished tall structure is blocked. Because the arm reaches outward
horizontally (ROS2 doc §9.4), the secondary rule is **build far-from-robot
first, near-to-robot last** within a layer.

**C3 — Jaw clearance.** The jaws sit only ~51 mm above the stick's base — i.e.
right at the glue joint. The cone around each target vertex must be clear.
Placing the sticks that share a vertex in a bad order can make the last one
impossible to insert.

### 6.1 Proposed algorithm — greedy topological build with backtracking

```
vertices = merge mesh vertices within tolerance
grounded = { v : v.z <= epsilon }            # on the base plate
placed   = {}                                 # simulated scene
order    = []

while sticks remain:
    candidates = [ s for s in remaining
                   if s has an end that is grounded or at a placed vertex ]
    if not candidates:
        report "floating component — no stick can be placed next"; stop

    for s in sorted(candidates, key=cost):
        orient s so its supported end is `base`
        if validate(s, placed):               # IK + jaw clearance + collision
            order.append(s); placed.add(s); break
    else:
        backtrack()                           # undo the last choice, try the next best

cost(s) = ( max_z(s),                         # primary: build upward
            -distance_from_robot(s),          # secondary: far side first
            -support_count(s) )               # tertiary: prefer better-anchored
```

Notes:
- The `validate()` call is what makes this a *slicer* rather than a sort: the
  order and the buildability are the same problem (ROS2 doc §10.1).
- Backtracking keeps it correct without an exponential search in practice —
  cap the backtrack depth and report honestly if the cap is hit.
- Run it in chunks across timer ticks for big meshes (B3).

### 6.2 Structural cases to detect and warn about

| Case | Why it matters | Suggested handling |
|---|---|---|
| **Cantilever** — a stick glued at one end only, far from vertical | The robot releases it and gravity acts before the glue sets | Warn; prefer orders that give a stick two anchors where possible; suggest holding time |
| **Loop closure** — the last stick of a triangle/quad | Must fit exactly between two already-glued vertices; absorbs all accumulated error | Flag as high-risk; suggest cutting ~1 mm short (ROS2 doc §10.2) |
| **Floating component** — a sub-graph with no grounded vertex | Unbuildable | Hard error, name the component |
| **High-valence vertex** — many sticks meeting at one point | Jaw clearance collapses | Warn with the computed clearance |
| **Out-of-plane tilt** | 5-DOF arm cannot lean a stick sideways (ROS2 doc §9.4) | Hard error per stick, with the offending angle |

---

## 7. Validation & feedback

`core/validate.py` replays the build order through the vendored kinematics:

```
for stick in order:
    reachable?        -> closed-form IK (ROS2 doc §9.3)
    orientation ok?   -> tool axis must lie in the arm plane (§9.4)
    jaw clearance?    -> swept jaw box vs. already-placed sticks
    within envelope?  -> cached reachability map
    -> verdict, then add to the simulated scene
```

Per-stick verdicts drive everything the user sees: list icons, viewport
colours, and a summary (`42 sticks · 38 buildable · 4 impossible`). Reasons
must be specific and actionable — `"out of reach: 470 mm, max 442 mm at this
height"` rather than `"unreachable"`.

⚠ **This is an upper bound, not a guarantee.** It does not model self-collision
of the whole arm, the mount platform, or MoveIt's own path planning. ROS2
re-validates on load and MoveIt has the final word.

---

## 8. Coordinate transform (`core/transform.py`)

```python
def blender_to_robot(vec_world, base_matrix_world, scale_length):
    """Blender world-space point -> metres in base_link."""
    local = base_matrix_world.inverted() @ vec_world
    return tuple(c * scale_length for c in local)
```

- Blender and ROS are both **right-handed, Z-up** — no axis flip needed. This
  is a common source of bugs when people assume Y-up; do not "fix" it.
- Apply `scale_length` **after** the inverse transform, to translation only.
- Quaternion conversion (w,x,y,z) ↔ (x,y,z,w) lives here and nowhere else.

---

## 9. Scene data model

### 9.1 Conventions
- An empty named **`SO100_Base`** (scene pointer property) marks `base_link`.
  Moving it repositions the whole design relative to the robot without
  re-authoring anything.
- The wireframe mesh object is designated by a scene pointer property.
- The **build volume** is drawn as a reference box:
  ✅ **240 × 160 × 200 mm centred at Y = −370 mm** — i.e. X ∈ [−120, +120],
  Y ∈ [−290, −450], Z ∈ [0, 200] mm in `base_link` (ROS2 doc §1.2 N2).
  98 % of it is reachable for vertical sticks; the overlay must show the
  actual reachable region inside it, not just the box.

### 9.2 Per-stick state
Stored per edge (an edge attribute layer, or a dict keyed by a stable edge id
— note edge indices are **not** stable across mesh edits, so hash the vertex
positions or maintain an explicit id layer):

| Field | Meaning |
|---|---|
| `id` | Stable identifier, never reused |
| `order` | Build order index |
| `status` | `pending` / `buildable` / `impossible` / `placed` / `failed` / `skipped` |
| `reason` | Specific explanation for `impossible` / `failed` |
| `stick_length_mm` | Physical length to cut — the design edge length (§5.2.1), after any stock snapping |
| `expanded_edge_mm` | The solved edge length after mesh expansion, and its residual |
| `flip` | Manual override of which end is `base` |

### 9.3 Persistence (QB4 — both, as requested)
- **In the `.blend`**: the authoritative live state; travels with the design.
- **A compact JSON sidecar**, rewritten on every status change: survives "don't
  save changes", is diffable, and is what ROS2 reads. Format in
  [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) Part A.
- On load, if the two disagree, show both and let the user choose — never
  silently pick one.

---

## 10. UI specification

3D View sidebar (`N` panel), category **SO-100**:

### 10.1 `Design`
`SO100_Base` picker · mesh picker · stock section (6.45 mm) · length mode
(§5.3) · joint allowance · **Extract Sticks** · summary counts.

### 10.2 `Plan`
**Compute Build Order** · **Validate** · summary
(`42 sticks · 38 buildable · 4 impossible`) · warnings list from §6.2 ·
**Export Build File** · **Export Cut List**.

### 10.3 `Build`
`UIList` of sticks in build order: index, id, stick length, status icon, reason
tooltip. The current stick is highlighted in list and viewport.

Under Option C the panel shows the **next stick to load** prominently (id and
stick length in mm — this is what the human needs at the feeder) and offers
**Mark placed / Mark failed**, syncing status from the JSON sidecar written by
the ROS2 side.

*(Option A only)* a single state-dependent action button — **Pick next** →
**Place** → **Glue it, then Release** — mirroring the robot's state machine so
a wrong command is impossible, plus a red **Abort**.

### 10.4 `Preview`
- **Robot mirror** (QB3): a rig posed by the vendored FK. Scrub through the
  build order and watch the arm move to each placement — a printer-style
  preview of the whole job, before anything moves.
- **Reachability volume** overlay, toggleable.
- **Status colours**: grey pending · green buildable · red impossible ·
  blue placed · orange failed.

Keep the GPU overlay in its own module with a hard on/off switch — draw
handlers are the most common cause of addon crashes across Blender versions.

---

## 11. Implementation phases

### Phase A — Scaffolding & geometry *(no robot, no ROS)*
- [ ] Addon skeleton for Blender 5.2 (verify B8 first), preferences, panels.
- [ ] `core/transform.py` + unit tests via `blender --background`.
- [ ] `core/sticks.py`: edge extraction, joint allowance, both length modes,
      cut list.
- [ ] Per-stick state with **stable ids** (§9.2) and `.blend` persistence.
- **Done when:** a wireframe cube produces 12 sticks with correct metre
  coordinates in `base_link` and correct stick lengths, verified by hand.

### Phase B — Kinematics & validation
- [ ] Vendor `so_arm_100_kinematics` (available after ROS2 Phase 1); add the
      version check.
- [ ] `core/validate.py`; per-stick verdicts with specific reasons.
- [ ] Viewport overlay: build volume, reachability, status colours.
- **Done when:** dragging a vertex outside the envelope turns that stick red
  with a specific reason, live, with no ROS running.

### Phase C — Build order
- [ ] `core/order.py` per §6, including the §6.2 warnings.
- [ ] Chunked execution so large meshes don't freeze the UI.
- **Done when:** a multi-layer test mesh produces an order that is
  support-valid and fully buildable, and a deliberately-floating component is
  correctly rejected.

### Phase D — Export & the build loop
- [ ] Build-file export + cut-list export.
- [ ] JSON status sidecar round-trip; resume a partially-built job.
- [ ] `Build` panel with the next-stick-to-load display.
- **Done when:** a design is exported, built on real hardware, and reopening
  the `.blend` shows the correct placed/pending state.

### Phase E — Preview & polish
- [ ] Robot mirror rig driven by the shared FK; scrub through the build order.
- [ ] *(If Option A/telemetry chosen)* the `net/` layer and live status.

---

## 12. Testing

| What | How |
|---|---|
| Transform & stick maths | `blender --background --python tests/run.py`, plain `unittest`. No UI, no robot. |
| Kinematics parity | Same test data run against the ROS2 copy of the module; results must match exactly. |
| Build order | Synthetic meshes with known-correct orders; a floating component; a loop closure; a high-valence vertex. |
| Full loop | Real robot, a 3–5 stick design, watched every step. |

Explicitly test: Blender closed and reopened mid-build; a stick marked
`placed` that was actually knocked over (needs a re-sync path); a mesh edited
*after* the order was computed (ids must survive, or the user must be warned
that the order is stale).

---

## 13. Open questions

N1–N4 are **decided** (ROS2 doc §1.2). **Nothing blocks addon work.**

N5 (is there a base plate with pre-drilled sockets?) and N6 (glue set time)
remain open — see [`ROS2_IMPLEMENTATION_PLAN.md`](ROS2_IMPLEMENTATION_PLAN.md)
§13. Neither affects the addon: N5 changes the first-layer strategy on the
robot side, N6 is a hardware concern.
