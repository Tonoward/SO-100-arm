"""The build file and its status sidecar -- BRIDGE_PROTOCOL.md Part A,
ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 5.

Blender writes the build file; this module reads it (never writes one) and
owns the status sidecar (reads it for resume, writes it after every stick).

**Ported from the Blender addon's own `so100_builder/io/build_file.py`, not
freshly reinterpreted from the prose spec.** That module already implements
this format in full, its test suite already exercises "resume by reading a
sidecar written as ROS2 would", and it is the addon's whole integration
surface with this side (Option C -- BRIDGE_PROTOCOL.md's own header).
Drifting from its exact conventions (sidecar filename derivation, which
fields get omitted, how an unknown status degrades) would silently break
round-tripping between the two sides. `status_path_for`/`status_document`/
`load_status_file`/`next_stick_to_load` below are behavior-for-behavior
ports; `load_build_file` adds the two checks the protocol assigns
specifically to ROS2 (`kinematics_version`, `frame`) on top of the same
format/version/order checks the addon's own loader makes.

Pure Python (`json` only, no `rclpy`) so the format is testable standalone,
same reasoning the addon's own module docstring gives.
"""
import datetime
import json
import os

import so_arm_100_kinematics as sak

BUILD_FORMAT = "so100_build"
BUILD_VERSION = 1
STATUS_FORMAT = "so100_build_status"
STATUS_VERSION = 1

FRAME = "base_link"

STATUS_PENDING = "pending"
STATUS_PLACED = "placed"
STATUS_FAILED = "failed"
STATUS_SKIPPED = "skipped"
SIDECAR_STATUSES = (STATUS_PENDING, STATUS_PLACED, STATUS_FAILED, STATUS_SKIPPED)

STATUS_SUFFIX = ".status.json"


class ProtocolError(Exception):
    """A file that does not conform to BRIDGE_PROTOCOL.md Part A."""


def _timestamp(when=None):
    """UTC, ISO 8601, `Z`-suffixed -- matching the protocol's examples."""
    when = when or datetime.datetime.now(datetime.timezone.utc)
    return when.astimezone(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def status_path_for(build_path):
    """`tower_v3.build.json` -> `tower_v3.build.status.json`.

    A.3 says the sidecar sits "next to the build file as
    `<name>.status.json`". A build file conventionally ends `.json`, so
    that suffix is replaced rather than appended -- otherwise the sidecar
    would be `tower_v3.build.json.status.json`. (Ported verbatim from the
    addon's own `status_path_for` -- same edge-case handling.)
    """
    root, extension = os.path.splitext(build_path)
    if extension.lower() == ".json":
        return root + STATUS_SUFFIX
    return build_path + STATUS_SUFFIX


def _dumps(document):
    """Pretty-printed, newline-terminated, insertion-ordered JSON -- matches
    the addon's own `dumps()` so a sidecar written by either side looks the
    same."""
    return json.dumps(document, indent=2, sort_keys=False) + "\n"


# --- reading a build file -----------------------------------------------


def load_build_file(text):
    """Parse and sanity-check a build file. Raises `ProtocolError`.

    Checks `format`/`version`/dense `order` -- the same invariants the
    addon's own loader enforces on its own output. Then two checks
    BRIDGE_PROTOCOL.md's A.2 table assigns specifically to this side:

    - `kinematics_version` must equal `so_arm_100_kinematics.__version__`
      EXACTLY. The protocol's own words: "ROS2 must compare it against its
      own and refuse to execute on mismatch" -- a hard error, not a
      compatible-version check, because the two sides sharing one
      kinematics module is the entire basis for trusting Blender's own
      validation of the file.
    - `frame` must be `"base_link"` -- everything on the wire is assumed to
      already be in robot coordinates (A.1 rule 2); a file claiming a
      different frame was never converted and must not be executed as-is.
    """
    try:
        document = json.loads(text)
    except ValueError as exc:
        raise ProtocolError(f"not valid JSON: {exc}")

    if document.get("format") != BUILD_FORMAT:
        raise ProtocolError(
            f"not a build file: format is {document.get('format')!r}, expected {BUILD_FORMAT!r}")
    if document.get("version") != BUILD_VERSION:
        raise ProtocolError(
            f"build file version {document.get('version')!r}, this node reads version {BUILD_VERSION}")

    kinematics_version = document.get("kinematics_version")
    if kinematics_version != sak.__version__:
        raise ProtocolError(
            f"kinematics_version mismatch: build file says {kinematics_version!r}, "
            f"this workspace's so_arm_100_kinematics is {sak.__version__!r} -- refusing to "
            "execute (the two sides sharing one kinematics module is the whole basis for "
            "trusting the file's own validation; re-export from Blender with a matching "
            "kinematics_version)")

    frame = document.get("frame")
    if frame != FRAME:
        raise ProtocolError(f"frame is {frame!r}, expected {FRAME!r} -- refusing to execute")

    sticks = document.get("sticks")
    if not isinstance(sticks, list):
        raise ProtocolError("'sticks' must be a list")
    for position, entry in enumerate(sticks):
        if entry.get("order") != position:
            raise ProtocolError(
                f"stick {entry.get('id')!r} has order {entry.get('order')!r} at array position "
                f"{position} -- A.2 requires `order` to be dense and match array position")
    return document


# --- the status sidecar ---------------------------------------------------


def status_document(build_file_name, statuses, current_index=0, updated=None):
    """Assemble a status sidecar, per A.3.

    `statuses` maps stick id -> `{"status", "at", "reason"}`. Entries that
    are `pending` are **omitted**: A.3 says "anything absent is pending",
    so writing them would just be noise in a file whose whole point is to
    be small and diffable. (Ported behavior from the addon's own
    `status_document`.)
    """
    entries = {}
    for stick_id, entry in sorted(statuses.items()):
        status = entry.get("status", STATUS_PENDING)
        if status == STATUS_PENDING:
            continue
        record = {"status": status, "at": entry.get("at") or _timestamp(updated)}
        if entry.get("reason"):
            record["reason"] = entry["reason"]
        entries[stick_id] = record

    return {
        "format": STATUS_FORMAT,
        "version": STATUS_VERSION,
        "build_file": build_file_name,
        "updated": _timestamp(updated),
        "current_index": int(current_index),
        "sticks": entries,
    }


def write_status_file(path, document):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(_dumps(document))


def load_status_file(text):
    """Parse a status sidecar. Returns `(current_index, {id: entry})`.

    Unknown statuses degrade to `pending` rather than raising: the
    protocol's additive-evolution rule means a newer Blender addon may
    write a state this node has never heard of, and refusing to open the
    file would be a worse failure than treating that stick as not-yet-done.
    (Ported behavior from the addon's own `load_status_file`.)
    """
    try:
        document = json.loads(text)
    except ValueError as exc:
        raise ProtocolError(f"not valid JSON: {exc}")

    if document.get("format") != STATUS_FORMAT:
        raise ProtocolError(
            f"not a status sidecar: format is {document.get('format')!r}, expected {STATUS_FORMAT!r}")

    entries = {}
    for stick_id, entry in (document.get("sticks") or {}).items():
        if not isinstance(entry, dict):
            continue
        status = entry.get("status", STATUS_PENDING)
        if status not in SIDECAR_STATUSES:
            status = STATUS_PENDING
        entries[stick_id] = {
            "status": status,
            "at": entry.get("at", ""),
            "reason": entry.get("reason", ""),
        }
    return document.get("current_index", 0), entries


# --- resuming --------------------------------------------------------------


def next_stick_to_load(ordered_ids, statuses):
    """A.5 / Sec 10.3: the id the operator should load next.

    The first stick in build order that is not already `placed`. `failed`
    and `skipped` sticks are **not** skipped over here -- the operator
    decides what to do about them, and silently stepping past a failure
    would hide exactly the thing they need to see. Returns `None` when the
    build is finished. (Ported verbatim from the addon's own
    `next_stick_to_load`.)
    """
    for stick_id in ordered_ids:
        status = (statuses.get(stick_id) or {}).get("status", STATUS_PENDING)
        if status != STATUS_PLACED:
            return stick_id
    return None
