# PR-B reference implementation — `single.py` → Aorta-only

Reviewable drafts for the one-shot migration (see `../../aorta-migration-plan.md` §4).
These are **not yet wired into** `robo_orchard_piper_ros2`; they are the code PR-B
will land, kept here so the working ROS2 package stays untouched until the real-hw
gate passes.

## Files

| File | Role | Status |
|---|---|---|
| `arm_bridge.py` | Transport-neutral Piper CAN layer. Same `piper_sdk` math as `ros_bridge.py`, returns **dataclasses** instead of ROS msgs (drops `rclpy`/`sensor_msgs`/`geometry_msgs`). | syntax-checked ✅ |
| `single_aorta.py` | Aorta-only rewrite of `single.py` (master+slave, one node). Pull-sub `joint_cmd` drained in 200 Hz timer; `ExecutionGroup(SERIALIZED)` for CAN safety; typed pub/sub/service. | syntax-checked ✅; fill/decode validated ✅ |
| `take_over_aorta.py` | Aorta-only rewrite of `take_over/node.py` (muxer). 3-mode state machine + replay `deque` unchanged; **raw byte pass-through** for the command stream (`publish_bytes`); callbacks serialized via `ExecutionGroup`. | syntax-checked ✅; fill paths validated ✅ |
| `keyboard_trigger_aorta.py` | Aorta-only rewrite of `take_over/trigger/keyboard.py`. Typed Trigger clients (`create_client_typed`); `sshkeyboard` input unchanged; blocking `call()` dispatched on worker threads. | syntax-checked ✅; request fill validated ✅ |
| `start_robo_orchard_slave.aorta.sh` | Reference S100 launcher (lands in **vita-robot**). Drops `--ros-args` remaps + AMENT/ROS env for Aorta env + topic args. | `bash -n` OK ✅ |
| `validate_fill.py` | Builds every `single_aorta.py` FlatBuffers message via the exact fill bodies and decodes them back — validates builder/accessor names, vectors, Vec3/Quat structs, roundtrip values, without `libaorta_core`. | passes ✅ |
| `validate_muxer_fills.py` | Same for the muxer's ControlMode / TakeOverEvent typed publishes. | passes ✅ |

## What's validated vs what still needs the runtime

**Validated locally (no Aorta runtime needed):**
- All `.fbs` compile with `flatc` (see `../schemas/`).
- Every fill/decode path in `single_aorta.py` roundtrips correct values through
  the generated FlatBuffers bindings (`validate_fill.py`). Measured frame sizes:
  JointState **416 B**, EePose 160 B, PiperStatus 136 B, TriggerResp 120 B
  (all include the 72 B `AortaHeader`; node name `takeover_muxer_node`).
- Syntax of both modules (`python -m py_compile`).

**Still requires the device / Aorta runtime (the §4.4 gate):**
- `libaorta_core` on the host (`AORTA_CORE_FFI_LIB`) — real pub/sub/service.
- The generated `*_schema_meta` Python modules from the aorta repo (below).
- `piper_sdk` + CAN + a physical arm.

## Generating the `*_schema_meta` modules

`single_aorta.py` imports `arm_joint_state_schema_meta`, `arm_ee_pose_schema_meta`,
`piper_status_schema_meta`, `arm_trigger_schema_meta`. In the aorta repo these come
from `aorta_fbs_library(..., cpp=True, cpp_wrapper=True)` → `*_schema_meta_py_lib`
targets (they bundle the flatc accessors/builders **and** `SCHEMA_BFBS` + hash).

For the **desktop (non-Bazel)** side, generate an equivalent from the staged `.fbs`:

```bash
flatc --python -I ../schemas ../schemas/topic/arm/arm_joint_state.fbs   # accessors/builders
flatc --binary --schema --bfbs-builtins -I ../schemas \
      ../schemas/topic/arm/arm_joint_state.fbs                           # -> .bfbs
# then a tiny shim: SCHEMA_BFBS = <bytes of .bfbs>; SCHEMA_HASH_HEX = sha256(...)
```

> ⚠️ Master (desktop) and slave (S100) MUST use the **same** generated bindings
> (same `.bfbs` → same schema hash) or Aorta schema validation will reject the
> cross-host messages. Canonical source = the aorta-repo release (Tier 1).

## PR-B wiring TODO (when the §4.4 gate is green to land)

1. Move `arm_bridge.py` + `single_aorta.py` into `robo_orchard_piper_ros2/`.
2. Add the `*_schema_meta` deps (Bazel `*_schema_meta_py_lib` on S100; vendored/pip on desktop).
3. `setup.py`: add `single_ctrl` console-entry → `single_aorta:main` (drop the rclpy one after the gate).
4. `start_robo_orchard_slave.sh`: replace `--ros-args -r ... :=/puppet/...` remaps with
   `--joint-state-topic aorta/puppet/joint_left` etc.; export `AORTA_GROUP`, `AORTA_CORE_FFI_LIB`.
5. Move `take_over_aorta.py` → `robo_orchard_teleop_ros2/take_over/`; `take_over` entry → `take_over_aorta:main`.
6. Move `keyboard_trigger_aorta.py` → `robo_orchard_teleop_ros2/take_over/trigger/`; `keyboard_trigger` entry → `keyboard_trigger_aorta:main`.
7. In vita-robot: swap `start_robo_orchard_slave.sh` for `start_robo_orchard_slave.aorta.sh`; add the aorta Python SDK + `*_schema_meta` to the S100 image.

All core master/slave + takeover code is now drafted (single / muxer / keyboard trigger + slave launcher). Remaining is the device-side wiring above + the §4.1 spike + §4.4 gate.

## Run the local validation

```bash
cd docs/aorta-migration
flatc --python -I schemas schemas/topic/arm/*.fbs schemas/service/arm/*.fbs schemas/system/aorta_base.fbs
cp reference-impl/validate_fill.py . && python3 validate_fill.py   # expects generated arm/ + aorta/ dirs
```
