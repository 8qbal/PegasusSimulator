# Isaac Sim 6.0 Migration — Debug & Fix Log

This document records the issues found and fixed while porting Pegasus Simulator from
Isaac Sim 5.1 to **Isaac Sim 6.0**, specifically for the "PX4 connects but the drone
cannot take off" problem.

Date: 2026-07-03

**Environment:**
- Isaac Sim **6.0.0** (build `6.0.0-rc.59+release.41464.5f2772bc.gl`), bundled Python 3.12
- PX4-Autopilot **v1.16.2** (`~/PX4-Autopilot`, built with `make px4_sitl_default none`)
- Ubuntu 22.04 LTS

---

## Symptom

- Isaac Sim GUI / standalone examples start, PX4 SITL auto-launches and connects on
  TCP port 4560 (`Simulator connected on TCP port 4560`).
- PX4 boots fully and reports `Ready for takeoff!`.
- Arming works, `MAV_CMD_NAV_TAKEOFF` is **accepted**, PX4 enters AUTO.TAKEOFF and
  commands the motors to near-maximum PWM…
- …but the drone never moves. Altitude stays at exactly 0 m.
- The Kit log is flooded with:
  `[pegasus.simulator.logic.backends.px4_mavlink_backend] Could not send groundtruth through mavlink`

---

## Root cause 1 — Forces never reached the physics engine (the takeoff blocker)

**File:** `extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/vehicle.py`

The 6.0 port had replaced the old Dynamic Control force calls with writes to a
`physxForce:force` USD attribute:

```python
force_attr = prim.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Vector3f)
force_attr.Set(Gf.Vec3f(*world_force))
```

PhysX **ignores** this attribute unless the `PhysxForceAPI` schema is actually applied
to the prim — `CreateAttribute` alone just creates an inert, free-standing attribute.
As a result rotor thrust, body drag, and the rolling-moment torque were all silently
discarded. PX4 was healthy the entire time; the simulated vehicle simply had no motors
as far as PhysX was concerned.

**Fix:** `apply_force()` / `apply_torque()` now use the Isaac Sim 6.0
`isaacsim.core.experimental.prims.RigidPrim` PhysX **tensor API**:

- `apply_force()` → `RigidPrim.apply_forces(force, local_frame=True)`, or
  `RigidPrim.apply_forces_and_torques_at_pos(forces=..., positions=..., local_frame=True)`
  when a position offset is given.
- `apply_torque()` → `RigidPrim.apply_forces_and_torques_at_pos(torques=..., local_frame=True)`.
- `local_frame=True` preserves the original upstream body-frame semantics (the old
  `dc_interface.apply_body_force(..., False)` behaviour). Thrust `[0, 0, f]` is applied
  along each rotor's local z-axis, drag and rolling moment in the body FLU frame.
- One `RigidPrim` wrapper per body part (`/body`, `/rotor0` … `/rotor3`) is created
  lazily and cached in `self._rigid_prims`; the cache is cleared when the simulation
  stops. Calls are guarded by `is_physics_tensor_entity_valid()` so nothing fires
  before the physics views exist.

## Root cause 2 — MAVLink groundtruth message crashed on every frame

**File:** `extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py`

`HIL_STATE_QUATERNION.ind_airspeed` / `true_airspeed` are **uint16** fields. While the
drone rests on the ground, physics jitter makes the body x-velocity slightly negative,
so `int(body_vel[0] * 100)` produced a negative number and pymavlink raised
`struct.error: 'H' format requires 0 <= number <= 65535` on **every physics step** —
that was the "Could not send groundtruth through mavlink" warning wall. The bare
`except:` swallowed the actual reason.

**Fixes in `update_state()`:**

| Field | MAVLink type | Fix |
|---|---|---|
| `ind_airspeed`, `true_airspeed` | uint16 (cm/s) | clamped to `[0, 65535]` |
| `vx`, `vy`, `vz` | int16 (cm/s) | clamped to `[-32767, 32767]` |
| `xacc`, `yacc`, `zacc` | int16 (**mG**) | converted from m/s² to mG (`/ 9.80665 * 1000`) and clamped — previous code sent mm/s², the wrong unit |

**Fixes in `update_gps_data()`** (same overflow class for `HIL_GPS`):

| Field | MAVLink type | Fix |
|---|---|---|
| `vel` (speed) | uint16 (cm/s) | clamped to `[0, 65535]` |
| `vn`, `ve`, `vd` | int16 (cm/s) | clamped to `[-32767, 32767]` |

**Error visibility:** all bare `except:` blocks in the send path
(`send_ground_truth`, `send_gps_msgs`, `send_vision_msgs`) now log the actual
exception message instead of swallowing it.

## Root cause 3 — Pegasus extension failed to load in the Isaac Sim GUI

**File:** `extensions/pegasus.simulator/config/extension.toml`

In Isaac Sim 6.0, `isaacsim.core.api` (which provides `World`, `Robot`,
`isaacsim.core.utils.*`, `isaacsim.core.prims`) was moved to `extsDeprecated/` and is
**no longer auto-loaded** in GUI mode. Pegasus imports it but never declared it as a
dependency, so Kit never loaded it and the extension failed on import with no visible
panel.

**Fix:** added to `[dependencies]`:

```toml
"isaacsim.core.api" = {}
```

This pulls in the deprecated core stack transitively before Pegasus starts.

---

## Verification performed

1. `~/isaacsim/python.sh examples/1_px4_single_vehicle.py` — PX4 auto-launches,
   connects on TCP 4560, lockstep runs, EKF initializes, home position set, PX4 prints
   `Ready for takeoff!`. No poll timeouts, no groundtruth send warnings.
2. A GCS test script (pymavlink on UDP 14550) confirmed: heartbeat OK, arming
   **accepted**, `MAV_CMD_NAV_TAKEOFF` **accepted**, PX4 switches to AUTO.TAKEOFF and
   drives the motors (SERVO_OUTPUT_RAW ≈ 1250–2000 µs).
   - Before the `RigidPrim` fix: altitude stayed at 0 m (forces discarded).
   - The force-application fix makes the commanded thrust actually act on the vehicle.
3. QGroundControl connects automatically on UDP 14550 (PX4's GCS link broadcasts to
   localhost:14550; no extra configuration needed).

## Bonus fix — `isaacsim.robot_motion.pink` startup error (unrelated to Pegasus)

The red traceback `AttributeError: module 'pinocchio' has no attribute 'Model'` at GUI
startup comes from NVIDIA's stock Pink IK extension (manipulator arms — nothing to do
with drones). Root cause is an NVIDIA packaging bug **plus** leftover manual installs:

- The extension's `pip_prebundle/.../site-packages/pinocchio/` contained **only a
  `.pyi` type stub** (no `__init__.py`, no compiled module) → Python treated it as an
  empty namespace package with no attributes.
- Kit's site-packages had manually-installed `pin 2.7.0` + `pin-pink 3.1.0` (old,
  mutually incompatible with the bundled expectations of `pink 4.2.0` / `pin 4.0.0`).

Fix applied to `~/isaacsim/kit/python` (not this repo):

1. `pip uninstall pin pin-pink`, then install the versions the extension actually
   expects: `pin-pink==4.2.0`, `pin==4.0.0`, `libpinocchio==4.0.0`, `coal`/`libcoal`
   `3.0.3`, and the `cmeel-*` runtime library wheels (installed with `--no-deps` where
   needed — plain installs pull in **numpy 2.5**, which breaks IsaacLab/Isaac Sim;
   numpy was restored to `1.26.4` afterwards each time).
2. Renamed the stub-only bundle dir to
   `pip_prebundle/.../site-packages/pinocchio_stubs_disabled` so it can never shadow
   the real package as a namespace package.
3. Verified: `import pinocchio` has `Model`, `from pink.limits import
   AccelerationLimit` (the exact crash line) imports, numpy 1.26.4 and torch 2.7.0
   unaffected.

## Related earlier fixes (previous sessions, kept)

- **`tcpin` is the correct connection type** for the PX4 HIL link: PX4 v1.16 runs
  `simulator_mavlink start -c 4560` (TCP *client*), so the simulator must be the TCP
  *server* (`tcpin:localhost:4560`). `PX4_SIM_PROTOCOL` is unused in v1.16+ — UDP
  variants silently drop all HIL data.
- GPS groundtruth lat/lon converted from radians to degrees before the 1e7 integer
  encoding (`update_gps_data`).
- Physics callbacks registered only after backends start (`sim_start_stop`), so PX4 is
  listening before the first HIL_SENSOR is pushed.
- All example scripts use `from isaacsim.core.api import World`
  (`isaacsim.core.world` does not exist in 6.0).
- Isaac Sim's bundled broken PyTorch 2.11 (`omni.isaac.ml_archive/pip_prebundle/torch`,
  `undefined symbol: ncclCommShrink`) renamed to `torch_bundled_disabled` so the working
  system torch 2.7 is used — without this, Isaac Sim shuts itself down ~8 s after start.
