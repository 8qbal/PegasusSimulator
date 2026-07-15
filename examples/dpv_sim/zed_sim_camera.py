"""
| File: examples/dpv_sim/zed_sim_camera.py
| Description: Isaac-side half of the real ZED VIO chain (Phase 4 of PLAN.md).
|   References the Stereolabs ZED X USD on the V1 nose and streams its stereo
|   pair + IMU to the ZED SDK via the zed-isaac-sim extension (sl.sensor.camera),
|   so the REAL zed_wrapper can run with sim_mode:=true and produce real VIO:
|
|     Isaac render (CameraLeft/CameraRight) + IsaacReadIMU
|       -> OgnZEDSimCameraNode (IPC/RTSP stream, port 30000)
|       -> ZED SDK 5.x (/usr/local/zed)
|       -> zed_wrapper camera_model:=zedx sim_mode:=true  (tmux window "zed")
|       -> /zed/zed_node/odom -> zed_odom_to_fcu.py -> PX4 EV
|
|   The virtual camera model is the ZED X (the only family the extension
|   streams): same 12 cm baseline as the real drone's ZED 2i, slightly wider
|   FOV. That difference is irrelevant for VIO; it only matters to the
|   (stretch-goal) vision corrector.
|
|   ONE-TIME SETUP: the extension's C++ streamer plugin must be built first:
|     cd extensions/zed-isaac-sim && ./build.sh
|   (downloads Stereolabs' libsl_zed streaming runtime + compiles the OGN
|   plugin into exts/sl.sensor.camera/bin/).
"""
import os

# Repo root = two levels up from examples/dpv_sim/
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
ZED_EXTS_DIR = os.path.join(_REPO_ROOT, "extensions", "zed-isaac-sim", "exts")
ZED_X_USD = os.path.join(ZED_EXTS_DIR, "sl.sensor.camera", "data", "usd", "ZED_X.usdc")
_PLUGIN_BIN = os.path.join(ZED_EXTS_DIR, "sl.sensor.camera", "bin", "libsl.sensor.camera.plugin.so")


def enable_zed_extension(simulation_app):
    """Register the zed-isaac-sim exts folder and enable sl.sensor.camera.

    Must be called after SimulationApp() exists and before attach_zed_streamer().
    Raises RuntimeError with the build instruction if the C++ plugin was never built.
    """
    if not os.path.isfile(_PLUGIN_BIN):
        raise RuntimeError(
            "zed-isaac-sim streamer plugin not built: missing " + _PLUGIN_BIN +
            "\nRun once:  cd extensions/zed-isaac-sim && ./build.sh"
            "\n(downloads Stereolabs' libsl_zed runtime + builds the OGN plugin)."
        )

    import omni.kit.app
    from isaacsim.core.utils.extensions import enable_extension

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(ZED_EXTS_DIR)
    if not enable_extension("sl.sensor.camera"):
        raise RuntimeError(
            "Failed to enable extension 'sl.sensor.camera' from " + ZED_EXTS_DIR)
    # Let kit finish loading the extension (registers the OGN node types).
    simulation_app.update()


def attach_zed_streamer(
    world,
    body_prim_path,
    position=(0.355, 0.0, 0.0),
    resolution="SVGA",
    fps=30,
    port=30000,
    transport="IPC",
    mass_kg=0.15,
):
    """Mount a virtual ZED X on a vehicle rigid body and start streaming it.

    Args:
        world: the isaacsim World (stage source).
        body_prim_path: rigid-body prim to mount on (e.g. /World/quadrotor/body).
        position: mount offset in the body frame, metres (V1 nose = ZED 2i spot).
        resolution: SVGA (960x600) | HD1080 | HD1200 - streamer tokens; SVGA
            keeps RTF high, and the real drone only grabs HD720-class anyway.
        fps: 15 | 30 | 60 | 120 (streamer-validated).
        port: ZED SDK stream port (zed_wrapper sim_port must match).
        transport: IPC (same-machine, cheap - our case) | NETWORK | BOTH.

    Returns the ZEDAnnotator (owns the render products + stream graph nodes).

    Frames: the assembled ZED_X.usd looks along its own +X with +Z up (checked
    against the USD: base_link is yawed +90 deg and the camera prims look along
    base_link -Y), so identity orientation here means "looking out the nose",
    matching the MonocularCamera it replaces.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    # Public API of the sl.sensor.camera extension - the same class the
    # "ZED Camera Helper" OGN node constructs on its first compute().
    from sl.sensor.camera.annotators import ZEDAnnotator

    stage = world.stage
    zed_path = body_prim_path + "/zed_x"

    prim = stage.DefinePrim(zed_path, "Xform")
    prim.GetReferences().AddReference(ZED_X_USD)
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*position))

    # The referenced /Root carries PhysicsRigidBodyAPI but no MassAPI - PhysX
    # would otherwise derive the mass from the collision meshes at default
    # density (1000 kg/m3). Pin the real-ish camera mass so the V1 thrust
    # margin is unaffected deterministically.
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(mass_kg))

    # Rigidly attach the ZED body to the vehicle body. A FixedJoint (rather
    # than plain xform parenting) is required because the ZED root is a rigid
    # body of its own - unjointed it would just fall - and the IsaacReadIMU
    # node the streamer wires to Imu_Sensor needs a real physics body to read.
    # PhysX disables collision between jointed bodies by default, so the ZED
    # collision meshes cannot fight the V1 frame.
    joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(zed_path + "/mount_joint"))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body_prim_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(zed_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in position]))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # Build the stream: render products on CameraLeft/CameraRight + IMU reader
    # + OgnZEDSimCameraNode, all inside Isaac's SDG graph. Positional signature
    # mirrors SlCameraStreamer.compute().
    annotator = ZEDAnnotator(
        [Sdf.Path(zed_path)],   # camera_prim (list; entries need .pathString)
        "ZED_X",                # camera_model
        int(port),              # streaming_port
        resolution,             # resolution token
        int(fps),               # fps
        8000,                   # bitrate (unused over IPC)
        4096,                   # chunk size (unused over IPC)
        transport,              # transport layer mode
    )
    return annotator
