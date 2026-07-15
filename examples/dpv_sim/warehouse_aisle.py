#!/usr/bin/env python
"""
| File: warehouse_aisle.py
| Description: Builds the DPV warehouse aisle procedurally - two pallet racks facing
| each other across a drive aisle - from the measured site dimensions in aisle_spec.json.
|
| This replaces the canned NVIDIA warehouse USDs ("Warehouse with Shelves" et al) so the
| geometry the V1 actually flies through - aisle width, upright pitch, beam levels - is
| the real site's rather than a generic one. Only the ground plane and lighting still come
| from a stock environment; everything between the racks is built here.
|
| To change the aisle, edit aisle_spec.json (measured values) or the constants below
| (dimensions the site survey did not record).
"""

import json
import os

import numpy as np
from isaacsim.core.api.objects import FixedCuboid

DEFAULT_SPEC_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "aisle_spec.json")

# The one complete, pre-assembled rack row that ships with Isaac Sim. There is no
# standalone rack asset - the Simple_Warehouse props are loose parts (frames, beams,
# box piles) - but warehouse_multiple_shelves.usd contains three assembled rows, and
# /Root/Shelf_1 is a single-sided one: 1.39 m deep x 17.62 m long x 6.0 m tall, with
# mesh colliders on 482 of its prims (so the lidar and the airframe both see it).
STOCK_SHELF_PRIM = "/Root/Shelf_1"


def build_aisle_stock(env_usd: str, spec_path: str = DEFAULT_SPEC_PATH,
                      root: str = "/World/aisle", shelf_prim: str = STOCK_SHELF_PRIM,
                      rows_per_side: int = 2):
    """Build the aisle from Isaac's stock warehouse rack rows instead of procedural boxes.

    Places rows_per_side copies of the pre-assembled NVIDIA shelf row end-to-end along
    each side of the aisle, facing each other across the measured aisle width from
    aisle_spec.json. Rack construction (levels, beam pitch) is whatever the stock asset
    has - only the aisle geometry is the real site's. The aisle runs along Isaac +X
    (East), centred on the origin, with the racks at +-aisle_width/2 in Y.

    +X is not arbitrary. The DPV mission stack works in the bridge's FLU/ENU frame
    (warehouse_path_planner subscribes to /px4_ros2_bridge/odometry/fcu_odom_flu), whose
    axes are Isaac's ENU axes with the origin at the vehicle's spawn - NOT PX4's NED body
    frame; the bridge converts internally. So a mission waypoint's x is metres EAST, i.e.
    Isaac +X. The real mission files say as much ("aligned with the local +X aisle
    direction", and path_planner_params.yaml's aisle_yaw_map_deg: 0.0 with "Missions
    define aisles along mission +x"). An aisle built along +Y instead sends every mission
    sideways into a rack. Verified empirically: commanding x=6.5 moved the vehicle along
    Isaac +X.

    The stock rows' group pivots sit ~25 m away from their geometry (scene-editing
    leftovers), so placement here is measured-bbox-based, not pivot-based: reference the
    prim, measure where its geometry actually is, then translate the parent so the
    aisle-facing face lands exactly at +-aisle_width/2 and the row lands on its tile.

    Args:
        env_usd (str): Path to warehouse_multiple_shelves.usd (pass
            SIMULATION_ENVIRONMENTS["Warehouse with Shelves"]).
        spec_path (str): Measured aisle spec JSON (only aisle_width_m is used).
        root (str): Stage prefix to build under.
        shelf_prim (str): Prim path of the assembled rack row inside env_usd.
        rows_per_side (int): How many stock rows to chain end-to-end per side. Each row is
            ~17.62 m, so this multiplies the hall length. 2 (~35.2 m) is the practical
            maximum: it only fits because 12_px4_v1_vehicle.py yaws the whole warehouse
            shell 90 deg, putting the shell's long 38.82 m axis on X to receive the aisle
            (unrotated, X is only 24 m and a second row would go through the end walls).
            3 rows would be 52.9 m and does not fit either way.

    Returns:
        dict: Realised geometry (aisle length/width, rack height/depth, rows).
    """
    # Isaac-only imports, kept out of module scope so build_aisle stays stub-testable.
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    with open(spec_path) as f:
        spec = json.load(f)["aisle_measurement"]
    half_aisle = float(np.mean(spec["aisle_width_m"])) / 2.0

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    info = {}

    for side, tag in ((1.0, "left"), (-1.0, "right")):
        for i in range(rows_per_side):
            parent = UsdGeom.Xform.Define(stage, f"{root}/rack_{tag}_{i}")
            # The stock row runs along its own +Y, so yaw every copy 90 deg to lay it
            # along the aisle's +X; the far side gets another 180 deg so both rows
            # present their shelf face to the aisle. "left"/"right" are as seen by a
            # vehicle flying +X (East): left = +Y (north side), right = -Y.
            rot = parent.AddRotateZOp()
            rot.Set(90.0 if side > 0 else 270.0)

            shelf = stage.DefinePrim(f"{root}/rack_{tag}_{i}/shelf")
            shelf.GetReferences().AddReference(env_usd, shelf_prim)

            # Measure where the referenced geometry actually ended up (rotation included,
            # translation not set yet), then shift the parent onto its tile.
            rng = cache.ComputeWorldBound(parent.GetPrim()).ComputeAlignedRange()
            mn, mx = rng.GetMin(), rng.GetMax()
            row_len = mx[0] - mn[0]

            # Rows butt end-to-end and the whole run is centred on X=0, so row i's centre
            # is offset from the middle of the run.
            target_x = (i - (rows_per_side - 1) / 2.0) * row_len

            dx = target_x - (mn[0] + mx[0]) / 2.0
            dy = (half_aisle - mn[1]) if side > 0 else (-half_aisle - mx[1])
            dz = -mn[2]
            # The translate op must run before the rotate op already on the parent (USD
            # applies ops last-to-first), and dx/dy/dz were measured in world frame - so
            # prepend it to the op order.
            tr = parent.AddTranslateOp(opSuffix="place")
            tr.Set(Gf.Vec3d(dx, dy, dz))
            parent.GetPrim().GetAttribute("xformOpOrder").Set(
                ["xformOp:translate:place", "xformOp:rotateZ"])

            info[tag] = dict(depth=mx[1] - mn[1], row_len=row_len, height=mx[2] - mn[2])

    return {
        "aisle_length_m": info["left"]["row_len"] * rows_per_side,
        "aisle_width_m": half_aisle * 2.0,
        "rack_height_m": info["left"]["height"],
        "rack_depth_m": info["left"]["depth"],
        "rows_per_side": rows_per_side,
        "prims": 2 * rows_per_side,
    }

# --- Dimensions the site survey did not record -------------------------------------
# The bay arithmetic implies Euro pallets: a 2.68 m clear span split across the 3 bin
# positions leaves ~0.83 m per bin, which is an 0.8 m pallet width plus a finger gap.
# That puts the pallet's 1.2 m dimension front-to-back, which in turn sets rack depth.
RACK_DEPTH_M = 1.2
PALLET_WIDTH_M = 0.8
PALLET_DEPTH_M = 1.2
POST_DEPTH_M = 0.09  # posts are modelled square in plan: upright_width_m x this
BEAM_FACE_M = 0.05  # beam thickness front-to-back; each level gets a front/back pair

# Cosmetic, but the ZED renders these - keep uprights, beams and pallets separable.
_UPRIGHT_COLOR = np.array([0.90, 0.35, 0.05])
_BEAM_COLOR = np.array([0.05, 0.25, 0.60])
_PALLET_COLORS = {
    "wood": np.array([0.65, 0.50, 0.30]),
    "plastic": np.array([0.22, 0.26, 0.32]),
}


def build_aisle(n_bays: int = 8, with_pallets: bool = True, spec_path: str = DEFAULT_SPEC_PATH,
                root: str = "/World/aisle"):
    """Build two racks facing each other across the aisle, from the measured spec.

    The aisle runs along Isaac's +Y and is centred on the origin, so a vehicle at
    (0, y, z) for |y| < aisle_length_m / 2 is flying down the middle of it, with the
    "left" rack to the west (-X) and the "right" rack to the east (+X).

    +Y is deliberate, not arbitrary: Isaac's world is ENU but PX4's local frame is NED,
    so PX4's +X (North) is Isaac's +Y. A mission waypoint of x=10 flies 10 m North, i.e.
    10 m along Isaac +Y - down this aisle. It also lets the aisle use the warehouse's
    long axis (38.82 m) instead of its 24 m one.

    Args:
        n_bays (int): Number of bays per rack. Each bay is one upright pitch long, so the
            aisle is n_bays * (width_between_upright_m + upright_width_m) long.
        with_pallets (bool): Place pallets at the bin positions on every storage level.
            Adds ~n_bays * 54 prims; turn off to recover RTF if the racks alone suffice.
        spec_path (str): Path to the measured aisle spec JSON.
        root (str): Stage prefix to build the racks under.

    Returns:
        dict: Realised geometry (aisle length/width, rack height, prim count).
    """

    with open(spec_path) as f:
        spec = json.load(f)["aisle_measurement"]

    upright_w = spec["upright_width_m"]
    clear_span = spec["width_between_upright_m"]
    level_pitch = spec["height_between_horizontal_beam_m"]
    beam_count = spec["horizontal_beam_count"]
    first_level = spec["initial_height_m"]
    beam_low = spec["horizontal_beam_width"]["low_lvl_m"]
    beam_upper = spec["horizontal_beam_width"]["second_m"]
    bin_pattern = spec["bin_location_width_pattern_m"]
    pallet_heights = spec["pallet_height_m"]

    # An upright occupies upright_width_m and the next one starts a clear span later.
    bay_pitch = clear_span + upright_w

    # aisle_width_m is a measured min/max range across the aisle; build the midpoint.
    half_aisle = float(np.mean(spec["aisle_width_m"])) / 2.0

    # initial_height_m is the top of the lowest beam - the surface a pallet rests on -
    # and each level sits one pitch above the last.
    level_tops = [first_level + k * level_pitch for k in range(beam_count)]

    # Storage levels are the floor plus every beam level, which is what aisle_lvl counts.
    load_surfaces = [0.0] + level_tops
    if len(load_surfaces) != spec["aisle_lvl"]:
        raise ValueError(
            f"spec inconsistent: floor + {beam_count} beam levels = {len(load_surfaces)} "
            f"storage levels, but aisle_lvl says {spec['aisle_lvl']}"
        )

    # Uprights run one level module past the top beam, which is what upright_height_m
    # measures (it equals the beam pitch - it is the per-level module, not the total).
    total_height = first_level + beam_count * level_pitch

    count = [0]

    def box(name, center, dims, color):
        FixedCuboid(
            prim_path=f"{root}/{name}",
            name=name,
            position=np.array(center),
            scale=np.array(dims),
            size=1.0,
            color=color,
        )
        count[0] += 1

    # Centre the aisle on the origin so it sits in the middle of the building.
    y_start = -(n_bays * bay_pitch) / 2.0

    for side, tag in ((-1.0, "left"), (1.0, "right")):
        # Racks mirror about the aisle centreline (X=0); `side` flips the X sign. "front"
        # is the face on the aisle, "back" the far side of the rack.
        post_front_x = side * (half_aisle + POST_DEPTH_M / 2.0)
        post_back_x = side * (half_aisle + RACK_DEPTH_M - POST_DEPTH_M / 2.0)
        beam_front_x = side * (half_aisle + BEAM_FACE_M / 2.0)
        beam_back_x = side * (half_aisle + RACK_DEPTH_M - BEAM_FACE_M / 2.0)
        pallet_x = side * (half_aisle + RACK_DEPTH_M / 2.0)

        # Upright frames: one opening every bay, plus a closing frame at the far end.
        for i in range(n_bays + 1):
            y = y_start + i * bay_pitch + upright_w / 2.0
            for x, place in ((post_front_x, "f"), (post_back_x, "b")):
                box(
                    f"upright_{tag}_{i}_{place}",
                    (x, y, total_height / 2.0),
                    (POST_DEPTH_M, upright_w, total_height),
                    _UPRIGHT_COLOR,
                )

        # Beams: a front/back pair spanning each bay's clear span at every level. Only the
        # lowest level uses the thicker low_lvl_m section.
        for k, top in enumerate(level_tops):
            thickness = beam_low if k == 0 else beam_upper
            z = top - thickness / 2.0
            for i in range(n_bays):
                y = y_start + i * bay_pitch + upright_w + clear_span / 2.0
                for x, place in ((beam_front_x, "f"), (beam_back_x, "b")):
                    box(
                        f"beam_{tag}_L{k}_{i}_{place}",
                        (x, y, z),
                        (BEAM_FACE_M, clear_span, thickness),
                        _BEAM_COLOR,
                    )

        if with_pallets:
            for k, surface in enumerate(load_surfaces):
                for i in range(n_bays):
                    for j, offset in enumerate(bin_pattern):
                        # Alternate the two pallet types so levels are not uniform slabs
                        # to the ZED/lidar; the spec gives a height for each.
                        kind = "wood" if (i + j) % 2 == 0 else "plastic"
                        height = pallet_heights[kind]
                        y = y_start + i * bay_pitch + upright_w + offset
                        box(
                            f"pallet_{tag}_L{k}_{i}_{j}",
                            (pallet_x, y, surface + height / 2.0),
                            (PALLET_DEPTH_M, PALLET_WIDTH_M, height),
                            _PALLET_COLORS[kind],
                        )

    return {
        "aisle_length_m": n_bays * bay_pitch,
        "aisle_width_m": half_aisle * 2.0,
        "rack_height_m": total_height,
        "bay_pitch_m": bay_pitch,
        "storage_levels": len(load_surfaces),
        "prims": count[0],
    }
