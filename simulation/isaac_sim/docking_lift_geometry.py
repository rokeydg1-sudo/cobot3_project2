"""World-space geometry measurements for the IW Hub lift and Dolly."""

from __future__ import annotations

import math
from dataclasses import dataclass

from pxr import Gf, Usd, UsdGeom, UsdPhysics


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned world bounds."""

    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    @property
    def center(self):
        """Return the XYZ center of the bounds."""
        return tuple(
            (low + high) * 0.5
            for low, high in zip(self.minimum, self.maximum)
        )


def world_pose(prim):
    """Return world XYZ and yaw in radians for an Xformable prim."""
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    translation = matrix.ExtractTranslation()
    quaternion = matrix.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    x, y, z = map(float, imaginary)
    w = float(quaternion.GetReal())
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return tuple(map(float, translation)), yaw


def collision_shapes(root_prim):
    """Return boundable collision shapes below a root prim."""
    shapes = []
    for prim in Usd.PrimRange(root_prim):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not prim.IsA(UsdGeom.Boundable):
            continue
        shapes.append(prim)
    return shapes


def shape_world_bounds(prim):
    """Compute one collision shape's world AABB from its own local extent."""
    if prim.IsA(UsdGeom.Cube):
        half = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) * 0.5
        low = Gf.Vec3d(-half, -half, -half)
        high = Gf.Vec3d(half, half, half)
    else:
        boundable = UsdGeom.Boundable(prim)
        extent = boundable.GetExtentAttr().Get(Usd.TimeCode.Default())
        if extent is None:
            extent = boundable.ComputeExtentFromPlugins(
                Usd.TimeCode.Default()
            )
        if not extent or len(extent) != 2:
            raise RuntimeError(
                f'No local extent for collision shape {prim.GetPath()}'
            )
        low = extent[0]
        high = extent[1]
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    corners = [
        matrix.Transform(Gf.Vec3d(x, y, z))
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ]
    minimum = tuple(
        min(float(point[i]) for point in corners)
        for i in range(3)
    )
    maximum = tuple(
        max(float(point[i]) for point in corners)
        for i in range(3)
    )
    return Bounds(minimum, maximum)


def combined_bounds(prims):
    """Return the union AABB for collision shapes."""
    bounds = [shape_world_bounds(prim) for prim in prims]
    if not bounds:
        raise RuntimeError('No collision shapes supplied')
    minimum = tuple(
        min(bound.minimum[i] for bound in bounds)
        for i in range(3)
    )
    maximum = tuple(
        max(bound.maximum[i] for bound in bounds)
        for i in range(3)
    )
    return Bounds(minimum, maximum)


def lift_joint_bodies(stage, joint_path):
    """Resolve body0/body1 targets authored on a PhysicsJoint."""
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        raise RuntimeError(f'Missing lift joint: {joint_path}')
    joint = UsdPhysics.Joint(joint_prim)
    body0 = [str(path) for path in joint.GetBody0Rel().GetTargets()]
    body1 = [str(path) for path in joint.GetBody1Rel().GetTargets()]
    return body0, body1


def overlap_metrics(lift_bounds, dolly_bounds):
    """Calculate signed AABB overlap and vertical separation."""
    x_overlap = min(lift_bounds.maximum[0], dolly_bounds.maximum[0]) - max(
        lift_bounds.minimum[0], dolly_bounds.minimum[0]
    )
    y_overlap = min(lift_bounds.maximum[1], dolly_bounds.maximum[1]) - max(
        lift_bounds.minimum[1], dolly_bounds.minimum[1]
    )
    vertical_gap = dolly_bounds.minimum[2] - lift_bounds.maximum[2]
    return x_overlap, y_overlap, vertical_gap


def local_correction(amr_yaw, dx, dy):
    """Project a world XY correction into AMR forward/left coordinates."""
    forward = math.cos(amr_yaw) * dx + math.sin(amr_yaw) * dy
    lateral = -math.sin(amr_yaw) * dx + math.cos(amr_yaw) * dy
    return forward, lateral


def geometry_snapshot(
    stage,
    amr_path,
    base_path,
    joint_path,
    dolly_root_path,
    dolly_base_path,
):
    """Measure the exact lift-to-Dolly contact geometry in the live stage."""
    body0, body1 = lift_joint_bodies(stage, joint_path)
    moving_paths = body1 or body0
    if len(moving_paths) != 1:
        raise RuntimeError(f'Ambiguous lift body: {moving_paths}')

    lift_prim = stage.GetPrimAtPath(moving_paths[0])
    lift_shapes = [
        prim
        for prim in collision_shapes(lift_prim)
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
    ]
    if len(lift_shapes) != 1:
        raise RuntimeError(f'Ambiguous lift collisions: {lift_shapes}')

    dolly_base = stage.GetPrimAtPath(dolly_base_path)
    dolly_shapes = [
        prim
        for prim in collision_shapes(dolly_base)
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
    ]
    if len(dolly_shapes) != 1:
        raise RuntimeError(f'Ambiguous Dolly Base collisions: {dolly_shapes}')

    lift_bounds = combined_bounds(lift_shapes)
    dolly_bounds = combined_bounds(dolly_shapes)
    amr_pose, amr_yaw = world_pose(stage.GetPrimAtPath(amr_path))
    base_pose, base_yaw = world_pose(stage.GetPrimAtPath(base_path))
    lift_pose, lift_yaw = world_pose(lift_prim)
    dolly_root_pose, dolly_root_yaw = world_pose(
        stage.GetPrimAtPath(dolly_root_path)
    )
    dolly_pose, dolly_yaw = world_pose(dolly_base)
    dx = dolly_bounds.center[0] - lift_bounds.center[0]
    dy = dolly_bounds.center[1] - lift_bounds.center[1]
    horizontal_distance = math.hypot(dx, dy)
    relative_yaw = math.atan2(
        math.sin(dolly_yaw - lift_yaw),
        math.cos(dolly_yaw - lift_yaw),
    )
    x_overlap, y_overlap, vertical_gap = overlap_metrics(
        lift_bounds,
        dolly_bounds,
    )
    forward, lateral = local_correction(base_yaw, dx, dy)
    return {
        'joint_path': joint_path,
        'body0': body0,
        'body1': body1,
        'lift_body_path': str(lift_prim.GetPath()),
        'lift_collision_paths': [str(prim.GetPath()) for prim in lift_shapes],
        'dolly_collision_paths': [
            str(prim.GetPath()) for prim in dolly_shapes
        ],
        'amr_pose': amr_pose,
        'amr_yaw': amr_yaw,
        'base_pose': base_pose,
        'base_yaw': base_yaw,
        'lift_pose': lift_pose,
        'lift_yaw': lift_yaw,
        'dolly_root_pose': dolly_root_pose,
        'dolly_root_yaw': dolly_root_yaw,
        'dolly_pose': dolly_pose,
        'dolly_yaw': dolly_yaw,
        'lift_bounds': lift_bounds,
        'dolly_bounds': dolly_bounds,
        'dx': dx,
        'dy': dy,
        'horizontal_distance': horizontal_distance,
        'relative_yaw': relative_yaw,
        'x_overlap': x_overlap,
        'y_overlap': y_overlap,
        'xy_overlap': x_overlap > 0.0 and y_overlap > 0.0,
        'vertical_gap': vertical_gap,
        'required_forward': forward,
        'required_lateral': lateral,
    }


def format_snapshot(snapshot):
    """Format a live geometry snapshot for logs and review."""
    lines = []
    for key, value in snapshot.items():
        if isinstance(value, float):
            lines.append(f'{key}={value:.9f}')
        else:
            lines.append(f'{key}={value}')
    return '\n'.join(lines) + '\n'


def apply_lift_geometry_offset(stage, joint_path, offset_x, offset_y):
    """Shift lift visual/collision branches together in the session layer."""
    body0, body1 = lift_joint_bodies(stage, joint_path)
    moving_paths = body1 or body0
    if len(moving_paths) != 1:
        raise RuntimeError(f'Ambiguous lift body: {moving_paths}')
    moving_prim = stage.GetPrimAtPath(moving_paths[0])

    geometry_roots = {}
    for prim in Usd.PrimRange(moving_prim):
        if prim == moving_prim:
            continue
        is_geometry = (
            prim.HasAPI(UsdPhysics.CollisionAPI)
            or prim.IsA(UsdGeom.Mesh)
        )
        if not is_geometry:
            continue
        branch = prim
        while branch.GetParent() != moving_prim:
            branch = branch.GetParent()
        geometry_roots[str(branch.GetPath())] = branch

    if not geometry_roots:
        raise RuntimeError(f'No lift geometry below {moving_prim.GetPath()}')

    shifted_paths = []
    for path in sorted(geometry_roots):
        prim = geometry_roots[path]
        xformable = UsdGeom.Xformable(prim)
        translate_op = next(
            (
                op
                for op in xformable.GetOrderedXformOps()
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
            ),
            None,
        )
        if translate_op is None:
            translate_op = xformable.AddTranslateOp(
                opSuffix='runtimeContactOffset'
            )
            current = Gf.Vec3d(0.0, 0.0, 0.0)
        else:
            current = translate_op.Get()
        vector_type = type(current)
        shifted = vector_type(
            float(current[0]) + float(offset_x),
            float(current[1]) + float(offset_y),
            float(current[2]),
        )
        translate_op.Set(shifted)
        shifted_paths.append(path)
    return shifted_paths
