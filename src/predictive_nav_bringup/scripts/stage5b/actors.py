#!/usr/bin/env python3
"""
Stage-5B actor catalogue: footprints AND their ground-truth semantic class.

SINGLE SOURCE OF TRUTH. The SDF models the simulator spawns and the
ground-truth label the evaluation scores against are both generated from the
ACTORS table below, so a geometry change cannot silently disagree with its
label -- the same reason make_stage4f_world.py emits the world and its
occupancy map together.

GROUND TRUTH COMES FROM MODEL IDENTITY, NEVER FROM THE CLASSIFIER. Each entry
states the class the object IS, by construction; `gt_class_of(model_name)`
resolves a spawned model name to that class and is the only labelling path the
evaluation uses.

WHY THESE FOOTPRINTS
--------------------
The TurtleBot3 LDS sweeps ONE horizontal plane ~0.16 m above the floor, so the
only thing an actor can present to the tracker is its cross-section at that
height. Every actor below is therefore built as prisms of CONSTANT
cross-section spanning z in [0, 0.45]: the slice the sensor measures is exactly
the footprint written here, with no dependence on where in that band the plane
happens to fall. The footprints are the real scan-plane signatures of the
objects they stand for, not scale models of their silhouettes:

  pedestrian  two 0.12 m calves, offset BOTH along the walking direction
              (0.28 m, a mid-stride stride length) and across it (0.16 m, hip
              separation) -- what the plane cuts on a person is shins, not a
              torso. The lateral offset is anatomy, not convenience, and it
              matters: with the legs on a single radial line the near shin
              fully occludes the far one, leaving one 0.12 m target that
              subtends under 3 beams beyond ~2.5 m and so falls below the
              frozen cluster_min_points. Real hips are not collinear with the
              beam, and modelling them as if they were would manufacture a
              detection failure the sensor does not have.
  bike        the two WHEEL CHORDS. A 0.67 m wheel cut 0.175 m below its hub
              leaves 2*sqrt(0.335^2 - 0.175^2) = 0.57 m of tyre along the
              travel direction, 0.04 m thick, and the 1.05 m wheelbase puts a
              0.48 m hole between the two where the frame passes overhead.
  vehicle     a 3.8 x 1.7 m car body, and a 1.0 x 0.5 m compact body. BOTH are
              ground-truth VEHICLE: the compact one exists because the frozen
              tracker rejects any cluster whose bounding-box diagonal exceeds
              cluster_max_diameter (1.2 m), so it is the only vehicle-class
              object that the UNMODIFIED v1 clustering can represent at all.
  planter     a 0.90 m diameter round planter: a real obstacle that is NOT one
              of the three target classes, included so the UNKNOWN rate is
              measured against something rather than asserted.
"""

#: Scan-plane prism band, m. Chosen to straddle the ~0.16 m LDS plane with
#: margin for the 0.01 m range noise and any spawn-height settling.
PRISM_Z0 = 0.0
PRISM_Z1 = 0.45
PRISM_H = PRISM_Z1 - PRISM_Z0
PRISM_ZC = 0.5 * (PRISM_Z0 + PRISM_Z1)

#: Ground-truth semantic classes. These strings are the evaluation's labels and
#: match TrackedObject.CLASS_* by name.
PEDESTRIAN = 'PEDESTRIAN'
BIKE = 'BIKE'
VEHICLE = 'VEHICLE'
UNKNOWN = 'UNKNOWN'

#: model name -> (ground-truth class, [parts]) where each part is
#: ('box', dx, dy, xoff, yoff) or ('cyl', radius, xoff, yoff), in metres, in
#: the model's own frame with +x along its direction of travel.
ACTORS = {
    'actor_pedestrian': (PEDESTRIAN, [
        ('box', 0.12, 0.12, -0.14, -0.08),
        ('box', 0.12, 0.12, +0.14, +0.08),
    ]),
    'actor_bike': (BIKE, [
        ('box', 0.57, 0.04, -0.525, 0.0),
        ('box', 0.57, 0.04, +0.525, 0.0),
    ]),
    'actor_vehicle': (VEHICLE, [
        ('box', 3.80, 1.70, 0.0, 0.0),
    ]),
    'actor_vehicle_compact': (VEHICLE, [
        ('box', 1.00, 0.50, 0.0, 0.0),
    ]),
    'actor_planter': (UNKNOWN, [
        ('cyl', 0.45, 0.0, 0.0),
    ]),
}

#: Bounding-box diagonal of each actor's footprint, m. Compared against the
#: tracker's cluster_max_diameter to predict which actors the frozen
#: clustering envelope can represent at all.


def footprint_diagonal(name):
    """Return the bounding-box diagonal of `name`'s scan-plane footprint, m."""
    _, parts = ACTORS[name]
    xs, ys = [], []
    for part in parts:
        if part[0] == 'box':
            _, dx, dy, ox, oy = part
            xs += [ox - dx / 2.0, ox + dx / 2.0]
            ys += [oy - dy / 2.0, oy + dy / 2.0]
        else:
            _, r, ox, oy = part
            xs += [ox - r, ox + r]
            ys += [oy - r, oy + r]
    return ((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2) ** 0.5


def gt_class_of(model_name):
    """
    Ground-truth semantic class of a spawned model, from its identity alone.

    Raises KeyError for an unknown model rather than guessing, so a typo in a
    scenario can never silently become a mislabelled sample.
    """
    return ACTORS[model_name][0]


_PART_COLLISION = """      <collision name="{cname}">
        <pose>{ox:.4f} {oy:.4f} {zc:.4f} 0 0 0</pose>
        <geometry>
{geom}
        </geometry>
      </collision>
      <visual name="{vname}">
        <pose>{ox:.4f} {oy:.4f} {zc:.4f} 0 0 0</pose>
        <geometry>
{geom}
        </geometry>
        <material>
          <ambient>{rgb} 1</ambient>
          <diffuse>{rgb} 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
"""

_COLOURS = {
    PEDESTRIAN: '0.15 0.85 0.25',
    BIKE: '0.20 0.45 1.00',
    VEHICLE: '1.00 0.65 0.10',
    UNKNOWN: '0.65 0.65 0.65',
}

_MODEL = """<?xml version="1.0" ?>
<!-- GENERATED by scripts/stage5b/actors.py -- do not edit by hand.
     Ground-truth class: {gt}
     Footprint bounding-box diagonal: {diag:.3f} m
     Constant cross-section over z in [{z0:.2f}, {z1:.2f}] m, so the slice the
     ~0.16 m LiDAR plane measures is exactly this footprint. -->
<sdf version="1.9">
  <model name="{name}">
    <pose>0 0 0 0 0 0</pose>
    <static>false</static>
    <self_collide>false</self_collide>

    <link name="link">
      <inertial>
        <pose>0 0 {zc:.4f} 0 0 0</pose>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.15</ixx><iyy>0.15</iyy><izz>0.10</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
{parts}    </link>

    <!-- Exactly the plugin pair the validated Stage-4B dynamic_obstacle uses:
         a commanded planar velocity (known ground-truth motion) and a
         ground-truth odometry stream used ONLY for evaluation, never by the
         tracker. -->
    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl">
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>{name}</robot_base_frame>
      <odom_publish_frequency>20</odom_publish_frequency>
      <dimensions>2</dimensions>
    </plugin>
  </model>
</sdf>
"""


def model_sdf(name):
    """Render the SDF for one actor."""
    gt, parts = ACTORS[name]
    rendered = []
    for i, part in enumerate(parts):
        if part[0] == 'box':
            _, dx, dy, ox, oy = part
            geom = ('          <box><size>'
                    f'{dx:.4f} {dy:.4f} {PRISM_H:.4f}'
                    '</size></box>')
        else:
            _, r, ox, oy = part
            geom = ('          <cylinder>'
                    f'<radius>{r:.4f}</radius><length>{PRISM_H:.4f}</length>'
                    '</cylinder>')
        rendered.append(_PART_COLLISION.format(
            cname=f'collision_{i}', vname=f'visual_{i}',
            ox=ox, oy=oy, zc=PRISM_ZC, geom=geom, rgb=_COLOURS[gt]))
    return _MODEL.format(
        name=name, gt=gt, diag=footprint_diagonal(name),
        z0=PRISM_Z0, z1=PRISM_Z1, zc=PRISM_ZC, parts=''.join(rendered))


def main():
    """Write every actor model into models/stage5b/."""
    import argparse
    import os
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='output directory')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for name in sorted(ACTORS):
        path = os.path.join(args.out, f'{name}.sdf')
        with open(path, 'w') as handle:
            handle.write(model_sdf(name))
        print(f'{path}  gt={gt_class_of(name):10s} '
              f'diag={footprint_diagonal(name):.3f} m')


if __name__ == '__main__':
    main()
