# Nira CAD design notes

All CAD commands below must be run inside the verified resource wrapper described
in [AGENTS.md](AGENTS.md). Run test modules separately; do not run a full geometry
suite concurrently with another heavy workload.

## Vendor bracket registration

Nira's `vendor_brackets.py` measures mounting-hole centres and planar mounting
faces from the vendor STEP files. Both brackets expose `servo_mount` and
`plate_mount` joints. The side bracket aligns its vertical hole pair with the
servo's rear flange; the bottom bracket aligns its horizontal pair with the
servo's lower front row. Its outer plate joint seats the inclined bracket.
The camera adapter bores follow the side bracket's measured hole pattern.
Spline hole outlines in the bottom STEP are measured by enclosed area and
centroid. Feature selection checks the hole count and servo mounting pitch,
and rejects missing or incompatible patterns. Physical fit still needs verification.

## Nira shared layout

- `torso_layout.py` defines body layer elevations, spacer lengths, the cover's
  seating offset and the front deck boundary. Body, servo and standalone
  chassis assemblies use these same elevations.
- Chassis edges follow slot-row centres and widths. Canopy corners follow
  the body width, height and chamfer; the removable cover shares the deck boundary.
- `board_layout.py` lists every PCB placement and hole pattern for both the
  base plate and populated assembly.
- `foot_common.CHEEK_OUTER_SPAN` sizes both the foot armor and toe connection;
  connection notches follow cheek thickness.
- Body toe positions and hole radius are separate. Toe mounting faces sit
  against the underside of layer 0, independent of the bore radius.

The bracket mounting frames replace empirical placement, so the bottom
bracket and outgoing femur interface move slightly. The body toes also move
up 1 mm to contact the base plate. Material thicknesses, clearances, decorative
outlines and display poses remain explicit design parameters.

Run the shared-layout and mounting regression tests with:

```bash
.venv/bin/python -m unittest discover -s tests -p test_nira_layout.py -v
```

The headless tests check joint coincidence and shaft alignment, foot mounting
hole alignment, bracket width, drawing coordinates, solid validity and the
complete robot's default foot placement. They do not certify collision-free
motion or manufacturing tolerances.

## Nira forward lidar deck

Nira's T-mini sits on its interface housing at **(93, 0, 67) mm**
(+X is forward), with its scan plane at Z=93.3 mm. The interface module
rests on layer 2 inside a cream, open-bottom `lidar_interface_housing`.
Its four laser-cut panels have finger joints and six tabs that engage the
deck. The lid and side walls extend rearward to the slope cover, with an open
rear end and a matching notch through the slope for USB cables. The scanner
and wall-to-deck mounts retain their positions. Two wine-red cylindrical
spacers reuse the foot's 4 mm outer diameter and 2.5 mm bore, shortened to
14 mm, with 0 mm clearance to the housing side walls. Matching holes in the
lid and deck secure them. The lid reuses the chamfered-octagon plate
outline with a square tunnel mouth. Nine wall fingers enter enclosed roof
slots and finish flush with its upper face. The solid front panel has no
connector cutout; its corner fingers fit enclosed slots in the side panels.
The front and side panels meet the roof without upper corner cutbacks, and
the roof extends over the closed side-slot rims. The cover exports four flat-panel
DXFs under `generated/nira/dxf/dxf_1p5mm/lidar_interface_housing/`.
Fits are nominal, without kerf allowance; interface dimensions remain
photo-based estimates pending hardware measurement.

The layer-3 support frame ends at X=15 mm, where the layer-4 roof's triangular
tip begins; the cream layer-4 roof ends at X=80 mm.
The upper spacers are 50 mm long, placing the frame underside at Z=101.5 mm
and roof underside at Z=103 mm. The roof has 2.1 mm vertical clearance above
the scanner envelope. The frame is behind the scanner, so its physical
separation is checked using the solids. The front diagonal braces have two
bottom tabs and a compact solid profile. Rear walls, diagonal braces and four
rear upper spacers support the canopy.

The sloped speaker cover is recessed 3 mm rearward (-X) from the side-plate
shoulders. Its five fingers on each side engage enclosed through-slots,
leaving the sloping side edges continuous. The speakers and their vents
move with the cover; finger ends remain flush with the outer side faces.

The forward **270° sector (-135° to +135° from +X)** is clear in the default
complete-robot pose. The geometry test intersects a continuous 4 mm high
scan band with the robot, from 20 to approximately 600 mm radius. It also
checks interface seating, housing bores and connector access, canopy-tip
alignment, scanner clearance, single-solid components, body collisions,
and retained tab engagement. This does not verify clearance throughout
head/leg motion, structural strength, optical tolerances or manufacturing fit.

Layer 4 contains the original 1.5 mm vented roof and a separate 3 mm frame
on its underside. The frame fits the central opening of layer 3 and has a
5 mm rim. Running its exporter writes the two-part STEP assembly and the
separate `torso_layer_4_frame.dxf` in `generated/nira/dxf/dxf_3mm/`;
the roof DXF remains in `dxf_1p5mm/`.

Export the revised parts and assembly with:

```bash
.venv/bin/python -m robot_nira.torso_layer_2
.venv/bin/python -m robot_nira.torso_layer_3
.venv/bin/python -m robot_nira.torso_layer_4
.venv/bin/python -m robot_nira.chassis_back
.venv/bin/python -m robot_nira.chassis_front
.venv/bin/python -m robot_nira.chassis_side
.venv/bin/python -m robot_nira.chassis_diagonal_back
.venv/bin/python -m robot_nira.chassis_diagonal_front
.venv/bin/python -m robot_nira.chassis_slope_cover
.venv/bin/python -m robot_nira.lidar_interface_housing
.venv/bin/python -m robot_nira.assembly_torso
.venv/bin/python robot_nira.py
.venv/bin/python -m unittest discover -s tests -p test_nira_lidar.py -v
```

Outputs are under `generated/nira/`. Side exports distinguish `chassis_front`,
`chassis_back`, `chassis_side`, `chassis_diagonal_front`, and
`chassis_diagonal_back`; the front and rear parts are no longer interchangeable.
Each plate has a STEP model and a flat DXF drawing. The complete STEP includes
the lidar and interface housing. `lidar_body_preview.png` shows the body and
an exploded view of the sensor, cover, and interface module.

## Nira vented foot armor

The foot cheeks use a swept toe outline, a broad lower rail and four angled
vents matching the layer-4 canopy. A new cream `foot_outer` plate bridges the
cheeks on their outer (negative drawing-Y) edge. Its paired gills surround a
solid spine, with clipped ends keeping the servo and toe connection accessible.
The existing servo cutouts, mounting holes, spacers and toe interface are retained.

The 1.5 mm outer plate has three 8 mm fingers on each edge, seated in matching
through-slots in the cheeks. The fingers finish flush with the outside faces;
the clear span is 32 mm and overall height is 35 mm. Fits are nominal, without
kerf compensation; physical fit and retention need checking for the chosen
material and laser process.

```bash
.venv/bin/python -m robot_nira.foot_front
.venv/bin/python -m robot_nira.foot_back
.venv/bin/python -m robot_nira.foot_outer
.venv/bin/python -m robot_nira.assembly_foot
.venv/bin/python -m unittest discover -s tests -p test_nira_foot.py -v
```

STEP and flat DXF exports are under `generated/nira/step/` and
`generated/nira/dxf/`; `foot_preview.png` shows
the assembled armor and the three flat profiles. Geometry tests check valid
single-solid plates, six fully engaged tabs, flush ends and outer-plate
clearance from the foot hardware and tibia servo. They do not establish load
capacity or clearance throughout leg motion.


## Nira test scope

### Nira URDF descriptions

`urdf/nira.urdf.xacro` uses primitive envelopes and
`urdf/nira_mesh.urdf.xacro` uses the Nira STL assemblies. Both include
`urdf/nira_model.xacro`, so their joint trees and sensor frames are identical.
After building and sourcing the workspace, display either with:

```bash
ros2 launch nira_viz display.launch.py
ros2 launch nira_viz display_mesh.launch.py
```

The joint and mesh registrations come from the CAD horn interfaces, including
the measured vendor bracket placement. Zero joint angles reproduce
`assembly_complete.py`'s standing pose and its -60 degree CAD head pitch.
Positive URDF angles increase the corresponding CAD connection angle. By convention, `right_*` names refer to the -Y side (+X forward, +Z up). These CAD joint frames use the measured mounting interfaces.

The torso STL includes the mounted T-mini and boards; `lidar_link` is at the
scan centre and `imu_link` follows the BNO055 board placement. The separate
head STL includes the camera pod and pitch servo. Collision envelopes,
estimated masses and provisional motion limits support model development;
they are not calibrated dynamics or verified mechanical travel limits.
`use_sim:=true` adds the shared ros2_control interfaces and the Nira controller
configuration, a planar T-mini lidar and an ideal BNO055 IMU. Hardware calibration must still be verified. See `nira_gazebo/README.md` for
sensor topics, mounting conventions and simulation limitations.

The description package's colcon tests check both variants, mesh resources,
the connected joint tree and simulation configuration. The additional
`tests/test_nira_urdf.py` CAD test checks exported mesh bounds and all leg/head
registrations at zero and nonzero joint angles. Run it with the ROS environment
sourced and `xacro` and `PyYAML` available to the CAD Python environment:

```bash
.venv/bin/python -m unittest discover -s tests -p test_nira_urdf.py -v
```

### Mechanical tests

Run all Nira tests with:

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_nira_*.py' -v
```

Keep the four suites: foot interfaces and tab engagement (`test_nira_foot`),
camera support and cable access (`test_nira_head`), shared mounting geometry
(`test_nira_layout`), and housing/chassis fit plus optical clearance
(`test_nira_lidar`). They exercise built geometry and mechanical relationships.
The fixed foot mounting coordinates intentionally protect the existing hardware
interface. Decorative opening counts are not a mechanical contract.

The canopy tests require physical separation and a roof above the scanner;
the continuous scan-band test separately protects the optical path. They do
not impose the former 3 mm envelope gap or certify manufacturing clearance.
The head cage collision test excludes the vendor bracket/simplified servo pair,
whose case and flange overlap slightly; separate tests verify its bore alignment.

