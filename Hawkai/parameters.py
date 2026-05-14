## Hawkai — All numeric constants in one place
## Change values here; vehicle.py / mission.py import them.

from SUAVE.Core import Units

# ── Vehicle mass ──────────────────────────────────────────────────────────────
TAKEOFF_MASS         = 340.   * Units.lb   # NDARC DGW
OPERATING_EMPTY_MASS = 260.   * Units.lb   # NDARC WE
MAX_TAKEOFF_MASS     = 340.   * Units.lb   # fWMTO=1.0 → MTOW=DGW
MAX_PAYLOAD_MASS     = 80.    * Units.lb   # DGW − WE
CENTER_OF_GRAVITY    = [[0.933, 0., 0.]]   # NDARC SL=3.06 ft

# ── Main wing ─────────────────────────────────────────────────────────────────
WING_ORIGIN                  = [[0.933, 0., 0.244]]   # NDARC SL=3.06 ft, WL=0.80 ft
WING_INCIDENCE               = 5.0 * Units.degrees    # NDARC wing incidence
WING_SPAN                    = 23.   * Units.feet
WING_ROOT_CHORD              = 2.1   * Units.feet
WING_EXPOSED_ROOT_CHORD_OFFSET = 0.5

ROOT_SEG_DIHEDRAL   = 1.0  * Units.degrees
ROOT_SEG_SWEEP_QC   = 8.5  * Units.degrees
ROOT_SEG_T_C        = 0.12

SEC2_SEG_PCT_SPAN   = 4. / 11.
SEC2_SEG_DIHEDRAL   = 1.0  * Units.degrees
SEC2_SEG_SWEEP_QC   = 0.0  * Units.degrees
SEC2_SEG_T_C        = 0.12

TIP_SEG_DIHEDRAL    = 0.0  * Units.degrees
TIP_SEG_SWEEP_QC    = 0.0  * Units.degrees
TIP_SEG_T_C         = 0.12

# ── Horizontal tail ───────────────────────────────────────────────────────────
HTAIL_AREA         = 10.0 * Units.feet * Units.feet
HTAIL_TAPER        = 1.
HTAIL_SWEEP_QC     = 0.
HTAIL_ASPECT_RATIO = 5.0
HTAIL_T_C          = 0.09
HTAIL_DIHEDRAL     = 5.   * Units.degrees

# ── Vertical tail ─────────────────────────────────────────────────────────────
VTAIL_AREA         = 8.0  * Units.feet * Units.feet
VTAIL_TAPER        = 1.
VTAIL_SWEEP_QC     = 0.
VTAIL_ASPECT_RATIO = 5.0
VTAIL_T_C          = 0.09
VTAIL_DIHEDRAL     = 5.   * Units.degrees
VTAIL_ORIGIN_X        = 8.75 * Units.feet
HTAIL_STATION_LINE    = 8.75 * Units.feet   # NDARC SL for htail/vtail AC

# ── Fuselage ──────────────────────────────────────────────────────────────────
FUSELAGE_TOTAL_LENGTH  = 8.0                   # feet (scalar, used in segment fractions); NDARC
FUSELAGE_WIDTH         = 2.0   * Units.feet    # NDARC
FUSELAGE_HEIGHT        = 2.0   * Units.feet    # NDARC
FUSELAGE_WETTED_AREA   = 20.   * Units.feet**2
FUSELAGE_EFF_DIAMETER  = 2.0   * Units.feet    # matches width; NDARC
FUSELAGE_FRONT_AREA    = 0.14  * Units.feet**2

# ── Booms ─────────────────────────────────────────────────────────────────────
BOOM_ORIGIN_Y  = 3.0
BOOM_ORIGIN    = [[0.525, BOOM_ORIGIN_Y, -0.35]]
BOOM_LENGTH    = 5.
BOOM_WIDTH     = 0.15
BOOM_RADIUS    = BOOM_WIDTH / 2.

# ── Propulsion network ────────────────────────────────────────────────────────
N_LIFT_ROTORS  = 4
N_PROPELLERS   = 1
BUS_VOLTAGE    = 400.   # Volts

ESC_EFFICIENCY = 0.95

AVIONICS_POWER = 300. * Units.watts
PAYLOAD_POWER  = 0.                        # Watts
EXCRESCENCE_AREA = 0.1                     # m²

# ── Battery ───────────────────────────────────────────────────────────────────
BATTERY_MASS   = 1000. * Units.lb

# ── Tractor propeller ─────────────────────────────────────────────────────────
PROP_ORIGIN            = [[0., 0., -0.325]]
PROP_N_BLADES          = 2
PROP_TIP_RADIUS        = 0.5090             # m  (1.67 ft per NDARC)
PROP_HUB_RADIUS        = 0.0509             # m  (10% of tip radius)
PROP_RPM               = 572.  * Units.rpm  # Vtip=100 ft/s, R=1.67 ft → 572 rpm
PROP_FREESTREAM_VEL    = 100.  * Units.knots
PROP_DESIGN_CL         = 0.7
PROP_DESIGN_ALTITUDE   = 5000. * Units.feet
PROP_DESIGN_THRUST     = 68.   * Units.lbf  # scaled from 500 lbf × (340/2500)

# ── Lift rotors ───────────────────────────────────────────────────────────────
LIFT_ROTOR_TIP_RADIUS      = 0.6096         # m  (2 ft per NDARC)
LIFT_ROTOR_HUB_RADIUS      = 0.0610         # m  (10% root cutout)
LIFT_ROTOR_N_BLADES        = 2
LIFT_ROTOR_DESIGN_TIP_MACH = 0.0746         # Vtip_ref=83.3 ft/s=25.39 m/s; M=25.39/340.3
LIFT_ROTOR_FREESTREAM_VEL  = 500.  * Units['ft/min']
LIFT_ROTOR_DESIGN_CL       = 0.7
LIFT_ROTOR_DESIGN_ALTITUDE = 3000. * Units.feet
LIFT_ROTOR_DESIGN_THRUST   = TAKEOFF_MASS / N_LIFT_ROTORS  # balanced hover

LIFT_ROTOR_ROTATIONS = [1, -1, -1, 1]
LIFT_ROTOR_ORIGINS   = [
    [0.6,  3.,  -0.125],
    [4.5,  3.,  -0.125],
    [0.6, -3.,  -0.125],
    [4.5, -3.,  -0.125],
]

# ── Propeller motor ───────────────────────────────────────────────────────────
PROP_MOTOR_EFFICIENCY     = 0.90   # NDARC eta_motor=0.90
PROP_MOTOR_MASS           = 2.0  * Units.kg
PROP_MOTOR_NO_LOAD_CURRENT = 2.0

# ── Lift-rotor motor ──────────────────────────────────────────────────────────
LIFT_MOTOR_EFFICIENCY          = 0.90   # NDARC eta_motor=0.90
LIFT_MOTOR_VOLTAGE_FRACTION    = 0.75   # fraction of bus voltage
LIFT_MOTOR_MASS                = 3.0   * Units.kg
LIFT_MOTOR_GEARBOX_EFFICIENCY  = 1.0
LIFT_MOTOR_NO_LOAD_CURRENT     = 4.0

# ── Mission ───────────────────────────────────────────────────────────────────
N_CONTROL_POINTS      = 8
INITIAL_BATTERY_SOC   = 0.95              # fraction of max energy

HOVER_CLIMB_START_ALT = 0.0   * Units.ft
HOVER_CLIMB_END_ALT   = 100.  * Units.ft
HOVER_CLIMB_RATE      = 200.  * Units['ft/min']

WING_CLIMB_AIR_SPEED  = 70.   * Units.knots
WING_CLIMB_END_ALT    = 3000. * Units.ft
WING_CLIMB_RATE       = 500.  * Units['ft/min']

CRUISE_DISTANCE       = 50.   * Units.nautical_miles
CRUISE_SPEED          = 100.  * Units.knots

WING_DESCENT_AIR_SPEED = 100. * Units.knots
WING_DESCENT_END_ALT   = 100. * Units.ft
WING_DESCENT_RATE      = 300. * Units['ft/min']

HOVER_DESCENT_END_ALT  = 0.   * Units.ft
HOVER_DESCENT_RATE     = 100. * Units['ft/min']

# ── Mission case parameters ────────────────────────────────────────────────────
CRUISE_DISTANCE_SHORT  = 150. * Units.miles                  # Cases 2, 4
CRUISE_DISTANCE_LONG   = 300. * Units.miles                  # Cases 1, 3
CRUISE_SPEED_60MPH     = 60.  * Units.miles / Units.hour     # all four cases
HOVER_DURATION         = 5.   * Units.minutes                # Case 4 mid-route hover
