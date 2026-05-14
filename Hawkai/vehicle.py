## Hawkai — Vehicle and configurations setup

import os
import copy as cp
import numpy as np

import SUAVE
from SUAVE.Core import Units
from SUAVE.Attributes.Gases import Air
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import (
    segment_properties, wing_segmented_planform, wing_planform,
)
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_optimal_motor
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass

import Hawkai.parameters as P

# Airfoil files live in regression/scripts/Vehicles/Airfoils/ relative to repo root
_AIRFOIL_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'regression', 'scripts', 'Vehicles', 'Airfoils',
) + os.sep


# ──────────────────────────────────────────────────────────────────────────────
def vehicle_setup():
    """Build and return a fully configured Hawkai SUAVE Vehicle."""

    # ── Vehicle object ────────────────────────────────────────────────────────
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'eVTOL'

    vehicle.mass_properties.takeoff         = P.TAKEOFF_MASS
    vehicle.mass_properties.operating_empty = P.OPERATING_EMPTY_MASS
    vehicle.mass_properties.max_takeoff     = P.MAX_TAKEOFF_MASS
    vehicle.mass_properties.max_payload     = P.MAX_PAYLOAD_MASS
    vehicle.mass_properties.center_of_gravity = P.CENTER_OF_GRAVITY

    # ── Main wing ─────────────────────────────────────────────────────────────
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.origin                    = P.WING_ORIGIN
    wing.spans.projected           = P.WING_SPAN
    wing.chords.root               = P.WING_ROOT_CHORD
    wing.exposed_root_chord_offset = P.WING_EXPOSED_ROOT_CHORD_OFFSET

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                    = 'Root'
    segment.percent_span_location  = 0.
    segment.twist                  = P.WING_INCIDENCE
    segment.root_chord_percent     = 1.
    segment.dihedral_outboard      = P.ROOT_SEG_DIHEDRAL
    segment.sweeps.quarter_chord   = P.ROOT_SEG_SWEEP_QC
    segment.thickness_to_chord     = P.ROOT_SEG_T_C
    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                    = 'Section_2'
    segment.percent_span_location  = P.SEC2_SEG_PCT_SPAN
    segment.twist                  = P.WING_INCIDENCE
    segment.root_chord_percent     = 1.
    segment.dihedral_outboard      = P.SEC2_SEG_DIHEDRAL
    segment.sweeps.quarter_chord   = P.SEC2_SEG_SWEEP_QC
    segment.thickness_to_chord     = P.SEC2_SEG_T_C
    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                    = 'Tip'
    segment.percent_span_location  = 1.0
    segment.twist                  = P.WING_INCIDENCE
    segment.root_chord_percent     = 1.
    segment.dihedral_outboard      = P.TIP_SEG_DIHEDRAL
    segment.sweeps.quarter_chord   = P.TIP_SEG_SWEEP_QC
    segment.thickness_to_chord     = P.TIP_SEG_T_C
    wing.Segments.append(segment)

    wing = segment_properties(wing)
    wing = wing_segmented_planform(wing)

    vehicle.reference_area = wing.areas.reference
    vehicle.append_component(wing)

    # ── Horizontal tail ───────────────────────────────────────────────────────
    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag                   = 'horizontal_tail'
    wing.areas.reference       = P.HTAIL_AREA
    wing.taper                 = P.HTAIL_TAPER
    wing.sweeps_quarter_chord  = P.HTAIL_SWEEP_QC
    wing.aspect_ratio          = P.HTAIL_ASPECT_RATIO
    wing.thickness_to_chord    = P.HTAIL_T_C
    wing.dihedral              = P.HTAIL_DIHEDRAL
    wing = wing_planform(wing)
    wing.origin = [[P.HTAIL_STATION_LINE - 0.25 * wing.chords.root, 0., 0.]]
    vehicle.append_component(wing)

    # ── Vertical tail ─────────────────────────────────────────────────────────
    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag                   = 'vertical_tail'
    wing.areas.reference       = P.VTAIL_AREA
    wing.taper                 = P.VTAIL_TAPER
    wing.sweeps_quarter_chord  = P.VTAIL_SWEEP_QC
    wing.aspect_ratio          = P.VTAIL_ASPECT_RATIO
    wing.thickness_to_chord    = P.VTAIL_T_C
    wing.dihedral              = P.VTAIL_DIHEDRAL
    wing.origin                = [[P.VTAIL_ORIGIN_X, 0., 0.]]
    wing = wing_planform(wing)
    vehicle.append_component(wing)

    # ── Fuselage ──────────────────────────────────────────────────────────────
    fus = SUAVE.Components.Fuselages.Fuselage()
    fus.tag                                = 'fuselage'
    fus.seats_abreast                      = 2.
    fus.fineness.nose                      = .88
    fus.fineness.tail                      = 1.
    fus.lengths.nose                       = 0.0 * Units.feet
    fus.lengths.tail                       = 0.0 * Units.feet
    fus.lengths.cabin                      = P.FUSELAGE_TOTAL_LENGTH * Units.feet
    fus.lengths.total                      = P.FUSELAGE_TOTAL_LENGTH * Units.feet
    fus.width                              = P.FUSELAGE_WIDTH
    fus.heights.maximum                    = P.FUSELAGE_HEIGHT
    fus.heights.at_quarter_length          = P.FUSELAGE_HEIGHT
    fus.heights.at_wing_root_quarter_chord = P.FUSELAGE_HEIGHT
    fus.heights.at_three_quarters_length   = P.FUSELAGE_HEIGHT
    fus.areas.wetted                       = P.FUSELAGE_WETTED_AREA
    fus.effective_diameter                 = P.FUSELAGE_EFF_DIAMETER
    fus.areas.front_projected              = P.FUSELAGE_FRONT_AREA
    fus.differential_pressure              = 0.

    L = P.FUSELAGE_TOTAL_LENGTH  # shorthand for segment fractions

    seg = SUAVE.Components.Lofted_Body_Segment.Segment()
    seg.tag = 'segment_0'; seg.percent_x_location = 0.; seg.percent_z_location = 0.
    seg.height = 0.1 * Units.feet; seg.width = 0.1 * Units.feet
    fus.Segments.append(seg)

    seg = SUAVE.Components.Lofted_Body_Segment.Segment()
    seg.tag = 'segment_1'; seg.percent_x_location = (.286 + .678 * .5) / L; seg.percent_z_location = 0.
    seg.height = .709 * Units.feet; seg.width = .709 * Units.feet
    fus.Segments.append(seg)

    seg = SUAVE.Components.Lofted_Body_Segment.Segment()
    seg.tag = 'segment_2'; seg.percent_x_location = (1.6 + .678 * .5) / L; seg.percent_z_location = 0.
    seg.height = P.FUSELAGE_HEIGHT; seg.width = P.FUSELAGE_WIDTH
    fus.Segments.append(seg)

    seg = SUAVE.Components.Lofted_Body_Segment.Segment()
    seg.tag = 'segment_3'; seg.percent_x_location = (5.745 + .678 * .5) / L; seg.percent_z_location = 0.
    seg.height = P.FUSELAGE_HEIGHT; seg.width = P.FUSELAGE_WIDTH
    fus.Segments.append(seg)

    seg = SUAVE.Components.Lofted_Body_Segment.Segment()
    seg.tag = 'segment_4'; seg.percent_x_location = (6.735 + .678 * .5) / L; seg.percent_z_location = 0.
    seg.height = .453 * Units.feet; seg.width = .453 * Units.feet
    fus.Segments.append(seg)

    seg = SUAVE.Components.Lofted_Body_Segment.Segment()
    seg.tag = 'segment_5'; seg.percent_x_location = 1.; seg.percent_z_location = 0.
    seg.height = .453 * Units.feet; seg.width = .453 * Units.feet
    fus.Segments.append(seg)

    vehicle.append_component(fus)

    # ── Booms ─────────────────────────────────────────────────────────────────
    boom = SUAVE.Components.Fuselages.Fuselage()
    boom.tag                                = 'boom_R'
    boom.origin                             = P.BOOM_ORIGIN
    boom.lengths.nose                       = 0.2
    boom.lengths.tail                       = 0.2
    boom.lengths.total                      = P.BOOM_LENGTH
    boom.width                              = P.BOOM_WIDTH
    boom.heights.maximum                    = P.BOOM_WIDTH
    boom.heights.at_quarter_length          = P.BOOM_WIDTH
    boom.heights.at_three_quarters_length   = P.BOOM_WIDTH
    boom.heights.at_wing_root_quarter_chord = P.BOOM_WIDTH
    boom.effective_diameter                 = P.BOOM_WIDTH
    boom.areas.wetted      = 2 * np.pi * P.BOOM_RADIUS * (P.BOOM_LENGTH - 0.5)
    boom.areas.front_projected = np.pi * P.BOOM_WIDTH
    boom.fineness.nose     = 0.75
    boom.fineness.tail     = 0.75
    vehicle.append_component(boom)

    other_boom = cp.deepcopy(boom)
    other_boom.tag = 'boom_L'
    other_boom.origin[0][1] = -boom.origin[0][1]
    vehicle.append_component(other_boom)

    # ── Propulsion network ────────────────────────────────────────────────────
    net = SUAVE.Components.Energy.Networks.Lift_Cruise()
    net.number_of_lift_rotor_engines = P.N_LIFT_ROTORS
    net.number_of_propeller_engines  = P.N_PROPELLERS
    net.identical_propellers         = True
    net.identical_lift_rotors        = True
    net.voltage                      = P.BUS_VOLTAGE

    # ESCs
    lift_rotor_esc            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    lift_rotor_esc.efficiency = P.ESC_EFFICIENCY
    net.lift_rotor_esc        = lift_rotor_esc

    propeller_esc            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    propeller_esc.efficiency = P.ESC_EFFICIENCY
    net.propeller_esc        = propeller_esc

    # Payload & avionics
    payload           = SUAVE.Components.Energy.Peripherals.Avionics()
    payload.power_draw = P.PAYLOAD_POWER
    net.payload       = payload

    avionics            = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = P.AVIONICS_POWER
    net.avionics        = avionics

    # Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()
    bat.mass_properties.mass = P.BATTERY_MASS
    bat.max_voltage          = net.voltage
    initialize_from_mass(bat)
    net.battery = bat

    # Tractor propeller
    propeller = SUAVE.Components.Energy.Converters.Propeller()
    propeller.origin              = P.PROP_ORIGIN
    propeller.number_of_blades   = P.PROP_N_BLADES
    propeller.tip_radius          = P.PROP_TIP_RADIUS
    propeller.hub_radius          = P.PROP_HUB_RADIUS
    propeller.angular_velocity    = P.PROP_RPM
    propeller.freestream_velocity = P.PROP_FREESTREAM_VEL
    propeller.design_Cl           = P.PROP_DESIGN_CL
    propeller.design_altitude     = P.PROP_DESIGN_ALTITUDE
    propeller.design_thrust       = P.PROP_DESIGN_THRUST

    prop_airfoil = SUAVE.Components.Airfoils.Airfoil()
    prop_airfoil.coordinate_file = _AIRFOIL_DIR + 'NACA_4412.txt'
    prop_airfoil.polar_files = [
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_50000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_100000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_200000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_500000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_1000000.txt',
    ]
    propeller.Airfoils.append(prop_airfoil)
    propeller.airfoil_polar_stations = np.zeros(20, dtype=np.int8).tolist()
    propeller = propeller_design(propeller)
    net.propellers.append(propeller)

    # Lift rotors
    lift_rotor = SUAVE.Components.Energy.Converters.Lift_Rotor()
    lift_rotor.tip_radius         = P.LIFT_ROTOR_TIP_RADIUS
    lift_rotor.hub_radius         = P.LIFT_ROTOR_HUB_RADIUS
    lift_rotor.number_of_blades   = P.LIFT_ROTOR_N_BLADES
    lift_rotor.design_tip_mach    = P.LIFT_ROTOR_DESIGN_TIP_MACH
    lift_rotor.freestream_velocity = P.LIFT_ROTOR_FREESTREAM_VEL
    lift_rotor.angular_velocity   = (
        P.LIFT_ROTOR_DESIGN_TIP_MACH * Air().compute_speed_of_sound() / P.LIFT_ROTOR_TIP_RADIUS
    )
    lift_rotor.design_Cl          = P.LIFT_ROTOR_DESIGN_CL
    lift_rotor.design_altitude    = P.LIFT_ROTOR_DESIGN_ALTITUDE
    lift_rotor.design_thrust      = P.LIFT_ROTOR_DESIGN_THRUST
    lift_rotor.variable_pitch     = False

    lr_airfoil = SUAVE.Components.Airfoils.Airfoil()
    lr_airfoil.coordinate_file = _AIRFOIL_DIR + 'NACA_4412.txt'
    lr_airfoil.polar_files = [
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_50000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_100000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_200000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_500000.txt',
        _AIRFOIL_DIR + 'Polars/NACA_4412_polar_Re_1000000.txt',
    ]
    lift_rotor.Airfoils.append(lr_airfoil)
    lift_rotor.airfoil_polar_stations = np.zeros(20, dtype=np.int8).tolist()
    lift_rotor = propeller_design(lift_rotor)

    for k in range(P.N_LIFT_ROTORS):
        rotor = cp.deepcopy(lift_rotor)
        rotor.tag      = 'lift_rotor'
        rotor.rotation = P.LIFT_ROTOR_ROTATIONS[k]
        rotor.origin   = [P.LIFT_ROTOR_ORIGINS[k]]
        net.lift_rotors.append(rotor)

    # Propeller motor
    prop_motor = SUAVE.Components.Energy.Converters.Motor()
    prop_motor.efficiency          = P.PROP_MOTOR_EFFICIENCY
    prop_motor.nominal_voltage     = bat.max_voltage
    prop_motor.mass_properties.mass = P.PROP_MOTOR_MASS
    prop_motor.origin              = propeller.origin
    prop_motor.propeller_radius    = propeller.tip_radius
    prop_motor.no_load_current     = P.PROP_MOTOR_NO_LOAD_CURRENT
    prop_motor = size_optimal_motor(prop_motor, propeller)
    net.propeller_motors.append(prop_motor)

    # Lift-rotor motors
    lr_motor = SUAVE.Components.Energy.Converters.Motor()
    lr_motor.efficiency          = P.LIFT_MOTOR_EFFICIENCY
    lr_motor.nominal_voltage     = bat.max_voltage * P.LIFT_MOTOR_VOLTAGE_FRACTION
    lr_motor.mass_properties.mass = P.LIFT_MOTOR_MASS
    lr_motor.origin              = lift_rotor.origin
    lr_motor.propeller_radius    = lift_rotor.tip_radius
    lr_motor.gearbox_effiicency  = P.LIFT_MOTOR_GEARBOX_EFFICIENCY   # note: SUAVE typo retained
    lr_motor.no_load_current     = P.LIFT_MOTOR_NO_LOAD_CURRENT
    lr_motor = size_optimal_motor(lr_motor, lift_rotor)

    for _ in range(P.N_LIFT_ROTORS):
        motor = cp.deepcopy(lr_motor)
        motor.tag = 'motor'
        net.lift_rotor_motors.append(motor)

    vehicle.append_component(net)
    vehicle.excrescence_area = P.EXCRESCENCE_AREA

    return vehicle


# ──────────────────────────────────────────────────────────────────────────────
def configs_setup(vehicle):
    """Return a Config.Container with vehicle configurations.

    Currently a stub — only the base config is defined.
    Extend here when hover-only or other variants are needed.
    """
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    return configs
