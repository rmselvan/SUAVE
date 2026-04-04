## Live tutorial - Basic eVTOL Analysis - https://suave.stanford.edu/tutorials/basic_eVTOL.html

# --------------------------------------------------------------------------------------------------
# Imports
# --------------------------------------------------------------------------------------------------

## Note: reference frame origin located at nose. X-axis pointed from nose toward tail. Z-axis pointed up

# package imports
import SUAVE
import numpy as np
import copy as cp
import matplotlib.pyplot as plt
import os

from SUAVE.Core import Units
from SUAVE.Attributes.Gases import Air
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Input_Output.OpenVSP import write
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties, wing_segmented_planform, wing_planform
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_optimal_motor
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass

# --------------------------------------------------------------------------------------------------
# Main 
# --------------------------------------------------------------------------------------------------
def main():
  # Setup a Vehicle
  vehicle = setup_vehicle()
  wing = vehicle.wings.main_wing
  # export the vehicle to OpenVSP
  print("spans.projected (m)  :", wing.spans.projected)
  print("spans.projected (ft) :", wing.spans.projected / Units.feet)
  print("symmetric            :", wing.symmetric)
  print("chords.root (m)      :", wing.chords.root)
  print("chords.tip (m)       :", wing.chords.tip)
  for seg in wing.Segments:
      print(f"  Segment {seg.tag}: pct_span={seg.percent_span_location}, root_chord_pct={seg.root_chord_percent}")
  write(vehicle, 'Hawkai')

  # Setup analyses
  # analyses = setup_analyses(vehicle)
  # analyses.finalize() # <-- this builds surrogate models! Important to call this
  
  # Setup a mission
  # mission = setup_mission(vehicle,analyses)
  # results = mission.evaluate()

  # make_plots(results)

# --------------------------------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------------------------------
def make_plots(results):
  plot_flight_conditions(results)
  plot_aerodynamic_coefficients(results)
  plot_battery_pack_conditions(results)
  plot_lift_cruise_network(results)

# --------------------------------------------------------------------------------------------------
# Analyses
# --------------------------------------------------------------------------------------------------
def setup_analyses(vehicle):
  # Initialize the analyses:
  analyses = SUAVE.Analyses.Vehicle()

  # Weights:
  weights = SUAVE.Analyses.Weights.Weights_eVTOL()
  weights.vehicle = vehicle
  analyses.append(weights)

  # Aerodynamic Analyses:
  aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
  aerodynamics.geometry = vehicle
  aerodynamics.settings.drag_coefficient_incremenet = 0.4 * vehicle.excrescence_area/vehicle.reference_area
  analyses.append(aerodynamics)

  # Energy:
  energy = SUAVE.Analyses.Energy.Energy()
  energy.network = vehicle.networks
  analyses.append(energy)

  # Noise Analysis:
  # noise = SUAVE.Analyses.Noise.Fidelity_One()
  # noise.network = vehicle
  # noise.geometry = vehicle
  # analyses.append(noise)

  # Planet Analysis:
  planet = SUAVE.Analyses.Planets.Planet()
  analyses.append(planet)

  # Atmosphere Analysis:
  atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
  atmosphere.features.planet = planet.features
  analyses.append(atmosphere)

  return analyses


# --------------------------------------------------------------------------------------------------
# Vehicle 
# --------------------------------------------------------------------------------------------------

def setup_vehicle():
  # ------------------------------------------------------------------------------------------------
  # Initialize the Vehicle
  # ------------------------------------------------------------------------------------------------
  # Create a vehicle and set level properties
  vehicle = SUAVE.Vehicle()
  vehicle.tag = 'eVTOL'

  # ------------------------------------------------------------------------------------------------
  # Vehicle-level properties
  # ------------------------------------------------------------------------------------------------
  # mass properties
  vehicle.mass_properties.takeoff = 2500. * Units.lb
  vehicle.mass_properties.operating_empty = 2150. * Units.lb
  vehicle.mass_properties.max_takeoff = 2500. * Units.lb
  vehicle.mass_properties.max_payload = 100. * Units.lb
  vehicle.mass_properties.center_of_gravity = [[2.0,0.,0.]] # TODO: make this accurate

  # ------------------------------------------------------------------------------------------------
  # WINGS
  # ------------------------------------------------------------------------------------------------
  # TODO:
  # - wing incidence needs to be added
  wing = SUAVE.Components.Wings.Main_Wing()
  wing.origin = [[1.215,0.,0.313]]
  wing.spans.projected = 23. * Units.feet
  wing.chords.root = 2.1 * Units.feet
  wing.exposed_root_chord_offset = 0.5

  # Segment
  segment = SUAVE.Components.Wings.Segment()
  segment.tag = 'Root'
  segment.percent_span_location = 0.
  segment.twist = 0.
  segment.root_chord_percent = 1.
  # not adjusted -----------------------------------
  segment.dihedral_outboard = 1.0 * Units.degrees
  segment.sweeps.quarter_chord = 8.5 * Units.degrees
  # not adjusted -----------------------------------
  segment.thickness_to_chord = 0.12
  wing.Segments.append(segment)

  # Segment
  segment = SUAVE.Components.Wings.Segment()
  segment.tag = 'Section_2'
  segment.percent_span_location = 4. / 11.
  segment.twist = 0.
  segment.root_chord_percent = 1.
  # not adjusted -----------------------------------
  segment.dihedral_outboard = 1.0 * Units.degrees
  segment.sweeps.quarter_chord = 0.0 * Units.degrees
  # not adjusted -----------------------------------
  segment.thickness_to_chord = 0.12
  wing.Segments.append(segment)

  # Segment
  segment = SUAVE.Components.Wings.Segment()
  segment.tag = 'Tip'
  segment.percent_span_location = 1.0
  segment.twist = 0.
  segment.root_chord_percent = 1.
  segment.dihedral_outboard = 0.0 * Units.degrees
  # not adjusted -----------------------------------
  segment.sweeps.quarter_chord = 0.0 * Units.degrees
  segment.thickness_to_chord = 0.12
  # not adjusted -----------------------------------
  wing.Segments.append(segment)

  # Fill out more segment properties automatically
  wing = segment_properties(wing)
  wing = wing_segmented_planform(wing)

  ## ALSO SET VEHICLE REFERENCE AREA:
  vehicle.reference_area = wing.areas.reference

  # Add to vehicle
  vehicle.append_component(wing)
  
  # ------------------------------------------------------------------------------------------------
  # TAILS
  # ------------------------------------------------------------------------------------------------

  # Add a horizontal tail
  # WING PROPERTIES
  wing = SUAVE.Components.Wings.Horizontal_Tail()
  wing.tag = 'horizontal_tail'
  wing.areas.reference = 10.0 * Units.feet * Units.feet
  wing.taper = 1.
  # wing.sweeps_quarter_chord = 20. * Units.degrees
  wing.sweeps_quarter_chord = 0.
  wing.aspect_ratio = 5.0
  wing.thickness_to_chord = 0.09
  # not adjusted -----------------------------------
  wing.dihedral = 5. * Units.degrees
  # ------------------------------------------------
  wing.origin = [[((7.324 + .678*.5) - .25*wing.chords.root) * Units.feet ,0.,0.]]

  # Fill out wing properties automatically
  wing = wing_planform(wing)

  # add to vehicle
  vehicle.append_component(wing)

  # Add a vertical tail
  wing = SUAVE.Components.Wings.Vertical_Tail()
  wing.tag = 'vertical_tail'
  wing.areas.reference = 8. * Units.feet * Units.feet
  wing.taper = 1.
  # wing.sweeps_quarter_chord = 20. * Units.degrees
  wing.sweeps_quarter_chord = 0.
  wing.aspect_ratio = 5.0
  wing.thickness_to_chord = 0.09
  # not adjusted -----------------------------------
  wing.dihedral = 5. * Units.degrees
  # ------------------------------------------------
  wing.origin = [[8.75 * Units.feet,0.,0.]]

  # Fill out wing properties automatically
  wing = wing_planform(wing)

  # add to vehicle
  vehicle.append_component(wing)

  # ------------------------------------------------------------------------------------------------
  # FUSELAGE
  # ------------------------------------------------------------------------------------------------
  # FUSELAGE PROPERTIES ----------------------------------------------------------------------------
  fuselage = SUAVE.Components.Fuselages.Fuselage()
  fuselage.tag = 'fuselage'
  fuselage.seats_abreast = 2.
  fuselage.fineness.nose = .88
  fuselage.fineness.tail = 1.
  fuselage.lengths.nose  = 1.0 * Units.feet
  fuselage.lengths.tail  = 1.8 * Units.feet
  fuselage.lengths.nose  = 0.0 * Units.feet
  fuselage.lengths.tail  = 0.0 * Units.feet
  fuselage_total_length = 7.56 + (0.678 * .5)
  fuselage.lengths.cabin = fuselage_total_length * Units.feet
  # not adjusted -----------------------------------
  fuselage.lengths.total = fuselage_total_length * Units.feet
  # ------------------------------------------------
  fuselage.width = 1.333 * Units.feet
  fuselage.heights.maximum = 1.333 * Units.feet
  fuselage.heights.at_quarter_length = 1.333 * Units.feet
  fuselage.heights.at_wing_root_quarter_chord = 1.333  * Units.feet
  fuselage.heights.at_three_quarters_length = 1.333 * Units.feet
  fuselage.areas.wetted = 20. * Units.feet**2
  fuselage.effective_diameter = 5.85 * Units.feet # TODO: verify this is appropriate
  # not adjusted -----------------------------------
  fuselage.areas.front_projected = .14 * Units.feet**2
  fuselage.differential_pressure = 0.
  # ------------------------------------------------

  # Segment - nose
  segment = SUAVE.Components.Lofted_Body_Segment.Segment()
  segment.tag = 'segment_0'
  segment.percent_x_location = 0.
  segment.percent_z_location = 0.
  segment.height = 0.1 * Units.feet
  segment.width = 0.1  * Units.feet
  fuselage.Segments.append(segment)

  # Segment - first bulkhead
  segment = SUAVE.Components.Lofted_Body_Segment.Segment()
  segment.tag = 'segment_1'
  segment.percent_x_location = (.286 + .678*.5) / fuselage_total_length
  segment.percent_z_location = 0.
  segment.height = .709 * Units.feet
  segment.width =  .709 * Units.feet
  fuselage.Segments.append(segment)

  # Segment - second bulkhead
  segment = SUAVE.Components.Lofted_Body_Segment.Segment()
  segment.tag = 'segment_2'
  segment.percent_x_location = (1.6 + .678*.5) / fuselage_total_length
  segment.percent_z_location = 0.
  segment.height = 1.333 * Units.feet
  segment.width =  1.333 * Units.feet
  fuselage.Segments.append(segment)

  # Segment - Largest diameter H2 tank farthest away from nose to nose tip
  segment = SUAVE.Components.Lofted_Body_Segment.Segment()
  segment.tag = 'segment_3'
  segment.percent_x_location = (5.745 + .678*.5) / fuselage_total_length
  segment.percent_z_location = 0.
  segment.height = 1.333 * Units.feet
  segment.width =  1.333 * Units.feet
  fuselage.Segments.append(segment)

  # Segment - Beginning of tail section closest to tip to nose tip
  segment = SUAVE.Components.Lofted_Body_Segment.Segment()
  segment.tag = 'segment_4'
  segment.percent_x_location = (6.735 + .678*.5) / fuselage_total_length
  segment.percent_z_location = 0.
  segment.height = .453 * Units.feet
  segment.width =  .453 * Units.feet
  fuselage.Segments.append(segment)

  # Segment - Beginning of tail section closest to tip to nose tip
  segment = SUAVE.Components.Lofted_Body_Segment.Segment()
  segment.tag = 'segment_5'
  segment.percent_x_location = 1.
  segment.percent_z_location = 0.
  segment.height = .453 * Units.feet
  segment.width =  .453 * Units.feet
  fuselage.Segments.append(segment)
  
  # Add to vehicle:
  vehicle.append_component(fuselage)
  
  # ------------------------------------------------------------------------------------------------
  # BOOMS
  # ------------------------------------------------------------------------------------------------
  # Add booms for the motors -----------------------------------------------------------------------
  boom = SUAVE.Components.Fuselages.Fuselage()
  boom.tag = 'boom_R'
  boom.origin = [[0.525, 3.0, -0.35]]
  boom.lengths.nose = 0.2
  boom.lengths.tail = 0.2
  boom.lengths.total = 5
  boom.width = 0.15
  boom.heights.maximum = 0.15
  boom.heights.at_quarter_length = 0.15
  boom.heights.at_three_quarters_length = 0.15
  boom.heights.at_wing_root_quarter_chord = 0.15
  boom.effective_diameter = 0.15
  boom.areas.wetted = 2 * np.pi * (0.075) * 3.5
  boom.areas.front_projected = np.pi * 0.15
  boom.fineness.nose = 0.75
  boom.fineness.tail = 0.75

  vehicle.append_component(boom)

  # Now attached the mirrored boom
  other_boom = cp.deepcopy(boom)
  other_boom.origin[0][1] = -boom.origin[0][1]
  other_boom.tag = 'boom_L'
  vehicle.append_component(other_boom)

  # ------------------------------------------------------------------------------------------------
  # NETWORK
  # ------------------------------------------------------------------------------------------------
  net = SUAVE.Components.Energy.Networks.Lift_Cruise()
  net.number_of_lift_rotor_engines = 4
  net.number_of_propeller_engines = 1
  net.identical_propellers = True
  net.identical_lift_rotors = True
  net.voltage = 400.

  # ------------------------------------------------------------------------------------------------
  # Electronic Speed Controller
  # ------------------------------------------------------------------------------------------------
  lift_rotor_esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
  lift_rotor_esc.efficiency = 0.95
  net.lift_rotor_esc = lift_rotor_esc

  propeller_esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
  propeller_esc.efficiency = 0.95
  net.propeller_esc = propeller_esc

  # ------------------------------------------------------------------------------------------------
  # Payload 
  # ------------------------------------------------------------------------------------------------
  payload = SUAVE.Components.Energy.Peripherals.Avionics()
  payload.power_draw = 0.
  net.payload = payload

  # ------------------------------------------------------------------------------------------------
  # Avionics
  # ------------------------------------------------------------------------------------------------
  avionics = SUAVE.Components.Energy.Peripherals.Avionics()
  avionics.power_draw = 300. * Units.watts
  net.avionics = avionics

  # ------------------------------------------------------------------------------------------------
  # Design Battery
  # ------------------------------------------------------------------------------------------------
  bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()
  bat.mass_properties.mass = 1000. * Units.lb 
  bat.max_voltage = net.voltage
  initialize_from_mass(bat)
  net.battery = bat

  # ------------------------------------------------------------------------------------------------
  # DESIGN ROTORS AND PROPELLERS
  # ------------------------------------------------------------------------------------------------
  # The tractor propeller --------------------------------------------------------------------------
  propeller = SUAVE.Components.Energy.Converters.Propeller()
  propeller.origin = [[0.,0.,-0.325]]
  propeller.number_of_blades = 3
  propeller.tip_radius = 0.9
  propeller.hub_radius = 0.1
  propeller.angular_velocity = 2200 * Units.rpm
  propeller.freestream_velocity = 100. * Units.knots
  propeller.design_Cl = 0.7
  propeller.design_altitude = 5000. * Units.feet
  propeller.design_thrust = 500. * Units.lbf
  ospath    = os.path.abspath(__file__)
  separator = os.path.sep 
  rel_path = ospath.split('Hawkai.py')[0] + 'regression' + separator + 'scripts' + separator + 'Vehicles' + separator
  # propeller.airfoil_geometry = [rel_path + 'Airfoils/NACA_4412.txt']
  # propeller.airfoil_polars = [[rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_50000.txt', 
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]
  propeller_airfoil = SUAVE.Components.Airfoils.Airfoil()
  propeller_airfoil.coordinate_file = rel_path + 'Airfoils/NACA_4412.txt'
  propeller_airfoil.polar_files = [rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_50000.txt', 
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']
  propeller.Airfoils.append(propeller_airfoil)
  propeller.airfoil_polar_stations = np.zeros((20),dtype=np.int8).tolist()
  propeller = propeller_design(propeller)
  net.propellers.append(propeller)

  # The lifter rotors ------------------------------------------------------------------------------
  lift_rotor = SUAVE.Components.Energy.Converters.Lift_Rotor()
  lift_rotor.tip_radius = 1.5
  lift_rotor.hub_radius = 0.15
  lift_rotor.number_of_blades = 4
  lift_rotor.design_tip_mach = 0.65
  lift_rotor.freestream_velocity = 500. * Units['ft/min']
  lift_rotor.angular_velocity = lift_rotor.design_tip_mach * Air().compute_speed_of_sound()/lift_rotor.tip_radius
  lift_rotor.design_Cl = 0.7
  lift_rotor.design_altitude = 3000. * Units.feet
  lift_rotor.design_thrust = 2500. * Units.lbf/4
  lift_rotor.variable_pitch = False 
  # lift_rotor.airfoil_geometry = [rel_path + 'Airfoils/NACA_4412.txt']
  # lift_rotor.airfoil_polars = [[rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_50000.txt', 
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
  #                              rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]
  lift_rotor_airfoil = SUAVE.Components.Airfoils.Airfoil()
  lift_rotor_airfoil.coordinate_file = rel_path + 'Airfoils/NACA_4412.txt'
  lift_rotor_airfoil.polar_files = [rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_50000.txt', 
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                               rel_path + 'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']
  lift_rotor.Airfoils.append(lift_rotor_airfoil)
  lift_rotor.airfoil_polar_stations = np.zeros((20),dtype=np.int8).tolist()
  lift_rotor = propeller_design(lift_rotor) # don't use propeller design on lift rotors generally!

  rotations = [1,-1,-1,1]
  origins = [[0.6,3.,-0.125],[4.5,3.,-0.125],[0.6,-3,-0.125],[4.5,-3.,-0.125]]
  for k in range(4):
    lift_rotor = cp.deepcopy(lift_rotor)
    lift_rotor.tag = 'lift_rotor'
    lift_rotor.rotation = rotations[k]
    lift_rotor.origin = [origins[k]]
    net.lift_rotors.append(lift_rotor)

  # ------------------------------------------------------------------------------------------------
  # DESIGN MOTORS
  # ------------------------------------------------------------------------------------------------
  # Propeller (Thrust) motor -----------------------------------------------------------------------
  propeller_motor = SUAVE.Components.Energy.Converters.Motor()
  propeller_motor.efficiency = 0.95
  propeller_motor.nominal_voltage = bat.max_voltage
  propeller_motor.mass_properties.mass = 2.0 * Units.kg
  propeller_motor.origin = propeller.origin
  propeller_motor.propeller_radius = propeller.tip_radius
  propeller_motor.no_load_current = 2.0
  propeller_motor = size_optimal_motor(propeller_motor,propeller)
  net.propeller_motors.append(propeller_motor)

  # Rotor (Lift) motor -----------------------------------------------------------------------------
  lift_rotor_motor = SUAVE.Components.Energy.Converters.Motor()
  lift_rotor_motor.efficiency = 0.85
  lift_rotor_motor.nominal_voltage = bat.max_voltage * 0.75
  lift_rotor_motor.mass_properties.mass = 3.0 * Units.kg
  lift_rotor_motor.origin = lift_rotor.origin
  lift_rotor_motor.propeller_radius = lift_rotor.tip_radius
  lift_rotor_motor.gearbox_effiicency = 1.0
  lift_rotor_motor.no_load_current = 4.0
  lift_rotor_motor = size_optimal_motor(lift_rotor_motor,lift_rotor)

  for k in range(4):
    lift_rotor_motor = cp.deepcopy(lift_rotor_motor)
    lift_rotor_motor.tag = 'motor'
    net.lift_rotor_motors.append(lift_rotor_motor)

  # Add energy network to the vehicle
  vehicle.append_component(net)

  # Account for additional drag area by things that have been overlooked 
  #   (e.g antennae, proper wetted area, landing gear)
  vehicle.excrescence_area = 0.1

  return vehicle


def setup_mission(vehicle,analyses):
  # --------------------------------------------------------------------------------------------------
  # Initialize the Mission
  # --------------------------------------------------------------------------------------------------
  mission = SUAVE.Analyses.Mission.Sequential_Segments()
  mission.tag = 'the_mission'

  # unpack Segments module
  Segments = SUAVE.Analyses.Mission.Segments

  # base segment
  base_segment = Segments.Segment()
  base_segment.state.numerics.number_control_points = 8
  base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
  base_segment.process.iterate.conditions.planet_position = SUAVE.Methods.skip
  base_segment.process.iterate.conditions.stability = SUAVE.Methods.skip
  base_segment.process.finalize.post_process.stability = SUAVE.Methods.skip
  ones_row = base_segment.state.ones_row

  # --------------------------------------------------------------------------------------------------
  # Hover Climb Segment
  # --------------------------------------------------------------------------------------------------
  segment = Segments.Hover.Climb(base_segment)
  segment.tag = "hover_climb"
  segment.analyses.extend(analyses)
  segment.altitude_start = 0.0* Units.ft
  segment.altitude_end = 100. * Units.ft
  segment.climb_rate = 200. * Units['ft/min']
  segment.battery_energy = vehicle.networks.lift_cruise.battery.max_energy*0.95
  segment.process.iterate.unknowns.mission = SUAVE.Methods.skip
  segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)

  # add to mission
  mission.append_segment(segment)

  # --------------------------------------------------------------------------------------------------
  # Second Climb Segment: Speed, Constant Rate
  # --------------------------------------------------------------------------------------------------
  segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
  segment.tag = "wing_climb"
  segment.analyses.extend(analyses)
  segment.air_speed = 70. * Units.knots
  segment.altitude_end = 3000. * Units.ft
  segment.climb_rate = 500. * Units['ft/min']
  segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)

  # add to mission
  mission.append_segment(segment)

  # --------------------------------------------------------------------------------------------------
  # Cruise
  # --------------------------------------------------------------------------------------------------
  segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
  segment.tag = "Cruise"
  segment.analyses.extend(analyses)
  segment.distance = 50. * Units.nautical_miles
  segment.air_speed = 100. * Units.knots
  segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)

  # add to mission
  mission.append_segment(segment)

  # --------------------------------------------------------------------------------------------------
  # Descent
  # --------------------------------------------------------------------------------------------------
  segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
  segment.tag = "wing_descent"
  segment.analyses.extend(analyses)
  segment.air_speed = 100. * Units.knots
  segment.altitude_end = 100. * Units.ft
  segment.climb_rate = 300. * Units['ft/min']
  segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)

  # add to mission
  mission.append_segment(segment)

  # --------------------------------------------------------------------------------------------------
  # Hover Descent
  # --------------------------------------------------------------------------------------------------
  segment = Segments.Hover.Descent(base_segment)
  segment.tag = "hover_descent"
  segment.analyses.extend(analyses)
  segment.altitude_end = 0. * Units.ft
  segment.descent_rate = 100. * Units['ft/min']
  segment.process.iterate.unknowns.mission = SUAVE.Methods.skip
  segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)

  # add to mission
  mission.append_segment(segment)

  return mission


if __name__=="__main__":
  main()
  plt.show()