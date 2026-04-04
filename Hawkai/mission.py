## Hawkai — Mission setup

import SUAVE
from SUAVE.Core import Units
import Hawkai.parameters as P


# ──────────────────────────────────────────────────────────────────────────────
def _make_base_segment():
    Segments = SUAVE.Analyses.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_control_points = P.N_CONTROL_POINTS
    base_segment.process.iterate.initials.initialize_battery = (
        SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    )
    base_segment.process.iterate.conditions.planet_position = SUAVE.Methods.skip
    base_segment.process.iterate.conditions.stability       = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability    = SUAVE.Methods.skip
    return base_segment


def add_hover_climb(mission, base_segment, configs_analyses, vehicle, tag=None):
    Segments = SUAVE.Analyses.Mission.Segments
    segment = Segments.Hover.Climb(base_segment)
    segment.tag            = tag if tag is not None else 'hover_climb'
    segment.analyses.extend(configs_analyses.base)
    segment.altitude_start = P.HOVER_CLIMB_START_ALT
    segment.altitude_end   = P.HOVER_CLIMB_END_ALT
    segment.climb_rate     = P.HOVER_CLIMB_RATE
    segment.battery_energy = vehicle.networks.lift_cruise.battery.max_energy * P.INITIAL_BATTERY_SOC
    segment.process.iterate.unknowns.mission = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)
    mission.append_segment(segment)


def add_wing_climb(mission, base_segment, configs_analyses, vehicle, tag=None):
    Segments = SUAVE.Analyses.Mission.Segments
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag          = tag if tag is not None else 'wing_climb'
    segment.analyses.extend(configs_analyses.base)
    segment.air_speed    = P.WING_CLIMB_AIR_SPEED
    segment.altitude_end = P.WING_CLIMB_END_ALT
    segment.climb_rate   = P.WING_CLIMB_RATE
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)
    mission.append_segment(segment)


def add_cruise(mission, base_segment, configs_analyses, vehicle,
               distance=None, air_speed=None, tag=None):
    Segments = SUAVE.Analyses.Mission.Segments
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag          = tag       if tag       is not None else 'Cruise'
    segment.analyses.extend(configs_analyses.base)
    segment.distance     = distance  if distance  is not None else P.CRUISE_DISTANCE
    segment.air_speed    = air_speed if air_speed is not None else P.CRUISE_SPEED
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)
    mission.append_segment(segment)


def add_wing_descent(mission, base_segment, configs_analyses, vehicle, tag=None):
    Segments = SUAVE.Analyses.Mission.Segments
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag          = tag if tag is not None else 'wing_descent'
    segment.analyses.extend(configs_analyses.base)
    segment.air_speed    = P.WING_DESCENT_AIR_SPEED
    segment.altitude_end = P.WING_DESCENT_END_ALT
    segment.climb_rate   = P.WING_DESCENT_RATE
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)
    mission.append_segment(segment)


def add_hover_descent(mission, base_segment, configs_analyses, vehicle, tag=None):
    Segments = SUAVE.Analyses.Mission.Segments
    segment = Segments.Hover.Descent(base_segment)
    segment.tag           = tag if tag is not None else 'hover_descent'
    segment.analyses.extend(configs_analyses.base)
    segment.altitude_end  = P.HOVER_DESCENT_END_ALT
    segment.descent_rate  = P.HOVER_DESCENT_RATE
    segment.process.iterate.unknowns.mission = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)
    mission.append_segment(segment)


def add_hover(mission, base_segment, configs_analyses, vehicle,
              duration=None, tag=None):
    Segments = SUAVE.Analyses.Mission.Segments
    segment = Segments.Hover.Hover(base_segment)
    segment.tag      = tag      if tag      is not None else 'hover'
    segment.analyses.extend(configs_analyses.base)
    segment.altitude = P.WING_CLIMB_END_ALT
    segment.time     = duration if duration is not None else P.HOVER_DURATION
    segment.process.iterate.unknowns.mission = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)
    mission.append_segment(segment)


# ──────────────────────────────────────────────────────────────────────────────
def mission_setup(configs_analyses, vehicle):
    """Build and return the primary Sequential_Segments mission.

    configs_analyses is the Analysis.Container returned by analyses_setup().
    Each segment extends configs_analyses.base.
    """
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    base_segment = _make_base_segment()

    add_hover_climb(mission, base_segment, configs_analyses, vehicle)
    add_wing_climb(mission, base_segment, configs_analyses, vehicle)
    add_cruise(mission, base_segment, configs_analyses, vehicle)
    add_wing_descent(mission, base_segment, configs_analyses, vehicle)
    add_hover_descent(mission, base_segment, configs_analyses, vehicle)

    return mission


# ──────────────────────────────────────────────────────────────────────────────
def mission_case1(configs_analyses, vehicle):
    """Case 1 — Linear Asset Inspection Mission A.

    Hover climb → wing climb → cruise 300 mi @ 60 mph → wing descent → hover descent.
    """
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'case1_asset_inspection'
    base_segment = _make_base_segment()

    add_hover_climb  (mission, base_segment, configs_analyses, vehicle)
    add_wing_climb   (mission, base_segment, configs_analyses, vehicle)
    add_cruise       (mission, base_segment, configs_analyses, vehicle,
                      distance=P.CRUISE_DISTANCE_LONG, air_speed=P.CRUISE_SPEED_60MPH)
    add_wing_descent (mission, base_segment, configs_analyses, vehicle)
    add_hover_descent(mission, base_segment, configs_analyses, vehicle)

    return mission


def mission_case2(configs_analyses, vehicle):
    """Case 2 — Part Delivery Mission.

    Two identical legs, each: hover climb → wing climb → cruise 150 mi @ 60 mph
    → wing descent → hover descent.
    """
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'case2_part_delivery'
    base_segment = _make_base_segment()

    # Leg 1
    add_hover_climb  (mission, base_segment, configs_analyses, vehicle, tag='hover_climb_1')
    add_wing_climb   (mission, base_segment, configs_analyses, vehicle, tag='wing_climb_1')
    add_cruise       (mission, base_segment, configs_analyses, vehicle,
                      distance=P.CRUISE_DISTANCE_SHORT, air_speed=P.CRUISE_SPEED_60MPH,
                      tag='cruise_1')
    add_wing_descent (mission, base_segment, configs_analyses, vehicle, tag='wing_descent_1')
    add_hover_descent(mission, base_segment, configs_analyses, vehicle, tag='hover_descent_1')

    # Leg 2
    add_hover_climb  (mission, base_segment, configs_analyses, vehicle, tag='hover_climb_2')
    add_wing_climb   (mission, base_segment, configs_analyses, vehicle, tag='wing_climb_2')
    add_cruise       (mission, base_segment, configs_analyses, vehicle,
                      distance=P.CRUISE_DISTANCE_SHORT, air_speed=P.CRUISE_SPEED_60MPH,
                      tag='cruise_2')
    add_wing_descent (mission, base_segment, configs_analyses, vehicle, tag='wing_descent_2')
    add_hover_descent(mission, base_segment, configs_analyses, vehicle, tag='hover_descent_2')

    return mission


def mission_case3(configs_analyses, vehicle):
    """Case 3 — Dual Inspection and Delivery Route.

    Identical flight profile to Case 1: 300 mi @ 60 mph.
    Separate function for distinct operational context and tagging.
    """
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'case3_dual_inspection_delivery'
    base_segment = _make_base_segment()

    add_hover_climb  (mission, base_segment, configs_analyses, vehicle)
    add_wing_climb   (mission, base_segment, configs_analyses, vehicle)
    add_cruise       (mission, base_segment, configs_analyses, vehicle,
                      distance=P.CRUISE_DISTANCE_LONG, air_speed=P.CRUISE_SPEED_60MPH)
    add_wing_descent (mission, base_segment, configs_analyses, vehicle)
    add_hover_descent(mission, base_segment, configs_analyses, vehicle)

    return mission


def mission_case4(configs_analyses, vehicle):
    """Case 4 — Spot Inspection.

    Hover climb → wing climb → cruise 150 mi @ 60 mph → hover 5 min
    → cruise 150 mi @ 60 mph → wing descent → hover descent.
    """
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'case4_spot_inspection'
    base_segment = _make_base_segment()

    add_hover_climb  (mission, base_segment, configs_analyses, vehicle)
    add_wing_climb   (mission, base_segment, configs_analyses, vehicle)
    add_cruise       (mission, base_segment, configs_analyses, vehicle,
                      distance=P.CRUISE_DISTANCE_SHORT, air_speed=P.CRUISE_SPEED_60MPH,
                      tag='cruise_1')
    add_hover        (mission, base_segment, configs_analyses, vehicle,
                      duration=P.HOVER_DURATION)
    add_cruise       (mission, base_segment, configs_analyses, vehicle,
                      distance=P.CRUISE_DISTANCE_SHORT, air_speed=P.CRUISE_SPEED_60MPH,
                      tag='cruise_2')
    add_wing_descent (mission, base_segment, configs_analyses, vehicle)
    add_hover_descent(mission, base_segment, configs_analyses, vehicle)

    return mission


# ──────────────────────────────────────────────────────────────────────────────
def missions_setup(base_mission, configs_analyses, vehicle):
    """Return a Mission.Container holding all missions."""
    missions = SUAVE.Analyses.Mission.Mission.Container()
    missions.base  = base_mission
    missions.case1 = mission_case1(configs_analyses, vehicle)
    missions.case2 = mission_case2(configs_analyses, vehicle)
    missions.case3 = mission_case3(configs_analyses, vehicle)
    missions.case4 = mission_case4(configs_analyses, vehicle)
    return missions
