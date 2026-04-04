## Hawkai — Analyses setup

import SUAVE
import Hawkai.parameters as P


# ──────────────────────────────────────────────────────────────────────────────
def base_analysis(vehicle):
    """Return a SUAVE.Analyses.Vehicle configured for the given vehicle."""
    analyses = SUAVE.Analyses.Vehicle()

    # Weights
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)

    # Aerodynamics
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_incremenet = (
        0.4 * vehicle.excrescence_area / vehicle.reference_area
    )
    analyses.append(aerodynamics)

    # Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)

    # Planet
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # Atmosphere
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    return analyses


# ──────────────────────────────────────────────────────────────────────────────
def analyses_setup(configs):
    """Return an Analysis.Container — one analyses object per config."""
    analyses = SUAVE.Analyses.Analysis.Container()

    for tag, config in configs.items():
        analyses[tag] = base_analysis(config)

    return analyses
