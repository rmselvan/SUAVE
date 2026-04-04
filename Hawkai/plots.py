## Hawkai — Plot helpers

import matplotlib.pyplot as plt

from SUAVE.Plots.Performance.Mission_Plots import (
    plot_flight_conditions,
    plot_aerodynamic_coefficients,
    plot_battery_pack_conditions,
    plot_lift_cruise_network,
)

# ──────────────────────────────────────────────────────────────────────────────
def make_plots(results, line_color='bo-', title=None):
    """Generate standard mission performance plots."""
    figs_before = set(plt.get_fignums())

    plot_flight_conditions(results,         line_color=line_color)
    plot_aerodynamic_coefficients(results,  line_color=line_color)
    plot_battery_pack_conditions(results,   line_color=line_color)
    plot_lift_cruise_network(results,       line_color=line_color)

    if title:
        figs_after = set(plt.get_fignums())
        for fnum in (figs_after - figs_before):
            fig = plt.figure(fnum)
            existing = fig._suptitle.get_text() if fig._suptitle else None
            fig.suptitle(f'{title}\n{existing}' if existing else title, fontsize=9)
