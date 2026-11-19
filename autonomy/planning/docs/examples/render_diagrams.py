#!/usr/bin/env python3
"""Rebuild the architecture and data-flow PNG/SVG in ../images using Matplotlib."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).resolve().parent.parent / 'images'
GREEN, BLUE, GRAY, ORANGE = '#238466', '#2866ad', '#586671', '#c58020'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'svg.fonttype': 'none'})


def canvas(title, subtitle):
    fig, ax = plt.subplots(figsize=(16, 10), dpi=180)
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis('off')
    ax.text(.3, 9.7, title, fontsize=21, weight='bold', color='#26364a')
    ax.text(.3, 9.22, subtitle, fontsize=11, color=GRAY)
    return fig, ax


def box(ax, x, y, w, h, title, body, color=BLUE):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.025,rounding_size=.12',
                                facecolor=color + '10', edgecolor=color, linewidth=1.3))
    ax.text(x + .18, y + h - .27, title, fontsize=12, weight='bold', color=color, va='top')
    ax.text(x + .18, y + h - .72, body, fontsize=10.5, color='#26364a', va='top', linespacing=1.5)


def arrow(ax, start, end, color=BLUE, dashed=False, bend=0):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=14,
                                linewidth=1.4, color=color,
                                linestyle='--' if dashed else '-',
                                connectionstyle=f'arc3,rad={bend}'))


def save(fig, name):
    ROOT.mkdir(exist_ok=True)
    fig.subplots_adjust(left=.025, right=.98, top=.98, bottom=.025)
    for suffix in ['png', 'svg']:
        fig.savefig(ROOT / (name + '.' + suffix), facecolor='white')
    plt.close(fig)


def architecture():
    fig, ax = canvas('Fields2Cover in Autonomy: architecture',
                     'Source: autonomy/planning/planner/fields2cover | Optional module in autonomy_planning')
    box(ax, .3, 7.45, 4.1, 1.25, 'Application / task adapter',
        'Region + holes + robot parameters\nTask status, cancellation, control semantics', GRAY)
    box(ax, 5.3, 7.45, 5.25, 1.25, 'fields2cover.hpp / Options',
        'PlanCovRoute()   /   PlanCovPath()\nCell, Cells and Field overloads')
    box(ax, 11.4, 7.45, 4.0, 1.25, 'Route  /  Path',
        'Ordered swaths and connections\nPose, direction, section, speed', GREEN)
    arrow(ax, (4.45, 8.05), (5.25, 8.05))
    arrow(ax, (10.6, 8.05), (11.35, 8.05), GREEN)

    box(ax, .3, 4.9, 3.4, 1.75, 'Headland generation',
        'hg::ConstHL\nCore inset + connection rings\nOuter border / holes')
    box(ax, 4.2, 4.9, 3.4, 1.75, 'Swath generation',
        'sg::BruteForce\nobj::SGObjective\nDirection + phase + clipping')
    box(ax, 8.1, 4.9, 3.4, 1.75, 'Route planning',
        'rp::RoutePlannerBase\nBorder / coverage graphs\nGreedy + timed 2-opt')
    box(ax, 12.0, 4.9, 3.4, 1.75, 'Path planning',
        'pp::PathPlanning\nTurningBase: Dubins / RS\nChecked constrained search')
    for x in [3.75, 7.65, 11.55]:
        arrow(ax, (x, 5.75), (x + .4, 5.75))
    arrow(ax, (7.5, 7.4), (7.5, 6.75), GRAY, dashed=True)

    box(ax, .3, 2.4, 7.3, 1.55, 'Types and geometry',
        'Point / Ring / Cell / Cells / Field / Robot / Swaths / Route / Path\n'
        'NativeGeom + Boost.Geometry: buffer, boolean, clipping, WKT', GREEN)
    box(ax, 8.1, 2.4, 7.3, 1.55, 'Supporting utilities and steering',
        'Parser: nlohmann JSON / pugixml GML; limited CRS transforms\n'
        'autonomy/common/math/steering: turning solvers', ORANGE)
    for x in [2.0, 5.9]:
        arrow(ax, (x, 4.85), (x, 4.02), GREEN, dashed=True)
    for x in [9.8, 13.7]:
        arrow(ax, (x, 4.85), (x, 4.02), ORANGE, dashed=True)
    ax.text(.35, 1.4, 'Integration boundary', fontsize=12, weight='bold', color=GRAY)
    ax.text(.35, .9, 'Fields2Cover has no built-in PlannerServer registration or coverage task protocol.\n'
            'Applications must preserve reverse direction and work/turn semantics when adapting PathState.',
            fontsize=11, color=GRAY, linespacing=1.5)
    save(fig, 'fields2cover-architecture')


def dataflow():
    fig, ax = canvas('Fields2Cover: data flow and geometric constraints',
                     'Core area scores coverage; the original field constrains connecting paths')
    box(ax, .3, 7.45, 3.6, 1.3, 'Cell / Cells', 'Outer boundary + holes\nMetric coordinates', GREEN)
    box(ax, 5.1, 7.45, 4.0, 1.3, 'ConstHL', 'Negative polygon buffer\nVehicle width x headland count')
    box(ax, 11.0, 7.45, 4.4, 1.3, 'Robot + Options', 'Work width, turning radius, speed\nHeadland, angle, objective, turn type', GRAY)
    arrow(ax, (3.95, 8.05), (5.05, 8.05), GREEN)
    arrow(ax, (10.95, 8.05), (9.15, 8.05), GRAY, dashed=True)
    box(ax, .3, 5.0, 3.6, 1.3, 'mainland / core', 'Inset area; holes enlarged\nCan split or become empty', GREEN)
    box(ax, 5.1, 5.0, 4.0, 1.3, 'SwathsByCells', 'Direction / phase / clipping\nActual buffered coverage scoring')
    box(ax, 11.0, 5.0, 4.4, 1.3, 'fast_hl_rings', 'Reference rings for connections\nNot the original free-space boundary')
    arrow(ax, (6.0, 7.4), (2.1, 6.35), bend=.1)
    arrow(ax, (8.2, 7.4), (12.9, 6.35), bend=-.1)
    arrow(ax, (3.95, 5.65), (5.05, 5.65), GREEN)
    box(ax, .3, 2.5, 3.6, 1.3, 'Route', 'Ordered swath groups\nConnections + optional real field')
    box(ax, 5.1, 2.5, 4.0, 1.3, 'PathPlanning', 'Dubins / RS + candidate checks\nInvalid -> constrained pose search')
    box(ax, 11.0, 2.5, 4.4, 1.3, 'Path', 'Point, heading, length, direction\nWork / turn section, velocity', GREEN)
    arrow(ax, (6.9, 4.95), (2.1, 3.85), bend=.08)
    arrow(ax, (12.9, 4.95), (3.3, 3.85))
    arrow(ax, (3.95, 3.15), (5.05, 3.15))
    arrow(ax, (9.15, 3.15), (10.95, 3.15), GREEN)
    arrow(ax, (.35, 7.4), (.35, 3.85), GREEN, dashed=True, bend=.12)
    ax.text(.6, 4.0, 'SetField(original Cells)', fontsize=9, color=GREEN)
    ax.text(.35, 1.35, 'Checks: finite states, nonnegative lengths, boundary / holes, joins and sampled curvature.',
            fontsize=11, color=GRAY)
    ax.text(.35, .87, 'Invalid connection: search or explicit exception. No feasible candidate is a task failure.\n'
            'Full vehicle swept volume and real dynamics remain application validation responsibilities.',
            fontsize=11, color=GRAY, linespacing=1.5)
    save(fig, 'fields2cover-dataflow')


if __name__ == '__main__':
    architecture()
    dataflow()
    print(ROOT)
