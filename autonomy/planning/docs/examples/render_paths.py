#!/usr/bin/env python3
"""Render actual C++ PathState outputs; requires matplotlib and shapely."""
from pathlib import Path
import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from shapely import wkt


def polygons(geometry):
    return list(geometry.geoms) if geometry.geom_type == 'MultiPolygon' else [geometry]


def draw_polygon(ax, geometry, core=False):
    for polygon in polygons(geometry):
        x, y = polygon.exterior.xy
        if core:
            ax.plot(x, y, '--', color='#238466', lw=1.2)
        else:
            ax.fill(x, y, color='#eef2f6', zorder=0)
            ax.plot(x, y, color='#586671', lw=1.2)
        for hole in polygon.interiors:
            x, y = hole.xy
            if core:
                ax.plot(x, y, '--', color='#238466', lw=1.2)
            else:
                ax.fill(x, y, color='white', hatch='///', edgecolor='#586671', zorder=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_directory', type=Path)
    args = parser.parse_args()
    root = args.output_directory
    field = wkt.loads((root / 'field.wkt').read_text())
    core = wkt.loads((root / 'core.wkt').read_text())
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), dpi=170)
    for ax, filename, name in zip(axes, ['dubins.path.txt', 'reeds_shepp.path.txt'],
                                  ['Dubins', 'Reeds-Shepp']):
        draw_polygon(ax, field)
        draw_polygon(ax, core, core=True)
        work, connections, reverse = [], [], []
        for line in (root / filename).read_text().splitlines():
            x, y, z, angle, velocity, length, direction, section = map(float, line.split())
            end = (x + math.cos(angle) * length * direction,
                   y + math.sin(angle) * length * direction)
            segment = [(x, y), end]
            (work if int(section) == 1 else connections).append(segment)
            if int(direction) < 0 and length > 0:
                reverse.append(segment)
        ax.add_collection(LineCollection(work, colors='#2866ad', linewidths=.8))
        ax.add_collection(LineCollection(connections, colors='#d48b26', linewidths=.9))
        ax.add_collection(LineCollection(reverse, colors='#a446a5', linewidths=1.1,
                                         linestyles='dashed'))
        ax.set_title(name, loc='left', weight='bold')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.grid(alpha=.15)
        ax.autoscale_view()
    fig.suptitle('Fields2Cover documentation example: actual C++ paths',
                 fontsize=17, weight='bold', x=.055, ha='left')
    fig.legend(handles=[Line2D([], [], color='#2866ad', label='Work swath'),
                        Line2D([], [], color='#d48b26', label='Turn / connection'),
                        Line2D([], [], color='#238466', ls='--', label='3 m core inset'),
                        Line2D([], [], color='#a446a5', ls='--', label='Reverse segment')],
               loc='upper center', bbox_to_anchor=(.5, .92), ncol=4, frameon=False)
    fig.text(.055, .025, 'Metric coordinates; robot 1.5 m, implement 3 m, radius 1 m. '
             'Drawing only; not a full vehicle swept-volume validation.', fontsize=9, color='#586671')
    fig.tight_layout(rect=(0, .12, 1, .86))
    target = root / 'paths.png'
    fig.savefig(target, facecolor='white')
    plt.close(fig)
    print(target)


if __name__ == '__main__':
    main()
