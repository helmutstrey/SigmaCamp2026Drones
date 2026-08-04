"""
Session 3 · Script 2 — Fly a named figure  (FLIES)

Flies any figure defined in config.FIGURES. This is the template each group
copies to design their own solo: add a new entry to config.FIGURES, then run
it here. Motion + timing is the whole language.

Run:  python s3_02_figure.py            (defaults to 'triangle')
      python s3_02_figure.py spiral
      python s3_02_figure.py square
Prereq: preflight (s2_00) passes.
"""

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'triangle'
    if name not in config.FIGURES:
        print('Unknown figure "{}". Options: {}'.format(
            name, ', '.join(config.FIGURES)))
        return
    figure = config.FIGURES[name]

    cf_utils.init()
    with cf_utils.make_scf(config.DRONE) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck — aborting.'); return
        cf_utils.prepare_for_flight(scf)
        print('  flying figure: {}'.format(name))
        cf_utils.fly_figure(scf, figure)
        print('  landed.')


if __name__ == '__main__':
    main()
