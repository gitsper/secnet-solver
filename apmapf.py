__copyright__ = """
    Copyright 2020 Boston University Board of Trustees

    Author: Kacper Wardega
"""

__license__ = """
    This file is part of secnet.

    secnet is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    secnet is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with secnet.  If not, see <https://www.gnu.org/licenses/>.
"""

import argparse
import os
import sys
import json
import math
import logging
import timeout_decorator
sys.stderr = open(os.devnull, 'w')
from pysmt.shortcuts import (
    Symbol,
    Int,
    get_model,
    ForAll,
    Ite,
    And,
    Or,
    Not,
    Implies,
    Equals)
from pysmt.typing import INT
sys.stderr = sys.__stderr__
from bfs import BFS


def Abs(x):
    return Ite(x >= 0, x, -x)


def Adj(c0, c1):
    return 1 >= Abs(c1[0] - c0[0]) + Abs(c1[1] - c0[1])


def IsOnGrid(coord, gridsize):
    return And(
        coord[0] >= 0,
        coord[1] >= 0,
        coord[0] < gridsize,
        coord[1] < gridsize)


def IsConnected(path):
    return And([Adj(path[i-1], path[i]) for i in range(1, len(path))])


def SamePosition(c0, c1):
    return And(Equals(c0[0], c1[0]), Equals(c0[1], c1[1]))


def IntifyCoords(ary):
    return [[Int(c0), Int(c1)] for c0, c1 in ary]


def IsPlan(plan, gridsize, starts, goals, obstacles):
    starts = IntifyCoords(starts)
    goals = IntifyCoords(goals)
    obstacles = IntifyCoords(obstacles)

    bounds = And([And([IsOnGrid(step, gridsize) for step in path])
                  for path in plan])
    adjacency = And([IsConnected(path) for path in plan])
    init = And([SamePosition(path[0], starts[i])
                for i, path in enumerate(plan)])
    reach = And([Or([SamePosition(coord, goals[i]) for coord in path])
                 for i, path in enumerate(plan)])
    avoid = And([
                 Implies(
                     SamePosition(plan[i][k],
                                  plan[j][k]),
                     Equals(Int(i),
                            Int(j)))
                 for k in range(len(plan[0]))
                 for i in range(len(plan)) for j in range(len(plan))])
    obstacles = And([Not(SamePosition(coord, obstacle))
                     for path in plan
                     for coord in path
                     for obstacle in obstacles])

    return And(bounds, adjacency, init, reach, avoid, obstacles)


def IsAttack(attack, plan, gridsize, obstacles, safes):
    obstacles = IntifyCoords(obstacles)
    safes = IntifyCoords(safes)

    bounds = And([IsOnGrid(step, gridsize) for step in attack])
    adjacency = IsConnected(attack)
    init = Or([SamePosition(path[0], attack[0]) for path in plan])
    reach = Or([SamePosition(coord, safe)
                for coord in attack for safe in safes])
    avoid = And([
                 Implies(
                     SamePosition(path[i],
                                  attack[i]),
                     SamePosition(path[0],
                                  attack[0]))
                 for i in range(len(attack)) for path in plan])
    obstacles = And(
        [Not(SamePosition(coord, obstacle))
         for coord in attack for obstacle in obstacles])
    # i is the attacker, j the observer, k the timestep
    undetected = And([Implies(SamePosition(plan[i][0], attack[0]),
                              And([
                                  And(
                                      Implies(
                                          Adj(plan[j][k], plan[i][k]),
                                          SamePosition(coord, plan[i][k])),
                                      Implies(
                                          Adj(coord, plan[j][k]),
                                          SamePosition(coord, plan[i][k]))
                                  )
                                  for k, coord in enumerate(attack)
                                  for j in range(len(plan)) if j != i])
                              ) for i in range(len(plan))])

    return And(bounds, adjacency, init, reach, avoid, obstacles, undetected)


class GridWorld():

    def __init__(self, content, h_mult=1.5, search='linear', timeout=0):
        self.content = content
        self.h_mult = h_mult
        self.search = search
        @timeout_decorator.timeout(timeout)
        def timed_get_model(*args, **kwargs):
            return get_model(*args, **kwargs)
        self.get_model = timed_get_model
        logging.log(logging.INFO, 'Accepted by solver.')

    def run(self):
        R = len(self.content['starts'])        # robots
        N = self.content['N']        # grid size

        distance_to_target = []
        bfs = BFS(N, self.content['starts'],
                  self.content['obstacles'])
        for i, source in enumerate(bfs.starts):
            bfs.search(source)
            distance_to_target.append(
                bfs.G[tuple(self.content['goals'][i])]['d'])

        H_MIN = max(distance_to_target)  # horizon
        H_MAX = int(H_MIN * self.h_mult)
        H_MAX_ORIG = H_MAX
        logging.log(logging.INFO, 'BFS lower bound: {}.'.format(H_MIN))
        logging.log(logging.INFO, 'Setting H_MAX to h_mult * {}: {}'.format(H_MIN, H_MAX))
        if self.search == 'binary':
            H = (H_MAX - H_MIN) // 2 + H_MIN
        elif self.search == 'linear':
            H = H_MIN
        else:
            logging.log(logging.ERROR, 'Search method is not one of "linear", "binary".')
            exit()
        logging.log(logging.INFO, 'Beginning search with horizon: {}'.format(H))
        while H < H_MAX:
            # make the EF-SMT instance here
            plan = [[[Symbol('p_%d^%d-0' %
                             (i, j), INT), Symbol('p_%d^%d-1' %
                                                  (i, j), INT)]
                     for j in range(H)] for i in range(R)]
            attack = [[Symbol('a_%d-0' %
                              i, INT), Symbol('a_%d-1' %
                                              i, INT)]
                      for i in range(H)]

            formula = ForAll([symvar for coordinate in attack
                              for symvar in coordinate],
                             And(IsPlan(plan,
                                        N,
                                        self.content['starts'],
                                        self.content['goals'],
                                        self.content['safes'] +
                                        self.content['obstacles']),
                                 Not(IsAttack(attack,
                                              plan,
                                              N,
                                              self.content['obstacles'],
                                              self.content['safes']))))
            try:
                model = self.get_model(formula)
            except timeout_decorator.TimeoutError:
                model = None
            if model:
                control = [[[model.get_py_value(plan[i][j][0]),
                             model.get_py_value(plan[i][j][1])]
                            for j in range(H)] for i in range(R)]
                self.content['control'] = list(zip(*control))
                logging.log(logging.INFO, 'Found solution for horizon: {}'.format(H))
                if self.search == 'linear':
                    break
                else:
                    H_MAX = H
                    H = (H_MAX - H_MIN) // 2 + H_MIN
            else:
                logging.log(logging.INFO, 'UNSAT for H={}, updating H.'.format(H))
                if self.search == 'linear':
                    H += 1
                else:
                    H_MIN = H + 1
                    H = (H_MAX - H_MIN) // 2 + H_MIN

        if 'control' not in self.content:
            logging.log(logging.INFO, 'UNSAT for H_MAX={}.'.format(H_MAX_ORIG))


if __name__ == '__main__':

    parser = argparse.ArgumentParser(prog='APMAPF',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('experiment', help='Experiment json path')
    parser.add_argument('-t', '--timeout', help='Assume UNSAT for a given horizon after timeout (s)', default=0, type=int)
    parser.add_argument('-v', '--verbosity', help='Verbosity level', default=2, type=int)
    parser.add_argument('-m', help='Horizon multiplier', default=2.0, type=float)
    parser.add_argument('-s', '--search', help='Path length optimization strategy.', default='linear', choices=['linear','binary'])
    # DEBUG, INFO, WARNING, ERROR, CRITICAL
    args = parser.parse_args()
    logging.basicConfig(level=10*args.verbosity, format='%(asctime)s: %(levelname)s - %(message)s')
    if not os.path.isfile(args.experiment):
        logging.log(logging.ERROR, 'No experiment file found at "{}", exiting.'.format(args.experiment))
        exit()
    logging.log(logging.INFO, 'Reading "{}".'.format(args.experiment))
    with open(args.experiment) as f:
        content = json.load(f)
    grid = GridWorld(content, h_mult=args.m, search=args.search, timeout=args.timeout)
    grid.run()
    logging.log(logging.INFO, 'Writing "{}".'.format(args.experiment))
    with open(args.experiment, 'w') as f:
        json.dump(grid.content, f)
