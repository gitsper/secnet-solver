__copyright__ = """
    Copyright 2020 Boston University Board of Trustees

    Author: Kacper Wardega
    Contributors: Max von Hippel
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

# We don't care about errors thrown during import of pysmt.*.
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

# Return to showing errors again.
sys.stderr = sys.__stderr__

from bfs import BFS

# Absolute value of x = Ite(x >= 0, x, -x) = if x >= 0 then x else -x
def Abs(x):
    return Ite(x >= 0, x, -x)

# c0, c1 are "adjacent" iff d_x(c0, c1) + d_y(c0, c1) <= 1.
# That is, if their Manhattan distance is <= 1. 
def Adj(c0, c1):
    return 1 >= Abs(c1[0] - c0[0]) + Abs(c1[1] - c0[1])

# coord is on the grid iff its x, y coordinates are bounded below by the origin
# (0, 0), and above by the width = height of the (square) grid.
def IsOnGrid(coord, gridsize):
    return And(
        coord[0] >= 0,
        coord[1] >= 0,
        coord[0] < gridsize,
        coord[1] < gridsize)

# The path is connected iff each point in the path is adjacent to its
# predecessor (if it has one) and successor (if it has one).
def IsConnected(path):
    return And([Adj(path[i-1], path[i]) for i in range(1, len(path))])

# c0 is literally the same position as c1 if their x and y coordinates are both
# equal.
def SamePosition(c0, c1):
    return And(Equals(c0[0], c1[0]), Equals(c0[1], c1[1]))

# IntifyCoords takes a vector art in (R^2)^n and returns the closest vector to
# it in (Z^2)^n.
def IntifyCoords(ary):
    return [[Int(c0), Int(c1)] for c0, c1 in ary]

# IsPlan decides if a plan is valid.  It takes 5 inputs:
#   plan      = (x_i)_{i=1}^R := a set of T-length paths <x_i^1,...,x_i^T>.    
#   gridsize  = sqrt(|W|)     := some integer 0 < n, s.t. W = Z_n x Z_n.
#   starts    = (S_i)_{i=1}^R := the initial locations occupied by the robots
#   goals     = (G_i)_{i=1}^R := the set of goal locations of the robots
#   obstacles = Omega         := the set of obstacles in the world.
def IsPlan(plan, gridsize, starts, goals, obstacles):

    # Begin by snapping all coordinates to N x N space.  This is correct 
    # according to the problem formulation given in Definition 1
    starts    = IntifyCoords(starts)
    goals     = IntifyCoords(goals)
    obstacles = IntifyCoords(obstacles)

    # (1) A i in N_R : x_i^1 = S_i
    init = And([SamePosition(path[0], starts[i])
                for i, path in enumerate(plan)])

    # (2) A i in N_r : A t in N_T : x_i^t in W = Z_n x Z_n
    bounds = And([And([IsOnGrid(step, gridsize) for step in path])
                  for path in plan])

    # (3) A i in N_R : A t in N_{T-1} : E u in U : delta(x_i^t, u) = x_i^{t+1}
    # Note this isn't exactly the same, but it's morally equivalent, just
    # modulo the knowledge of the function delta.
    adjacency = And([IsConnected(path) for path in plan])

    # (4) A i in N_R : E t in N_T : G_i in x_i^T
    reach = And([Or([SamePosition(coord, goals[i]) for coord in path])
                 for i, path in enumerate(plan)])

    # (5) A (i, j) in N_R x N_R : A t in N_T : 
    #       x_i^t \cap x_j^t \neq \emptyset => i = j
    avoid = And([
                 Implies(
                     SamePosition(plan[i][k],
                                  plan[j][k]),
                     Equals(Int(i),
                            Int(j)))
                 for k in range(len(plan[0]))
                 for i in range(len(plan)) for j in range(len(plan))])

    # (6) A i in N_R : A t in N_T : x_i^t \cap \Omega = \emptyset
    obstacles = And([Not(SamePosition(coord, obstacle))
                     for path in plan
                     for coord in path
                     for obstacle in obstacles])

    # valid(x) = (1) ^  (2)  ^   (3)   ^   (4)  ^  (5)  ^  (6)
    return And(init, bounds, adjacency, reach, avoid, obstacles)

# Takes as inputs attack, plan, gridsize, obstacles, safes; where:
# 
#   > attack is a finite list of coordinates on Z_gridsize x Z_gridsize;
#   > plan consists of paths (like attack) for all the non-adversarial bots;
#   > the attack and the paths in plan are over the space Z_gridsize x Z_gridsize;
#   > obstacles is a set of "off-limits" grid coordinates where no bot (or 
#     attacker can go; and
#   > safes is a set of coordinates the attacker may pass through undetected.
# 
# Returns the attack validity, i.e., (1) ^ (2) ^ (3) ^ (4) ^ (5) ^ (6) ^ (7).
def IsAttack(attack, plan, gridsize, obstacles, safes):
    obstacles = IntifyCoords(obstacles)
    safes = IntifyCoords(safes)

    # (1) All attack moves are in Z_gridsize x Z_gridsize.
    bounds = And([IsOnGrid(step, gridsize) for step in attack])

    # (2) The attacker never moves more than 1 grid-square at a time.
    adjacency = IsConnected(attack)

    # (3) The attack begins in the same place as some path in the plan.
    init = Or([SamePosition(path[0], attack[0]) for path in plan])

    # (4) The attack enters a "safe" zone at some point.
    reach = Or([SamePosition(coord, safe)
                for coord in attack for safe in safes])
    
    # (5) The attack does not cross paths with any plans other than the one it
    #     replaces.
    avoid = And([
                 Implies(
                     SamePosition(path[i],
                                  attack[i]),
                     SamePosition(path[0],
                                  attack[0]))
                 for i in range(len(attack)) for path in plan])
    
    # (6) The attack never runs into any obstacle.
    obstacles = And(
        [Not(SamePosition(coord, obstacle))
         for coord in attack for obstacle in obstacles])

    # (7) Any time the attack is adjascent to some plan p, it is in the same 
    #     place as another plan p' and therefore could be mistaken for the
    #     innocent plan p'.
    #     i is the attacker, j the observer, k the time-step.
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
        R = len(self.content['starts']) # robots
        N = self.content['N']           # grid size

        # We use a breadth-first search to determine the horizon, i.e., 
        # upper complexity bound, of the problem, based on the maximum distance
        # of a goal from the start positions of the paths.
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

        logging.log(logging.INFO, 
                    'BFS lower bound: {}.'.format(H_MIN))

        logging.log(logging.INFO, 
                    'Setting H_MAX to h_mult * {}: {}'.format(H_MIN, H_MAX))

        if self.search == 'binary':
            H = (H_MAX - H_MIN) // 2 + H_MIN
        elif self.search == 'linear':
            H = H_MIN
        else:
            logging.log(logging.ERROR, 
                        'Search method is not one of "linear", "binary".')
            exit()
        
        logging.log(logging.INFO, 'Beginning search with horizon: {}'.format(H))
        
        # H denotes the complexity we have checked 'up to', and H_MAX the
        # complexity at which we can terminate.
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

            # formula says that forall components of all coordinates in the
            # attack, the plan is a valid plan, but the attack is not a valid
            # attack.  So, a violation (which z3 would detect) would mean an
            # attack.
            # @Kacper can you double check this logic?
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
            # If a model, i.e., a satisfaction of the formula, is recoverable,
            # then we produce the solution.  This is of course within the H_MAX
            # complexity bound.
            if model:
                control = [[[model.get_py_value(plan[i][j][0]),
                             model.get_py_value(plan[i][j][1])]
                            for j in range(H)] for i in range(R)]
                self.content['control'] = list(zip(*control))
                logging.log(logging.INFO, 
                            'Found solution for horizon: {}'.format(H))
                if self.search == 'linear':
                    break
                else:
                    H_MAX = H
                    H = (H_MAX - H_MIN) // 2 + H_MIN
            else:
                logging.log(logging.INFO, 
                            'UNSAT for H={}, updating H.'.format(H))
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

    timeoutHelp = 'Assume UNSAT for a given horizon after timeout (s)'
    searchHelp  = 'Path length optimization strategy.'
    
    # Self-explanatory.
    parser.add_argument('experiment', 
                        help='Experiment json path')
    parser.add_argument('-t', 
                        '--timeout', 
                        help=timeoutHelp, 
                        default=0, 
                        type=int)
    parser.add_argument('-v', 
                        '--verbosity', 
                        help='Verbosity level', 
                        default=2, 
                        type=int)
    parser.add_argument('-m', 
                        help='Horizon multiplier', 
                        default=2.0, 
                        type=float)
    parser.add_argument('-s', 
                        '--search', 
                        help=searchHelp, 
                        default='linear', 
                        choices=['linear','binary'])
    
    # DEBUG, INFO, WARNING, ERROR, CRITICAL
    args = parser.parse_args()
    logging.basicConfig(level=10*args.verbosity, 
                        format='%(asctime)s: %(levelname)s - %(message)s')
    if not os.path.isfile(args.experiment):
        logging.log(logging.ERROR, 
                    'No experiment file found at "{}", exiting.'\
                    .format(args.experiment))
        exit()
    logging.log(logging.INFO, 'Reading "{}".'.format(args.experiment))
    with open(args.experiment) as f:
        content = json.load(f)
    grid = GridWorld(content, 
                     h_mult=args.m, 
                     search=args.search, 
                     timeout=args.timeout)
    grid.run()
    logging.log(logging.INFO, 'Writing "{}".'.format(args.experiment))
    with open(args.experiment, 'w') as f:
        json.dump(grid.content, f)
