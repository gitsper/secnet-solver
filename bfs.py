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

from enum import Enum
from queue import Queue


class Color(Enum):
    white = 0
    gray = 1
    black = 2


class BFS():

    def __init__(self, N, starts, obstacles):
        self.starts = list(map(tuple, starts))
        self.obstacles = list(map(tuple, obstacles))
        self.G = {(i, j): {'d': float('inf'),
                           'color': Color.white,
                           'pi': None,
                           'adj': []}
                  for i in range(N) for j in range(N)}
        for vertex in self.G.keys():
            self.G[vertex]['adj'] += [(vertex[0]+u, vertex[1]+v)
                                      for u in range(-1, 2)
                                      for v in range(-1, 2)
                                      if (abs(u)+abs(v) == 1 and
                                          vertex[0]+u >= 0 and
                                          vertex[0]+u < N and
                                          vertex[1]+v >= 0 and
                                          vertex[1]+v < N and
                                          (vertex[0]+u, vertex[1]+v) not
                                          in self.obstacles)]
        self.Q = None

    def search(self, s):
        for vertex in self.G.keys():
            self.G[vertex]['color'] = Color.white
            self.G[vertex]['d'] = float('inf')
            self.G[vertex]['pi'] = None
        self.G[s]['color'] = Color.gray
        self.G[s]['d'] = 0
        self.Q = Queue()
        self.Q.put(s)
        while not self.Q.empty():
            u = self.Q.get()
            for v in self.G[u]['adj']:
                if self.G[v]['color'] == Color.white:
                    self.G[v]['color'] = Color.gray
                    self.G[v]['d'] = self.G[u]['d'] + 1
                    self.G[v]['pi'] = u
                    self.Q.put(v)
            self.G[u]['color'] = Color.black

if __name__ == '__main__':
    bfs = BFS(10, [[0, 0], [9, 0]], [[5, i] for i in range(9)])
    bfs.search((0, 0))

    print(bfs.G[(9, 0)])
