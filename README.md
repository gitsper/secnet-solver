# secnet

Discrete-space physical masquerade attack-proof multi-agent path-finding
planner described in our SafeThings 2019 paper "Resilience of
Multi-Robot Systems to Physical Masquerade Attacks."

### Install

Software was tested with python 3.8 and pip 19.3. First, to install the python dependencies:

```console
$ pip install -r requirements.txt
```

Install the solver:

```console
$ pysmt-install --z3
```

See software help and small example:

```console
$ python apmapf.py -h
usage: APMAPF [-h] [-t TIMEOUT] [-v VERBOSITY] [-m M] [-s {linear,binary}] experiment

positional arguments:
  experiment            Experiment json path

optional arguments:
  -h, --help            show this help message and exit
  -t TIMEOUT, --timeout TIMEOUT
                        Assume UNSAT for a given horizon after timeout (s) (default: 0)
  -v VERBOSITY, --verbosity VERBOSITY
                        Verbosity level (default: 2)
  -m M                  Horizon multiplier (default: 2.0)
  -s {linear,binary}, --search {linear,binary}
                        Path length optimization strategy. (default: linear)
$ python apmapf.py small.dat
2020-02-12 13:59:04,157: INFO - Reading "small.dat".
2020-02-12 13:59:04,165: INFO - Accepted by solver.
2020-02-12 13:59:04,168: INFO - BFS lower bound: 10.
2020-02-12 13:59:04,168: INFO - Setting H_MAX to h_mult * 10: 20
2020-02-12 13:59:04,168: INFO - Beginning search with horizon: 10
2020-02-12 13:59:04,597: INFO - UNSAT for H=10, updating H.
2020-02-12 13:59:04,767: INFO - UNSAT for H=11, updating H.
2020-02-12 13:59:04,930: INFO - UNSAT for H=12, updating H.
2020-02-12 13:59:05,232: INFO - UNSAT for H=13, updating H.
2020-02-12 13:59:05,541: INFO - UNSAT for H=14, updating H.
2020-02-12 13:59:06,893: INFO - Found solution for horizon: 15
2020-02-12 13:59:06,893: INFO - Writing "small.dat".
```

Input/output data format is a json file. Input files have the following required fields:

|field|type|description|
| - |- | -|
|N|integer|grid size|
|starts|length-R array of length-2 integer arrays|starting positions for each of the R robots|
|goals|length-R array of length-2 integer arrays|goal positions for each of the R robots|
|obstacles|length-k array of length-2 integer arrays|positions of the k obstacles in the environment|
|safes|length-k array of length-2 integer arrays|positions of the k safe zones in the environment|

If the solver succeeds in finding an attack-proof plan, the program will add a field to the json file:

|field|type|description|
|-|-|-|
|control|length-H array of length-R arrays of length-2 integer arrays|positions of each of the R robots for every time step 0<=t<H|
