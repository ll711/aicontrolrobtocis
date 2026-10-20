"""Compare Java geometry/actions/PWM against the real C++ adapters."""
import argparse
import json
import math
from pathlib import Path
import random
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--java-home', type=Path, required=True)
    args = parser.parse_args()
    rng = random.Random(20261019)
    records = []
    paths = [((0, 0), (.3, 0)), ((0, 0), (.2, 0), (.2, .18)),
             ((0, 0), (.2, 0), (.2, .16), (0, .16), (0, 0)),
             ((0, 0), (.18, .18), (0, .18), (.18, 0), (.3, 0))]
    for path in paths:
        for repeat in range(5):
            records.append('R ' + str(len(path)) + ' ' + ' '.join(str(v) for p in path for v in p))
            for a, b in zip(path, path[1:]):
                steps = math.ceil(math.dist(a, b)/.001)
                for i in range(steps):
                    t = i/steps
                    x = a[0]+t*(b[0]-a[0])+rng.uniform(-.0001, .0001)
                    y = a[1]+t*(b[1]-a[1])+rng.uniform(-.0001, .0001)
                    yaw = math.atan2(b[1]-a[1], b[0]-a[0])
                    if repeat == 1: yaw += math.pi
                    elif repeat == 2: yaw += math.pi/2
                    elif repeat == 3: yaw += rng.uniform(-math.pi, math.pi)
                    records.append(f'P {x:.17g} {y:.17g} {yaw:.17g} {rng.randint(-100,100)} {rng.randint(-100,100)}')
            records.append(f'P {path[-1][0]} {path[-1][1]} 0 0 0')
    payload = '\n'.join(records)+'\n'
    java = subprocess.run([str(args.java_home/'bin/java.exe'), '-cp', str(args.build_dir),
                           'ProcessingParity'], input=payload, text=True, capture_output=True, check=True)
    cpp = subprocess.run([str(ROOT/'build-host/Release/processing_parity.exe')],
                         input=payload, text=True, capture_output=True, check=True)
    actual = [[float(v) for v in row.split()] for row in java.stdout.splitlines()]
    expected = [[float(v) for v in row.split()] for row in cpp.stdout.splitlines()]
    assert len(actual) == len(expected)
    maximum = 0
    for index, (ja, cc) in enumerate(zip(actual, expected)):
        assert len(ja) == len(cc) == 18
        for col, (a, b) in enumerate(zip(ja, cc)):
            if col in (0, 1, 2, 4, 5, 6):
                assert a == b, (index, col, a, b)
            else:
                maximum = max(maximum, abs(a-b))
                assert abs(a-b) <= 1e-10*(1+abs(b)), (index, col, a, b)
    report = {'passed': True, 'cases': len(actual), 'max_geometry_error': maximum,
              'discrete_mismatches': 0, 'coverage': 'continuous progress, crossings, closed paths, forward/reverse/rotation, random previous PWM, endpoints'}
    output = ROOT/'build-host/simulation-results/java_cpp_parity.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
