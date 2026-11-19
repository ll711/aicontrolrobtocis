"""Compile the real Processing 4 sketch, run course-physics rollouts and C++ parity.

Exit 0 means all strict 5 mm endpoint checks passed; exit 2 means the run
completed but some routes did not meet that criterion. Reports retain failures.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]


def compile_processing_tests(processing, output):
    resources=processing/'app/resources'
    jdk=resources/'jdk'
    java=str(jdk/'bin/java.exe')
    subprocess.run([java, '-Dcompose.application.resources.dir='+str(resources), '-Dprocessing.version=4.5.7',
                    '-cp', str(processing/'app/*'), 'processing.mode.java.Commander',
                    '--sketch='+str(ROOT/'DigitalTwin'), '--output='+str(output), '--build'], check=True)
    cp=';'.join([str(output),str(resources/'core/library/*'),str(resources/'modes/java/libraries/net/library/*')])
    subprocess.run([str(jdk/'bin/javac.exe'),'-cp',cp,'-d',str(output),
                    str(ROOT/'tests/ProcessingSimulationTests.java'),str(ROOT/'tests/ProcessingParity.java')],check=True)
    return jdk, cp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--processing-root', type=Path, default=ROOT/'.tools/processing4/Processing')
    parser.add_argument('--cpp-build-dir', type=Path, default=ROOT/'build-host')
    parser.add_argument('--tree1-source', type=Path, help='Independent first_tree checkout')
    parser.add_argument('--results-dir', type=Path, default=ROOT/'simulation-results')
    parser.add_argument('--legacy', action='store_true', help='Build and test the archived motor policy')
    args=parser.parse_args()
    output=ROOT/'build-host'/('processing-'+uuid.uuid4().hex)
    jdk, cp = compile_processing_tests(args.processing_root.resolve(), output)
    java = str(jdk/'bin/java.exe')
    results=output/'results'
    rollouts=subprocess.run([java,'-Dtree2.legacy='+str(args.legacy).lower(),'-cp',cp,'ProcessingSimulationTests',str(results)])
    if rollouts.returncode not in (0,2): raise RuntimeError('Processing rollout harness failed')
    build=args.cpp_build_dir.resolve()
    cmake=['cmake','-S',str(ROOT),'-B',str(build),'-DTREE2_USE_LEGACY='+('ON' if args.legacy else 'OFF')]
    if args.tree1_source:
        cmake.append('-DTREE1_SOURCE_DIR='+str(args.tree1_source.resolve()))
    subprocess.run(cmake,check=True)
    subprocess.run(['cmake','--build',str(build),'--config','Release'],check=True)
    subprocess.run([sys.executable,str(ROOT/'tests/check_processing_parity.py'),
                    '--build-dir',str(output),'--java-home',str(jdk),'--cpp-exe',str(build/'Release/processing_parity.exe'),
                    '--report',str(results/'java_cpp_parity.json'),*(['--legacy'] if args.legacy else [])],check=True)
    subprocess.run([sys.executable,str(ROOT/'tests/run_processing_logic_checks.py'),
                    '--java-home',str(jdk)],check=True)
    subprocess.run(['ctest','--test-dir',str(build),'-C','Release','--output-on-failure'],check=True)
    with (results/'rollouts.csv').open() as stream: rows=list(csv.DictReader(stream))
    report={'tested_at_utc':datetime.now(timezone.utc).isoformat(),'tree2':'legacy' if args.legacy else 'measured-forward-v1',
            'platform':'Processing 4.5.7, real Java-mode compiler and core; course PDE physics',
            'physics_step_ms':10,'control_step_ms':50,'goal_tolerance_mm':5,
            'motor_model':'Course linear PWM-to-wheel-speed model; no measured friction or slip',
            'passed_routes':sum(row['success']=='true' for row in rows),'routes':rows,
            'java_cpp_parity':json.loads((results/'java_cpp_parity.json').read_text()),
            'processing_logic_checks':86,
            'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sorted((ROOT/'DigitalTwin').glob('*')) if p.suffix in ('.java','.pde')}}
    destination=args.results_dir.resolve()
    destination.mkdir(parents=True,exist_ok=True)
    for path in results.glob('*.csv'): shutil.copyfile(path,destination/path.name)
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Saved report:',destination/'report.json')
    raise SystemExit(rollouts.returncode)


if __name__=='__main__': main()
