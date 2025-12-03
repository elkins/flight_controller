#!/usr/bin/env python3
"""
Advanced Flight Scenario Tests

Runs comprehensive flight tests including:
- Takeoff and landing
- Circle pattern with rotation
- Figure-8 aerobatics
- Square waypoint navigation
- Triangle waypoint course
- Emergency landing procedures

Usage:
    python examples/test_advanced_scenarios.py [--gui] [--scenario SCENARIO]
    
Scenarios:
    all       - Run all tests sequentially (default)
    landing   - Test safe landing from hover
    circle    - Rotating hover pattern
    figure8   - Figure-8 aerobatic pattern
    square    - Square waypoint course
    waypoint  - Triangle waypoint navigation
"""

import sys
import os
import time
import subprocess
import argparse

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


SCENARIOS = {
    'landing': {
        'name': 'Safe Landing Test',
        'description': 'Takeoff, hover, and controlled landing',
        'duration': 25,
        'mode': 'hover',
        'expected': 'Smooth descent with <0.3 m/s touchdown'
    },
    'circle': {
        'name': 'Rotating Circle',
        'description': 'Hover with 360° rotation',
        'duration': 35,
        'mode': 'circle',
        'expected': 'Stable altitude while rotating heading'
    },
    'figure8': {
        'name': 'Figure-8 Aerobatics',
        'description': 'Figure-8 pattern with banked turns',
        'duration': 45,
        'mode': 'figure8',
        'expected': 'Smooth coordinated turns with gentle banking'
    },
    'square': {
        'name': 'Square Waypoint Course',
        'description': '4-leg square pattern with 90° turns',
        'duration': 50,
        'mode': 'square',
        'expected': 'Precise heading changes at each corner'
    },
    'waypoint': {
        'name': 'Triangle Waypoint Navigation',
        'description': '3-point triangle course',
        'duration': 50,
        'mode': 'waypoint',
        'expected': '120° heading changes at each waypoint'
    }
}


def run_scenario(scenario_name, gui=True, verbose=True):
    """Run a single flight scenario"""
    if scenario_name not in SCENARIOS:
        print(f"✗ Unknown scenario: {scenario_name}")
        return False
    
    scenario = SCENARIOS[scenario_name]
    
    print(f"\n{'='*70}")
    print(f"  {scenario['name'].upper()}")
    print(f"{'='*70}")
    print(f"Description: {scenario['description']}")
    print(f"Duration:    {scenario['duration']}s")
    print(f"Mode:        {scenario['mode']}")
    print(f"Expected:    {scenario['expected']}")
    print(f"{'='*70}\n")
    
    # Build command
    cmd = [
        'python',
        'examples/pybullet_qgroundcontrol.py',
        '--duration', str(scenario['duration']),
        '--mode', scenario['mode'],
        '--auto-start'
    ]
    
    if gui:
        cmd.append('--gui')
    
    # Run simulation
    try:
        print(f"🚁 Starting {scenario['name']}...\n")
        result = subprocess.run(cmd, check=True, capture_output=not verbose)
        
        if result.returncode == 0:
            print(f"\n✓ {scenario['name']} completed successfully!\n")
            return True
        else:
            print(f"\n✗ {scenario['name']} failed with code {result.returncode}\n")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"\n✗ {scenario['name']} crashed: {e}\n")
        return False
    except KeyboardInterrupt:
        print(f"\n⚠ {scenario['name']} interrupted by user\n")
        return False


def run_all_scenarios(gui=True):
    """Run all scenarios sequentially"""
    print("\n" + "="*70)
    print("  ADVANCED FLIGHT SCENARIO TEST SUITE")
    print("="*70)
    print(f"Total scenarios: {len(SCENARIOS)}")
    print(f"GUI mode: {'Enabled' if gui else 'Disabled'}")
    print("="*70)
    
    results = {}
    start_time = time.time()
    
    for scenario_name in ['landing', 'circle', 'figure8', 'square', 'waypoint']:
        success = run_scenario(scenario_name, gui=gui, verbose=True)
        results[scenario_name] = success
        
        if not success:
            print(f"\n⚠ Stopping test suite after failure in {scenario_name}")
            break
        
        # Brief pause between scenarios
        if scenario_name != 'waypoint':  # Don't pause after last test
            print("\nWaiting 3 seconds before next scenario...")
            time.sleep(3)
    
    # Summary
    elapsed = time.time() - start_time
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    print("\n" + "="*70)
    print("  TEST SUMMARY")
    print("="*70)
    for scenario_name, success in results.items():
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"{status}  {SCENARIOS[scenario_name]['name']}")
    print("="*70)
    print(f"Results: {passed}/{total} scenarios passed")
    print(f"Time: {elapsed:.1f}s")
    print("="*70 + "\n")
    
    return passed == total


def main():
    parser = argparse.ArgumentParser(
        description='Advanced flight scenario tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--scenario',
        choices=['all'] + list(SCENARIOS.keys()),
        default='all',
        help='Scenario to run (default: all)'
    )
    parser.add_argument(
        '--gui',
        action='store_true',
        default=True,
        help='Show PyBullet GUI (default: True)'
    )
    parser.add_argument(
        '--headless',
        action='store_true',
        help='Run without GUI (faster)'
    )
    
    args = parser.parse_args()
    
    # Handle GUI flag
    gui = args.gui and not args.headless
    
    # Run scenarios
    if args.scenario == 'all':
        success = run_all_scenarios(gui=gui)
    else:
        success = run_scenario(args.scenario, gui=gui, verbose=True)
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
