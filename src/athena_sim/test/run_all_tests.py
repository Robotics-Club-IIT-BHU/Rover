#!/usr/bin/env python3
"""
run_all_tests.py

Unified test runner for the athena_sim package.
Runs all unit tests in test/ without requiring ROS 2 or Gazebo binaries.
"""

import os
import sys
import unittest

def main():
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(test_dir)
    src_dir = os.path.dirname(pkg_dir)

    # Ensure python path includes the source trees
    sys.path.insert(0, pkg_dir)
    sys.path.insert(0, src_dir)

    loader = unittest.TestLoader()
    suite = loader.discover(start_dir=test_dir, pattern='test_*.py')

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    if result.wasSuccessful():
        print("\n=======================================================")
        print(">>> ALL ATHENA_SIM TESTS PASSED SUCCESSFULLY! (EXIT 0)")
        print("=======================================================\n")
        sys.exit(0)
    else:
        print("\n>>> SOME TESTS FAILED!")
        sys.exit(1)

if __name__ == '__main__':
    main()
