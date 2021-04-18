#!/usr/bin/env python3

"""Main."""

import sys

from cpu import CPU

if __name__ == "__main__":
    # Initialize CPU
    cpu = CPU()

    # Load instruction set
    if len(sys.argv) != 2:
        print(f"Usage: python3 {os.path.basename(__file__)} <path to instruction set>")
    else:
        cpu.load(sys.argv[1])

    # Run
    cpu.run()
