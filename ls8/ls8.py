#!/usr/bin/env python3

"""Main."""

import sys
from cpu import *

cpu = CPU()

if len(sys.argv) == 1:
    print("You have not provided a program file as argument")
    print(f"Usage: python3 {os.path.basename(__file__)} <path to instruction set file>")
    sys.exit(1)
  
elif len(sys.argv) > 2:
    print("Too many arguments provided")
    print(f"Usage: python3 {os.path.basename(__file__)} <path to instruction set file>")
    sys.exit(1)
  
elif len(sys.argv) == 2:
    filename = sys.argv[1]
    cpu.load(filename)

else:
    print(f"Usage: python3 {os.path.basename(__file__)} <path to instruction set file>")
    sys.exit(1)
  
cpu.run()