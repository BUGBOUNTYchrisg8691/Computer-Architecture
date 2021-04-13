#!/usr/bin/env python3

"""Main."""

import sys
from cpu import *

cpu = CPU()

if len(sys.argv) == 1:
    print("You have not provided a program file as argument")
    print("Running the hardcoded print8.ls8 program")
    cpu.load()
  
elif len(sys.argv) > 2:
    print("Too many arguments provided")
    sys.exit(1)
  
elif len(sys.argv) == 2:
    filename = sys.argv[1]
    cpu.load(filename)

else:
    print("Running the hardcoded print8.ls8 program")
    cpu.load()
  
cpu.run()