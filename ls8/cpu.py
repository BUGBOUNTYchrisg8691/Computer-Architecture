"""CPU functionality."""

import os
import re
import sys


class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256
        self.reg = [0] * 8
        self.pc = 0
        self.ir = None
        self.__fl = 0b000
        self.__ie = False
        self.__im = 5
        self.__is = 6
        self.__sp = 7

    def load(self, filename):
        """
        Load a program into memory using a path to an external file as a
        command line argument.
        """
        regex_patt = re.compile("[0-1]{8}")
        try:
            with open(filename, "r") as file:
                program = regex_patt.findall(file.read())

        except FileNotFoundError:
            print(
                f"Usage: python3 {os.path.basename(__file__)} "
                "<path to instruction set>"
            )
            sys.exit(2)

        # Load program into memory
        address = 0
        for instruction in program:
            self.ram[address] = instruction
            address += 1

    def ram_read(self, mar):
        pass

    def ram_write(self, mdr, mar):
        pass

    def alu(self, op, reg_a, reg_b):
        """ALU operations."""

        if op == "ADD":
            self.reg[reg_a] += self.reg[reg_b]
        # elif op == "SUB": etc
        else:
            raise Exception("Unsupported ALU operation")

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(
            f"TRACE: %02X | %02X %02X %02X |"
            % (
                self.pc,
                # self.fl,
                # self.ie,
                self.ram_read(self.pc),
                self.ram_read(self.pc + 1),
                self.ram_read(self.pc + 2),
            ),
            end="",
        )

        for i in range(8):
            print(" %02X" % self.reg[i], end="")

        print()

    def run(self):
        """Run the CPU."""
        print(self.ram)
