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

        # Flags reg for CMP
        self.__fl = 0b000
        self.__lt_mask = 0b100
        self.__gt_mask = 0b10
        self.__eq_mask = 0b1

        # Interrupt Enabled?, Mask and Status regs
        self.__ie = False
        self.__im = 5
        self.__is = 6

        # Stack Pointer reg
        self.__sp = 7
        self.reg[self.__sp] = 0xF4

        # Reserved
        self.ram_write("RESERVED", 0xF5)
        self.ram_write("RESERVED", 0xF6)
        self.ram_write("RESERVED", 0xF7)

        # Interrupt Vector Table
        self.i0 = 0xF8
        self.i1 = 0xF9
        self.i2 = 0xFA
        self.i3 = 0xFB
        self.i4 = 0xFC
        self.i5 = 0xFD
        self.i6 = 0xFE
        self.i7 = 0xFF

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
            print(f"File not found: `{sys.argv[1]}`")
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
        return self.ram[mar]

    def ram_write(self, mdr, mar):
        self.ram[mar] = mdr

    def reg_read(self, reg_idx):
        return self.reg[reg_idx]

    def reg_write(self, val, reg_idx):
        self.reg[reg_idx] = val

    def push_stack(self, val):
        self.reg[self.__sp] -= 1
        self.ram_write(val, self.reg[self.__sp])

    def pop_stack(self):
        val = self.ram_read(self.reg[self.__sp])
        self.reg[self.__sp] += 1

        return val

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
