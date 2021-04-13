"""CPU functionality."""

import sys
import re
class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256
        self.reg = [0] * 8
        self.reg[7] = 0xF4
        self.pc = 0
        self.fl = 0

    def load(self, program_file=None):
        """Load a program into memory."""

        address = 0

        # For now, we've just hardcoded a program:
        if program_file is not None:
            program = self.parse_instructions(program_file)
        
        else:
            program = [
                # From print8.ls8
                0b10000010, # LDI R0,8
                0b00000000,
                0b00001000,
                0b01000111, # PRN R0
                0b00000000,
                0b00000001, # HLT
            ]

        print(program)
        for instruction in program:
            self.ram_write(instruction, address)
            # self.ram[address] = instruction
            address += 1


    def alu(self, op, reg_a, reg_b):
        """ALU operations."""

        if op == "ADD":
            self.reg[reg_a] += self.reg[reg_b]
        #elif op == "SUB": etc
        else:
            raise Exception("Unsupported ALU operation")

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(f"TRACE: %02X | %02X %02X %02X |" % (
            self.pc,
            #self.fl,
            #self.ie,
            self.ram_read(self.pc),
            self.ram_read(self.pc + 1),
            self.ram_read(self.pc + 2)
        ), end='')

        for i in range(8):
            print(" %02X" % self.reg[i], end='')

        print()
        
    def ram_read(self, mar):
        """
        Accepts a Memory Address Register and returns 
        the Memory Data Register stored there.
        """
        return self.ram[mar]
    
    def ram_write(self, mdr, mar):
        """
        Accepts a Memory Data Register and 
        a Memory Address Register and writes the MDR to
        the MAR in memory.
        """
        self.ram[mar] = mdr
        
    def parse_instructions(self, filename):
        """
        Accepts a file name and parses it to gather the
        binary program instruction set as an array of 8bit
        binary literals.
        """
        parsed = []
        regex_match = '[0-1]{8}'
        
        try:
            with open(filename, 'r') as file:
                for line in file.read().splitlines():
                    parsed.append([s for s in line.split(" ") if re.match(regex_match, s)])

            program = [i[0] for i in parsed if i]
            
        except FileNotFoundError:
            print("No such file... Exiting...")
            sys.exit(1)
            
        if len(program) > 0:
            return program
        else:
            return [
                # From print8.ls8
                0b10000010, # LDI R0,8
                0b00000000,
                0b00001000,
                0b01000111, # PRN R0
                0b00000000,
                0b00000001, # HLT
            ]
                
    def run(self):
        """Run the CPU."""
        ir = None
        running = True
        
        # Instruction cases
        LDI = 0b10000010
        PRN = 0b01000111
        HLT = 0b00000001
        
        while running:
            # Fetch the next instruction and store in
            # in the Instruction Register
            ir = self.ram_read(self.pc)
            
            # Decode the instruction
            if ir == HLT:
                running = False
                sys.exit(0)

            elif ir == LDI:
                mar = self.ram_read(self.pc + 1)
                mdr = self.ram_read(self.pc + 2)
                self.reg[mar] = mdr
                self.pc += 3
                
            elif ir == PRN:
                mar = self.ram_read(self.pc + 1)
                print(self.reg[mar])
                self.pc += 2
                
            else:
                print("Invalid instruction... Exiting...")
                running = False
                sys.exit(1)