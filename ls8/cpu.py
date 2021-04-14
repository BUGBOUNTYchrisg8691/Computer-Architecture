"""CPU functionality."""

import sys
import re
import os

class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256
        self.reg = [0] * 8
        self.reg[7] = 0xF4
        self.pc = 0
        self.fl = 0
        self.ie = 0
        self.stack = []
        self.sp = self.reg[7]

    def load(self, program_file=None):
        """Load a program into memory."""

        address = 0

        # For now, we've just hardcoded a program:
        if program_file is not None:
            program = self.parse_instructions(program_file)
        
        else:
            print(f"Usage: python3 {os.path.basename(__file__)} <path to instruction set file>")
            sys.exit(1)

        for instruction in program:
            self.ram_write(instruction, address)
            address += 1


    def alu(self, mov_pc, instr_ident):
        """ALU operations."""
        regs= []
        
        for i in range(self.pc + 1, self.pc + mov_pc):
            regs.append(i)
        
        # Instruction Identifiers
        ADD = 0b0000
        AND = 0b1000
        CMP = 0b0111
        DEC = 0b0110
        DIV = 0b0011
        INC = 0b0101
        MOD = 0b0100
        MUL = 0b0010
        NOT = 0b1001
        OR  = 0b1010
        SHL = 0b1100
        SHR = 0b1101
        SUB = 0b0001
        XOR = 0b1011
        
        # Maybe I will choose to decode these in this function
        # instead of passing the decoded parts in in a future
        # iteration
        # ADD = 0b10100000
        # AND = 0b10101000
        # CMP = 0b10100111
        # DEC = 0b01100110
        # DIV = 0b10100011
        # INC = 0b01100101
        # MOD = 0b10100100
        # MUL = 0b10100010
        # NOT = 0b01101001
        # OR  = 0b10101010
        # SHL = 0b10101100
        # SHR = 0b10101101
        # SUB = 0b10100001
        # XOR = 0b10101011

        if instr_ident == ADD:
            self.reg[self.ram[regs[0]]] += self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == AND:
            self.reg[self.ram[regs[0]]] &= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == DEC:
            self.reg[self.ram[regs[0]]] -= 0b00000001
            self.pc += mov_pc
            
        elif instr_ident == DIV:
            self.reg[self.ram[regs[0]]] /= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == INC:
            self.reg[self.ram[regs[0]]] += 0b00000001
            self.pc += mov_pc
            
        elif instr_ident == MOD:
            self.reg[self.ram[regs[0]]] %= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == MUL:
            self.reg[self.ram[regs[0]]] *= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == NOT:
            xor_mask = 0b11111111
            self.reg[self.ram[regs[0]]] = self.reg[self.ram[regs[0]]] ^ xor_mask
            self.pc += mov_pc
            
        elif instr_ident == OR:
            self.reg[self.ram[regs[0]]] |= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == SHL:
            self.reg[self.ram[regs[0]]] <<= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == SHR:
            self.reg[self.ram[regs[0]]] >>= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == SUB:
            self.reg[self.ram[regs[0]]] -= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        elif instr_ident == XOR:
            self.reg[self.ram[regs[0]]] ^= self.reg[self.ram[regs[1]]]
            self.pc += mov_pc
            
        else:
            raise Exception("Unsupported ALU operation")

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(f"TRACE: %02X %02X %02X | %02X %02X %02X |" % (
            self.pc,
            self.fl,
            self.ie,
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

            program = [int(i[0], 2) for i in parsed if i]
            
        except FileNotFoundError:
            print("No such file... Exiting...")
            sys.exit(2)
            
        return program
    
    def decode_instr(self, instr):
        mov_pc_mask = 0b00000011
        first_bit_instruction_mask = 0b00000001
        instr_ident_mask = 0b00001111
        is_alu = ((instr >> 5) & first_bit_instruction_mask)
        sets_pc = ((instr >> 4) & first_bit_instruction_mask)
        mov_pc = ((instr >> 6) & mov_pc_mask) + 1
        instr_ident = (instr & instr_ident_mask)
        
        return is_alu, sets_pc, mov_pc, instr_ident
                
    def run(self):
        """Run the CPU."""
        ir = None
        running = True
        
        # Instruction cases
        LDI  = 0b0010
        PRN  = 0b0111
        HLT  = 0b0001
        PUSH = 0b0101
        POP  = 0b0110
        
        while running:
            # self.trace()
            # Fetch the next instruction and store in
            # in the Instruction Register
            ir = self.ram_read(self.pc)
            is_alu, sets_pc, mov_pc, instr_ident = self.decode_instr(ir)
            
            # Decode the instruction
            if instr_ident == HLT:
                running = False
                sys.exit(0)
            
            elif is_alu:
                self.alu(mov_pc, instr_ident)

            elif instr_ident == LDI:
                reg_idx = self.ram_read(self.pc + 1)
                val = self.ram_read(self.pc + 2)
                self.reg[reg_idx] = val
                self.pc += mov_pc
                
            elif instr_ident == PRN:
                reg_idx = self.ram_read(self.pc + 1)
                print(self.reg[reg_idx])
                self.pc += mov_pc

            elif instr_ident == PUSH:
                reg_idx = self.ram_read(self.pc + 1)
                val = self.reg[reg_idx]
                self.sp -= 1
                self.ram_write(val, self.sp)
                self.pc += mov_pc

            elif instr_ident == POP:
                reg_idx = self.ram_read(self.pc + 1)
                self.reg[reg_idx] = self.ram_read(self.sp)
                self.sp += 1
                self.pc += mov_pc

            else:
                print("Invalid instruction... Exiting...")
                running = False
                sys.exit(1)