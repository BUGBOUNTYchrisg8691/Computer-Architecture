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
        self.running = False
        self.pc = 0
        self.ir = None

        # Flags reg for CMP and masks
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
            self.ram_write(int(instruction, 2), address)
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

    def handle_alu(self, op, reg_a, reg_b):
        """ALU operations."""

        # ALU ops
        ADD = 0b0000
        SUB = 0b0001
        MUL = 0b0010
        DIV = 0b0011
        MOD = 0b0100
        INC = 0b0101
        DEC = 0b0110
        CMP = 0b0111
        AND = 0b1000
        NOT = 0b1001
        OR = 0b1010
        XOR = 0b1011
        SHL = 0b1100
        SHR = 0b1101

        if instr_ident == ADD:
            self.reg[reg_a] += self.reg[reg_b]
        # elif op == "SUB": etc
        else:
            raise Exception("Unsupported ALU operation")

    def handle_pc_mutators(self, instruction):
        """Handles ops that mutate the Program Counter."""
        # PC mutator ops
        CALL = 0b0000
        RET = 0b0001
        INT = 0b0010
        IRET = 0b0011
        JMP = 0b0100
        JEQ = 0b0101
        JNE = 0b0110
        JGT = 0b0111
        JLT = 0b1000
        JLE = 0b1001
        JGE = 0b1010

        pass

    def get_instr_args(self, mov_pc):
        args = []
        for i in range(self.pc + 1, self.pc + mov_pc, 1):
            args.append(self.ram_read(i))

        return args

    def decode_prog_instr(self, instruction):
        # Masks for decoding
        instr_ident_mask = 0b1111
        instr_arg_cnt_mask = 0b11
        single_bit_mask = 0b1

        # Shift bits appropriate amount and bitwise-& to get only needed bits
        mov_pc = ((instruction >> 6) & instr_arg_cnt_mask) + 1
        is_alu = (instruction >> 5) & single_bit_mask
        sets_pc = (instruction >> 4) & single_bit_mask
        instr_ident = instruction & instr_ident_mask

        # Possible option, don't know if this is beneficial yet
        # return {
        #     "mov_pc": mov_pc,
        #     "is_alu": is_alu,
        #     "sets_pc": sets_pc,
        #     "instr_ident": instr_ident,
        # }

        return mov_pc, is_alu, sets_pc, instr_ident

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
        # Other ops
        NOP = 0b0000
        HLT = 0b0001
        LDI = 0b0010
        LD = 0b0011
        ST = 0b0100
        PUSH = 0b0101
        POP = 0b0110
        PRN = 0b0111
        PRA = 0b1000

        self.running = True

        while self.running:
            # Read instruction from memory
            self.ir = self.ram_read(self.pc)

            # decode instruction
            dec_instr = self.decode_prog_instr(self.ir)
            mov_pc, is_alu, sets_pc, instr_ident = dec_instr

            # Logic for ops
            if instr_ident == HLT:
                self.running = False
                sys.exit(0)

            elif instr_ident == LDI:
                args = self.get_instr_args(mov_pc)
                self.reg_write(args[1], args[0])

            elif instr_ident == PRN:
                args = self.get_instr_args(mov_pc)
                print(self.reg_read(args[0]))

            else:
                print("Invalid instruction")
                print("Trace:")
                print(self.trace())
                self.running = False
                sys.exit(1)

            if not sets_pc:
                self.pc += mov_pc
