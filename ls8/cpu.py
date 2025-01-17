"""CPU functionality."""

# Maybe I will choose to decode these in the alu()
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

# Built-in imports
import os
import re
import sys

# Libraries
import pygame

# Custom dep imports
from interrupt_timer import InterruptTimer


class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256
        self.reg = [0] * 8

        # Program Counter (start at 0)
        self.PC = 0

        # CMP Flags
        # reg[4]
        self.FL = 4
        # Create FL reg masks
        self.LT_MASK = 0b100
        self.GT_MASK = 0b10
        self.EQ_MASK = 0b1

        # interrupt Mask and Status
        # Interrupts enabled (start enabled)
        self.IE = True
        # reg[5]
        self.IM = 5
        # reg[6]
        self.IS = 6

        # Stack Pointer
        # reg[7]
        self.SP = 7
        self.reg[self.SP] = 0xF4

        # Interrupt Vector Table
        self.I7 = 0xFF
        self.I6 = 0xFE
        self.I5 = 0xFD
        self.I4 = 0xFC
        self.I3 = 0xFB
        self.I2 = 0xFA
        self.I1 = 0xF9
        self.I0 = 0xF8

    def load(self, program_file=None):
        """Load a program into memory."""

        address = 0

        # For now, we've just hardcoded a program:
        if program_file is not None:
            program = self.parse_instructions(program_file)

        else:
            print(
                f"Usage: python3 {os.path.basename(__file__)} <path to "
                "instruction set file>"
            )
            sys.exit(1)

        for instruction in program:
            self.ram_write(instruction, address)
            address += 1

    def alu(self, mov_pc, instr_ident):
        """ALU operations."""
        operands = []

        for i in range(self.PC + 1, self.PC + mov_pc):
            operands.append(i)

        # Instruction Identifiers (remaining instr_ident `0b1111`)
        ADD = 0b0
        ADDI = 0b1110
        AND = 0b1000
        CMP = 0b111
        DEC = 0b110
        DIV = 0b11
        INC = 0b101
        MOD = 0b100
        MUL = 0b10
        NOT = 0b1001
        OR = 0b1010
        SHL = 0b1100
        SHR = 0b1101
        SUB = 0b1
        XOR = 0b1011

        if instr_ident == ADD:
            self.reg[self.ram_read(operands[0])] += self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == ADDI:
            self.reg[self.ram_read(operands[0])] += self.ram_read(operands[1])
            self.PC += mov_pc

        elif instr_ident == AND:
            self.reg[self.ram_read(operands[0])] &= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == CMP:
            # Clear FL reg
            self.reg[self.FL] = 0b0

            # Set L bit in FL reg
            if (
                self.reg[self.ram_read(operands[0])]
                < self.reg[self.ram_read(operands[1])]
            ):
                self.reg[self.FL] |= self.LT_MASK

            # Set G bit in FL reg
            elif (
                self.reg[self.ram_read(operands[0])]
                > self.reg[self.ram_read(operands[1])]
            ):
                self.reg[self.FL] |= self.GT_MASK

            # Set E bit in FL reg
            elif (
                self.reg[self.ram_read(operands[0])]
                == self.reg[self.ram_read(operands[1])]
            ):
                self.reg[self.FL] |= self.EQ_MASK

            self.PC += mov_pc

        elif instr_ident == DEC:
            self.reg[self.ram_read(operands[0])] -= 1
            self.PC += mov_pc

        elif instr_ident == DIV:
            self.reg[self.ram_read(operands[0])] /= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == INC:
            self.reg[self.ram_read(operands[0])] += 1
            self.PC += mov_pc

        elif instr_ident == MOD:
            self.reg[self.ram_read(operands[0])] %= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == MUL:
            self.reg[self.ram_read(operands[0])] *= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == NOT:
            xor_mask = 0b11111111
            self.reg[self.ram_read(operands[0])] = (
                self.reg[self.ram_read(operands[0])] ^ xor_mask
            )
            self.PC += mov_pc

        elif instr_ident == OR:
            self.reg[self.ram_read(operands[0])] |= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == SHL:
            self.reg[self.ram_read(operands[0])] <<= self.reg[
                self.ram_read(operands[1])
            ]
            self.PC += mov_pc

        elif instr_ident == SHR:
            self.reg[self.ram_read(operands[0])] >>= self.reg[
                self.ram_read(operands[1])
            ]
            self.PC += mov_pc

        elif instr_ident == SUB:
            self.reg[self.ram_read(operands[0])] -= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        elif instr_ident == XOR:
            self.reg[self.ram_read(operands[0])] ^= self.reg[self.ram_read(operands[1])]
            self.PC += mov_pc

        else:
            raise Exception("Unsupported ALU operation")

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(
            f"TRACE: PC: %02X | RAM@PC: %02X, RAM@PC+1: %02X, RAM@PC+2: %02X |"
            % (
                self.PC,
                # self.FL,
                # self.IS,
                self.ram_read(self.PC),
                self.ram_read(self.PC + 1),
                self.ram_read(self.PC + 2),
            ),
            end="",
        )

        for i in range(8):
            print(" %02X" % self.reg[i], end="")

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
        regex_match = "[0-1]{8}"

        try:
            with open(filename, "r") as file:
                for line in file.read().splitlines():
                    parsed.append(
                        [s for s in line.split(" ") if re.match(regex_match, s)]
                    )

            program = [int(i[0], 2) for i in parsed if i]

        except FileNotFoundError:
            print("No such file... Exiting...")
            sys.exit(2)

        return program

    def decode_instr(self, instr):
        """
        Accepts 8bit intruction byte literal and decodes info
        using masks, return is_alu instruction flag, if the
        instruction sets the program counter manually, the
        amount the program counter should move and the
        instruction identifier.
        """
        mov_pc_mask = 0b11
        first_bit_instruction_mask = 0b1
        instr_ident_mask = 0b1111
        is_alu = (instr >> 5) & first_bit_instruction_mask
        sets_pc = (instr >> 4) & first_bit_instruction_mask
        mov_pc = ((instr >> 6) & mov_pc_mask) + 1
        instr_ident = instr & instr_ident_mask

        return is_alu, sets_pc, mov_pc, instr_ident

    def set_IS_reg(self):
        """
        Sets 0 bit of IS every 1 seconds.
        """
        self.reg[self.IS] = 0b1

    def run(self):
        """Run the CPU."""
        ir = None
        running = True

        # InterruptTimer initialization
        it = InterruptTimer(1, self.set_IS_reg)
        it.start()

        # Keyboard Interrupt polling initialization

        # Instruction cases
        HLT = 0b1
        LD = 0b11
        LDI = 0b10
        POP = 0b110
        PRA = 0b1000
        PRN = 0b111
        PUSH = 0b101
        ST = 0b100

        # instructions that set pc manually
        CALL = 0b0
        INT = 0b10
        IRET = 0b11
        JEQ = 0b101
        JGE = 0b1010
        JGT = 0b0111
        JLE = 0b1001
        JLT = 0b1000
        JMP = 0b100
        JNE = 0b110
        RET = 0b1

        while running:
            # trace for debugging
            # self.trace()

            # interrupt check
            if self.reg[self.IS] and self.IE:
                masked_interrupts = self.reg[self.IM] & self.reg[self.IS]

                # check all 8 bits of the IS register
                for i in range(8):
                    interrupt_occurred = ((masked_interrupts >> i) & 1) == 1

                    if interrupt_occurred:
                        # if interrupt occurred, disable further interrupts
                        self.IE = False

                        # clear IS register
                        self.reg[self.IS] = 0b0

                        # push PC reg, FL reg, and R0 through R6 onto stack
                        # in that order
                        self.reg[self.SP] -= 1
                        self.ram_write(self.PC, self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[self.FL], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[0], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[1], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[2], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[3], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[4], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[self.IM], self.reg[self.SP])
                        self.reg[self.SP] -= 1
                        self.ram_write(self.reg[self.IS], self.reg[self.SP])

                        # PC is set to vector from interrupt vector table
                        self.PC = self.ram_read(self.I0)
                        break

            # Fetch the next instruction and store in
            # in the Instruction Register
            ir = self.ram_read(self.PC)

            # Decode the instruction
            is_alu, sets_pc, mov_pc, instr_ident = self.decode_instr(ir)

            # Checks sets_pc bit and then instruction identity bits
            # and then run logic per that instruction
            if sets_pc:
                if instr_ident == CALL:
                    self.reg[self.SP] -= 1
                    self.ram_write(self.PC + mov_pc, self.reg[self.SP])
                    reg_idx = self.ram_read(self.PC + 1)
                    self.PC = self.reg[reg_idx]

                elif instr_ident == INT:
                    n = self.ram_read(self.PC + 1)
                    self.reg[self.IS] |= n
                    self.PC += mov_pc

                elif instr_ident == IRET:
                    # Pop R6 through R0 and put them back in the registers
                    self.reg[self.IS] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1
                    self.reg[self.IM] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1
                    self.reg[4] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1
                    self.reg[3] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1
                    self.reg[2] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1
                    self.reg[1] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1
                    self.reg[0] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1

                    # Pop FL off the stack
                    self.reg[self.FL] = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1

                    # Pop return address off stack and set to PC reg
                    self.PC = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1

                    # Re-enable interrupts
                    self.IE = True

                elif instr_ident == JEQ:
                    if self.reg[self.FL] & self.EQ_MASK:
                        reg_idx = self.ram_read(self.PC + 1)
                        self.PC = self.reg[reg_idx]
                    else:
                        self.PC += mov_pc

                elif instr_ident == JGE:
                    if (
                        self.reg[self.FL] & self.GT_MASK
                        or self.reg[self.FL] & self.EQ_MASK
                    ):
                        reg_idx = self.ram_read(self.PC + 1)
                        self.PC = self.reg[reg_idx]
                    else:
                        self.PC += mov_pc

                elif instr_ident == JGT:
                    if self.reg[self.FL] & self.GT_MASK:
                        reg_idx = self.ram_read(self.PC + 1)
                        self.PC = self.reg[reg_idx]
                    else:
                        self.PC += mov_pc

                elif instr_ident == JLE:
                    if (
                        self.reg[self.FL] & self.LT_MASK
                        or self.reg[self.FL] & self.EQ_MASK
                    ):
                        reg_idx = self.ram_read(self.PC + 1)
                        self.PC = self.reg[reg_idx]
                    else:
                        self.PC += mov_pc

                elif instr_ident == JLT:
                    if self.reg[self.FL] & self.LT_MASK:
                        reg_idx = self.ram_read(self.PC + 1)
                        self.PC = self.reg[reg_idx]
                    else:
                        self.PC += mov_pc

                elif instr_ident == JMP:
                    reg_idx = self.ram_read(self.PC + 1)
                    self.PC = self.reg[reg_idx]

                elif instr_ident == JNE:
                    if not self.reg[self.FL] & self.EQ_MASK:
                        reg_idx = self.ram_read(self.PC + 1)
                        self.PC = self.reg[reg_idx]
                    else:
                        self.PC += mov_pc

                elif instr_ident == RET:
                    self.PC = self.ram_read(self.reg[self.SP])
                    self.reg[self.SP] += 1

            # Checks is_alu bit then sends the mov_pc bits and
            # instruction identifier bits to the alu method
            elif is_alu:
                self.alu(mov_pc, instr_ident)

            elif instr_ident == HLT:
                it.stop()
                running = False
                sys.exit(0)

            elif instr_ident == LD:
                reg_idx_a = self.ram_read(self.PC + 1)
                reg_idx_b = self.ram_read(self.PC + 2)
                self.reg[reg_idx_a] = self.ram_read(self.reg[reg_idx_b])
                self.PC += mov_pc

            elif instr_ident == LDI:
                reg_idx = self.ram_read(self.PC + 1)
                val = self.ram_read(self.PC + 2)
                self.reg[reg_idx] = val
                self.PC += mov_pc

            elif instr_ident == PRN:
                reg_idx = self.ram_read(self.PC + 1)
                print(self.reg[reg_idx])
                self.PC += mov_pc

            elif instr_ident == PUSH:
                reg_idx = self.ram_read(self.PC + 1)
                val = self.reg[reg_idx]
                self.reg[self.SP] -= 1
                self.ram_write(val, self.reg[self.SP])
                self.PC += mov_pc

            elif instr_ident == POP:
                reg_idx = self.ram_read(self.PC + 1)
                self.reg[reg_idx] = self.ram_read(self.reg[self.SP])
                self.reg[self.SP] += 1
                self.PC += mov_pc

            elif instr_ident == ST:
                reg_idx_a = self.ram_read(self.PC + 1)
                reg_idx_b = self.ram_read(self.PC + 2)
                self.ram_write(self.reg[reg_idx_b], self.reg[reg_idx_a])
                self.PC += mov_pc

            elif instr_ident == PRA:
                reg_idx = self.ram_read(self.PC + 1)
                print(chr(self.reg[reg_idx]))
                self.PC += mov_pc

            else:
                print("Invalid instruction... Exiting...")
                it.stop()
                running = False
                sys.exit(1)
