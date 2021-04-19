"""CPU functionality."""

import os
import re
import sys

# Threaded Interrupt Timer Class
from interrupt_timer import InterruptTimer


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

        # Interrupt Enabled?, Mask and Status regs, Timer
        self.__ie = False
        self.__im = 5
        self.__is = 6
        self.__timer_int = 0b00000001
        self.__key_int = 0b00000010
        self.__it = InterruptTimer(1, self.handle_timer_interrupt)

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

        # Instruction Identifiers
        # Main ops
        self.NOP = 0b0000
        self.HLT = 0b0001
        self.LDI = 0b0010
        self.LD = 0b0011
        self.ST = 0b0100
        self.PUSH = 0b0101
        self.POP = 0b0110
        self.PRN = 0b0111
        self.PRA = 0b1000

        # ALU ops
        self.ADD = 0b0000
        self.SUB = 0b0001
        self.MUL = 0b0010
        self.DIV = 0b0011
        self.MOD = 0b0100
        self.INC = 0b0101
        self.DEC = 0b0110
        self.CMP = 0b0111
        self.AND = 0b1000
        self.NOT = 0b1001
        self.OR = 0b1010
        self.XOR = 0b1011
        self.SHL = 0b1100
        self.SHR = 0b1101

        # PC Mutator ops
        self.CALL = 0b0000
        self.RET = 0b0001
        self.INT = 0b0010
        self.IRET = 0b0011
        self.JMP = 0b0100
        self.JEQ = 0b0101
        self.JNE = 0b0110
        self.JGT = 0b0111
        self.JLT = 0b1000
        self.JLE = 0b1001
        self.JGE = 0b1010

        # Branch Tables
        # Main Branch Table
        self.main_branchtable = {}
        self.main_branchtable[self.NOP] = self.handle_nop
        self.main_branchtable[self.HLT] = self.handle_hlt
        self.main_branchtable[self.LDI] = self.handle_ldi
        self.main_branchtable[self.LD] = self.handle_ld
        self.main_branchtable[self.ST] = self.handle_st
        self.main_branchtable[self.PUSH] = self.handle_push
        self.main_branchtable[self.POP] = self.handle_pop
        self.main_branchtable[self.PRN] = self.handle_prn
        self.main_branchtable[self.PRA] = self.handle_pra

        # ALU Branch Table
        self.alu_branchtable = {}
        self.alu_branchtable[self.ADD] = self.handle_add
        self.alu_branchtable[self.SUB] = self.handle_sub
        self.alu_branchtable[self.MUL] = self.handle_mul
        self.alu_branchtable[self.DIV] = self.handle_div
        self.alu_branchtable[self.MOD] = self.handle_mod
        self.alu_branchtable[self.INC] = self.handle_inc
        self.alu_branchtable[self.DEC] = self.handle_dec
        self.alu_branchtable[self.CMP] = self.handle_cmp
        self.alu_branchtable[self.AND] = self.handle_and
        self.alu_branchtable[self.NOT] = self.handle_not
        self.alu_branchtable[self.OR] = self.handle_or
        self.alu_branchtable[self.XOR] = self.handle_xor
        self.alu_branchtable[self.SHL] = self.handle_shl
        self.alu_branchtable[self.SHR] = self.handle_shr

        # PC Mutator Branch Table
        self.pcm_branchtable = {}
        self.pcm_branchtable[self.CALL] = self.handle_call
        self.pcm_branchtable[self.RET] = self.handle_ret
        self.pcm_branchtable[self.INT] = self.handle_int
        self.pcm_branchtable[self.IRET] = self.handle_iret
        self.pcm_branchtable[self.JMP] = self.handle_jmp
        self.pcm_branchtable[self.JEQ] = self.handle_jeq
        self.pcm_branchtable[self.JNE] = self.handle_jne
        self.pcm_branchtable[self.JGT] = self.handle_jgt
        self.pcm_branchtable[self.JLT] = self.handle_jlt
        self.pcm_branchtable[self.JLE] = self.handle_jle
        self.pcm_branchtable[self.JGE] = self.handle_jge

    # Main op handlers
    def handle_nop(self, mov_pc):
        pass

    def handle_hlt(self, mov_pc):
        self.running = False
        self.__it.stop()
        sys.exit(0)

    def handle_ldi(self, mov_pc):
        [reg_idx, imm] = self.get_instr_args(mov_pc)
        self.reg[reg_idx] = imm

    def handle_ld(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] = self.ram_read(self.reg[reg_idx_b])

    def handle_st(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.ram_write(self.reg[reg_idx_b], self.reg[reg_idx_a])

    def handle_push(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        self.push_stack(self.reg[reg_idx])

    def handle_pop(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        self.reg[reg_idx] = self.pop_stack()

    def handle_prn(self, mov_pc):
        [arg] = self.get_instr_args(mov_pc)
        print(self.reg[arg])

    def handle_pra(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        print(chr(self.reg[reg_idx]))

    # ALU op handlers
    def handle_add(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] += self.reg[reg_idx_b]

    def handle_sub(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] -= self.reg[reg_idx_b]

    def handle_mul(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] *= self.reg[reg_idx_b]

    def handle_div(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] /= self.reg[reg_idx_b]

    def handle_mod(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] %= self.reg[reg_idx_b]

    def handle_inc(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        self.reg[reg_idx] += 1

    def handle_dec(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        self.reg[reg_idx] -= 1

    def handle_cmp(self, mov_pc):
        # cmp masks
        lt_mask = 0b100
        gt_mask = 0b10
        eq_mask = 0b1

        # Clear fl reg
        self.__fl = 0b0

        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)

        if self.reg[reg_idx_a] == self.reg[reg_idx_b]:
            self.__fl |= eq_mask
        elif self.reg[reg_idx_a] < self.reg[reg_idx_b]:
            self.__fl |= lt_mask
        elif self.reg[reg_idx_a] > self.reg[reg_idx_b]:
            self.__fl |= gt_mask
        else:
            self.__fl = 0b0

    def handle_and(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] &= self.reg[reg_idx_b]

    def handle_not(self, mov_pc):
        xor_mask = 0b11111111
        [reg_idx] = self.get_instr_args(mov_pc)
        self.reg[reg_idx] ^= xor_mask

    def handle_or(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] |= self.reg[reg_idx_b]

    def handle_xor(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] ^= self.reg[reg_idx_b]

    def handle_shl(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] << self.reg[reg_idx_b]

    def handle_shr(self, mov_pc):
        [reg_idx_a, reg_idx_b] = self.get_instr_args(mov_pc)
        self.reg[reg_idx_a] >> self.reg[reg_idx_b]

    # PC Mutator op handlers
    def handle_call(self, mov_pc):
        self.push_stack(self.pc + mov_pc)
        [reg_idx] = self.get_instr_args(mov_pc)
        self.pc = self.reg[reg_idx]

    def handle_ret(self, mov_pc):
        self.pc = self.pop_stack()

    def handle_int(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        self.__is |= self.reg[reg_idx]

    def handle_iret(self, mov_pc):
        # pop R6 through R0, FL reg, and PC off stack
        # in that order
        for i in range(6, -1, -1):
            self.reg[i] = self.pop_stack()

        self.__fl = self.pop_stack()
        self.pc = self.pop_stack()

        # re-enable interrupts
        self.__ie = True

    def handle_jmp(self, mov_pc):
        [reg_idx] = self.get_instr_args(mov_pc)
        self.pc = self.reg[reg_idx]

    def handle_jeq(self, mov_pc):
        pass

    def handle_jne(self, mov_pc):
        pass

    def handle_jgt(self, mov_pc):
        pass

    def handle_jlt(self, mov_pc):
        pass

    def handle_jle(self, mov_pc):
        pass

    def handle_jge(self, mov_pc):
        pass

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

    def push_stack(self, val):
        self.reg[self.__sp] -= 1
        self.ram_write(val, self.reg[self.__sp])

    def pop_stack(self):
        val = self.ram_read(self.reg[self.__sp])
        self.reg[self.__sp] += 1

        return val

    def handle_timer_interrupt(self):
        self.reg[self.__is] |= self.__timer_int

    def handle_key_interrupt(self):
        pass

    def alu(self, op, reg_a, reg_b):
        """ALU operations."""
        if op == self.ADD:
            self.reg[reg_a] += self.reg[reg_b]
        # elif op == "SUB": etc
        else:
            raise Exception("Unsupported ALU operation")

    def handle_pc_mutators(self, instruction):
        """Handles ops that mutate the Program Counter."""
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

        return mov_pc, is_alu, sets_pc, instr_ident

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(
            f"TRACE: %02X %02X %02X | %02X %02X %02X |"
            % (
                self.pc,
                self.__fl,
                self.__ie,
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
        self.running = True
        self.__ie = True
        self.__it.start()

        while self.running:
            # Trace for debugging
            # self.trace()

            # Interrupt check
            if self.__ie:
                masked_interrupts = self.reg[self.__im] & self.reg[self.__is]

                # check all bits of IS register
                for i in range(8):
                    if masked_interrupts & (1 << i):
                        # disable further interrupts
                        self.__ie = False

                        # clear IS reg
                        self.reg[self.__is] &= ~(1 << i)

                        # push PC reg, FL reg and R0 through R6 onto stack
                        # in that order
                        self.push_stack(self.pc)
                        self.push_stack(self.__fl)
                        for j in range(7):
                            self.push_stack(self.reg[j])

                        # set PC to vector from interrupt vector table
                        self.pc = self.ram_read(self.i0 + i)

                        # break out of IS reg check
                        break

            # Read instruction from memory
            self.ir = self.ram_read(self.pc)

            # decode instruction
            dec_instr = self.decode_prog_instr(self.ir)
            mov_pc, is_alu, sets_pc, instr_ident = dec_instr

            # Logic for ops
            try:
                if is_alu:
                    self.alu_branchtable[instr_ident](mov_pc)
                elif sets_pc:
                    self.pcm_branchtable[instr_ident](mov_pc)
                else:
                    self.main_branchtable[instr_ident](mov_pc)

            except KeyError:
                print("Invalid instruction")
                print("Trace:")
                print(self.trace())
                self.running = False
                self.__it.stop()
                sys.exit(1)

            finally:
                if not sets_pc:
                    self.pc += mov_pc
