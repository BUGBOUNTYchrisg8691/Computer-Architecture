class BranchTable:
    OP1 = 0b10101010
    OP2 = 0b11110000

    def __init__(self):
        self.branch_table = {}
        self.branch_table[OP1] = self.handle_op1
        self.branch_table[OP2] = self.handle_op2

