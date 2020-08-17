#include "Core.h"

Core *initCore(Instruction_Memory *i_mem){
    Core *core = (Core *)malloc(sizeof(Core));
    core->st2_en = core->st3_en = core->st4_en = core->st5_en = core->last_instr = 0;
    core->st1_en = 1;
    core->clk = 0;
    core->PC = 0;
    core->instr_mem = i_mem;
    core->tick = tickFunc;
    core->ifid_reg = (IFIDRegister *)malloc(sizeof(IFIDRegister));
    core->idex_reg = (IDEXRegister *)malloc(sizeof(IDEXRegister));
    core->exmem_reg = (EXMEMRegister *)malloc(sizeof(EXMEMRegister));
    core->memwb_reg = (MEMWBRegister *)malloc(sizeof(MEMWBRegister));

    // 40(x1) = -63
    core->data_mem[40] = -63; 
    core->data_mem[41] = 0; 
    core->data_mem[42] = 0; 
    core->data_mem[43] = 0; 
    core->data_mem[44] = 0; 
    core->data_mem[45] = 0; 
    core->data_mem[46] = 0; 
    core->data_mem[47] = 0; 

    // 48(x1) = 63
    core->data_mem[48] = 63; 
    core->data_mem[49] = 0; 
    core->data_mem[50] = 0; 
    core->data_mem[51] = 0; 
    core->data_mem[52] = 0; 
    core->data_mem[53] = 0; 
    core->data_mem[54] = 0; 
    core->data_mem[55] = 0; 

    // initialize reg file;
    core->reg_file[0] = 0; // hard-wire x0 to zero
    core->reg_file[1] = 0;
    core->reg_file[2] = 10;
    core->reg_file[3] = -15;
    core->reg_file[4] = 20;
    core->reg_file[5] = 30;
    core->reg_file[6] = -35;


   return core;
}

bool tickFunc(Core *core){
    if (core->st5_en == 1) {
        // Stage 5 WB
        writeback(core->memwb_reg, core->reg_file);
        if (core->last_instr == 1 && core->st4_en == 0)
            return false;
    }
    if (core->st4_en == 1) {
        //Stage 4 MEM
        memoryAccess(core->exmem_reg, core->memwb_reg, core->data_mem);
        if (core->last_instr == 1 && core->st3_en == 0)
            core->st4_en = 0;
        else
            core->st5_en = 1;
    }
    if (core->st3_en == 1) {
        //Stage 3 EX
        execution(core->idex_reg, core->exmem_reg);
        if (core->last_instr == 1 && core->st2_en == 0)
            core->st3_en = 0;
        else
            core->st4_en = 1;
    }
    if (core->st2_en == 1) {
        //Stage 2 ID
        instructionDecode(core->ifid_reg, core->idex_reg, core->reg_file);
        if (core->last_instr == 1)
            core->st2_en = 0;
        else
            core->st3_en = 1;
    }
    if (core->st1_en == 1) {
        //Stage 1 IF
        instructionFetch(core->ifid_reg, core->instr_mem, core->PC);
        if (core->PC > core->instr_mem->last->addr){
            core->last_instr = 1;
            core->st1_en = 0;
        }
        else
            core->st2_en = 1;
    }
    Signal pc_add = MUX((core->exmem_reg->Branch && core->exmem_reg->Zero), (core->PC+4), (core->PC+core->exmem_reg->AddSum));
    core->PC = pc_add;
    ++core->clk;
    return true;
}

void instructionFetch(IFIDRegister *ifid_reg, Instruction_Memory *instr_mem, Addr PC) {
    ifid_reg->PC = PC;
    ifid_reg->instruction = instr_mem->instructions[PC / 4].instruction;
}

void instructionDecode(IFIDRegister *ifid_reg, IDEXRegister *idex_reg, Register *reg_file) {
    Signal opcode = (ifid_reg->instruction & 0b1111111);
    ControlUnit(opcode, idex_reg, ifid_reg->instruction);
    idex_reg->PC = ifid_reg->PC;
    readRegisters(ifid_reg->instruction, &idex_reg->ReadData1, &idex_reg->ReadData2, reg_file);
    idex_reg->Funct7 = (ifid_reg->instruction & 0XFE000000) >> 25;
    idex_reg->Funct3 = (ifid_reg->instruction & 0X7000) >> 12;
    idex_reg->Immediate = ImmeGen((Signal) ifid_reg->instruction);
    idex_reg->writeIndex = (ifid_reg->instruction & 0XF80) >> 7;
}

void execution(IDEXRegister *idex_reg, EXMEMRegister *exmem_reg) {
    exmem_reg->writeIndex = idex_reg->writeIndex;
    exmem_reg->MemWrite = idex_reg->MemWrite;
    exmem_reg->MemRead = idex_reg->MemRead;
    exmem_reg->Branch = idex_reg->Branch;
    exmem_reg->RegWrite = idex_reg->RegWrite;
    exmem_reg->MemtoReg = idex_reg->MemtoReg;
    exmem_reg->ReadData2 = idex_reg->ReadData2;
    Signal ALU_operand1 = MUX(idex_reg->ALUSrc, idex_reg->ReadData2, idex_reg->Immediate);
    Signal ALU_ctrl_signal = ALUControlUnit(idex_reg->ALUOp, idex_reg->Funct7, idex_reg->Funct3);
    ALU(idex_reg->ReadData1, ALU_operand1, ALU_ctrl_signal, &exmem_reg->ALU, &exmem_reg->Zero);
    Signal shifted = ShiftLeft1(idex_reg->Immediate);
    exmem_reg->AddSum = idex_reg->PC + shifted;
}

void memoryAccess(EXMEMRegister *exmem_reg, MEMWBRegister *memwb_reg, Byte *data_mem) {
    memwb_reg->writeIndex = exmem_reg->writeIndex;
    memwb_reg->RegWrite = exmem_reg->RegWrite;
    memwb_reg->MemtoReg = exmem_reg->MemtoReg;
    memwb_reg->ALU = exmem_reg->ALU;
    readDataFromMemory(exmem_reg->MemRead, exmem_reg->ALU, &memwb_reg->MemData, data_mem);
    writeDataToMem(exmem_reg->MemWrite, exmem_reg->ALU, exmem_reg->ReadData2, data_mem);
}

void writeback(MEMWBRegister *memwb_reg, Register *reg_file) {
    Signal new_reg_data = MUX(memwb_reg->MemtoReg, memwb_reg->ALU, memwb_reg->MemData);
    writeDataToReg(memwb_reg->RegWrite, memwb_reg->writeIndex, new_reg_data, reg_file);
}

/* Register r/w operations */ 
void readRegisters(unsigned instruction, Signal *reg1, Signal *reg2, Register *reg_file){
    int reg1_idx = (instruction & 0XF8000) >> 15;
    *reg1 = reg_file[reg1_idx];
    int reg2_idx = (instruction & 0X1F00000) >> 20;
    *reg2 = reg_file[reg2_idx];
}

void writeDataToReg(Signal RegWrite, int reg_idx, Signal data, Register *reg_file){
    if (RegWrite == 1){
        if (reg_idx != 0) { // we can't overwrite x0
            reg_file[reg_idx] = data;
        }
    }
}

/* Memory r/w operations */
void readDataFromMemory(Signal MemRead, Signal mem_addr, Signal *mem_data, Byte *data_mem){
/* operates as ld */
   if (MemRead == 1){
        *mem_data = 0;
        *mem_data = (((Signal)data_mem[mem_addr+7] << 56) |
                    ((Signal)data_mem[mem_addr+6] << 48) |
                    ((Signal)data_mem[mem_addr+5] << 40) |
                    ((Signal)data_mem[mem_addr+4] << 32) |
                    ((Signal)data_mem[mem_addr+3] << 24) |
                    ((Signal)data_mem[mem_addr+2] << 16) |
                    ((Signal)data_mem[mem_addr+1] << 8) |
                    (Signal)data_mem[mem_addr]); 
   }
}

void writeDataToMem(Signal MemWrite, Signal mem_addr, Signal data, Byte *data_mem){
/* operates as sd */
    if (MemWrite == 1){
        data_mem[mem_addr] = (data & 0xFF); // start with LSB
        data_mem[mem_addr+1] = (data >> 8) & 0xFF;
        data_mem[mem_addr+2] = (data >> 16) & 0xFF;
        data_mem[mem_addr+3] = (data >> 24) & 0xFF;
        data_mem[mem_addr+4] = (data >> 32) & 0xFF;
        data_mem[mem_addr+5] = (data >> 40) & 0xFF;
        data_mem[mem_addr+6] = (data >> 48) & 0xFF;
        data_mem[mem_addr+7] = (data >> 56) & 0xFF;
    }
}

// (1). Control Unit. Refer to Figure 4.18.
void ControlUnit(Signal opcode, IDEXRegister *idex_reg, unsigned instruction){
    // For R-type
    if (opcode == 51) {
        idex_reg->ALUSrc = 0;
        idex_reg->MemtoReg = 0;
        idex_reg->RegWrite = 1;
        idex_reg->MemRead = 0;
        idex_reg->MemWrite = 0;
        idex_reg->Branch = 0;
        idex_reg->ALUOp = 2;
    }
    // For SB-type
    else if (opcode == 0b1100011){
        idex_reg->ALUSrc = 0;
        idex_reg->MemtoReg = 0; // dont-care
        idex_reg->RegWrite = 0;
        idex_reg->MemRead = 0;
        idex_reg->MemWrite = 0;
        idex_reg->Branch = 1;
        idex_reg->ALUOp = 1;
    }
    // For I-type
    else if (opcode == 0b11 || opcode == 0b10011){
        idex_reg->ALUSrc = 1;
        idex_reg->RegWrite = 1;
        if (opcode == 0b11){ //ld
            idex_reg->MemtoReg = 1;  
            idex_reg->MemRead = 1; 
        }
        else if (opcode == 0b10011){ //addi, slli
            idex_reg->MemtoReg = 0; 
            idex_reg->MemRead = 0;
        }
        idex_reg->MemWrite = 0;
        idex_reg->Branch = 0;
        idex_reg->ALUOp = 0;
    }
    // For S-type 
    else if (opcode == 0b100011){
        idex_reg->ALUSrc = 1;
        idex_reg->MemtoReg = 0; //don't care
        idex_reg->RegWrite = 0;
        idex_reg->MemRead = 0;
        idex_reg->MemWrite = 1;
        idex_reg->Branch = 0;
        idex_reg->ALUOp = 0; 
    }
    // For jalr 
    else if (opcode == 0b1100111){
        idex_reg->ALUSrc = 1;
        idex_reg->MemtoReg = 0;
        idex_reg->RegWrite = 1;
        idex_reg->MemRead = 0;
        idex_reg->MemWrite = 0;
        idex_reg->Branch = 1;
        idex_reg->ALUOp = 0;
    }
    // For jal
    else if (opcode == 0b1101111){
        idex_reg->ALUSrc = 0; //
        idex_reg->MemtoReg = 0; // dont care - muxed out
        idex_reg->RegWrite = 1;
        idex_reg->MemRead = 0;
        idex_reg->MemWrite = 0;
        idex_reg->Branch = 1; // use this to mux on data for rd
        idex_reg->ALUOp = 3; // automatic zero - just jump 
    }
}

// (2). ALU Control Unit. Refer to Figure 4.12.
Signal ALUControlUnit(Signal ALUOp, Signal Funct7, Signal Funct3){
    // For add
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 0)
        return 2;
    // For sll
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 1)
        return 4;
    // for slli
    else if (ALUOp == 0 && Funct3 == 1)
        return 4;
    // for ld, sd, addi, and jalr, ALU should add
    else if (ALUOp == 0)
        return 2;
    // for beq
    else if (ALUOp == 1 && Funct3 == 0)
        return 6;
    // for bne (like beq, but flips zero bit)
    else if (ALUOp == 1 && Funct3 == 1)
        return 7;
    // for jal
    else if (ALUOp == 3)
        return 8;
}

// (3). Imme. Generator
Signal ImmeGen(Signal instr){
    Signal imm;
    Signal mask = 0b1111111; //mask for opcode

    switch (instr & mask){
        case 0b11: // mem ops I-types
        case 0b10011: // generic I-types
        case 0b1100111: //jalr
            imm = (instr & 0XFFF00000) >> 20;
            if ((imm >> 11) == 1)
                imm = (imm | 0XFFFFFFFFFFFFF000);
            break;
        case 0b1100011: // sb types
            imm = ((instr & 0XFE000000) >> 20) | ((instr & 0X00000F80) >> 7); 
            if ((imm >> 11) == 1)
                imm = (imm | 0XFFFFFFFFFFFFF000);
            break;
        case 0b1101111: // jal
            imm = instr >> 12;
            if ((imm >> 19) == 1)
                imm = (imm | 0XFFFFFFFFFFF00000);
            break;
        case 0b100011: // S-type
            imm = ((instr & 0XFE000000) >> 20) | ((instr & 0X00000F80) >> 7);
             // sign extension
            if ((imm >> 11) == 1)
                imm = (imm | 0XFFFFFFFFFFFFF000);
            break;
    }
    return imm;
}

// (4). ALU
void ALU(Signal input_0, Signal input_1, Signal ALU_ctrl_signal,
         Signal *ALU_result, Signal *zero){

    // For addition
    if (ALU_ctrl_signal == 2) {
        *ALU_result = (input_0 + input_1);
        if (*ALU_result == 0)
            *zero = 1;
        else
            *zero = 0;
    }
    // For shift left
    if (ALU_ctrl_signal == 4) {
        *ALU_result = input_0 << input_1;
        *zero = 0;
    }
    // For subtract and beq
    if (ALU_ctrl_signal == 6){
        *ALU_result = (input_0 - input_1);
        if (*ALU_result == 0)
            *zero = 1;
        else
            *zero = 0;
    }
    // For bne
    if (ALU_ctrl_signal == 7){
        *ALU_result = (input_0 - input_1);
        if (*ALU_result == 0)
            *zero = 0;
        else
            *zero = 1;
    }
    // For jal - we dont care about the ALU result; we just wanna jump
    if (ALU_ctrl_signal == 8){
        *ALU_result = 0;
        *zero = 1;
    }
}

// (4). MUX
Signal MUX(Signal sel, Signal input_0, Signal input_1){
    if (sel == 0) { return input_0; } else { return input_1; }
}

// (5). Add
Signal Add(Signal input_0, Signal input_1) {
    return (input_0 + input_1);
}

// (6). ShiftLeft1
Signal ShiftLeft1(Signal input) {
    return input << 1;
}
