#include "Core.h"

Core *initCore(Instruction_Memory *i_mem){
    Core *core = (Core *)malloc(sizeof(Core));
    core->clk = 0;
    core->PC = 0;
    core->instr_mem = i_mem;
    core->tick = tickFunc;

    // initialize reg file;
    core->reg_file[0] = 0; // hard-wire x0 to zero
    core->reg_file[2] = 1023; // sp points to last byte address in data mem
   return core;
}

bool tickFunc(Core *core){
    // (Step 1) Reading instruction from instruction memory
    unsigned instruction = core->instr_mem->instructions[core->PC / 4].instruction;
    
    // (Step 2) Setting control bits
    Signal opcode = (instruction & 0b1111111);
    ControlSignals control_signals;
    ControlUnit(opcode, &control_signals, instruction);

    // (Step 3) Read values from reg1 and reg2
    Signal reg1data, reg2data;
    readRegisters(instruction, &reg1data, &reg2data, core->reg_file);

    // (Step 4) Generate Immediate
    Signal immediate = ImmeGen((Signal) instruction);

    // (Step 5) Use mux to choose between reg2 or immediate for input to ALU    
    Signal ALU_operand1 = MUX(control_signals.ALUSrc, reg2data, immediate);

    // (Step 6) Setting ALU Control Unit bits
    Signal Funct7 = (instruction & 0XFE000000) >> 25;
    Signal Funct3 = (instruction & 0X7000) >> 12;
    Signal ALU_ctrl_signal = ALUControlUnit(control_signals.ALUOp, Funct7, Funct3);

    // (Step 7) Get result from ALU
    Signal ALU_result, zero;
    ALU(reg1data, ALU_operand1, ALU_ctrl_signal, &ALU_result, &zero);

    // (Step 8a) Get data from data memory (if applicable)
    Signal mem_data;
    readDataFromMemory(control_signals.MemRead, ALU_result, &mem_data, core->data_mem);

    // (Step 8b.) Write reg2 to data memory (if applicable)
    writeDataToMem(control_signals.MemWrite, ALU_result, reg2data, core->data_mem);

    // (Step 9) Write Data to register (if applicable)
    Signal new_reg_data = MUX(control_signals.MemtoReg, ALU_result, mem_data);
    new_reg_data = MUX(control_signals.Branch, new_reg_data, (core->PC+4)); //for jal - writes to x1 
    writeDataToReg(control_signals.RegWrite, instruction, new_reg_data, core->reg_file);

    // (Step 10) Increment PC 
    immediate = ShiftLeft1(immediate);
    Signal pc_add = MUX((control_signals.Branch && zero), (core->PC+4), (core->PC+immediate));
    Signal new_pc = MUX((control_signals.Branch && control_signals.ALUSrc), pc_add, reg1data); // for jalr
    core->PC = new_pc;

    ++core->clk;
    // Are we reaching the final instruction?
    if (core->PC > core->instr_mem->last->addr)
        return false;
    return true;
}

/* Register r/w operations */ 
void readRegisters(unsigned instruction, Signal *reg1, Signal *reg2, Register *reg_file){
    int reg1_idx = (instruction & 0XF8000) >> 15;
    *reg1 = reg_file[reg1_idx];
    int reg2_idx = (instruction & 0X1F00000) >> 20;
    *reg2 = reg_file[reg2_idx];
}

void writeDataToReg(Signal RegWrite, unsigned instruction, Signal data, Register *reg_file){
    if (RegWrite == 1){
        int reg_idx = (instruction & 0XF80) >> 7;
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
void ControlUnit(Signal opcode, ControlSignals *signals, unsigned instruction){
    // For R-type
    if (opcode == 51) {
        signals->ALUSrc = 0;
        signals->MemtoReg = 0;
        signals->RegWrite = 1;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 2;
    }
    // For SB-type
    else if (opcode == 0b1100011){
        signals->ALUSrc = 0;
        signals->MemtoReg = 0; // dont-care
        signals->RegWrite = 0;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 1;
        signals->ALUOp = 1;
    }
    // For I-type
    else if (opcode == 0b11 || opcode == 0b10011){
        signals->ALUSrc = 1;
        signals->RegWrite = 1;
        if (opcode == 0b11){ //ld
            signals->MemtoReg = 1;  
            signals->MemRead = 1; 
        }
        else if (opcode == 0b10011){ //addi, slli
            signals->MemtoReg = 0; 
            signals->MemRead = 0;
        }
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 0;
    }
    // For S-type 
    else if (opcode == 0b100011){
        signals->ALUSrc = 1;
        signals->MemtoReg = 0; //don't care
        signals->RegWrite = 0;
        signals->MemRead = 0;
        signals->MemWrite = 1;
        signals->Branch = 0;
        signals->ALUOp = 0; 
    }
    // For jalr 
    else if (opcode == 0b1100111){
        signals->ALUSrc = 1;
        signals->MemtoReg = 0;
        signals->RegWrite = 1;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 1;
        signals->ALUOp = 0;
    }
    // For jal
    else if (opcode == 0b1101111){
        signals->ALUSrc = 0; //
        signals->MemtoReg = 0; // dont care - muxed out
        signals->RegWrite = 1;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 1; // use this to mux on data for rd
        signals->ALUOp = 3; // automatic zero - just jump 
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
