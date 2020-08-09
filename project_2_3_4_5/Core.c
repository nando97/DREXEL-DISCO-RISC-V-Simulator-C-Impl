#include "Core.h"

Core *initCore(Instruction_Memory *i_mem){
    Core *core = (Core *)malloc(sizeof(Core));
    core->clk = 0;
    core->PC = 0;
    core->instr_mem = i_mem;
    core->tick = tickFunc;

    // array is uint64_t arr[] = {16, 128, 8, 4}
    // so, each element requires 8 bytes

    // arr[0] = 16
    core->data_mem[0] = 16; 
    core->data_mem[1] = 0; 
    core->data_mem[2] = 0; 
    core->data_mem[3] = 0; 
    core->data_mem[4] = 0; 
    core->data_mem[5] = 0; 
    core->data_mem[6] = 0; 
    core->data_mem[7] = 0; 

    // arr[1] = 128
    core->data_mem[8] = 128; 
    core->data_mem[9] = 0;
    core->data_mem[10] = 0; 
    core->data_mem[11] = 0; 
    core->data_mem[12] = 0; 
    core->data_mem[13] = 0; 
    core->data_mem[14] = 0; 
    core->data_mem[15] = 0; 

    // arr[2] = 8
    core->data_mem[16] = 8;
    core->data_mem[17] = 0; 
    core->data_mem[18] = 0; 
    core->data_mem[19] = 0; 
    core->data_mem[20] = 0; 
    core->data_mem[21] = 0; 
    core->data_mem[22] = 0; 
    core->data_mem[23] = 0; 

    // arr[3] = 4 
    core->data_mem[24] = 4;
    core->data_mem[25] = 0; 
    core->data_mem[26] = 0; 
    core->data_mem[27] = 0; 
    core->data_mem[28] = 0; 
    core->data_mem[29] = 0; 
    core->data_mem[30] = 0; 
    core->data_mem[31] = 0;

    // initialize reg file;
    core->reg_file[0] = 0; // hard-wire x0 to zero
    core->reg_file[2] = 1023; // sp points to last byte address in data mem
    core->reg_file[10] = 4;
    core->reg_file[22] = 1;
    core->reg_file[25] = 4;
   return core;
}

bool tickFunc(Core *core){
    // (Step 1) Reading instruction from instruction memory
    unsigned instruction = core->instr_mem->instructions[core->PC / 4].instruction;

    // (Step 2) Setting control bits
    Signal opcode = (instruction & 0X7F);
    ControlSignals control_signals;
    ControlUnit(opcode, &control_signals);

    // (Step 3) Read values from reg1 and reg2
    Signal reg1, reg2;
    readRegisters(instruction, &reg1, &reg2, core->reg_file);

    // (Step 4) Generate Immediate
    Signal immediate = ImmeGen((Signal) instruction);

    // (Step 5) Use mux to choose between reg2 or immediate for input to ALU    
    Signal ALU_operand1 = MUX(control_signals.ALUSrc, reg2, immediate);

    // (Step 6) Setting ALU Control Unit bits
    Signal Funct7 = (instruction & 0XFE000000) >> 25;
    Signal Funct3 = (instruction & 0X7000) >> 12;
    Signal ALU_ctrl_signal = ALUControlUnit(control_signals.ALUOp, Funct7, Funct3);

    // (Step 7) Get result from ALU
    Signal ALU_result, zero;
    ALU(reg1, ALU_operand1, ALU_ctrl_signal, &ALU_result, &zero);

    // (Step 8) get data from data memory (if applicable)
    Signal mem_data;
    readDataFromMemory(control_signals.MemRead, ALU_result, &mem_data, core->data_mem);

    // (Step 9) Write Data to register
    Signal reg_data = MUX(control_signals.MemtoReg, ALU_result, mem_data);
    writeDataToReg(control_signals.RegWrite, instruction, reg_data, core->reg_file);

    // (Step 10) Increment PC 
    Signal pc_immediate = ShiftLeft1(immediate);
    pc_immediate = MUX((control_signals.Branch && zero), 4, pc_immediate);
    core->PC += pc_immediate;

    ++core->clk;
    // Are we reaching the final instruction?
    if (core->PC > core->instr_mem->last->addr)
        return false;
    return true;
}

// Read data from specified registers
void readRegisters(unsigned instruction, Signal *reg1, Signal *reg2, Register *reg_file){
    int reg1_idx = (instruction & 0XF8000) >> 15;
    *reg1 = reg_file[reg1_idx];

    int reg2_idx = (instruction & 0X1F00000) >> 20;
    *reg2 = reg_file[reg2_idx];
}

// Read data from data memory
void readDataFromMemory(Signal MemRead, Signal mem_addr, Signal *mem_data, Byte *data_mem){
   if (MemRead == 1)
        *mem_data = (((Signal)data_mem[mem_addr+7] << 56) |
                    ((Signal)data_mem[mem_addr+6] << 48) |
                    ((Signal)data_mem[mem_addr+5] << 40) |
                    ((Signal)data_mem[mem_addr+4] << 32) |
                    ((Signal)data_mem[mem_addr+3] << 24) |
                    ((Signal)data_mem[mem_addr+2] << 16) |
                    ((Signal)data_mem[mem_addr+1] << 8) |
                    (Signal)data_mem[mem_addr]); 
}

// Write data to register
void writeDataToReg(Signal RegWrite, unsigned instruction, Signal data, Register *reg_file){
    if (RegWrite == 1){
        int reg_idx = (instruction & 0XF80) >> 7;
        reg_file[reg_idx] = data;
    }
}

// (1). Control Unit. Refer to Figure 4.18.
void ControlUnit(Signal input, ControlSignals *signals){
    // For R-type
    if (input == 51) {
        signals->ALUSrc = 0;
        signals->MemtoReg = 0;
        signals->RegWrite = 1;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 2;
    }
    // For SB-type
    else if (input == 103){
        signals->ALUSrc = 0;
        signals->MemtoReg = 0; // dont-care
        signals->RegWrite = 0;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 1;
        signals->ALUOp = 1;
    }
    // For I-type
    else if (input == 3 || input == 19){
        signals->ALUSrc = 1;
        signals->RegWrite = 1;
        if (input == 19){ //addi, slli
            signals->MemtoReg = 0; 
            signals->MemRead = 0;
        }
        else { //ld
            signals->MemtoReg = 1;  
            signals->MemRead = 1; 
        }
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 0;
    }
}

// (2). ALU Control Unit. Refer to Figure 4.12.
Signal ALUControlUnit(Signal ALUOp, Signal Funct7, Signal Funct3){
    // For add
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 0)
        return 2;
    // for slli
    else if (ALUOp == 0 && Funct3 == 1)
        return 4;
    // for ld and addi
    else if (ALUOp == 0)
        return 2;
    // for bne
    else if (ALUOp == 1)
        return 7;
}

// (3). Imme. Generator
Signal ImmeGen(Signal input){
    Signal imm;
    Signal mask = 0b1100000; // for the given set of instructions, only the two
                             // MSBs matter

    switch (input & mask){
        case 0: // load has bits 6 and 5 set to zero
            imm = (input & 0XFFF00000) >> 20;
            break;
        case 96: // branches have bits 6 and 5 set to one
            imm = ((input & 0XFE000000) >> 20) | ((input & 0X00000F80) >> 7); 
            break;
    }
    // sign extension
    if ((imm >> 11) == 1)
        imm = (imm | 0XFFFFFFFFFFFFF000);
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
    // For subtraction
    if (ALU_ctrl_signal == 6 || ALU_ctrl_signal == 7){
        *ALU_result = (input_0 - input_1);
        switch (ALU_ctrl_signal){
            case 6: //beq
                if (*ALU_result == 0)
                    *zero = 1;
                else
                    *zero = 0;
                break;
            case 7: //bne
               if (*ALU_result == 0)
                    *zero = 0;
                else
                    *zero = 1; 
                break;
        }
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
