#include "Core.h"

Core *initCore(Instruction_Memory *i_mem){
    Core *core = (Core *)malloc(sizeof(Core));
    core->clk = 0;
    core->PC = 0;
    core->instr_mem = i_mem;
    core->tick = tickFunc;

    // FIXME, initialize data memory here.
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

    // FIXME, initialize reg file here.
    // initialize x25 = 4, x22 = 1, x10 = 4;
    core->reg_file[0] = 0; // hard-wire x0 to zero
    core->reg_file[10] = 4;
    core->reg_file[22] = 1;
    core->reg_file[25] = 4;
   return core;
}

// FIXME, implement this function
bool tickFunc(Core *core){
    // Steps may include
    // (Step 1) Reading instruction from instruction memory
    unsigned instruction = core->instr_mem->instructions[core->PC / 4].instruction;
    
    // (Step 2) ...
    
    // (Step N) Increment PC. FIXME, is it correct to always increment PC by 4?!
    core->PC += 4;

    ++core->clk;
    // Are we reaching the final instruction?
    if (core->PC > core->instr_mem->last->addr)
        return false;
    return true;
}

// FIXME (1). Control Unit. Refer to Figure 4.18.
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
}

// FIXME (2). ALU Control Unit. Refer to Figure 4.12.
Signal ALUControlUnit(Signal ALUOp, Signal Funct7, Signal Funct3){
    // For add
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 0){
        return 2;
    }
}

// FIXME (3). Imme. Generator
Signal ImmeGen(Signal input){

}

// FIXME (4). ALU
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
