#ifndef __CORE_H__
#define __CORE_H__

#include "Instruction_Memory.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define BOOL bool

typedef uint8_t Byte;
typedef int64_t Signal;
typedef int64_t Register;

typedef struct ControlSignals
{
    Signal Branch;
    Signal MemRead;
    Signal MemtoReg;
    Signal ALUOp;
    Signal MemWrite;
    Signal ALUSrc;
    Signal RegWrite;
}ControlSignals;

typedef struct HazardDetectionSignals
{
    Signal PCWrite;
    Signal IFIDWrite;
    Signal StallPipeline;
} HazardDetectionSignals;

typedef struct ForwardingSignals
{
    Signal ForwardA;
    Signal ForwardB;
} ForwardingSignals;

typedef struct IFIDRegister {
    Addr PC;
    unsigned instruction;
} IFIDRegister;

typedef struct IDEXRegister {
    ControlSignals CtrlSignal;
    Signal Funct7;
    Signal Funct3;
    Addr PC;
    Signal ReadData1;
    Signal ReadData2;
    Signal Immediate;
    int writeIndex;
    int Rs1;
    int Rs2;
} IDEXRegister;

typedef struct EXMEMRegister {
    Signal RegWrite;
    Signal MemtoReg;
    Signal MemRead;
    Signal Branch;
    Signal MemWrite;
    Signal ALU;
    Signal Zero;
    Signal ReadData2;
    Signal AddSum;
    int writeIndex;
} EXMEMRegister;

typedef struct MEMWBRegister {
    Signal RegWrite;
    Signal MemtoReg;
    Signal ALU;
    Signal MemData;
    int writeIndex;
} MEMWBRegister;

struct Core;
typedef struct Core Core;
typedef struct Core
{
    Tick clk; // Keep track of core clock
    Addr PC; // Keep track of program counter

    // What else you need? Data memory? Register file?
    Instruction_Memory *instr_mem;
   
    Byte data_mem[1024]; // data memory

    Register reg_file[32]; // register file.

    bool (*tick)(Core *core);

    IFIDRegister *ifid_reg;
    IDEXRegister *idex_reg;
    EXMEMRegister *exmem_reg;
    MEMWBRegister *memwb_reg;

    
    int st5_en, st4_en, st3_en, st2_en, st1_en, last_instr;
} Core;

Core *initCore(Instruction_Memory *i_mem);
bool tickFunc(Core *core);

void instructionFetch(IFIDRegister *ifid_reg, Instruction_Memory *instr_mem, Addr PC, Signal IFIDWrite);
void instructionDecode(IFIDRegister *ifid_reg, IDEXRegister *idex_reg, Register *reg_file, HazardDetectionSignals *hazard_signals);
void execution(IDEXRegister *idex_reg, EXMEMRegister *exmem_reg, MEMWBRegister *memwb_reg, Signal ALU_input);
void memoryAccess(EXMEMRegister *exmem_reg, MEMWBRegister *memwb_reg, Byte *data_mem);

// data r/w operations
void readRegisters(unsigned instruction, Signal *reg1, Signal *reg2, Register *reg_file);
void writeDataToReg(Signal RegWrite, int writeIndex, Signal data, Register *reg_file);
void readDataFromMemory(Signal MemRead, Signal mem_addr, Signal *mem_data, Byte *data_mem);
void writeDataToMem(Signal MemWrite, Signal mem_addr, Signal data, Byte *data_mem);
void writeback(MEMWBRegister *memwb_reg, Register *reg_file, Signal *mux_output);

// used in the ID stage
void HazardDetectionUnit(IFIDRegister *ifid_reg, IDEXRegister *idex_reg, HazardDetectionSignals *hazard_signals);
void Stall(ControlSignals *CtrlSignal);

// used in the EX stage
void ForwardingUnit(IDEXRegister *idex_reg, EXMEMRegister *exmem_reg,
                    MEMWBRegister *memwb_reg, ForwardingSignals *fw_signals);

// FIXME. Implement the following functions in Core.c
// FIXME (1). Control Unit.

void ControlUnit(Signal input, ControlSignals *signals, unsigned instruction);

// FIXME (2). ALU Control Unit.
Signal ALUControlUnit(Signal ALUOp, Signal Funct7, Signal Funct3);

// FIXME (3). Imme. Generator
Signal ImmeGen(Signal input);

// FIXME (4). ALU
void ALU(Signal input_0,
         Signal input_1,
         Signal ALU_ctrl_signal,
         Signal *ALU_result,
         Signal *zero);

// (4). MUX
Signal MUX(Signal sel,
           Signal input_0,
           Signal input_1);

// (5). Add
Signal Add(Signal input_0,
           Signal input_1);

// (6). ShiftLeft1
Signal ShiftLeft1(Signal input);

// (7). 4-2 mux
Signal MUX4_2(Signal sel, Signal input_0, Signal input_1, Signal input_2, Signal input_3);
#endif
