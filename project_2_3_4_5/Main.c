#include <stdio.h>

#include "Core.h"
#include "Parser.h"

int main(int argc, const char *argv[])
{	
    int test;
    if (argc < 2 || argc > 3)
    {
        printf("Usage: %s %s %s\n", argv[0], "<trace-file>", "-testI");

        return 0;
    }
    else if (argc == 3){
        if (strcmp(argv[2], "-testI") == 0){
            test = 1;
        }
        else{
            test = 0;
        }
    }
    else{
        test = 0;
    }

    /* Task One */
    // TODO, (1) parse and translate all the assembly instructions into binary format;
    // (2) store the translated binary instructions into instruction memory.
    Instruction_Memory instr_mem;
    instr_mem.last = NULL;
    loadInstructions(&instr_mem, argv[1]);

    // (3) print all the instruction in binary
    if (test == 1) { // used to test parser
        unsigned PC = 0;
        while (1)
        {
            Instruction *instr = &(instr_mem.instructions[PC / 4]);
            printf("\nInstruction at PC: %u\n", PC);
            unsigned mask = (1 << 31);
            for (int i = 31; i >= 0; i--)
            {
                if (instr->instruction & mask) { printf("1 ");}
                else { printf("0 "); }

                mask >>= 1;
            }
            printf("\n");
            if (instr == instr_mem.last) { break; }
            PC += 4;
        }
    }
    else {
        /* Task Two */
        // TODO, implement Core.{h,c}
        Core *core = initCore(&instr_mem);

        /* Task Three - Simulation */
        while (core->tick(core));
        Signal out;
        Signal mat;
        Signal readData = 1;

        readDataFromMemory(readData, 0, &mat, core->data_mem);
        printf("\nValue of mem[0:7]: %lld", mat);
        readDataFromMemory(readData, 8, &mat, core->data_mem);
        printf("\nValue of mem[8:15]: %lld", mat);
        readDataFromMemory(readData, 16, &mat, core->data_mem);
        printf("\nValue of mem[16:23]: %lld", mat);
        readDataFromMemory(readData, 24, &mat, core->data_mem);
        printf("\nValue of mem[24:31]: %lld", mat);
        readDataFromMemory(readData, 32, &mat, core->data_mem);
        printf("\nValue of mem[32:39]: %lld", mat);
        readDataFromMemory(readData, 40, &mat, core->data_mem);
        printf("\nValue of mem[40:47]: %lld", mat);
        readDataFromMemory(readData, 48, &mat, core->data_mem);
        printf("\nValue of mem[48:53]: %lld", mat);
        readDataFromMemory(readData, 56, &mat, core->data_mem);
        printf("\nValue of mem[56:63]: %lld", mat);
        readDataFromMemory(readData, 64, &mat, core->data_mem);
        printf("\nValue of mem[64:71]: %lld", mat);
        readDataFromMemory(readData, 72, &mat, core->data_mem);
        printf("\nValue of mem[72:79]: %lld", mat);
        readDataFromMemory(readData, 80, &mat, core->data_mem);
        printf("\nValue of mem[80:87]: %lld", mat);
        readDataFromMemory(readData, 88, &mat, core->data_mem);
        printf("\nValue of mem[88:95]: %lld", mat);
        readDataFromMemory(readData, 96, &mat, core->data_mem);
        printf("\nValue of mem[96:103]: %lld", mat);
        readDataFromMemory(readData, 104, &mat, core->data_mem);
        printf("\nValue of mem[104:111]: %lld", mat);
        readDataFromMemory(readData, 112, &mat, core->data_mem);
        printf("\nValue of mem[112:119]: %lld", mat);
        readDataFromMemory(readData, 120, &mat, core->data_mem);
        printf("\nValue of mem[120:127]: %lld", mat);
        

        readDataFromMemory(readData, 128, &out, core->data_mem);
        printf("\nValue of mem[128:135]: %lld", out);
        readDataFromMemory(readData, 136, &out, core->data_mem);
        printf("\nValue of mem[136:143]: %lld", out);
        readDataFromMemory(readData, 144, &out, core->data_mem);
        printf("\nValue of mem[144:151]: %lld", out);
        readDataFromMemory(readData, 152, &out, core->data_mem);
        printf("\nValue of mem[152:159]: %lld", out);
        
       
       
    
        printf("\nSimulation is finished.\n");

        free(core);
    }
}
