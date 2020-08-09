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
    if (test == 1) {
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

        printf("\nValue of Register x9: %lld", core->reg_file[9]);
        printf("\nValue of Register x11: %lld", core->reg_file[11]);
        printf("\nValue of Register x10: %lld", core->reg_file[10]);
        printf("\nValue of Register x22: %lld", core->reg_file[22]);
        printf("\nSimulation is finished.\n");

        free(core);
    }
}
