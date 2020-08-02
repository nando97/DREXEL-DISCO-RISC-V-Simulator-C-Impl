#include <stdio.h>

#include "Core.h"
#include "Parser.h"

int main(int argc, const char *argv[])
{	
    if (argc != 2)
    {
        printf("Usage: %s %s\n", argv[0], "<trace-file>");

        return 0;
    }

    /* Task One */
    // TODO, (1) parse and translate all the assembly instructions into binary format;
    // (2) store the translated binary instructions into instruction memory.
    Instruction_Memory instr_mem;
    instr_mem.last = NULL;
    loadInstructions(&instr_mem, argv[1]);

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
