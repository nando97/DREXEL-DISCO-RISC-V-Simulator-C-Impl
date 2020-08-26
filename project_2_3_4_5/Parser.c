#include <stdio.h>
#include "Parser.h"

// FIXME, implement this function.
// Here shows an example on how to translate "add x10, x10, x25"
void loadInstructions(Instruction_Memory *i_mem, const char *trace)
{
    printf("Loading trace file: %s\n", trace);

    FILE *fd = fopen(trace, "r");
    if (fd == NULL)
    {
        perror("Cannot open trace file. \n");
        exit(EXIT_FAILURE);
    }

    // Iterate all the assembly instructions
    char *line = NULL;
    size_t len = 0;
    ssize_t read;

    Addr PC = 0; // program counter points to the zeroth location initially.
    int IMEM_index = 0;
    while ((read = getline(&line, &len, fd)) != -1)
    {
        // Assign program counter
        i_mem->instructions[IMEM_index].addr = PC;

        // Extract operation
        char *raw_instr = strtok(line, " ");

        //R-types
        if (strcmp(raw_instr, "add") == 0 ||
            strcmp(raw_instr, "sub") == 0 ||
            strcmp(raw_instr, "sll") == 0 ||
            strcmp(raw_instr, "srl") == 0 ||
            strcmp(raw_instr, "xor") == 0 ||
            strcmp(raw_instr, "or")  == 0 ||
            strcmp(raw_instr, "and") == 0)
        {
            parseRType(raw_instr, &(i_mem->instructions[IMEM_index]));
            i_mem->last = &(i_mem->instructions[IMEM_index]);
	    }
        //I-types
        else if (strcmp(raw_instr, "lb") == 0 ||
            strcmp(raw_instr, "lh") == 0 ||
            strcmp(raw_instr, "ld") == 0 ||
            strcmp(raw_instr, "lw") == 0 ||
            strcmp(raw_instr, "slli") == 0 ||
            strcmp(raw_instr, "srli")  == 0 ||
            strcmp(raw_instr, "addi") == 0 ||
            strcmp(raw_instr, "jalr") == 0 )
        {
            parseIType(raw_instr, &(i_mem->instructions[IMEM_index]));
            i_mem->last = &(i_mem->instructions[IMEM_index]);
	    } 
        //SB-types
        else if (strcmp(raw_instr, "beq") == 0 ||
            strcmp(raw_instr, "bne") == 0 ||
            strcmp(raw_instr, "blt") == 0 ||
            strcmp(raw_instr, "bge") == 0 ||
            strcmp(raw_instr, "bltu") == 0 ||
            strcmp(raw_instr, "bgeu")  == 0)
        {
            parseSBType(raw_instr, &(i_mem->instructions[IMEM_index]));
            i_mem->last = &(i_mem->instructions[IMEM_index]);
	    }
        //S-types
        else if (strcmp(raw_instr, "sb") == 0 ||
            strcmp(raw_instr, "sh") == 0 ||
            strcmp(raw_instr, "sb") == 0 ||
            strcmp(raw_instr, "sw") == 0 ||
            strcmp(raw_instr, "sd") == 0 )
        {
            parseSType(raw_instr, &(i_mem->instructions[IMEM_index]));
            i_mem->last = &(i_mem->instructions[IMEM_index]);
	    }
        else if (strcmp(raw_instr, "jal") == 0)
        {
            parseUJType(raw_instr, &(i_mem->instructions[IMEM_index]));
            i_mem->last = &(i_mem->instructions[IMEM_index]);
        }
        IMEM_index++;
        PC += 4;
    }

    fclose(fd);
}

void parseRType(char *opr, Instruction *instr)
{
    instr->instruction = 0;
    unsigned opcode = 0;
    unsigned funct3 = 0;
    unsigned funct7 = 0;

    if (strcmp(opr, "add") == 0)
    {
        opcode = 51;
        funct3 = 0;
        funct7 = 0;
    }
    else if (strcmp(opr, "sll") == 0){
        opcode = 0b110011;
        funct3 = 0b1;
        funct7 = 0b0;
    }else if (strcmp(opr, "sub") == 0)
    {
        opcode = 51;
        funct3 = 0;
        funct7 = 0b0100000;
    }
    else if (strcmp(opr, "or") == 0){
        opcode = 0b110011;
        funct3 = 0b110;
        funct7 = 0;
    }
    else if (strcmp(opr, "and") == 0){
        opcode = 0b110011;
        funct3 = 0b111;
        funct7 = 0;
    }

    char *tok = strtok(NULL, ", ");
    unsigned rd = regIndex(tok);

    tok = strtok(NULL, ", ");
    unsigned rs_1 = regIndex(tok);

    tok = strtok(NULL, ", ");
    if (tok[strlen(tok)-1] == '\n')
        tok[strlen(tok)-1] = '\0';
    unsigned rs_2 = regIndex(tok);

    // Contruct instruction
    instr->instruction |= opcode;
    instr->instruction |= (rd << 7);
    instr->instruction |= (funct3 << (7 + 5));
    instr->instruction |= (rs_1 << (7 + 5 + 3));
    instr->instruction |= (rs_2 << (7 + 5 + 3 + 5));
    instr->instruction |= (funct7 << (7 + 5 + 3 + 5 + 5));
}

void parseIType(char *opr, Instruction *instr)
{
    instr->instruction = 0;
    unsigned funct3 = 0;
    unsigned opcode = 0;
    int imm=0;
    unsigned rd, rs_1;
    
    char *tok = strtok(NULL, ", ");
    rd = regIndex(tok);

    // ld requires diff handling of strtok
    if (strcmp(opr, "ld") == 0 ||
        strcmp(opr, "jalr") == 0)
    {
        if (strcmp(opr, "ld") == 0){
            opcode = 3;
            funct3 = 3;
        }
        else if (strcmp(opr, "jalr") == 0){
            opcode = 0b1100111;
            funct3 = 0b0;
        }

        tok = strtok(NULL, "(");
        imm = atoi(tok);
        
        tok = strtok(NULL, ")");
        rs_1 = regIndex(tok);
    } 
    else 
    {
        if (strcmp(opr, "addi") == 0)
        {
            opcode = 19;
        }
        else if (strcmp(opr, "slli") == 0)
        {
            opcode = 19;
            funct3 = 1;
        }
        tok = strtok(NULL, ", ");
        rs_1 = regIndex(tok);

        tok = strtok(NULL, ", ");
        if (tok[strlen(tok)-1] == '\n')
            tok[strlen(tok)-1] = '\0';
        imm = atoi(tok);
    }
    
    instr->instruction |= opcode;
    instr->instruction |= (rd << 7);
    instr->instruction |= (funct3 << (7 + 5));
    instr->instruction |= (rs_1 << (7 + 5 + 3));
    instr->instruction |= (imm << (7 + 5 + 3 + 5));
}

void parseSBType(char *opr, Instruction *instr)
{
    instr->instruction = 0;
    unsigned funct3 = 0;
    unsigned opcode = 0b1100011;

    if (strcmp(opr, "bne") == 0)
        funct3 = 1;

    else if (strcmp(opr, "beq") == 0)
        funct3 = 0;

    char *tok = strtok(NULL, ", ");
    unsigned rs_1 = regIndex(tok);

    tok = strtok(NULL, ", ");
    unsigned rs_2 = regIndex(tok);

    tok = strtok(NULL, ", ");
    if (tok[strlen(tok)-1] == '\n')
        tok[strlen(tok)-1] = '\0';
    int imm = atoi(tok);

    instr->instruction |= opcode;
    instr->instruction |= ((imm & 31) << 7); //keep 5 LSB from immediate: 31 == 0b00011111
    instr->instruction |= (funct3 << (7 + 5));
    instr->instruction |= (rs_1 << (7 + 5 + 3));
    instr->instruction |= (rs_2 << (7 + 5 + 3 + 5));
    instr->instruction |= ((imm >> 5) << (7 + 5 + 3 + 5 + 5)); //remove 5 LSB from immediate
}

void parseSType(char *opr, Instruction *instr)
{
    instr->instruction = 0;
    unsigned funct3 = 0;
    unsigned opcode = 0b100011;

    if (strcmp(opr, "sb") == 0){
        funct3 = 0;
    }
    else if (strcmp(opr, "sh") == 0){
        funct3 = 0b1;
    }
    else if (strcmp(opr, "sw") == 0){
        funct3 = 0b10;
    }
    else if (strcmp(opr, "sd") == 0){
        funct3 = 0b11;
    }

    char *tok = strtok(NULL, ", ");
    unsigned rs_2 = regIndex(tok);

    tok = strtok(NULL, "(");
    int imm = atoi(tok);

    tok = strtok(NULL, ")");
    unsigned rs_1 = regIndex(tok);

    instr->instruction |= opcode;
    instr->instruction |= ((imm & 0b11111) << 7); 
    instr->instruction |= (funct3 << (7 + 5));
    instr->instruction |= (rs_1 << (7 + 5 + 3));
    instr->instruction |= (rs_2 << (7 + 5 + 3 + 5));
    instr->instruction |= ((imm >> 5) << (7 + 5 + 3 + 5 + 5)); //remove 5 LSB from immediate
}

void parseUJType(char *opr, Instruction *instr)
{
    instr->instruction = 0;
    unsigned opcode = 0b1101111;

    char *tok = strtok(NULL, ", ");
    unsigned rd = regIndex(tok);

    tok = strtok(NULL, ", ");
    if (tok[strlen(tok)-1] == '\n')
        tok[strlen(tok)-1] = '\0';
    int imm = atoi(tok);

    instr->instruction |= opcode;
    instr->instruction |= (rd << 7);
    instr->instruction |= (imm << (7 + 5));
}

int regIndex(char *reg)
{
    if (strcmp(reg, "sp") == 0){
        return 2; // sp register is x2
    }
    unsigned i = 0;
    for (i; i < NUM_OF_REGS; i++)
    {
        if (strcmp(REGISTER_NAME[i], reg) == 0)
        {
            break;
        }
    }

    return i;
}