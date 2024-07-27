#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rv_emu.h"
#include "bits.h"

#define DEBUG 0

static void unsupported(char *s, uint32_t n) {
    printf("unsupported %s 0x%x\n", s, n);
    exit(-1);
}

void emu_r_type(rv_state *state, uint32_t iw) {


	state->analysis.ir_count++;
	
    uint32_t rd = (iw >> 7) & 0b11111;
    uint32_t rs1 = (iw >> 15) & 0b11111;
    uint32_t rs2 = (iw >> 20) & 0b11111;
    uint32_t funct3 = (iw >> 12) & 0b111;
    uint32_t funct7 = (iw >> 25) & 0b1111111;
	
    if (funct3 == 0b000 && funct7 == 0b0000000) {			//add
        state->regs[rd] = state->regs[rs1] + state->regs[rs2];
    } else if (funct3 == 0b000 && funct7 == 0b0000001){ 	//mul
    	state->regs[rd] = state->regs[rs1] * state->regs[rs2];
    } else if (funct3 == 0b000 && funct7 == 0b0100000){		//sub
    	state->regs[rd] = state->regs[rs1] - state->regs[rs2];
    } else if (funct3 == 0b101){							//srl
    	state->regs[rd] = state->regs[rs1] >> state->regs[rs2];
    } else if (funct3 == 0b001){ 							//sll
    	state->regs[rd] = state->regs[rs1] << state->regs[rs2];
    } else if (funct3 == 0b111){							//and
    	state->regs[rd] = state->regs[rs1] & state->regs[rs2];
    } else if (funct3 == 0b100 && funct7 == 0b0000001) { 	//div
    	state->regs[rd] = state->regs[rs1] / state->regs[rs2];
  	} else if (funct3 == 0b110 && funct7 == 0b0000001) { 	//rem
       	state->regs[rd] = state->regs[rs1] % state->regs[rs2];	
    } else {
        unsupported("R-type funct3", funct3);
    }

    state->pc += 4; // Next instruction
}

void emu_i_type(rv_state *state, uint32_t iw, rv_format fmt) {


	uint32_t rd = (iw >> 7) & 0b11111;	
	uint32_t rs1 = (iw >> 15) & 0b11111;
	uint32_t shamt = (iw >> 20) & 0b11111;
	uint32_t funct3 = (iw >> 12) & 0b111;
    uint32_t funct7 = (iw >> 25) & 0b1111111;
    uint64_t im = (iw >> 20) & 0b111111111111;

    int64_t imm = sign_extend(im, 12);
    
	//for 'li' rs1 = 0 therefore 0 + imm = imm
	//for 'mv' im = 0 so rd = rs1 + 0 aka rd = copy of rs1

	switch (fmt) {
	
		case FMT_I_JALR:
			state->pc = state->regs[rs1] + imm;					//jalr (ret)
           	state->analysis.j_count++;
			break;
			
		case FMT_I_ARITH:
			
			if (funct3 == 0b101 && funct7 == 0b0000000) {		//srli
				state->regs[rd] = state->regs[rs1] >> shamt;
			} else if (funct3 == 0b000) {
				state->regs[rd] = state->regs[rs1] + imm;			//addi and li		
			} else if (funct3 == 0b001) {
				state->regs[rd] = state->regs[rs1] << shamt; 		//slli
			}
			
			state->analysis.ir_count++;

			break;
			
		case FMT_I_LOAD:
			if (funct3 == LDST_DOUBLE) {		//ld
				state->regs[rd] = *(uint64_t *) (state->regs[rs1] + imm);			
			} else if (funct3 == LDST_BYTE) {	//lb
				state->regs[rd] = *(uint8_t *) (state->regs[rs1] + imm);	
			} else if (funct3 == LDST_WORD) {	//lw
				state->regs[rd] = *(uint32_t *) (state->regs[rs1] + imm);
			}

           	state->analysis.ld_count++;

			break;
			
		default: 
			unsupported("I-type",funct3);
		}
	
	if (fmt != FMT_I_JALR) {
		state->pc += 4;
	}

}

void emu_s_type(rv_state *state, uint32_t iw) {

	uint32_t bits4_0 = get_bits(iw, 7, 5);
	uint32_t bits11_5 = get_bits(iw, 25, 7) << 5;

	uint32_t rs1 = get_bits(iw, 15, 5);
	uint32_t rs2 = get_bits(iw, 20, 5);
	uint32_t funct3 = get_bits(iw, 12, 3);	
	uint64_t im = bits11_5 | bits4_0;

    int64_t imm = sign_extend(im, 12);
    
	if (funct3 == LDST_DOUBLE) { 					//sd
						
		*(uint64_t *) (state->regs[rs1] + imm) = state->regs[rs2];		

	} else if (funct3 == LDST_BYTE) {		//sb

		*(uint8_t *) (state->regs[rs1] + imm) = state->regs[rs2];

	} else if (funct3 == LDST_WORD) {   	//sw
		*(uint32_t *) (state->regs[rs1] + imm) = state->regs[rs2];

	} else {
		unsupported("s-type",funct3);
	}

	state->pc += 4;
	state->analysis.st_count++;

}

void emu_sb_type(rv_state *state, uint32_t iw) {
	
	uint32_t bit12 = ((iw >> 31) & 0b1) << 11; //msb = 12th imm bit

	uint32_t bits10_5 = ((iw >> 25) & 0b111111) << 5; //bits 25-30 = 10-5 imm bits

	uint32_t bit11 = ((iw >> 7) & 0b1) << 11; // bit 7 = 11 imm bit

	uint32_t bits4_1 = ((iw >> 8) & 0b1111) << 1; //bits 11-8 = 4-1 imm bits
	
	uint32_t imm = bit12 | bit11 | bits10_5 | bits4_1;

	uint32_t rs1 = (iw >> 15) & 0b11111;
	uint32_t rs2 = (iw >> 20) & 0b11111;
	uint32_t funct3 = (iw >> 12) & 0b111;

	bool branch_taken = false;
	
    //bgt has funct3 of blt
    //but its passed with the registers swapped
    //ie rs2 <-> rs1 meaning 
    //doing rs1 < rs2 == rs2 > rs1 

    switch (funct3) {
    	case 0b100:		//blt + bgt
			branch_taken = (int64_t)state->regs[rs1] < (int64_t)state->regs[rs2];
			break;
		case 0b001:		//bne
			branch_taken = (int64_t)state->regs[rs1] != (int64_t)state->regs[rs2];
			break;
		case 0b000:		// beq + beqz
			branch_taken = (int64_t)state->regs[rs1] == (int64_t)state->regs[rs2];
			break;
		case 0b101:		//bgte
			branch_taken = (int64_t)state->regs[rs1] >= (int64_t)state->regs[rs2];
			break;
		default:
			unsupported("sb-type funct3", funct3);
    }
	
	if(branch_taken) {
		state->pc += imm;
       	state->analysis.b_taken++;

	} else {
		state->pc+= 4;
       	state->analysis.b_not_taken++;
	}
}

void emu_uj_type(rv_state *state, uint32_t iw){


   	state->analysis.j_count++;
	
	uint32_t rd = get_bits(iw,7,5);

	if (rd != 0) {
		state->regs[rd] = ((uint64_t) state->pc) + 4;
	}

	uint32_t offset = 0;
	uint32_t bit20 = get_bit(iw,31);
	uint32_t bit11 = get_bit(iw, 20);
	uint32_t bits10_1 = get_bits(iw, 21, 10); 
	uint32_t bits19_12 = get_bits(iw, 12, 8); 

 	offset |= (bit20 << 19) | (bits19_12 << 11) | (bit11 << 10) | bits10_1;

 	int64_t signed_offset = sign_extend(offset, 20) * 2;
 	
 	state->pc += signed_offset;		//jal
					
}

void rv_init(rv_state *state, uint32_t *target, 
             uint64_t a0, uint64_t a1, uint64_t a2, uint64_t a3) {
             
    state->pc = (uint64_t) target;
    state->regs[RV_A0] = a0;
    state->regs[RV_A1] = a1;
    state->regs[RV_A2] = a2;
    state->regs[RV_A3] = a3;

    state->regs[RV_ZERO] = 0;  // zero is always 0  (:
    state->regs[RV_RA] = RV_STOP;
    state->regs[RV_SP] = (uint64_t) &state->stack[STACK_SIZE];
    
    memset(&state->analysis, 0, sizeof(rv_analysis));
    cache_init(&state->i_cache);
}


static void rv_one(rv_state *state) {


    uint32_t iw  = *((uint32_t*) state->pc);
    iw = cache_lookup(&state->i_cache, (uint64_t) state->pc);
	
    uint32_t opcode = get_bits(iw, 0, 7);


	if (state->verbose) {
	    printf("\niw: %x\n", iw);	
	}

	rv_format fmt = opcode;


  switch (opcode) {
            case 0b0110011:
                // R-type instructions have two register operands
                emu_r_type(state, iw);
                break;
        	case 0b0010011:
        		emu_i_type(state, iw, fmt);
        		break; 
        	case 0b1100011:
    			emu_sb_type(state, iw);
        		break;
            case 0b1100111:
                // JALR (aka RET) is a variant of I-type instructions
                // emu_jalr(state, iw);
                emu_i_type(state, iw, fmt);
                break;
            case 0b1101111:
            	emu_uj_type(state, iw);
            	break;
            case 0b0100011:
            	emu_s_type(state, iw);
            	break;
            case 0b0000011:
            	//ld or lb
            	emu_i_type(state, iw, fmt);
            	break;
            default:
                unsupported("Unknown opcode: ", opcode);
        }

    	state->analysis.i_count++;

}

uint64_t rv_emulate(rv_state *state) {
    while (state->pc != RV_STOP) {
        rv_one(state);
    }
    return state->regs[RV_A0];
}

static void print_pct(char *fmt, int numer, int denom) {
    double pct = 0.0;

    if (denom)
        pct = (double) numer / (double) denom * 100.0;
    printf(fmt, numer, pct);
}

void rv_print(rv_analysis *a) {
    int b_total = a->b_taken + a->b_not_taken;

    printf("=== Analysis\n");
    print_pct("Instructions Executed  = %d\n", a->i_count, a->i_count);
    print_pct("R-type + I-type        = %d (%.2f%%)\n", a->ir_count, a->i_count);
    print_pct("Loads                  = %d (%.2f%%)\n", a->ld_count, a->i_count);
    print_pct("Stores                 = %d (%.2f%%)\n", a->st_count, a->i_count);    
    print_pct("Jumps/JAL/JALR         = %d (%.2f%%)\n", a->j_count, a->i_count);
    print_pct("Conditional branches   = %d (%.2f%%)\n", b_total, a->i_count);
    print_pct("  Branches taken       = %d (%.2f%%)\n", a->b_taken, b_total);
    print_pct("  Branches not taken   = %d (%.2f%%)\n", a->b_not_taken, b_total);
}
