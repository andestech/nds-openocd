#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <helper/log.h>
#include "nds32.h"
#include "bytecode-ax.h"
//#include "nds32_v3.h"
//#include "nds32_v3_common.h"


#define ax_debug    LOG_DEBUG  //printf
struct target *p_curr_target = NULL;

void agent_mem_read(unsigned char *p_buffer, unsigned int addr, unsigned int size)
{
	if (p_curr_target)
		p_curr_target->type->read_buffer(p_curr_target, addr, size, (uint8_t *)p_buffer);
	/*
	if (size == 1) {
		*p_buffer = 0x10;
	}
	else if (size == 2) {
		*p_buffer++ = 0x22;
		*p_buffer++ = 0x22;
	} else if (size == 4) {
		*p_buffer++ = 0x5a;
		*p_buffer++ = 0x00;
		*p_buffer++ = 0x00;
		*p_buffer++ = 0x00;
	}*/
}

void collect_register (int reg_num, unsigned char *p_buffer)
{
	unsigned int value_reg = 0;
	if (p_curr_target == NULL)
		return;
	struct nds32 *nds32 = target_to_nds32(p_curr_target);
	nds32_get_mapped_reg(nds32, reg_num, &value_reg);
  memcpy (p_buffer, &value_reg, 4);
}

static int ishex(int ch, int *val)
{
	if ((ch >= 'a') && (ch <= 'f')) {
		*val = ch - 'a' + 10;
		return 1;
	}
	if ((ch >= 'A') && (ch <= 'F')) {
		*val = ch - 'A' + 10;
		return 1;
	}
	if ((ch >= '0') && (ch <= '9')) {
		*val = ch - '0';
		return 1;
	}
	return 0;
}

static char *unpack_varlen_hex(char *buff, ULONGEST *result)
{
	int nibble;
	ULONGEST retval = 0;

	while (ishex (*buff, &nibble)) {
		buff++;
		retval = retval << 4;
		retval |= nibble & 0x0f;
	}
	*result = retval;
	return buff;
}

/* Convert hex digit A to a number.  */
static int fromhex(int a)
{
	if (a >= '0' && a <= '9')
		return a - '0';
	else if (a >= 'a' && a <= 'f')
		return a - 'a' + 10;
	else
		ax_debug ("Reply contains invalid hex digit");
	return 0;
}

static void convert_ascii_to_int (const char *from, unsigned char *to, int n)
{
	int nib1, nib2;
	while (n--) {
		nib1 = fromhex (*from++);
		nib2 = fromhex (*from++);
		*to++ = (((nib1 & 0x0f) << 4) & 0xf0) | (nib2 & 0x0f);
	}
}

/* This enum must exactly match what is documented in
   gdb/doc/agentexpr.texi, including all the numerical values.  */

enum gdb_agent_op
{
#define DEFOP(NAME, SIZE, DATA_SIZE, CONSUMED, PRODUCED, VALUE)  \
    gdb_agent_op_ ## NAME = VALUE,
#include "ax.def"
#undef DEFOP
    gdb_agent_op_last
};

static const char *gdb_agent_op_names [gdb_agent_op_last] =
{
    "?undef?"
#define DEFOP(NAME, SIZE, DATA_SIZE, CONSUMED, PRODUCED, VALUE)  , # NAME
#include "ax.def"
#undef DEFOP
};

static const unsigned char gdb_agent_op_sizes [gdb_agent_op_last] =
{
    0
#define DEFOP(NAME, SIZE, DATA_SIZE, CONSUMED, PRODUCED, VALUE)  , SIZE
#include "ax.def"
#undef DEFOP
};

/* A wrapper for gdb_agent_op_names that does some bounds-checking.  */
static const char *gdb_agent_op_name (int op)
{
	if (op < 0 || op >= gdb_agent_op_last || gdb_agent_op_names[op] == NULL)
		return "?undef?";
	return gdb_agent_op_names[op];
}

/* The packet form of an agent expression consists of an 'X', number
   of bytes in expression, a comma, and then the bytes.  */
struct agent_expr *gdb_parse_agent_expr(char **actparm)
{
	char *act = *actparm;
	ULONGEST xlen;
	struct agent_expr *aexpr;

	++act;  /* skip the X */
	act = unpack_varlen_hex (act, &xlen);
	++act;  /* skip a comma */
	aexpr = malloc (sizeof (struct agent_expr));
	aexpr->length = xlen;
	aexpr->bytes = malloc (xlen);
	convert_ascii_to_int (act, aexpr->bytes, xlen);
	*actparm = act + (xlen * 2);
	return aexpr;
}

/* The agent expression evaluator, as specified by the GDB docs. It
   returns 0 if everything went OK, and a nonzero error code
   otherwise.  */
enum eval_result_type
gdb_eval_agent_expr (struct agent_expr *aexpr, ULONGEST *rslt)
{
	int pc = 0;
#define STACK_MAX 100
	ULONGEST stack[STACK_MAX], top;
	int sp = 0;
	unsigned char op;
	int arg;

  /* This union is a convenient way to convert representations.  For
     now, assume a standard architecture where the hardware integer
     types have 8, 16, 32, 64 bit types.  A more robust solution would
     be to import stdint.h from gnulib.  */
	union
	{
		union
		{
			unsigned char bytes[1];
			unsigned char val;
		} u8;
		union
		{
			unsigned char bytes[2];
			unsigned short val;
		} u16;
		union
		{
			unsigned char bytes[4];
			unsigned int val;
		} u32;
		union
		{
			unsigned char bytes[8];
			ULONGEST val;
		} u64;
	} cnv;

	if (aexpr->length == 0) {
		ax_debug ("empty agent expression\n");
		return expr_eval_empty_expression;
	}

	/* Cache the stack top in its own variable. Much of the time we can
	   operate on this variable, rather than dinking with the stack. It
	   needs to be copied to the stack when sp changes.  */
	top = 0;

	while (1) {
		op = aexpr->bytes[pc++];
		ax_debug ("About to interpret byte 0x%x\n", op);

		switch (op)  {
			case gdb_agent_op_add:
			  top += stack[--sp];
			  break;

			case gdb_agent_op_sub:
			  top = stack[--sp] - top;
			  break;

			case gdb_agent_op_mul:
			  top *= stack[--sp];
			  break;

			case gdb_agent_op_div_signed:
			  if (top == 0)
			    {
			      ax_debug ("Attempted to divide by zero\n");
			      return expr_eval_divide_by_zero;
			    }
			  top = ((LONGEST) stack[--sp]) / ((LONGEST) top);
			  break;

			case gdb_agent_op_div_unsigned:
			  if (top == 0)
			    {
			      ax_debug ("Attempted to divide by zero\n");
			      return expr_eval_divide_by_zero;
			    }
			  top = stack[--sp] / top;
			  break;

			case gdb_agent_op_rem_signed:
			  if (top == 0)
			    {
			      ax_debug ("Attempted to divide by zero\n");
			      return expr_eval_divide_by_zero;
			    }
			  top = ((LONGEST) stack[--sp]) % ((LONGEST) top);
			  break;

			case gdb_agent_op_rem_unsigned:
			  if (top == 0)
			    {
			      ax_debug ("Attempted to divide by zero\n");
			      return expr_eval_divide_by_zero;
			    }
			  top = stack[--sp] % top;
			  break;

			case gdb_agent_op_lsh:
			  top = stack[--sp] << top;
			  break;

			case gdb_agent_op_rsh_signed:
			  top = ((LONGEST) stack[--sp]) >> top;
			  break;

			case gdb_agent_op_rsh_unsigned:
			  top = stack[--sp] >> top;
			  break;

			case gdb_agent_op_trace:
			  agent_mem_read (NULL, (CORE_ADDR) stack[--sp], (ULONGEST) top);
			  if (--sp >= 0)
			    top = stack[sp];
			  break;

			case gdb_agent_op_trace_quick:
			  arg = aexpr->bytes[pc++];
			  agent_mem_read (NULL, (CORE_ADDR) top, (ULONGEST) arg);
			  break;

			case gdb_agent_op_log_not:
			  top = !top;
			  break;

			case gdb_agent_op_bit_and:
			  top &= stack[--sp];
			  break;

			case gdb_agent_op_bit_or:
			  top |= stack[--sp];
			  break;

			case gdb_agent_op_bit_xor:
			  top ^= stack[--sp];
			  break;

			case gdb_agent_op_bit_not:
			  top = ~top;
			  break;

			case gdb_agent_op_equal:
			  top = (stack[--sp] == top);
			  break;

			case gdb_agent_op_less_signed:
			  top = (((LONGEST) stack[--sp]) < ((LONGEST) top));
			  break;

			case gdb_agent_op_less_unsigned:
			  top = (stack[--sp] < top);
			  break;

			case gdb_agent_op_ext:
			  arg = aexpr->bytes[pc++];
			  if ((unsigned int)arg < (sizeof (LONGEST) * 8))
			    {
			      LONGEST mask = 1 << (arg - 1);
			      top &= ((LONGEST) 1 << arg) - 1;
			      top = (top ^ mask) - mask;
			    }
			  break;

			case gdb_agent_op_ref8:
			  agent_mem_read (cnv.u8.bytes, (CORE_ADDR) top, 1);
			  top = cnv.u8.val;
			  break;

			case gdb_agent_op_ref16:
			  agent_mem_read (cnv.u16.bytes, (CORE_ADDR) top, 2);
			  top = cnv.u16.val;
			  break;

			case gdb_agent_op_ref32:
			  agent_mem_read (cnv.u32.bytes, (CORE_ADDR) top, 4);
			  top = cnv.u32.val;
			  break;

			case gdb_agent_op_ref64:
			  agent_mem_read (cnv.u64.bytes, (CORE_ADDR) top, 8);
			  top = cnv.u64.val;
			  break;

			case gdb_agent_op_if_goto:
			  if (top)
			    pc = (aexpr->bytes[pc] << 8) + (aexpr->bytes[pc + 1]);
			  else
			    pc += 2;
			  if (--sp >= 0)
			    top = stack[sp];
			  break;

			case gdb_agent_op_goto:
			  pc = (aexpr->bytes[pc] << 8) + (aexpr->bytes[pc + 1]);
			  break;

			case gdb_agent_op_const8:
			  /* Flush the cached stack top.  */
			  stack[sp++] = top;
			  top = aexpr->bytes[pc++];
			  break;

			case gdb_agent_op_const16:
			  /* Flush the cached stack top.  */
			  stack[sp++] = top;
			  top = aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  break;

			case gdb_agent_op_const32:
			  /* Flush the cached stack top.  */
			  stack[sp++] = top;
			  top = aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  break;

			case gdb_agent_op_const64:
			  /* Flush the cached stack top.  */
			  stack[sp++] = top;
			  top = aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  top = (top << 8) + aexpr->bytes[pc++];
			  break;

			case gdb_agent_op_reg:
			  /* Flush the cached stack top.  */
			  stack[sp++] = top;
			  arg = aexpr->bytes[pc++];
			  arg = (arg << 8) + aexpr->bytes[pc++];
			  {
			    int regnum = arg;
			    collect_register (regnum, cnv.u32.bytes);
					top = cnv.u32.val;
			  }
			  break;

			case gdb_agent_op_end:
			  ax_debug ("At end of expression, sp=%d \n", sp);
			  if (rslt) {
			    if (sp <= 0) {
			      /* This should be an error */
			      ax_debug ("Stack is empty, nothing to return\n");
			      return expr_eval_empty_stack;
			    }
			    *rslt = top;
			  }
			  return expr_eval_no_error;

			case gdb_agent_op_dup:
			  stack[sp++] = top;
			  break;

			case gdb_agent_op_pop:
			  if (--sp >= 0)
			    top = stack[sp];
			  break;

			case gdb_agent_op_pick:
			  arg = aexpr->bytes[pc++];
			  stack[sp] = top;
			  top = stack[sp - arg];
			  ++sp;
			  break;

			case gdb_agent_op_rot:
			  {
			    ULONGEST tem = stack[sp - 1];
		
			    stack[sp - 1] = stack[sp - 2];
			    stack[sp - 2] = top;
			    top = tem;
			  }
			  break;

			case gdb_agent_op_zero_ext:
			  arg = aexpr->bytes[pc++];
			  if ((unsigned int)arg < (sizeof (LONGEST) * 8))
			    top &= ((LONGEST) 1 << arg) - 1;
			  break;

			case gdb_agent_op_swap:
			  /* Interchange top two stack elements, making sure top gets
			     copied back onto stack.  */
			  stack[sp] = top;
			  top = stack[sp - 1];
			  stack[sp - 1] = stack[sp];
			  break;

			case gdb_agent_op_getv:
			//  /* Flush the cached stack top.  */
			//  stack[sp++] = top;
			//  arg = aexpr->bytes[pc++];
			//  arg = (arg << 8) + aexpr->bytes[pc++];
			//  top = agent_get_trace_state_variable_value (arg);
			//  break;
			case gdb_agent_op_setv:
			//  arg = aexpr->bytes[pc++];
			//  arg = (arg << 8) + aexpr->bytes[pc++];
			//  agent_set_trace_state_variable_value (arg, top);
			//  /* Note that we leave the value on the stack, for the
			//     benefit of later/enclosing expressions.  */
			//  break;
			case gdb_agent_op_tracev:
			//  arg = aexpr->bytes[pc++];
			//  arg = (arg << 8) + aexpr->bytes[pc++];
			//  agent_tsv_read (ctx, arg);
			//  break;
			case gdb_agent_op_tracenz:
			//  agent_mem_read_string (NULL, (CORE_ADDR) stack[--sp], (ULONGEST) top);
			//  if (--sp >= 0)
			//    top = stack[sp];
			//  break;
			case gdb_agent_op_printf:
			//  break;
			/* GDB never (currently) generates any of these ops.  */
			case gdb_agent_op_float:
			case gdb_agent_op_ref_float:
			case gdb_agent_op_ref_double:
			case gdb_agent_op_ref_long_double:
			case gdb_agent_op_l_to_d:
			case gdb_agent_op_d_to_l:
			case gdb_agent_op_trace16:
			  ax_debug ("Agent expression op 0x%x valid, but not handled\n", op);
			  /* If ever GDB generates any of these, we don't have the
			     option of ignoring.  */
			  return 1;

			default:
			  ax_debug ("Agent expression op 0x%x not recognized\n", op);
			  /* Don't struggle on, things will just get worse.  */
			  return expr_eval_unrecognized_opcode;
		} // switch (op)

		/* Check for stack badness.  */
		if (sp >= (STACK_MAX - 1)) {
			ax_debug ("Expression stack overflow\n");
			return expr_eval_stack_overflow;
		}

		if (sp < 0) {
			ax_debug ("Expression stack underflow\n");
			return expr_eval_stack_underflow;
		}

		ax_debug ("Op %s -> sp=%d, top=0x%x\n",
			gdb_agent_op_name (op), sp, (int)top);
   } // while (1)
}

unsigned int gdb_bytecode_parsing(struct target *target, char *p_bytecode)
{
	struct agent_expr *tmp_aexpr;
	ULONGEST return_val = 0;
	char *actparm = p_bytecode;

	p_curr_target = target;
	tmp_aexpr = gdb_parse_agent_expr (&actparm);
	enum eval_result_type eval_result = gdb_eval_agent_expr (tmp_aexpr, &return_val);
	free(tmp_aexpr->bytes);
	free(tmp_aexpr);

	if ((eval_result == expr_eval_no_error) &&
		  (return_val == 0)) {
		/* do not hit conditional breakpoint */
		return 0;
	}
	/* always hit conditional breakpoint */
	return 1;
}
