/* Data structures and functions associated with agent expressions in GDB.
   Copyright (C) 2009-2014 Free Software Foundation, Inc.

   This file is part of GDB.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#if !defined (AX_H)
#define AX_H 1

//#include "server.h"
typedef unsigned int CORE_ADDR;
typedef long long LONGEST;
typedef unsigned long long ULONGEST;
//#include "regcache.h"
/*
enum gdb_agent_op
{
    gdb_agent_op_float = 0x01,
    gdb_agent_op_add = 0x02,
    gdb_agent_op_sub = 0x03,
    gdb_agent_op_mul = 0x04,
    gdb_agent_op_div_signed = 0x05,
    gdb_agent_op_div_unsigned = 0x06,
    gdb_agent_op_rem_signed = 0x07,
    gdb_agent_op_rem_unsigned = 0x08,
    gdb_agent_op_lsh = 0x09,
    gdb_agent_op_rsh_signed = 0x0a,
    gdb_agent_op_rsh_unsigned = 0x0b,
    gdb_agent_op_trace = 0x0c,
    gdb_agent_op_trace_quick = 0x0d,
    gdb_agent_op_log_not = 0x0e,
    gdb_agent_op_bit_and = 0x0f,
    gdb_agent_op_bit_or = 0x10,
    gdb_agent_op_bit_xor = 0x11,
    gdb_agent_op_bit_not = 0x12,
    gdb_agent_op_equal = 0x13,
    gdb_agent_op_less_signed = 0x14,
    gdb_agent_op_less_unsigned = 0x15,
    gdb_agent_op_ext = 0x16,
    gdb_agent_op_ref8 = 0x17,
    gdb_agent_op_ref16 = 0x18,
    gdb_agent_op_ref32 = 0x19,
    gdb_agent_op_ref64 = 0x1a,
    gdb_agent_op_ref_float = 0x1b,
    gdb_agent_op_ref_double = 0x1c,
    gdb_agent_op_ref_long_double = 0x1d,
    gdb_agent_op_l_to_d = 0x1e,
    gdb_agent_op_d_to_l = 0x1f,
    gdb_agent_op_if_goto = 0x20,
    gdb_agent_op_goto = 0x21,
    gdb_agent_op_const8 = 0x22,
    gdb_agent_op_const16 = 0x23,
    gdb_agent_op_const32 = 0x24,
    gdb_agent_op_const64 = 0x25,
    gdb_agent_op_reg = 0x26,
    gdb_agent_op_end = 0x27,
    gdb_agent_op_dup = 0x28,
    gdb_agent_op_pop = 0x29,
    gdb_agent_op_zero_ext = 0x2a,
    gdb_agent_op_swap = 0x2b,
    gdb_agent_op_getv = 0x2c,
    gdb_agent_op_setv = 0x2d,
    gdb_agent_op_tracev = 0x2e,
    gdb_agent_op_tracenz = 0x2f,
    gdb_agent_op_trace16 = 0x30,
    gdb_agent_op_invalid2 = 0x31,
    gdb_agent_op_pick = 0x32,
    gdb_agent_op_rot = 0x33,
    gdb_agent_op_printf = 0x34,
    gdb_agent_op_last
};

static const char *gdb_agent_op_names [gdb_agent_op_last] =
{
    "?undef?",
    "float",
    "add",
    "sub",
    "mul",
    "div_signed",
    "div_unsigned",
    "rem_signed",
    "rem_unsigned",
    "lsh",
    "rsh_signed",
    "rsh_unsigned",
    "trace",
    "trace_quick",
    "log_not",
    "bit_and",
    "bit_or",    // 0x10
    "bit_xor",
    "bit_not",
    "equal",
    "less_signed",
    "less_unsigned",
    "ext",
    "ref8",
    "ref16",
    "ref32",
    "ref64",
    "ref_float",
    "ref_double",
    "ref_long_double",
    "l_to_d",
    "d_to_l",
    "if_goto",   // 0x20
    "goto",
    "const8",
    "const16",
    "const32",
    "const64",
    "reg",
    "end",
    "dup",
    "pop",
    "zero_ext",
    "swap",
    "getv",      // 0x2c
    "setv",
    "tracev",
    "tracenz",
    "trace16",   // 0x30
    "invalid2",
    "pick",
    "rot",
    "printf",
};

*/

struct agent_expr
{
  unsigned int length;
  unsigned char *bytes;
};

/* Enumeration of the different kinds of things that can happen during
   agent expression evaluation.  */

enum eval_result_type
  {
    expr_eval_no_error,
    expr_eval_empty_expression,
    expr_eval_empty_stack,
    expr_eval_stack_overflow,
    expr_eval_stack_underflow,
    expr_eval_unhandled_opcode,
    expr_eval_unrecognized_opcode,
    expr_eval_divide_by_zero,
    expr_eval_invalid_goto
  };

extern unsigned int gdb_bytecode_parsing(struct target *target, char *p_bytecode);

#endif /* AX_H */
