/*
 *  Originally written by Glenn Engel, Lake Stevens Instrument Division
 *
 *  Contributed by HP Systems
 *
 *  Modified for Linux/MIPS (and MIPS in general) by Andreas Busse
 *  Send complaints, suggestions etc. to <andy@waldorf-gmbh.de>
 *
 *  Copyright (C) 1995 Andreas Busse
 *
 *  Copyright (C) 2003 MontaVista Software Inc.
 *  Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 *  Copyright (C) 2004-2005 MontaVista Software Inc.
 *  Author: Manish Lachwani, mlachwani@mvista.com or manish@koffee-break.com
 *
 *  Copyright (C) 2007-2008 Wind River Systems, Inc.
 *  Author/Maintainer: Jason Wessel, jason.wessel@windriver.com
 *
 *  This file is licensed under the terms of the GNU General Public License
 *  version 2. This program is licensed "as is" without any warranty of any
 *  kind, whether express or implied.
 */

#include <linux/ptrace.h>		/* for linux pt_regs struct */
#include <linux/kgdb.h>
#include <linux/kdebug.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/uaccess.h>
#include <asm/inst.h>
#include <asm/fpu.h>
#include <asm/cacheflush.h>
#include <asm/processor.h>
#include <asm/sigcontext.h>

/* <WRS_ADDED> */
#ifdef DEBUG_IT
#define PRINTK(args...) printk(KERN_ALERT args)
#else
#define PRINTK(args...)
#endif /* DEBUG_IT */

static unsigned long stepped_address;
static unsigned int  stepped_opcode;
static int stepped_cp0_status_ie;

static unsigned long mipsGetNpc(struct pt_regs *pRegs);
/* </WRS_ADDED> */

static struct hard_trap_info {
	unsigned char tt;	/* Trap type code for MIPS R3xxx and R4xxx */
	unsigned char signo;	/* Signal that we map this trap into */
} hard_trap_info[] = {
	{ 6, SIGBUS },		/* instruction bus error */
	{ 7, SIGBUS },		/* data bus error */
	{ 9, SIGTRAP },		/* break */
/*	{ 11, SIGILL }, */	/* CPU unusable */
	{ 12, SIGFPE },		/* overflow */
	{ 13, SIGTRAP },	/* trap */
	{ 14, SIGSEGV },	/* virtual instruction cache coherency */
	{ 15, SIGFPE },		/* floating point exception */
	{ 23, SIGSEGV },	/* watch */
	{ 31, SIGSEGV },	/* virtual data cache coherency */
	{ 0, 0}			/* Must be last */
};

struct dbg_reg_def_t dbg_reg_def[DBG_MAX_REG_NUM] =
{
	{ "zero", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[0]) },
	{ "at", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[1]) },
	{ "v0", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[2]) },
	{ "v1", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[3]) },
	{ "a0", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[4]) },
	{ "a1", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[5]) },
	{ "a2", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[6]) },
	{ "a3", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[7]) },
	{ "t0", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[8]) },
	{ "t1", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[9]) },
	{ "t2", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[10]) },
	{ "t3", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[11]) },
	{ "t4", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[12]) },
	{ "t5", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[13]) },
	{ "t6", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[14]) },
	{ "t7", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[15]) },
	{ "s0", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[16]) },
	{ "s1", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[17]) },
	{ "s2", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[18]) },
	{ "s3", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[19]) },
	{ "s4", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[20]) },
	{ "s5", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[21]) },
	{ "s6", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[22]) },
	{ "s7", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[23]) },
	{ "t8", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[24]) },
	{ "t9", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[25]) },
	{ "k0", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[26]) },
	{ "k1", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[27]) },
	{ "gp", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[28]) },
	{ "sp", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[29]) },
	{ "s8", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[30]) },
	{ "ra", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[31]) },
	{ "sr", GDB_SIZEOF_REG, offsetof(struct pt_regs, cp0_status) },
	{ "lo", GDB_SIZEOF_REG, offsetof(struct pt_regs, lo) },
	{ "hi", GDB_SIZEOF_REG, offsetof(struct pt_regs, hi) },
	{ "bad", GDB_SIZEOF_REG, offsetof(struct pt_regs, cp0_badvaddr) },
	{ "cause", GDB_SIZEOF_REG, offsetof(struct pt_regs, cp0_cause) },
	{ "pc", GDB_SIZEOF_REG, offsetof(struct pt_regs, cp0_epc) },
	{ "f0", GDB_SIZEOF_REG, 0 },
	{ "f1", GDB_SIZEOF_REG, 1 },
	{ "f2", GDB_SIZEOF_REG, 2 },
	{ "f3", GDB_SIZEOF_REG, 3 },
	{ "f4", GDB_SIZEOF_REG, 4 },
	{ "f5", GDB_SIZEOF_REG, 5 },
	{ "f6", GDB_SIZEOF_REG, 6 },
	{ "f7", GDB_SIZEOF_REG, 7 },
	{ "f8", GDB_SIZEOF_REG, 8 },
	{ "f9", GDB_SIZEOF_REG, 9 },
	{ "f10", GDB_SIZEOF_REG, 10 },
	{ "f11", GDB_SIZEOF_REG, 11 },
	{ "f12", GDB_SIZEOF_REG, 12 },
	{ "f13", GDB_SIZEOF_REG, 13 },
	{ "f14", GDB_SIZEOF_REG, 14 },
	{ "f15", GDB_SIZEOF_REG, 15 },
	{ "f16", GDB_SIZEOF_REG, 16 },
	{ "f17", GDB_SIZEOF_REG, 17 },
	{ "f18", GDB_SIZEOF_REG, 18 },
	{ "f19", GDB_SIZEOF_REG, 19 },
	{ "f20", GDB_SIZEOF_REG, 20 },
	{ "f21", GDB_SIZEOF_REG, 21 },
	{ "f22", GDB_SIZEOF_REG, 22 },
	{ "f23", GDB_SIZEOF_REG, 23 },
	{ "f24", GDB_SIZEOF_REG, 24 },
	{ "f25", GDB_SIZEOF_REG, 25 },
	{ "f26", GDB_SIZEOF_REG, 26 },
	{ "f27", GDB_SIZEOF_REG, 27 },
	{ "f28", GDB_SIZEOF_REG, 28 },
	{ "f29", GDB_SIZEOF_REG, 29 },
	{ "f30", GDB_SIZEOF_REG, 30 },
	{ "f31", GDB_SIZEOF_REG, 31 },
	{ "fsr", GDB_SIZEOF_REG, 0 },
	{ "fir", GDB_SIZEOF_REG, 0 },
};

int dbg_set_reg(int regno, void *mem, struct pt_regs *regs)
{
	int fp_reg;

	if (regno < 0 || regno >= DBG_MAX_REG_NUM)
		return -EINVAL;

	if (dbg_reg_def[regno].offset != -1 && regno < 38) {
		memcpy((void *)regs + dbg_reg_def[regno].offset, mem,
		       dbg_reg_def[regno].size);
	} else if (current && dbg_reg_def[regno].offset != -1 && regno < 72) {
		/* FP registers 38 -> 69 */
		if (!(regs->cp0_status & ST0_CU1))
			return 0;
		if (regno == 70) {
			/* Process the fcr31/fsr (register 70) */
			memcpy((void *)&current->thread.fpu.fcr31, mem,
			       dbg_reg_def[regno].size);
			goto out_save;
		} else if (regno == 71) {
			/* Ignore the fir (register 71) */
			goto out_save;
		}
		fp_reg = dbg_reg_def[regno].offset;
		memcpy((void *)&current->thread.fpu.fpr[fp_reg], mem,
		       dbg_reg_def[regno].size);
out_save:
		restore_fp(current);
	}

	return 0;
}

char *dbg_get_reg(int regno, void *mem, struct pt_regs *regs)
{
	int fp_reg;

	if (regno >= DBG_MAX_REG_NUM || regno < 0)
		return NULL;

	if (dbg_reg_def[regno].offset != -1 && regno < 38) {
		/* First 38 registers */
		memcpy(mem, (void *)regs + dbg_reg_def[regno].offset,
		       dbg_reg_def[regno].size);
	} else if (current && dbg_reg_def[regno].offset != -1 && regno < 72) {
		/* FP registers 38 -> 69 */
		if (!(regs->cp0_status & ST0_CU1))
			goto out;
		save_fp(current);
		if (regno == 70) {
			/* Process the fcr31/fsr (register 70) */
			memcpy(mem, (void *)&current->thread.fpu.fcr31,
			       dbg_reg_def[regno].size);
			goto out;
		} else if (regno == 71) {
			/* Ignore the fir (register 71) */
			memset(mem, 0, dbg_reg_def[regno].size);
			goto out;
		}
		fp_reg = dbg_reg_def[regno].offset;
		memcpy(mem, (void *)&current->thread.fpu.fpr[fp_reg],
		       dbg_reg_def[regno].size);
	}

out:
	return dbg_reg_def[regno].name;

}

struct kgdb_arch arch_kgdb_ops = {
#ifdef CONFIG_CPU_LITTLE_ENDIAN
	.gdb_bpt_instr = {0xd},
#else
	.gdb_bpt_instr = {0x00, 0x00, 0x00, 0x0d},
#endif
};

void arch_kgdb_breakpoint(void)
{
	__asm__ __volatile__(
		".globl breakinst\n\t"
		".set\tnoreorder\n\t"
		"nop\n"
		"breakinst:\tbreak\n\t"
		"nop\n\t"
		".set\treorder");
}

static void kgdb_call_nmi_hook(void *ignored)
{
	kgdb_nmicallback(raw_smp_processor_id(), NULL);
}

void kgdb_roundup_cpus(unsigned long flags)
{
	local_irq_enable();
	smp_call_function(kgdb_call_nmi_hook, NULL, 0);
	local_irq_disable();
}

static int compute_signal(int tt)
{
	struct hard_trap_info *ht;

	for (ht = hard_trap_info; ht->tt && ht->signo; ht++)
		if (ht->tt == tt)
			return ht->signo;

	return SIGHUP;		/* default for things we don't know about */
}

/*
 * Similar to regs_to_gdb_regs() except that process is sleeping and so
 * we may not be able to get all the info.
 */
void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	int reg;
	struct thread_info *ti = task_thread_info(p);
	unsigned long ksp = (unsigned long)ti + THREAD_SIZE - 32;
	struct pt_regs *regs = (struct pt_regs *)ksp - 1;
#if (KGDB_GDB_REG_SIZE == 32)
	u32 *ptr = (u32 *)gdb_regs;
#else
	u64 *ptr = (u64 *)gdb_regs;
#endif

	for (reg = 0; reg < 16; reg++)
		*(ptr++) = regs->regs[reg];

	/* S0 - S7 */
	for (reg = 16; reg < 24; reg++)
		*(ptr++) = regs->regs[reg];

	for (reg = 24; reg < 28; reg++)
		*(ptr++) = 0;

	/* GP, SP, FP, RA */
	for (reg = 28; reg < 32; reg++)
		*(ptr++) = regs->regs[reg];

	*(ptr++) = regs->cp0_status;
	*(ptr++) = regs->lo;
	*(ptr++) = regs->hi;
	*(ptr++) = regs->cp0_badvaddr;
	*(ptr++) = regs->cp0_cause;
	*(ptr++) = regs->cp0_epc;
}

void kgdb_arch_set_pc(struct pt_regs *regs, unsigned long pc)
{
	regs->cp0_epc = pc;
}

/*
 * Calls linux_debug_hook before the kernel dies. If KGDB is enabled,
 * then try to fall into the debugger
 */
static int kgdb_mips_notify(struct notifier_block *self, unsigned long cmd,
			    void *ptr)
{
	struct die_args *args = (struct die_args *)ptr;
	struct pt_regs *regs = args->regs;
	int trap = (regs->cp0_cause & 0x7c) >> 2;
	int ss_trap = trap;
	int error;

	PRINTK("%s at 0x%lx (trap %d)\n", __func__,
	       regs->cp0_epc, trap);

#ifdef CONFIG_KPROBES
	/*
	 * Return immediately if the kprobes fault notifier has set
	 * DIE_PAGE_FAULT.
	 */
	if (cmd == DIE_PAGE_FAULT)
		return NOTIFY_DONE;
#endif /* CONFIG_KPROBES */

	/* Userspace events, ignore. */
	if (user_mode(regs))
		return NOTIFY_DONE;

	if (atomic_read(&kgdb_active) != -1)
		kgdb_nmicallback(smp_processor_id(), regs);

	if (trap == 9 && stepped_opcode != 0) {
		PRINTK("Step done at 0x%lx putting back 0x%x\n",
		       stepped_address, stepped_opcode);

		/* restores original instruction */
		error = probe_kernel_write((char *)stepped_address,
					   (char *) &stepped_opcode,
					   BREAK_INSTR_SIZE);
		if (error != 0) {
			PRINTK("Unable to restore original instruction\n");
			printk(KERN_CRIT "KGDB: FATAL ERROR on instruction" \
			       "restore at 0x%lx", stepped_address);
		}

		flush_icache_range(stepped_address,
				   stepped_address + 4);

		if (regs->cp0_epc == stepped_address)
			ss_trap = 0;

		/* Restore original interrupts in cpsr regs */
		regs->cp0_status |= stepped_cp0_status_ie;
		stepped_opcode = 0;
	}
#ifdef DEBUG_IT
	else {
		int op;
		unsigned long addr = regs->cp0_epc;

		PRINTK("%s: It's very likely an actual BP.\n", __func__);

		if (probe_kernel_read((unsigned char *) &op,
				      (char *) addr,
				      BREAK_INSTR_SIZE) != 0) {
			PRINTK("Unable to read memory at 0x%lx\n", addr);
		} else {
			PRINTK("op is 0x%x\n", op);
			if (op != *(unsigned long *)arch_kgdb_ops.gdb_bpt_instr)
				PRINTK("OOPS this is not a bp !\n");
			else
				PRINTK("YES this is a bp !\n");
		}
	}
#endif /* DEBUG_IT */

	if (kgdb_handle_exception(ss_trap, compute_signal(trap), cmd, regs))
		return NOTIFY_DONE;

	if (atomic_read(&kgdb_setting_breakpoint))
		if ((trap == 9) && (regs->cp0_epc == (unsigned long)breakinst))
			regs->cp0_epc += 4;

	/* In SMP mode, __flush_cache_all does IPI */
	local_irq_enable();
	__flush_cache_all();

	return NOTIFY_STOP;
}

#ifdef CONFIG_KGDB_LOW_LEVEL_TRAP
int kgdb_ll_trap(int cmd, const char *str,
		 struct pt_regs *regs, long err, int trap, int sig)
{
	struct die_args args = {
		.regs	= regs,
		.str	= str,
		.err	= err,
		.trapnr = trap,
		.signr	= sig,

	};

	if (!kgdb_io_module_registered)
		return NOTIFY_DONE;

	return kgdb_mips_notify(NULL, cmd, &args);
}
#endif /* CONFIG_KGDB_LOW_LEVEL_TRAP */

static struct notifier_block kgdb_notifier = {
	.notifier_call = kgdb_mips_notify,
};

/*
 * Handle the 's' and 'c' commands
 */
int kgdb_arch_handle_exception(int vector, int signo, int err_code,
			       char *remcom_in_buffer, char *remcom_out_buffer,
			       struct pt_regs *regs)
{
	char *ptr;
	unsigned long address;
	int cpu = smp_processor_id();
	int error;

	switch (remcom_in_buffer[0]) {
	case 's': {
		unsigned long next_addr;

		PRINTK("KGDB: s command at 0x%lx\n", regs->cp0_epc);

		/* handle the optional parameter */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long (&ptr, &address))
			regs->cp0_epc = address;

		atomic_set(&kgdb_cpu_doing_single_step, -1);

		next_addr = mipsGetNpc(regs);
		stepped_address = next_addr;
		PRINTK("KGDB next pc 0x%lx\n", next_addr);

		/* Saves original instruction */
		error = probe_kernel_read((char *) &stepped_opcode,
					  (char *)next_addr,
					  BREAK_INSTR_SIZE);
		if (error != 0) {
			PRINTK("Unable to access opcode at next pc 0x%lx\n",
			       next_addr);
			return error;
		}

		/* Sets the temporary breakpoint */
		error = probe_kernel_write((char *)next_addr,
					   arch_kgdb_ops.gdb_bpt_instr,
					   BREAK_INSTR_SIZE);
		if (error != 0) {
			PRINTK("Unable to write tmp BP at next pc 0x%lx\n",
			       next_addr);
			return error;
		}

		stepped_cp0_status_ie = regs->cp0_status & ST0_IE;

		/* masks interrupts */
		regs->cp0_status &= ~ST0_IE;

		atomic_set(&kgdb_cpu_doing_single_step, cpu);

		PRINTK("step armed over 0x%lx\n", regs->cp0_epc);

		return 0;
	}
	case 'c':
		PRINTK("KGDB: c command at 0x%lx\n", regs->cp0_epc);

		/* handle the optional parameter */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &address))
			regs->cp0_epc = address;

		atomic_set(&kgdb_cpu_doing_single_step, -1);

		PRINTK("%s done OK\n", __func__);

		return 0;
	}

	return -1;
}

struct kgdb_arch arch_kgdb_ops;

/*
 * We use kgdb_early_setup so that functions we need to call now don't
 * cause trouble when called again later.
 */
int kgdb_arch_init(void)
{
	union mips_instruction insn = {
		.r_format = {
			.opcode = spec_op,
			.func	= break_op,
		}
	};
	memcpy(arch_kgdb_ops.gdb_bpt_instr, insn.byte, BREAK_INSTR_SIZE);

	register_die_notifier(&kgdb_notifier);

	return 0;
}

/*
 *	kgdb_arch_exit - Perform any architecture specific uninitalization.
 *
 *	This function will handle the uninitalization of any architecture
 *	specific callbacks, for dynamic registration and unregistration.
 */
void kgdb_arch_exit(void)
{
	unregister_die_notifier(&kgdb_notifier);
}

/* Copyright (c) 1996-2001 Wind River Systems, Inc. */
/* <WRS_ADDED> */
static unsigned long mipsGetNpc(struct pt_regs *pRegs)
{
	int	rsVal;
	int	rtVal;
	int ptr;
	unsigned long	disp;
	unsigned int  machInstr;
	unsigned long npc;
	unsigned long pc;

	if (pRegs == NULL)
		panic("%s: NULL pRegs !\n", __func__);

#if 0
	/*
	 * If we are in a branch delay slot, the pc has been changed
	 * in the breakpoint handler to match with the breakpoint
	 * address.  It is modified to have its normal value.
	 */

	if (pRegs->cp0_cause & CAUSE_BD)
		pRegs->cp0_epc--;
#endif	/* 0 */

	pc        = pRegs->cp0_epc;
	machInstr = *(unsigned int *)pc;

	/* Default instruction is the next one. */

	npc = pc + 4;

	/*
	 * Do not report the instruction in a branch delay slot as the
	 * next pc.  Doing so will mess up the WDB_STEP_OVER case as
	 * the branch instruction is re-executed.
	 */

	/*
	 * Check if we are on a branch likely instruction, which will nullify
	 * the instruction in the slot if the branch is taken.
	 * Also, pre-extract some of the instruction fields just to make coding
	 * easier.
	 */

	rsVal = pRegs->regs[(machInstr >> 21) & 0x1f];
	rtVal = pRegs->regs[(machInstr >> 16) & 0x1f];
	ptr   = (machInstr >> 16) & 0x1f;
	disp = ((int) ((machInstr & 0x0000ffff) << 16)) >> 14;
	if ((machInstr & 0xf3ff0000) == 0x41020000)	{
		/* BCzFL  */
		int copId = (machInstr >> 26) & 0x03;
		npc = pc + 8;
		switch (copId) {
		case 1:
#if 0
#ifndef SOFT_FLOAT
			if ((pRegs->fpcsr & FP_COND) != FP_COND)
				npc = disp + pc + 4;
#endif	/* !SOFT_FLOAT */
#endif /* 0 */
			break;
		}
	} else if ((machInstr & 0xf3ff0000) == 0x41030000) {
		/* BCzTL  */
		int copId = (machInstr >> 26) & 0x03;
		npc = pc + 8;
		switch (copId) {
		case 1:
#if 0
#ifndef SOFT_FLOAT
			if ((pRegs->fpcsr & FP_COND) == FP_COND)
				npc = disp + pc + 4;
#endif	/* !SOFT_FLOAT */
#endif /* 0 */
			break;
		}
	} else if (((machInstr & 0xfc1f0000) == 0x04130000)
		   || ((machInstr & 0xfc1f0000) == 0x04030000)) {
		/* BGEZALL*/
		/* BGEZL  */
		if (rsVal >= 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc1f0000) == 0x5c000000) {
		/* BGTZL  */
		if (rsVal > 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc1f0000) == 0x58000000) {
		/* BLEZL  */
		if (rsVal <= 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if (((machInstr & 0xfc1f0000) == 0x04120000)
		   || ((machInstr & 0xfc1f0000) == 0x04020000)) {
		/* BLTZALL*/
		/* BLTZL  */
		if (rsVal < 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc000000) == 0x50000000) {
		/* BEQL   */
		if (rsVal == rtVal)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc000000) == 0x54000000) {
		/* BNEL   */
		if (rsVal != rtVal)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if (((machInstr & 0xfc000000) == 0x08000000) ||
		   ((machInstr & 0xfc000000) == 0x0c000000)) {
		/* J    */
		/* JAL  */
		npc = ((machInstr & 0x03ffffff) << 2) |
#ifdef CONFIG_CPU_MIPS64
	       (pc        & 0xfffffffff0000000ULL);
#else
	       (pc        & 0xf0000000);
#endif
	} else if (((machInstr & 0xfc1f07ff) == 0x00000009)
		   || ((machInstr & 0xfc1fffff) == 0x00000008)) {
		/* JALR */
		/* JR   */
		npc = pRegs->regs[(machInstr >> 21) & 0x1f];
	} else if ((machInstr & 0xf3ff0000) == 0x41000000) {
		/* BCzF   */
		int copId = (machInstr >> 26) & 0x03;
		npc = pc + 8;
		switch (copId) {
		case 1:
#if 0
#ifndef SOFT_FLOAT
			if ((pRegs->fpcsr & FP_COND) != FP_COND)
				npc = disp + pc + 4;
#endif	/* !SOFT_FLOAT */
#endif /* 0 */
			break;
		}
	} else if ((machInstr & 0xf3ff0000) == 0x41010000) {
		/* BCzT   */
		int copId = (machInstr >> 26) & 0x03;
		npc = pc + 8;
		switch (copId) {
		case 1:
#if 0
#ifndef SOFT_FLOAT
			if ((pRegs->fpcsr & FP_COND) == FP_COND)
				npc = disp + pc + 4;
#endif	/* !SOFT_FLOAT */
#endif /* 0 */
			break;
		}
	} else if ((machInstr & 0xfc000000) == 0x10000000) {
		/* BEQ    */
		if (rsVal == rtVal)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if (((machInstr & 0xfc1f0000) == 0x04010000)
		   || ((machInstr & 0xfc1f0000) == 0x04110000)) {
		/* BGEZ   */
		/* BGEZAL */
		if (rsVal >= 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc1f0000) == 0x1c000000) {
		/* BGTZ   */
		if (rsVal > 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc1f0000) == 0x18000000) {
		/* BLEZ   */
		if (rsVal <= 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if (((machInstr & 0xfc1f0000) == 0x04000000)
		   || ((machInstr & 0xfc1f0000) == 0x04100000)) {
		/* BLTZ   */
		/* BLTZAL */
		if (rsVal < 0)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	} else if ((machInstr & 0xfc000000) == 0x14000000) {
		/* BNE    */
		if (rsVal != rtVal)
			npc = disp + pc + 4;
		else
			npc = pc + 8;
	}
   /* Cavium specific */

	else if ((machInstr & 0xfc000000) == 0xc8000000) {
		/* BBIT0  */
		/* branch if bit is Zero */
		if ((rsVal >> ptr) & 1)
			npc = pc + 8;
		else /* cond is true */
			npc = disp + pc + 4;
	} else if ((machInstr & 0xfc000000) == 0xd8000000) {
		/* BBIT032  */
		/* branch if bit is Zero */
		if ((rsVal >> (ptr + 32)) & 1)
			npc = pc + 8;
		else /* cond is true */
			npc = disp + pc + 4;
	} else if ((machInstr & 0xfc000000) == 0xe8000000) {
		/* BBIT1  */
		/* branch if bit is Set */
		if ((rsVal >> ptr) & 1)
			npc = disp + pc + 4;
		else /* cond is true */
			npc = pc + 8;
	} else if ((machInstr & 0xfc000000) == 0xf8000000) {
		/* BBIT132  */
		/* branch if bit is Set */
		if ((rsVal >> (ptr + 32)) & 1)
			npc = disp + pc + 4;
		else /* cond is true */
			npc = pc + 8;
	} else {
		/* normal instruction */
	}

	return npc;
}
/* </WRS_ADDED> */
