/*
 * Copyright (c) 2013
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _PROC_H_
#define _PROC_H_

/*
 * Definition of a process.
 *
 * Note: curproc is defined by <current.h>.
 */

#include <spinlock.h>
#include <thread.h> /* required for struct threadarray */
#include <synch.h> /* required for synchronization primitives*/
#include <limits.h> /* PID_MAX and PID_MIN are defined inside this header file*/
#include <array.h> /* Define a process array by this header file*/
#include <bitmap.h> /* Use bitmap to determine the pid whether used or not*/
#include <kern/errno.h>
#include <opt-A2.h>

struct addrspace;
struct vnode;
#ifdef UW
struct semaphore;
#endif // UW

#if OPT_A2
struct proc;        // forward declaration
#endif /* OPT_A2 */
/* Since a parent process will have multiple chilren processes, we use array to
 * store these children prcesses. And we will use the definition in array.h to initialize
 * the process array
 */

#if OPT_A2

// Bits for declaring and defining typed arrays
#ifndef PROCINLINE
#define PROCINLINE INLINE
#endif

DECLARRAY(proc);
DEFARRAY(proc, PROCINLINE);

#endif /* OPT_A2 */

/*
 * Process structure.
 */
struct proc {
	char *p_name;			             /* Name of this process */
	struct spinlock p_lock;		 /* Lock for this structure */
	struct threadarray p_threads;	             /* Threads in this process */

	/* VM */
	struct addrspace *p_addrspace;	  /* virtual address space */

	/* VFS */
	struct vnode *p_cwd;		               /* current working directory */

#ifdef UW
            /* a vnode to refer to the console device */
            /* this is a quick-and-dirty way to get console writes working */
            /* you will probably need to change this when implementing file-related
               system calls, since each process will need to keep track of all files
               it has opened, not just the console. */
            struct vnode *console;                          /* a vnode for the console device */
#endif

#if OPT_A2
            pid_t p_pid;                              /* the unique pid of the process */
            bool  p_exited;                         /* whether this process exited */
            int     p_exit_status;                 /* the exit status of the process */
            bool  p_parent_exited;             /* whther the parent of this process exited or not*/
            struct procarray p_children;    /* an array storing children processes */
            struct lock *p_child_lock;        /* ensure that everytime we only have one thread runs the child process*/
            struct lock *p_exit_lock;          /* ensure that there is only one process exiting every time*/
            struct cv *p_exit_cv;                /* ensure that when the process exited, wake all waiting processes*/

#endif /* OPT_A2*/
};

/* This is the process structure for the kernel and for kernel-only threads. */
extern struct proc *kproc;

/* Semaphore used to signal when there are no more processes */
#ifdef UW
extern struct semaphore *no_proc_sem;
#endif // UW

#if OPT_A2
struct bitmap *pid_map; // pid_map to assign pid, reused pid; pid is for the whole system, not a single user
struct spinlock pid_lock; // spinlock to ensure mutual exclusion

struct proc * proc_fork(struct proc * proc); // to fork the parent's process
int proc_assign_pid(pid_t *pid); // when creating the process, assign a pid to it, no matter it is fresh or reused before
void proc_set_pid_unused(pid_t pid); // when a proc exits, set the bit unmark, and then we can reuse it

void proc_signal_children(struct proc *proc);
void proc_destroy_holdon(struct proc *proc);
#endif /* OPT_A2*/

/* Call once during system startup to allocate data structures. */
void proc_bootstrap(void);

/* Create a fresh process for use by runprogram(). */
struct proc *proc_create_runprogram(const char *name);

/* Destroy a process. */
void proc_destroy(struct proc *proc);

/* Attach a thread to a process. Must not already have a process. */
int proc_addthread(struct proc *proc, struct thread *t);

/* Detach a thread from its process. */
void proc_remthread(struct thread *t);

/* Fetch the address space of the current process. */
struct addrspace *curproc_getas(void);

/* Change the address space of the current process, and return the old one. */
struct addrspace *curproc_setas(struct addrspace *);


#endif /* _PROC_H_ */
