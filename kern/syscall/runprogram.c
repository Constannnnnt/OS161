/*
 * Copyright (c) 2000, 2001, 2002, 2003, 2004, 2005, 2008, 2009
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

/*
 * Sample/test code for running a user program.  You can use this for
 * reference when implementing the execv() system call. Remember though
 * that execv() needs to do more than this function does.
 */

#include <types.h>
#include <kern/errno.h>
#include <kern/fcntl.h>
#include <lib.h>
#include <proc.h>
#include <current.h>
#include <addrspace.h>
#include <vm.h>
#include <vfs.h>
#include <syscall.h>
#include <test.h>
#include <copyinout.h>
#include <opt-A2.h>

/*
 * Load program "progname" and start running it in usermode.
 * Does not return except on error.
 *
 * Calls vfs_open on progname and thus may destroy it.
 */
int
runprogram(char *progname, unsigned long argc, char**argv)
{
	struct addrspace *as;
	struct vnode *v;
	vaddr_t entrypoint, stackptr, argvptr;
	int result;

	if (argc > 64)
	{
		return E2BIG;
	}

	/* Open the file. */
	result = vfs_open(progname, O_RDONLY, 0, &v);
	if (result) {
		return result;
	}

	/* We should be a new process. */
	KASSERT(curproc_getas() == NULL);

	/* Create a new address space. */
	as = as_create();
	if (as ==NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	/* Switch to it and activate it. */
	curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);

	/* Define the user stack in the address space */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}

#if OPT_A2

	//Now, we pass the argc and argv here, and we need to handle it
	// The difference here is that we now already know the argc
	// Also, pay attention to that the strlen does not include the null so we need to add it by ourself
	char* args = kmalloc(ARG_MAX);
	size_t* args_count = kmalloc(argc * sizeof(size_t));
	unsigned int count = 0;
	unsigned int counter = 0;
	while (counter < argc)
	{
		size_t arg_len = strlen(argv[counter]) + 1; // include the null-terminator
		if (count + arg_len > ARG_MAX) // exceed the ARG_MAX
		{
			kfree(args);
			kfree(args_count);
			return E2BIG;
		}
		strcpy(args + count, argv[counter]); // copy the arg into the argv
		args_count[counter] = count;
		count  = count + ROUNDUP(arg_len, 8);
		counter = counter + 1;
	}

	// First, we should know that we should store argv in kernel and then store in the user mode
	argvptr = stackptr;
	argvptr = argvptr - count;
	int prog_result = copyout(args, (userptr_t)argvptr, count);
	if (prog_result != 0)
	{
		kfree(args);
		kfree(args_count);
		return prog_result;
	}

	// Now, copy the argv
	userptr_t*args_ptr = kmalloc(sizeof(userptr_t) * (argc + 1)); // include the NULL pointer
	counter = 0;
	while (counter < argc)
	{
		userptr_t temp_ptr = (userptr_t)argvptr + args_count[counter];
		args_ptr[counter] = temp_ptr;
		counter  = counter + 1;
	}
	args_ptr[argc] = NULL; // the last arg is NULL

	// move the userstack downwards
	argvptr = argvptr - sizeof(userptr_t) * (argc + 1);
	int argv_result = copyout(args_ptr, (userptr_t)argvptr, sizeof(userptr_t) * (argc + 1)); //copy to the kernel
	if (argv_result != 0)
	{
		kfree(args_ptr);
		kfree(args_count);
		kfree(args);
		return argv_result;
	}
	// Finish Copying

	// Now, it is time to kree the allocated memory
	kfree(args_ptr);
	kfree(args_count);
	kfree(args);
#endif /*OPT_A2*/

	/* Warp to user mode. */
	enter_new_process(argc /*argc*/, (userptr_t)argvptr /*userspace addr of argv*/,
			  argvptr, entrypoint);

	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;
}

