#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <spl.h>
#include <mips/trapframe.h>
#include <synch.h>
#include <limits.h>
#include <kern/fcntl.h>
#include <vfs.h>
#include <opt-A2.h>
#include <opt-A3.h>
  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */


void sys__exit(int exitcode) {
    struct addrspace *as;
    struct proc *p = curproc;
    /* for now, just include this to keep the compiler from complaining about
      an unused variable */
    (void)exitcode;

    DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

    KASSERT(curproc->p_addrspace != NULL);
    as_deactivate();
    /*
    * clear p_addrspace before calling as_destroy. Otherwise if
    * as_destroy sleeps (which is quite possible) when we
    * come back we'll be calling as_activate on a
    * half-destroyed address space. This tends to be
    * messily fatal.
    */
    as = curproc_setas(NULL);
    as_destroy(as);

/* There are two situations when a current process exit: 1. its parent does not exit
 * 2. his parent exited. Note that we do not care much about the second situation.
 * The reasons are that if the parent exit, it will scan all of its children and destroy them.
 * However, if a child process exit but its parent exists, this parent may still waitpid on
 * this child process if the parent is too busy to know that this child has exited.
 */

#if OPT_A2

    lock_acquire(p->p_exit_lock);

    if (!p->p_parent_exited && p->p_pid > 1)
    {
        // well, the parent exits. we should wait for parent to deal with exit of child.
        // Step 1. handle the exitcode
#if OPT_A3
        p->p_exit_status = _MKWAIT_SIG(exitcode);
#else
        p->p_exit_status = _MKWAIT_EXIT(exitcode);
#endif /*OPT_A3*/
        // Step 2. wake up all waiting threads
        p->p_exited = true; // "Hurray, I exited", then "so what????"
        cv_broadcast(p->p_exit_cv, p->p_exit_lock);

        // Step 3. Notify this process its all children that their parent has exited
        proc_signal_children(p);

        // Step 4. /* detach this thread from its process, note: curproc cannot be used after this call */
        proc_remthread(curthread);

        // Step 5. destroy this process, but does not fully destoy,
        //              let the parent decides when it calls waitpid on it
        proc_destroy_holdon(p);

        lock_release(p->p_exit_lock);
    }
    else
    {
        proc_signal_children(p);
        /* detach this thread from its process */
        /* note: curproc cannot be used after this call */
        proc_remthread(curthread);

        lock_release(p->p_exit_lock);

        /* if this is the last user process in the system, proc_destroy()
            will wake up the kernel menu thread */
        proc_destroy(p);
    }
    thread_exit();
    /* thread_exit() does not return, so we should never get here */
    panic("return from thread_exit in sys_exit\n");

#else
    /* detach this thread from its process */
    /* note: curproc cannot be used after this call */
    proc_remthread(curthread);

    /* if this is the last user process in the system, proc_destroy()
      will wake up the kernel menu thread */
    proc_destroy(p);

    thread_exit();
    /* thread_exit() does not return, so we should never get here */
    panic("return from thread_exit in sys_exit\n");
#endif /* OPT_A2*/
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
#if OPT_A2

    *retval = curproc->p_pid;
    return 0;

#else
    /* for now, this is just a stub that always returns a PID of 1 */
    /* you need to fix this to make it work properly */
    *retval = 1;
    return(0);
#endif /*OPT_A2*/
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
    int exitstatus;
    int result;

/*Just acquire the exitlock and then see if the process has exited,
 *then wait it exit using cv_wait on exitcv and get it's exitcode.
 *See user/testbin/badcall/bad_waitpid.c
 */
#if OPT_A2

    // Step 1. Is the status pointer properly aligned (by 4) ?
    if (((uintptr_t)status % 4) != 0)
    {
        return EFAULT;
    }

    // Step 2. Is the status pointer a valid pointer anyway (NULL, point to kernel, ...)?
    if (status == NULL)
    {
        return EFAULT;
    }

    // Step 3. Is options valid? (More flags than WNOHANG | WUNTRACED )
    if (options != 0)
    {
        return EINVAL;
    }

    // Step4. Does the waited pid exist/valid?
    if (pid < 1 || pid >= PID_MAX)
    {
        return ESRCH;
    }

    // Step 5. If exist, are we allowed to wait it ? (Is it our child?)
    struct proc * child_proc = NULL;
    spinlock_acquire(&pid_lock);
    int set = bitmap_isset(pid_map, pid);
    spinlock_release(&pid_lock);
    if (set)
    {
        if (curproc == NULL)
        {
            return ESRCH;
        }

        lock_acquire(curproc->p_child_lock);
        unsigned int num = procarray_num(&curproc->p_children);
        for (unsigned int i = 0; i < num; i++)
        {
            struct proc* temp_proc= procarray_get(&curproc->p_children, i);
            if (temp_proc->p_pid == pid)
            {
                child_proc = temp_proc;
                break;
            }
        }
        if (child_proc == NULL)
        {
            return ECHILD;
        }
        lock_release(curproc->p_child_lock);
        lock_acquire(child_proc->p_exit_lock);
        if (!child_proc->p_exited)
        {
            cv_wait(child_proc->p_exit_cv, child_proc->p_exit_lock);
        }
        exitstatus = child_proc->p_exit_status;
        lock_release(child_proc->p_exit_lock);

        result = copyout((void *)&exitstatus,status,sizeof(int));
        if (result)
        {
            return result;
        }
        *retval = pid;
        return 0;

    }
    else
    {
        spinlock_release(&pid_lock);
        return ESRCH;
    }



#else
    /* this is just a stub implementation that always reports an
      exit status of 0, regardless of the actual exit status of
      the specified process.
      In fact, this will return 0 even if the specified process
      is still running, and even if it never existed in the first place.

      Fix this!
    */

    if (options != 0) {
      return(EINVAL);
    }
    /* for now, just pretend the exitstatus is 0 */
    exitstatus = 0;
    result = copyout((void *)&exitstatus,status,sizeof(int));
    if (result) {
      return(result);
    }
    *retval = pid;
    return(0);
#endif /*OPT_A2*/
}

#if OPT_A2
void child_fork_entry(void* p_tf, unsigned long p_as); // forward declaration
#endif

#if OPT_A2
int
sys_fork(pid_t *retval, struct trapframe *tf)
{
    // Step 1.pass parent's trap from to child thread
    struct trapframe *child_tf = kmalloc(sizeof(struct trapframe));
    if (child_tf == NULL)
    {
        //Sufficient virtual memory for the new process was not available.
        kfree(child_tf);
        return ENOMEM;
    }
    memcpy((void *)child_tf, (const void*) tf, sizeof(struct trapframe));

    // Step2. Copy parent's address space

    struct proc * child_proc = proc_fork(curproc);
    if (child_proc == NULL)
    {
        kfree(child_tf);
        *retval = -1;
        return ENOMEM;
    }

    // Step3. Create child thread
    // we need to prevent child thread from running until parent thread set everything up.
    // So, before thread_fork copying the threads, we need to disable all interrupts to fork safely.
    int spl = splhigh(); // disable interrupts before thread_fork using splhigh
    int ret = thread_fork(curthread->t_name, child_proc, &child_fork_entry, (void *)child_tf, (unsigned long) curproc->p_addrspace);
    splx(spl); // restore the old interrupt level using splx after parent thread is done

    // Step 4. Check the return value
    if (ret)
    {
        // if the return value is not equal to 0, the thread_fork fails
        proc_destroy(child_proc);
        kfree(child_tf);
        *retval = -1;
        return ret;
    }
    *retval = child_proc->p_pid;

    // the syscall code is 0, reserved for the fork, so basically, it should return 0.
    return 0;
}
#endif /* OPT_A2 */

#if OPT_A2
void
child_fork_entry(void* p_tf, unsigned long p_as)
{
    (void)p_as;
    // Activate our address space
    as_activate();

    struct trapframe tf;
    struct trapframe *tf_copy = (struct trapframe *) p_tf;
    memcpy((void *)&tf, (const void*) tf_copy, sizeof(struct trapframe));
    kfree(tf_copy);

    enter_forked_process(&tf);
}
#endif /* OPT_A2 */

#if OPT_A2
int
sys_execv(userptr_t progname, userptr_t args)
{
    // Step 1. Count the number of arguments and copy them into the kernel

    // Declare the new programe name, path, and related arguments
    char* new_progname = kmalloc(PATH_MAX); // by definition, Longest full path name
    size_t new_progname_len;
    char ** new_args = kmalloc(ARG_MAX); //Max bytes for an exec function */
    unsigned int argc= 0;
    // argc: number of arguments

    /*copyin: Copy a block of memory of length LEN from user-level address USERSRC
     * to kernel address DEST
     copyout: Copy a block of memory of length LEN from kernel address SRC to
     * user-level address USERDEST
     copyinstr: Copy a string from user-level address USERSRC to kernel address
     * DEST
     copyoutstr: Copy a string from kernel address SRC to user-level address
     * USERDEST*/

    // Make sure that none of the args are invalid pointer
    if ((char*)progname == NULL)
    {
        kfree(new_progname);
        kfree(new_args);
        return ENOENT;
    }
    if ((char**)args == NULL)
    {
        kfree(new_progname);
        kfree(new_args);
        return EFAULT;
    }

    // Now, we need to count the number of arguments and copy them into the kernel
    while (true)
    {
        char* temp_args = NULL;
        int copyarg_result = copyin(args+argc*sizeof(char*), &temp_args, sizeof(char*));
        if (copyarg_result != 0)
        {
            kfree(new_progname);
            kfree(new_args);
            return copyarg_result;
        }
        new_args[argc] = temp_args;
        if (temp_args != NULL)
        {
            argc += 1;
        }
        else
        {
            break;
        }
    }

    // Prepare to put items on stack by copying them
    char* argv = kmalloc(ARG_MAX); // the items' array we are going to store
    // size_t /* Size of a memory region */
    size_t* argv_count = kmalloc(argc * sizeof(size_t)); // pointer that points the item
    unsigned int count = 0;
    unsigned int counter = 0;
    while (counter < argc)
    {
        size_t argv_len;
        int argcopy_result = copyinstr((userptr_t)new_args[counter], argv+count, ARG_MAX - count, &argv_len);
        argv_len = argv_len + 1; // include the null-terminator
        if (argcopy_result != 0)
        {
            kfree(new_progname);
            kfree(new_args);
            kfree(argv);
            kfree(argv_count);
            return argcopy_result;
        }
        argv_count[counter] = count;
        count = count + ROUNDUP(argv_len, 8); // pad each item such that they are 8-byte aligned
        counter = counter + 1;
    }

    if (argc > 64) // The total size of the argument strings is too large
    {
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return E2BIG;   // By definition, ARG_MAX = 64*1024.
    }

    // Step 2.Copy the program path into the kernel
    int progkernel_result = copyinstr(progname, new_progname, PATH_MAX, &new_progname_len);
    if (progkernel_result != 0)
    {
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return progkernel_result;
    }

    // Step 3.  Need to copy the arguments into the new address space
    // But before that, we need to create new address and this part is very similar to the runprogram.c
    struct addrspace *new_as;
    // A struct vnode is an abstract representation of a file.
    struct vnode *new_v;
    //vaddr_t(virtual address pointer) : __u32 - unsigned integer type with 8,16,32 bits in it.
    vaddr_t new_entrypoint, new_stackptr, new_argvptr;
    int result;

    /* Open the file. */
    result = vfs_open(new_progname, O_RDONLY, 0, &new_v);
    if (result)
    {
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return result;
    }

    /* Create a new address space. */
    new_as = as_create();
    if (new_as ==NULL)
    {
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        vfs_close(new_v);
        return ENOMEM;
    }

    /* Switch to it and activate it. */
    // before swtich to the new address, we should get the curproc_as,
    // otherwise, we can not deleter the curproc_address later
    struct addrspace* curproc_as = curproc_getas();
    curproc_setas(new_as);
    as_activate();

    /* Load the executable. */
    result = load_elf(new_v, &new_entrypoint);
    if (result)
    {
        /* p_addrspace will go away when curproc is destroyed */
        vfs_close(new_v);
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return result;
    }

    /* Done with the file now. */
    vfs_close(new_v);

    /* Define the user stack in the address space */
    result = as_define_stack(new_as, &new_stackptr);
    if (result)
    {
        /* p_addrspace will go away when curproc is destroyed */
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return result;
    }

    // Now, we need to calculate the userspace address for argv
    // new_stackptr = USERSTACK, the starting value for the stack pointer at user leve
    // the stack is subtract-then-store, this can start as the next address after the stack area.
    // We put the stack at the very top of user virtual memory because it grows downwards.

    // First, we should know that we should store argv in kernel and then store in the user mode
    new_argvptr = new_stackptr;
    new_argvptr = new_argvptr - count;
    int prog_result = copyout(argv, (userptr_t)new_argvptr, (size_t)count);
    // copyout: from the kernel to the userdest, argv stores in the kernel address, okay, it is time to get it out
    if (prog_result != 0)
    {
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return prog_result;
    }

    // Now, copy the argv
    /*userptr_t: Define userptr_t as a pointer to a one-byte struct, so it won't mix
    * with other pointers.*/
    userptr_t * argv_ptr = kmalloc(sizeof(userptr_t) * (argc + 1)); // +1 indicates we should include the null-terminator
    counter = 0;
    while (counter < argc)
    {
        userptr_t temp_ptr = (userptr_t)new_argvptr + argv_count[counter];
        argv_ptr[counter] = temp_ptr;
        counter  = counter + 1;
    }
    argv_ptr[argc] = NULL; // the last one is NULL

    new_argvptr = new_argvptr - sizeof(userptr_t) * (argc + 1); // move downwards, new_argvptr now is the top of the stack
    int argv_result = copyout(argv_ptr, (userptr_t)new_argvptr, sizeof(userptr_t) * (argc+1));
    if (argv_result != 0)
    {
        kfree(new_progname);
        kfree(new_args);
        kfree(argv);
        kfree(argv_count);
        return argv_result;
    }
    // Now, we basically finishes passing the argv and progname

    // Step 4. Delete the old address space
    // Now, we need kfree all the allocated arrays or pointers
    kfree(argv_ptr);
    kfree(argv_count);
    kfree(argv);
    kfree(new_args);
    kfree(new_progname);
    as_destroy(curproc_as);

    // Step 5. Call enter_new_process with address to the arguments on the stack,
    // the stack pointer (from as_define_stack), and the program entry point (from vfs_open)

    /* Warp to user mode. */
    enter_new_process(argc /*argc*/, (userptr_t)new_argvptr /*userspace addr of argv*/,
              new_argvptr, new_entrypoint);

    /* enter_new_process does not return. */
    panic("enter_new_process returned\n");
    return EINVAL;
}
#endif

#if OPT_A3
void
kill_curthread_exit(int sig)
{
    sys__exit(sig);
}
#endif
