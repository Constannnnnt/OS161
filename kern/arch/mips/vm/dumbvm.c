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

#include <types.h>
#include <kern/errno.h>
#include <lib.h>
#include <spl.h>
#include <spinlock.h>
#include <proc.h>
#include <current.h>
#include <mips/tlb.h>
#include <addrspace.h>
#include <vm.h>
#include <opt-A3.h>

/*
 * Dumb MIPS-only "VM system" that is intended to only be just barely
 * enough to struggle off the ground.
 */

/* under dumbvm, always have 48k of user stack */
#define DUMBVM_STACKPAGES    12

/*
 * Wrap rma_stealmem in a spinlock.
 */
static struct spinlock stealmem_lock = SPINLOCK_INITIALIZER;

#if OPT_A3
static int vm_init = 0; // check whether the vm has been initialized already
static uint32_t *frame_size_list; // the list that stores the frame size
static uint32_t *frame_available_list; // mark the availbale list
static int frame_max_num; // the maximum number of frame
static int frame_available; // the first available frame
static paddr_t frame_mem_start; // start of the memory
static paddr_t frame_mem_end; // end of the memory
static bool check_frame_availability(int frame_position); // check whether this frame is available
static void set_frame_availablity(int frame_position, bool available); // set the frame available or unavailable
static paddr_t alloc_frame(int npages); // allocate a single frame
static void free_frame(paddr_t frame); // free a single frame
#endif /*OPT_A3*/

// Max mem is less than 512MB = 131072 frames -> one bit each :16384 bytes = 4 frames

void
vm_bootstrap(void)
{
#if OPT_A3
	// ram_getsize: in order to find out what memory it has available to manage.
	// should not be called once the VM system is initialized
	ram_getsize(&frame_mem_start, &frame_mem_end);

	// intialize the available frame list
	frame_available_list = (uint32_t *) PADDR_TO_KVADDR(frame_mem_start);
	frame_mem_start = frame_mem_start + sizeof(uint32_t) * PAGE_SIZE;
	// calculate the maximum frame number
	frame_max_num = (frame_mem_end - frame_mem_start) / (PAGE_SIZE + 4);
	frame_mem_end = frame_mem_end - sizeof(uint32_t) * frame_max_num;
	// initialize the first available frame
	frame_available = 0;
	//initialize the frame size list
	frame_size_list = (uint32_t *) PADDR_TO_KVADDR(frame_mem_end);
	int num_frame = frame_max_num / 32;
	int frame_counter = 0;
	// clear the bit in the frame size list
	while (frame_counter < frame_max_num)
	{
		frame_size_list[frame_counter] = 0;
		frame_counter = frame_counter + 1;
	}
	// clear the bit in the frame size list
	frame_counter = 0;
	while (frame_counter < num_frame)
	{
		frame_available_list[frame_counter] = 0;
		frame_counter = frame_counter + 1;
	}
	vm_init = 1;
#else
	/* Do nothing. */
#endif
}
#if OPT_A3
/* Allocate/free some frame for virtual memory*/
static
paddr_t
alloc_frame(int npages)
{
	// start with the available frame
	int frame_position;

	spinlock_acquire(&stealmem_lock);
	// set it to the first available frame
	frame_position = frame_available;
	while (frame_position < frame_max_num)
	{

		bool check_availablity = check_frame_availability(frame_position);;// check whether there is an available frame
		if (!check_availablity)
		{
			bool frame_continuous_space = true; // check whether we have enough continuous space
			int check_continuous_counter = 1;
			// check whether we have available continuous space for npages
			while (check_continuous_counter < npages)
			{
				if (check_frame_availability(frame_position + check_continuous_counter))
				{
					frame_position = frame_position + check_continuous_counter + 1;
					frame_continuous_space = false;
					break;
				}
				check_continuous_counter = check_continuous_counter + 1;
			}

			// If we found enough cotinuous frame space, we then use these positions
			if (frame_continuous_space)
			{
				int continuous_space_marker = 0;
				while (continuous_space_marker < npages)
				{
					// mark the space where we store the bits
					set_frame_availablity(frame_position + continuous_space_marker, true);
					// mark the frame size by used-bit
					frame_size_list[frame_position + continuous_space_marker] = 0;
					continuous_space_marker = continuous_space_marker + 1;
				}
				// mark the frame size
				frame_size_list[frame_position] = npages;

				// After store the address into the frame, we should update the first available position of the frame
				if (frame_position == frame_available)
				{
					int updated_position = frame_position + 1;
					while (updated_position < frame_max_num)
					{
						if (!check_frame_availability(updated_position))
						{
							frame_available = updated_position;
							break;
						}
						updated_position = updated_position + 1;
					}
				}
				spinlock_release(&stealmem_lock);
				return frame_mem_start + (frame_position * PAGE_SIZE);;
			}
		}
		else
		{
			frame_position = frame_position + 1;
		}
	}

	spinlock_release(&stealmem_lock);
	return 0;
}

static
void
free_frame(paddr_t frame)
{
	int frame_position = (frame - frame_mem_start) / PAGE_SIZE;
	int npages;
	int align = (frame - frame_mem_start) % PAGE_SIZE;

	KASSERT(align == 0);
	KASSERT(frame_position >= 0);
	KASSERT(frame_position < frame_max_num);
	KASSERT(check_frame_availability(frame_position));

	spinlock_acquire(&stealmem_lock);

	// update the first available frame
	if (frame_position < frame_available)
	{
		frame_available = frame_position;
	}
	npages = frame_size_list[frame_position];
	int frame_counter = 0;
	// clear the bit in the frame size list now.
	while (frame_counter < npages)
	{
		set_frame_availablity(frame_position + frame_counter, false);
		frame_size_list[frame_position + frame_counter] = 0;
		frame_counter = frame_counter + 1;
	}

	spinlock_release(&stealmem_lock);

}

static
bool
check_frame_availability(int frame_position)
{
	KASSERT(frame_position < frame_max_num);
	int index = frame_position / 32;
	int offset = frame_position % 32;
	uint32_t frame_check = ((uint32_t)1) << offset;
	return (frame_available_list[index] & frame_check) != 0;
}

static
void
set_frame_availablity(int frame_position, bool available)
{
	KASSERT(frame_position< frame_max_num);
	int index = frame_position / 32;
	int offset = frame_position % 32;
	uint32_t frame_check =  ((uint32_t)1) << offset;
	if (available)
	{
		// mark it
		frame_available_list[index] = frame_available_list[index] | frame_check;
	}
	else
	{
		// unmark it
		frame_available_list[index] = frame_available_list[index] & (~frame_check);
	}

}
#endif /*OPT_A3*/

static
paddr_t
getppages(unsigned long npages)
{
	paddr_t addr;

	spinlock_acquire(&stealmem_lock);

	addr = ram_stealmem(npages);

	spinlock_release(&stealmem_lock);
	return addr;
}

/* Allocate/free some kernel-space virtual pages */
vaddr_t
alloc_kpages(int npages)
{
#if OPT_A3
	paddr_t pa;
	if (vm_init == 1)
	{
		pa = alloc_frame(npages);
	}
	else
	{
		pa = getppages(npages);
	}
	if (pa == 0)
	{
		return 0;
	}
	return PADDR_TO_KVADDR(pa);
#else
	paddr_t pa;
	pa = getppages(npages);
	if (pa==0) {
		return 0;
	}
	return PADDR_TO_KVADDR(pa);
#endif /*OPT_A3*/
}

void
free_kpages(vaddr_t addr)
{
#if OPT_A3
	free_frame(KVADDR_TO_PADDR(addr));
#else
	/* nothing - leak the memory. */
	(void)addr;
#endif /*OPT_A3*/
}

void
vm_tlbshootdown_all(void)
{
	panic("dumbvm tried to do tlb shootdown?!\n");
}

void
vm_tlbshootdown(const struct tlbshootdown *ts)
{
	(void)ts;
	panic("dumbvm tried to do tlb shootdown?!\n");
}

int
vm_fault(int faulttype, vaddr_t faultaddress)
{
	vaddr_t vbase1, vtop1, vbase2, vtop2, stackbase, stacktop;
	paddr_t paddr;
	int i;
	uint32_t ehi, elo;
	struct addrspace *as;
	int spl;
#if OPT_A3
	uint32_t read_write_checker;
#endif /*OPT_A3*/

	faultaddress &= PAGE_FRAME;

	DEBUG(DB_VM, "dumbvm: fault: 0x%x\n", faultaddress);

	switch (faulttype) {
	    case VM_FAULT_READONLY:
#if OPT_A3
	    	return EROFS; // read_only file system
#else
		/* We always create pages read-write, so we can't get this */
		panic("dumbvm: got VM_FAULT_READONLY\n");
#endif /*OPT_A3*/
	    case VM_FAULT_READ:
	    case VM_FAULT_WRITE:
		break;
	    default:
		return EINVAL;
	}

	if (curproc == NULL) {
		/*
		 * No process. This is probably a kernel fault early
		 * in boot. Return EFAULT so as to panic instead of
		 * getting into an infinite faulting loop.
		 */
		return EFAULT;
	}

	as = curproc_getas();
	if (as == NULL) {
		/*
		 * No address space set up. This is probably also a
		 * kernel fault early in boot.
		 */
		return EFAULT;
	}
#if OPT_A3
	// Since we change the address space structure, pbase now fits segmentation
	// by using pointer, so we need to check every seg
	int seg_counter = 0;
	int pseg1 = (int)as->as_pbase1;
	int pseg2 = (int)as->as_pbase2;
	KASSERT(as->as_vbase1 != 0);
	KASSERT(as->as_pbase1 != NULL);
	KASSERT(as->as_npages1 != 0);
	KASSERT(as->as_vbase2 != 0);
	KASSERT(as->as_pbase2 != NULL);
	KASSERT(as->as_npages2 != 0);
	KASSERT(as->as_stackpbase != 0);
	KASSERT((as->as_vbase1 & PAGE_FRAME) == as->as_vbase1);
	while (seg_counter < pseg1)
	{
		KASSERT((as->as_pbase1[seg_counter] & PAGE_FRAME) == as->as_pbase1[seg_counter]);
		seg_counter = seg_counter + 1;
	}
	KASSERT((as->as_vbase2 & PAGE_FRAME) == as->as_vbase2);
	seg_counter = 0;
	while (seg_counter < pseg2)
	{
		KASSERT((as->as_pbase2[seg_counter] & PAGE_FRAME) == as->as_pbase2[seg_counter]);
		seg_counter = seg_counter + 1;
	}
	seg_counter = 0;
	while (seg_counter <DUMBVM_STACKPAGES)
	{
		KASSERT((as->as_stackpbase[seg_counter] & PAGE_FRAME) == as->as_stackpbase[seg_counter]);
		seg_counter = seg_counter + 1;
	}
#else
	/* Assert that the address space has been set up properly. */
	KASSERT(as->as_vbase1 != 0);
	KASSERT(as->as_pbase1 != 0);
	KASSERT(as->as_npages1 != 0);
	KASSERT(as->as_vbase2 != 0);
	KASSERT(as->as_pbase2 != 0);
	KASSERT(as->as_npages2 != 0);
	KASSERT(as->as_stackpbase != 0);
	KASSERT((as->as_vbase1 & PAGE_FRAME) == as->as_vbase1);
	KASSERT((as->as_pbase1 & PAGE_FRAME) == as->as_pbase1);
	KASSERT((as->as_vbase2 & PAGE_FRAME) == as->as_vbase2);
	KASSERT((as->as_pbase2 & PAGE_FRAME) == as->as_pbase2);
	KASSERT((as->as_stackpbase & PAGE_FRAME) == as->as_stackpbase);
#endif /*OPT_A3*/

	vbase1 = as->as_vbase1;
	vtop1 = vbase1 + as->as_npages1 * PAGE_SIZE;
	vbase2 = as->as_vbase2;
	vtop2 = vbase2 + as->as_npages2 * PAGE_SIZE;
	stackbase = USERSTACK - DUMBVM_STACKPAGES * PAGE_SIZE;
	stacktop = USERSTACK;

	// The TLBLO_DIRTY bit is actually a write privilege bit
	// If you set it, writes are permitted.
	// If you don't set it, you'll get a "TLB Modify" exception when a write is attempted.

#if OPT_A3
	if ((faultaddress >= vbase1) && (faultaddress < vtop1))
	{
		read_write_checker = 0;
		int frame = (faultaddress - vbase1) / PAGE_SIZE;
		paddr = as->as_pbase1[frame];
	}
	else if ((faultaddress >= vbase2) && (faultaddress < vtop2))
	{
		read_write_checker = TLBLO_DIRTY;
		int frame = (faultaddress - vbase2) / PAGE_SIZE;
		paddr = as->as_pbase2[frame];
	}
	else if ((faultaddress >= stackbase) && (faultaddress < stacktop))
	{
		read_write_checker = TLBLO_DIRTY;
		int frame = (faultaddress - stackbase) / PAGE_SIZE;
		paddr = as->as_stackpbase[frame];
	}
	else
	{
		return EFAULT;
	}
#else
	if (faultaddress >= vbase1 && faultaddress < vtop1) {
		paddr = (faultaddress - vbase1) + as->as_pbase1;

	}
	else if (faultaddress >= vbase2 && faultaddress < vtop2) {
		paddr = (faultaddress - vbase2) + as->as_pbase2;
	}
	else if (faultaddress >= stackbase && faultaddress < stacktop) {
		paddr = (faultaddress - stackbase) + as->as_stackpbase;
	}
	else {
		return EFAULT;
	}
#endif /*OPT_A3*/

	/* make sure it's page-aligned */
	KASSERT((paddr & PAGE_FRAME) == paddr);

#if OPT_A3
	if (as->init_status == 1)
	{
		read_write_checker = TLBLO_DIRTY;
	}
#endif /*OPT_A3*/

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_read(&ehi, &elo, i);
		if (elo & TLBLO_VALID) {
			continue;
		}
		ehi = faultaddress;
// Now,
#if OPT_A3
		elo = paddr | read_write_checker | TLBLO_VALID;
#else
		elo = paddr | TLBLO_DIRTY | TLBLO_VALID;
#endif /*OPT_A3*/
		DEBUG(DB_VM, "dumbvm: 0x%x -> 0x%x\n", faultaddress, paddr);
		tlb_write(ehi, elo, i);
		splx(spl);
		return 0;
	}
	//This part, we mainly deal with TLB replacement.
	// instead of running out of TLB entries, we replace a random entry with the new one
#if OPT_A3
	elo = paddr | read_write_checker | TLBLO_VALID;
	ehi = faultaddress;
	tlb_random(ehi, elo); // store the passed values into the entry registers
	splx(spl);
	return 0;
#else
	kprintf("dumbvm: Ran out of TLB entries - cannot handle page fault\n");
	splx(spl);
	return EFAULT;
#endif /*OPT_A3*/
}

struct addrspace *
as_create(void)
{
	struct addrspace *as = kmalloc(sizeof(struct addrspace));
	if (as==NULL) {
		return NULL;
	}
#if OPT_A3
	as->init_status = 0;
	as->as_vbase1 = 0;
	as->as_pbase1 = NULL;
	as->as_npages1 = 0;
	as->as_vbase2 = 0;
	as->as_pbase2 = NULL;
	as->as_npages2 = 0;
	as->as_stackpbase = NULL;
#else
	as->as_vbase1 = 0;
	as->as_pbase1 = 0;
	as->as_npages1 = 0;
	as->as_vbase2 = 0;
	as->as_pbase2 = 0;
	as->as_npages2 = 0;
	as->as_stackpbase = 0;
#endif /*OPT_A3*/
	return as;
}

void
as_destroy(struct addrspace *as)
{
#if OPT_A3
	int frame_counter = 0;
	int pbase1_num = (int) as->as_npages1;
	int pbase2_num = (int) as->as_npages2;
	int stackpbase_num = DUMBVM_STACKPAGES;
	// destroy the as->as_pbase1
	while (frame_counter < pbase1_num)
	{
		free_frame(as->as_pbase1[frame_counter]);
		frame_counter = frame_counter + 1;
	}
	kfree(as->as_pbase1);
	frame_counter = 0;
	// destroy the as->as_pbase2
	while (frame_counter < pbase2_num)
	{
		free_frame(as->as_pbase2[frame_counter]);
		frame_counter = frame_counter + 1;
	}
	kfree(as->as_pbase2);
	frame_counter = 0;
	// destroy the as->as_stackpbase
	while (frame_counter < stackpbase_num)
	{
		free_frame(as->as_stackpbase[frame_counter]);
		frame_counter = frame_counter + 1;
	}
	kfree(as->as_stackpbase);
	frame_counter = 0;
#endif /*OPT_A3*/
	kfree(as);
}

void
as_activate(void)
{
	int i, spl;
	struct addrspace *as;

	as = curproc_getas();
#ifdef UW
        /* Kernel threads don't have an address spaces to activate */
#endif
	if (as == NULL) {
		return;
	}

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_write(TLBHI_INVALID(i), TLBLO_INVALID(), i);
	}

	splx(spl);
}

void
as_deactivate(void)
{
#if OPT_A3
	// disable interrupts
	int spl;
	spl = splhigh();
	unsigned int i = 0;
	while (i < NUM_TLB)
	{
		tlb_write(TLBHI_INVALID(i), TLBLO_INVALID(), i);
		i = i + 1;
	}
	splx(spl);
#else
	/* nothing */
#endif /*OPT_A3*/
}

int
as_define_region(struct addrspace *as, vaddr_t vaddr, size_t sz,
		 int readable, int writeable, int executable)
{
	size_t npages;

	/* Align the region. First, the base... */
	sz += vaddr & ~(vaddr_t)PAGE_FRAME;
	vaddr &= PAGE_FRAME;

	/* ...and now the length. */
	sz = (sz + PAGE_SIZE - 1) & PAGE_FRAME;

	npages = sz / PAGE_SIZE;

	/* We don't use these - all pages are read-write */
	(void)readable;
	(void)writeable;
	(void)executable;

	if (as->as_vbase1 == 0) {
		as->as_vbase1 = vaddr;
		as->as_npages1 = npages;
		return 0;
	}

	if (as->as_vbase2 == 0) {
		as->as_vbase2 = vaddr;
		as->as_npages2 = npages;
		return 0;
	}

	/*
	 * Support for more than two regions is not available.
	 */
	kprintf("dumbvm: Warning: too many regions\n");
	return EUNIMP;
}
#if OPT_A3

static
void
as_zero_region(paddr_t* paddr_t, unsigned npages)
#else

static
void
as_zero_region(paddr_t paddr, unsigned npages)

#endif /*OPT_A3*/
{
#if OPT_A3
	unsigned int i = 0;
	unsigned int npage = (int) npages;
	while (i < npage)
	{
		bzero((void*)PADDR_TO_KVADDR(paddr_t[i]), PAGE_SIZE);
		i = i + 1;
	}
#else
	bzero((void *)PADDR_TO_KVADDR(paddr), npages * PAGE_SIZE);
#endif /*OPT_A3*/
}

int
as_prepare_load(struct addrspace *as)
{
#if OPT_A3
	KASSERT(as->as_pbase1 == NULL);
	KASSERT(as->as_pbase2 == NULL);
	KASSERT(as->as_stackpbase == NULL);

	// prepare_load with the as->as_pbase1
	paddr_t *new_pbase1;
	int pbase_flag = 1;
	new_pbase1 = kmalloc(as->as_npages1 * sizeof(paddr_t*));
	if (new_pbase1 == NULL)
	{
		pbase_flag = 0;
	}
	// initialize the as->as_npgaes1s frame
	int npage1_num = (int) as->as_npages1;
	for (int i = 0; i < npage1_num; i++)
	{
		new_pbase1[i] = alloc_frame(1);
		if (new_pbase1[i] == 0)
		{
			for (int j = 0; j < i; j++)
			{
				free_frame(new_pbase1[j]);
			}
			pbase_flag = 0;
			break;
		}
	}
	if (pbase_flag == 0)
	{
		kfree(new_pbase1);
		as->as_pbase1 = 0;
		return ENOMEM;
	}
	as->as_pbase1 = new_pbase1;

	// prepare_load with the as->as_pbase2
	paddr_t *new_pbase2;
	pbase_flag = 1;
	new_pbase2 = kmalloc(as->as_npages2 * sizeof(paddr_t*));
	if (new_pbase2 == NULL)
	{
		pbase_flag = 0;
	}
	// initialize the as->as_npgaes2s frame
	int npage2_num = (int) as->as_npages2;
	for (int i = 0; i < npage2_num; i++)
	{
		new_pbase2[i] = alloc_frame(1);
		if (new_pbase2[i] == 0)
		{
			for (int j = 0; j < i; j++)
			{
				free_frame(new_pbase2[j]);
			}
			pbase_flag = 0;
			break;
		}
	}
	if (pbase_flag == 0)
	{
		kfree(new_pbase2);
		as->as_pbase2 = 0;
		int npage1_num = (int)as->as_npages1;
		for (int i = 0; i < npage1_num; i++)
		{
			free_frame(as->as_pbase1[i]);
		}
		kfree(as->as_pbase1);
		as->as_pbase1 = 0;
		return ENOMEM;
	}
	as->as_pbase2 = new_pbase2;

	// prepare_load with as->as_stackpbase
	paddr_t *new_stackpbase;
	pbase_flag = 1;
	new_stackpbase = kmalloc(DUMBVM_STACKPAGES * sizeof(paddr_t*));
	if (new_stackpbase == NULL)
	{
		pbase_flag = 0;
	}
	// initialize the DUMBVM_STACKPAGES frame
	for (int i = 0; i < (int) DUMBVM_STACKPAGES; i++)
	{
		new_stackpbase[i] = alloc_frame(1);
		if (new_stackpbase[i] == 0)
		{
			for (int j = 0; j < i; j++)
			{
				free_frame(new_stackpbase[j]);
			}
			pbase_flag = 0;
			break;
		}
	}
	if (pbase_flag == 0)
	{
		kfree(new_stackpbase);
		as->as_stackpbase = 0;
		int npage1_num = (int)as->as_npages1;
		for (int i = 0; i < npage1_num; i++)
		{
			free_frame(as->as_pbase1[i]);
		}
		kfree(as->as_pbase1);
		as->as_pbase1 = 0;
		int npage2_num = (int)as->as_npages2;
		for (int i = 0; i < npage2_num; i++)
		{
			free_frame(as->as_pbase2[i]);
		}
		kfree(as->as_pbase2);
		as->as_pbase2 = 0;
		return ENOMEM;
	}
	as->as_stackpbase = new_stackpbase;
	as->init_status = 1; // finish preloading

#else
	KASSERT(as->as_pbase1 == 0);
	KASSERT(as->as_pbase2 == 0);
	KASSERT(as->as_stackpbase == 0);

	as->as_pbase1 = getppages(as->as_npages1);
	if (as->as_pbase1 == 0) {
		return ENOMEM;
	}

	as->as_pbase2 = getppages(as->as_npages2);
	if (as->as_pbase2 == 0) {
		return ENOMEM;
	}

	as->as_stackpbase = getppages(DUMBVM_STACKPAGES);
	if (as->as_stackpbase == 0) {
		return ENOMEM;
	}
#endif /*OPT_A3*/

	as_zero_region(as->as_pbase1, as->as_npages1);
	as_zero_region(as->as_pbase2, as->as_npages2);
	as_zero_region(as->as_stackpbase, DUMBVM_STACKPAGES);

	return 0;
}

int
as_complete_load(struct addrspace *as)
{
#if OPT_A3
	as->init_status = 0;
	return 0;
#else
	(void)as;
	return 0;
#endif /*OPT_A3*/
}

int
as_define_stack(struct addrspace *as, vaddr_t *stackptr)
{
	KASSERT(as->as_stackpbase != 0);

	*stackptr = USERSTACK;
	return 0;
}

int
as_copy(struct addrspace *old, struct addrspace **ret)
{
	struct addrspace *new;

	new = as_create();
	if (new==NULL) {
		return ENOMEM;
	}

	new->as_vbase1 = old->as_vbase1;
	new->as_npages1 = old->as_npages1;
	new->as_vbase2 = old->as_vbase2;
	new->as_npages2 = old->as_npages2;

	/* (Mis)use as_prepare_load to allocate some physical memory. */
	if (as_prepare_load(new)) {
		as_destroy(new);
		return ENOMEM;
	}
#if OPT_A3
	KASSERT(new->as_pbase1 != NULL);
	KASSERT(new->as_pbase2 != NULL);
	KASSERT(new->as_stackpbase != NULL);

	int frame_counter = 0;
	int pbase1_num = (int) old->as_npages1;
	int pbase2_num = (int) old->as_npages2;
	int stackpbase_num = DUMBVM_STACKPAGES;
	while (frame_counter < pbase1_num)
	{
		memmove((void*)PADDR_TO_KVADDR(new->as_pbase1[frame_counter]),
			(const void*)PADDR_TO_KVADDR(old->as_pbase1[frame_counter]),
			PAGE_SIZE);
		frame_counter = frame_counter + 1;
	}
	frame_counter = 0;
	while (frame_counter < pbase2_num)
	{
		memmove((void*)PADDR_TO_KVADDR(new->as_pbase2[frame_counter]),
			(const void*)PADDR_TO_KVADDR(old->as_pbase2[frame_counter]),
			PAGE_SIZE);
		frame_counter = frame_counter + 1;
	}
	frame_counter = 0;
	while (frame_counter < stackpbase_num)
	{
		memmove((void*)PADDR_TO_KVADDR(new->as_stackpbase[frame_counter]),
			(const void*)PADDR_TO_KVADDR(old->as_stackpbase[frame_counter]),
			PAGE_SIZE);
		frame_counter = frame_counter + 1;
	}
	frame_counter = 0;

	as_complete_load(new);
#else
	KASSERT(new->as_pbase1 != 0);
	KASSERT(new->as_pbase2 != 0);
	KASSERT(new->as_stackpbase != 0);

	memmove((void *)PADDR_TO_KVADDR(new->as_pbase1),
		(const void *)PADDR_TO_KVADDR(old->as_pbase1),
		old->as_npages1*PAGE_SIZE);

	memmove((void *)PADDR_TO_KVADDR(new->as_pbase2),
		(const void *)PADDR_TO_KVADDR(old->as_pbase2),
		old->as_npages2*PAGE_SIZE);

	memmove((void *)PADDR_TO_KVADDR(new->as_stackpbase),
		(const void *)PADDR_TO_KVADDR(old->as_stackpbase),
		DUMBVM_STACKPAGES*PAGE_SIZE);
#endif
	*ret = new;
	return 0;
}
