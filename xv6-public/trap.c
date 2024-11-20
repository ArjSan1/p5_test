// trap.c
#define MAX_PAGES_PER_MAPPING 4096 

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"
#include "fs.h"
#include "sleeplock.h"
#include "file.h"

// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[];  // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE<<3, vectors[i], 0);
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE<<3, vectors[T_SYSCALL], DPL_USER);

  initlock(&tickslock, "time");
}

void
idtinit(void)
{
  lidt(idt, sizeof(idt));
}

// Function to handle page faults
int handle_pagefault(uint addr) {
    struct proc *curproc = myproc();
    int handled = 0;

    // Iterate through all mappings to find the one containing the faulting address
    for (int i = 0; i < curproc->num_mappings; i++) {
        struct mapping *m = &curproc->mappings[i];
        uint map_start = m->addr;
        uint map_end = map_start + m->length;

        if (addr >= map_start && addr < map_end) {
            handled = 1;

            // Check if it's a file-backed mapping
            if (m->file != 0) {
                struct file *f = m->file;
                if (f == 0 || f->type != FD_INODE) {
                    cprintf("ERROR: handle_pagefault: Invalid file for mapping %d (fd=%d)\n", i + 1, m->fd);
                    goto segfault;
                }

                // **Load Data from File into Physical Page**

                // Calculate the page number and file offset
                int pg_num = (addr - m->addr) / PGSIZE;
                uint file_offset = pg_num * PGSIZE;

                // Allocate a physical page
                char *pa = kalloc();
                if (pa == 0) {
                    goto segfault;
                }

                // Convert kernel virtual address to physical address
                uint physical_addr = V2P(pa);

                // Read data from the file into the allocated memory
                int n = readi(f->ip, pa, file_offset, PGSIZE);

                if (n < 0) {
                    cprintf("ERROR: handle_pagefault: readi failed for mapping %d (fd=%d)\n", i + 1, m->fd);
                    kfree(pa);
                    goto segfault;
                }

                if (n < PGSIZE) {
                    memset(pa + n, 0, PGSIZE - n);
                }

                // **Retrieve the Page Table Entry (PTE)**
                pte_t *pte = walkpgdir(curproc->pgdir, (const void*)addr, 0);
                if (pte == 0) {
                    kfree(pa);
                    goto segfault;
                }

                // **Check if the Page is Already Present**
                if (*pte & PTE_P) {
                    kfree(pa);
                    goto segfault;
                }

                // **Map the Physical Page to the Virtual Address**
                *pte = physical_addr | PTE_U | PTE_W | PTE_P;

                // **Verify the Mapping**
                pte_t *verify_pte = walkpgdir(curproc->pgdir, (const void*)addr, 0);
                if (verify_pte == 0 || !(*verify_pte & PTE_P)) {
                    kfree(pa);
                    goto segfault;
                }

                // **Flush the TLB**
                lcr3(V2P(curproc->pgdir));

                // **Increment the Number of Loaded Pages with Overflow Check**
                if (m->n_loaded_pages < MAX_PAGES_PER_MAPPING) {
                    m->n_loaded_pages++;
                } else {
                    cprintf("ERROR: handle_pagefault: n_loaded_pages overflow for mapping %d\n", i + 1);
                    kfree(pa);
                    goto segfault;
                }

                // Successful handling
                return 1; // Success
            } else {
                // **Anonymous Mapping Handling**
                // Allocate a physical page
                char *pa = kalloc();
                if (pa == 0) {
                    goto segfault;
                }

                // Convert kernel virtual address to physical address
                uint physical_addr = V2P(pa);

                // Zero out the allocated memory
                memset(pa, 0, PGSIZE);

                // **Retrieve the Page Table Entry (PTE)**
                pte_t *pte = walkpgdir(curproc->pgdir, (const void*)addr, 0);
                if (pte == 0) {
                    kfree(pa);
                    goto segfault;
                }

                // **Check if the Page is Already Present**
                if (*pte & PTE_P) {
                    kfree(pa);
                    goto segfault;
                }

                // **Map the Physical Page to the Virtual Address**
                *pte = physical_addr | PTE_U | PTE_W | PTE_P;

                // **Verify the Mapping**
                pte_t *verify_pte = walkpgdir(curproc->pgdir, (const void*)addr, 0);
                if (verify_pte == 0 || !(*verify_pte & PTE_P)) {
                    kfree(pa);
                    goto segfault;
                }

                // **Flush the TLB**
                lcr3(V2P(curproc->pgdir));

                // **Increment the Number of Loaded Pages with Overflow Check**
                if (m->n_loaded_pages < MAX_PAGES_PER_MAPPING) {
                    m->n_loaded_pages++;
                } else {
                    cprintf("ERROR: handle_pagefault: n_loaded_pages overflow for mapping %d\n", i + 1);
                    kfree(pa);
                    goto segfault;
                }

                // Successful handling
                return 1; // Success
            }
        }
    }

    if (!handled) {
        // **Address Not Within Any Mapping**
        // No logging to minimize output
    }

segfault:
    // **Handle as a Standard Segmentation Fault**
    cprintf("Segmentation Fault: 0x%x\n", addr);
    curproc->killed = 1;
    return 0; // Failure
}
// Trap handler
void
trap(struct trapframe *tf)
{
    if(tf->trapno == T_SYSCALL){
        if(myproc()->killed)
            exit();
        myproc()->tf = tf;
        syscall();
        if(myproc()->killed)
            exit();
        return;
    }

    switch(tf->trapno){
    case T_IRQ0 + IRQ_TIMER:
        if(cpuid() == 0){
            acquire(&tickslock);
            ticks++;
            wakeup(&ticks);
            release(&tickslock);
        }
        lapiceoi();
        break;
    case T_IRQ0 + IRQ_IDE:
        ideintr();
        lapiceoi();
        break;
    case T_IRQ0 + IRQ_KBD:
        kbdintr();
        lapiceoi();
        break;
    case T_IRQ0 + IRQ_COM1:
        uartintr();
        lapiceoi();
        break;
    case T_IRQ0 + 7:
    case T_IRQ0 + IRQ_SPURIOUS:
        cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpuid(), tf->cs, tf->eip);
        lapiceoi();
        break;

    case T_PGFLT:
        {
            uint fault_addr;
            // On x86, CR2 contains the faulting address
            asm volatile("mov %%cr2, %0" : "=r" (fault_addr));
            // Minimal debug statement
            // cprintf("DEBUG: trap: Page fault at address 0x%x\n", fault_addr);
            if (handle_pagefault(fault_addr)) {
                break;
            } else {
                cprintf("ERROR: trap: Segmentation Fault at address 0x%x\n", fault_addr);
                // kill the process
                myproc()->killed = 1;
                // Exit if in user space
                if ((tf->cs & 3) == DPL_USER)
                    exit();
            }
        }
        break;

    default:
        if(myproc() == 0 || (tf->cs&3) == 0){
            // In kernel, unexpected trap: print and halt.
            cprintf("unexpected trap %d from cpu %d eip %x addr %x\n",
                tf->trapno, cpuid(), tf->eip, 0);
            panic("trap");
        }
        // In user space, assume process misbehaved.
        cprintf("pid %d %s: trap %d err %d on cpu %d eip 0x%x addr 0x%x--kill proc\n",
            myproc()->pid, myproc()->name, tf->trapno, tf->err, cpuid(), tf->eip, 0);
        myproc()->killed = 1;
    }

    // Force process exit if it has been killed and is in user space.
    // (If it is still executing in the kernel, let it keep running
    // until it gets to the regular system call return.)
    if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
        exit();

    // Force process to yield CPU on clock tick.
    if(myproc() && myproc()->state == RUNNING &&
        tf->trapno == T_IRQ0+IRQ_TIMER)
        yield();

    // Check if the process has been killed since we yielded
    if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
        exit();
}