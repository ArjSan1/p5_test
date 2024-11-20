#include "param.h"
#include "types.h"
#include "defs.h"
#include "x86.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "file.h"
#include "elf.h"

extern char data[];  // defined by kernel.ld
pde_t *kpgdir;  // for use in scheduler()
#define min(a, b) ((a) < (b) ? (a) : (b))


// Set up CPU's kernel segment descriptors.
// Run once on entry on each CPU.
void
seginit(void)
{
  struct cpu *c;

  // Map "logical" addresses to virtual addresses using identity map.
  // Cannot share a CODE descriptor for both kernel and user
  // because it would have to have DPL_USR, but the CPU forbids
  // an interrupt from CPL=0 to DPL=3.
  c = &cpus[cpuid()];
  c->gdt[SEG_KCODE] = SEG(STA_X|STA_R, 0, 0xffffffff, 0);
  c->gdt[SEG_KDATA] = SEG(STA_W, 0, 0xffffffff, 0);
  c->gdt[SEG_UCODE] = SEG(STA_X|STA_R, 0, 0xffffffff, DPL_USER);
  c->gdt[SEG_UDATA] = SEG(STA_W, 0, 0xffffffff, DPL_USER);
  lgdt(c->gdt, sizeof(c->gdt));
}

// Return the address of the PTE in page table pgdir
// that corresponds to virtual address va.  If alloc!=0,
// create any required page table pages.
pte_t *
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
  pde_t *pde;
  pte_t *pgtab;

  pde = &pgdir[PDX(va)];
  if(*pde & PTE_P){
    pgtab = (pte_t*)P2V(PTE_ADDR(*pde));
  } else {
    if(!alloc || (pgtab = (pte_t*)kalloc()) == 0)
      return 0;
    // Make sure all those PTE_P bits are zero.
    memset(pgtab, 0, PGSIZE);
    // The permissions here are overly generous, but they can
    // be further restricted by the permissions in the page table
    // entries, if necessary.
    *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
  }
  return &pgtab[PTX(va)];
}

// Create PTEs for virtual addresses starting at va that refer to
// physical addresses starting at pa. va and size might not
// be page-aligned.
int
mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm)
{
  char *a, *last;
  pte_t *pte;

  a = (char*)PGROUNDDOWN((uint)va);
  last = (char*)PGROUNDDOWN(((uint)va) + size - 1);
  for(;;){
    if((pte = walkpgdir(pgdir, a, 1)) == 0)
      return -1;
    if(*pte & PTE_P)
      panic("remap");
    *pte = pa | perm | PTE_P;
    if(a == last)
      break;
    a += PGSIZE;
    pa += PGSIZE;
  }
  return 0;
}

// There is one page table per process, plus one that's used when
// a CPU is not running any process (kpgdir). The kernel uses the
// current process's page table during system calls and interrupts;
// page protection bits prevent user code from using the kernel's
// mappings.
//
// setupkvm() and exec() set up every page table like this:
//
//   0..KERNBASE: user memory (text+data+stack+heap), mapped to
//                phys memory allocated by the kernel
//   KERNBASE..KERNBASE+EXTMEM: mapped to 0..EXTMEM (for I/O space)
//   KERNBASE+EXTMEM..data: mapped to EXTMEM..V2P(data)
//                for the kernel's instructions and r/o data
//   data..KERNBASE+PHYSTOP: mapped to V2P(data)..PHYSTOP,
//                                  rw data + free physical memory
//   0xfe000000..0: mapped direct (devices such as ioapic)
//
// The kernel allocates physical memory for its heap and for user memory
// between V2P(end) and the end of physical memory (PHYSTOP)
// (directly addressable from end..P2V(PHYSTOP)).

// This table defines the kernel's mappings, which are present in
// every process's page table.
static struct kmap {
  void *virt;
  uint phys_start;
  uint phys_end;
  int perm;
} kmap[] = {
 { (void*)KERNBASE, 0,             EXTMEM,    PTE_W}, // I/O space
 { (void*)KERNLINK, V2P(KERNLINK), V2P(data), 0},     // kern text+rodata
 { (void*)data,     V2P(data),     PHYSTOP,   PTE_W}, // kern data+memory
 { (void*)DEVSPACE, DEVSPACE,      0,         PTE_W}, // more devices
};

// Set up kernel part of a page table.
pde_t*
setupkvm(void)
{
  pde_t *pgdir;
  struct kmap *k;

  if((pgdir = (pde_t*)kalloc()) == 0)
    return 0;
  memset(pgdir, 0, PGSIZE);
  if (P2V(PHYSTOP) > (void*)DEVSPACE)
    panic("PHYSTOP too high");
  for(k = kmap; k < &kmap[NELEM(kmap)]; k++)
    if(mappages(pgdir, k->virt, k->phys_end - k->phys_start,
                (uint)k->phys_start, k->perm) < 0) {
      freevm(pgdir);
      return 0;
    }
  return pgdir;
}

// Allocate one page table for the machine for the kernel address
// space for scheduler processes.
void
kvmalloc(void)
{
  kpgdir = setupkvm();
  switchkvm();
}

// Switch h/w page table register to the kernel-only page table,
// for when no process is running.
void
switchkvm(void)
{
  lcr3(V2P(kpgdir));   // switch to the kernel page table
}

// Switch TSS and h/w page table to correspond to process p.
void
switchuvm(struct proc *p)
{
  if(p == 0)
    panic("switchuvm: no process");
  if(p->kstack == 0)
    panic("switchuvm: no kstack");
  if(p->pgdir == 0)
    panic("switchuvm: no pgdir");

  pushcli();
  mycpu()->gdt[SEG_TSS] = SEG16(STS_T32A, &mycpu()->ts,
                                sizeof(mycpu()->ts)-1, 0);
  mycpu()->gdt[SEG_TSS].s = 0;
  mycpu()->ts.ss0 = SEG_KDATA << 3;
  mycpu()->ts.esp0 = (uint)p->kstack + KSTACKSIZE;
  // setting IOPL=0 in eflags *and* iomb beyond the tss segment limit
  // forbids I/O instructions (e.g., inb and outb) from user space
  mycpu()->ts.iomb = (ushort) 0xFFFF;
  ltr(SEG_TSS << 3);
  lcr3(V2P(p->pgdir));  // switch to process's address space
  popcli();
}

// Load the initcode into address 0 of pgdir.
// sz must be less than a page.
void
inituvm(pde_t *pgdir, char *init, uint sz)
{
  char *mem;

  if(sz >= PGSIZE)
    panic("inituvm: more than a page");
  mem = kalloc();
  memset(mem, 0, PGSIZE);
  mappages(pgdir, 0, PGSIZE, V2P(mem), PTE_W|PTE_U);
  memmove(mem, init, sz);
}

// Load a program segment into pgdir.  addr must be page-aligned
// and the pages from addr to addr+sz must already be mapped.
int
loaduvm(pde_t *pgdir, char *addr, struct inode *ip, uint offset, uint sz, int flags)
{
    uint i, pa, n;
    pte_t *pte;

    for (i = 0; i < sz; i += PGSIZE) {
        uint va = (uint)addr + i;

        // Get the PTE
        pte = walkpgdir(pgdir, (void *)va, 0);
        if (pte == 0)
            panic("loaduvm: address should exist");

        // Get the physical address
        pa = PTE_ADDR(*pte);

        // Read from the inode into physical memory
        n = min(PGSIZE, sz - i);
        if (readi(ip, P2V(pa), offset + i, n) != n)
            return -1;

        // **Set the page permissions based on the ELF segment flags**
        // Clear existing permission bits
        *pte &= ~PTE_W;

        // If the segment is writable, set PTE_W
        if (flags & ELF_PROG_FLAG_WRITE)
            *pte |= PTE_W;
    }
    return 0;
}

// Allocate page tables and physical memory to grow process from oldsz to
// newsz, which need not be page aligned.  Returns new size or 0 on error.
int
allocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
  char *mem;
  uint a;

  if(newsz >= KERNBASE)
    return 0;
  if(newsz < oldsz)
    return oldsz;

  a = PGROUNDUP(oldsz);
  for(; a < newsz; a += PGSIZE){
    mem = kalloc();
    if(mem == 0){
      cprintf("allocuvm out of memory\n");
      deallocuvm(pgdir, newsz, oldsz);
      return 0;
    }
    memset(mem, 0, PGSIZE);
    if(mappages(pgdir, (char*)a, PGSIZE, V2P(mem), PTE_W|PTE_U) < 0){
      cprintf("allocuvm out of memory (2)\n");
      deallocuvm(pgdir, newsz, oldsz);
      kfree(mem);
      return 0;
    }
  }
  return newsz;
}

// Deallocate user pages to bring the process size from oldsz to
// newsz.  oldsz and newsz need not be page-aligned, nor does newsz
// need to be less than oldsz.  oldsz can be larger than the actual
// process size.  Returns the new process size.
int
deallocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
  pte_t *pte;
  uint a, pa;

  if(newsz >= oldsz)
    return oldsz;

  a = PGROUNDUP(newsz);
  for(; a  < oldsz; a += PGSIZE){
    pte = walkpgdir(pgdir, (char*)a, 0);
    if(!pte)
      a = PGADDR(PDX(a) + 1, 0, 0) - PGSIZE;
    else if((*pte & PTE_P) != 0){
      pa = PTE_ADDR(*pte);
      if(pa == 0)
        panic("kfree");
      char *v = P2V(pa);
      kfree(v);
      *pte = 0;
    }
  }
  return newsz;
}
void
freevm_pgdir(pde_t *pgdir)
{
    uint i;

    for(i = 0; i < NPDENTRIES; i++){
        if(pgdir[i] & PTE_P){
            char * v = P2V(PTE_ADDR(pgdir[i]));
            kfree(v);
        }
    }
    kfree((char*)pgdir);
}

// Free a page table and all the physical memory pages
// in the user part.
void
freevm(pde_t *pgdir)
{
    uint i;

    if(pgdir == 0)
        panic("freevm: no pgdir");

    // Deallocate pages from mappings first
    struct proc *p = myproc();
    for (i = 0; i < p->num_mappings; i++) {
        struct mapping *m = &p->mappings[i];
        uint addr = m->addr;
        uint length = m->length;
        for (uint va = addr; va < addr + length; va += PGSIZE) {
            pte_t *pte = walkpgdir(pgdir, (char *)va, 0);
            if (pte && (*pte & PTE_P)) {
                if (m->flags & MAP_SHARED) {
                    // Do not free shared pages here
                    // We'll handle them in wunmap or when all processes exit
                    *pte = 0; // Clear the PTE
                } else {
                    // Free private mapping pages
                    uint pa = PTE_ADDR(*pte);
                    char *v = P2V(pa);
                    kfree(v);
                    *pte = 0; // Clear the PTE
                }
            }
        }
    }

    // Deallocate pages for the rest of the process
    deallocuvm(pgdir, p->sz, 0);

    // Free the page directory
    freevm_pgdir(pgdir);
}



// Clear PTE_U on a page. Used to create an inaccessible
// page beneath the user stack.
void
clearpteu(pde_t *pgdir, char *uva)
{
  pte_t *pte;

  pte = walkpgdir(pgdir, uva, 0);
  if(pte == 0)
    panic("clearpteu");
  *pte &= ~PTE_U;
}

// Given a parent process's page table, create a copy
// of it for a child.
pde_t*
copyuvm(pde_t *pgdir, uint sz)
{
    pde_t *d;
    pte_t *pte;
    uint pa, i, flags;
    char *mem;

    if((d = setupkvm()) == 0)
        return 0;

    for(i = 0; i < sz; i += PGSIZE){
        if((pte = walkpgdir(pgdir, (void *) i, 0)) == 0)
            continue; // Skip if PTE does not exist
        if(!(*pte & PTE_P))
            continue; // Skip if page is not present
        pa = PTE_ADDR(*pte);
        flags = PTE_FLAGS(*pte);
        if((mem = kalloc()) == 0)
            goto bad;
        memmove(mem, (char*)P2V(pa), PGSIZE);
        if(mappages(d, (void*)i, PGSIZE, V2P(mem), flags) < 0) {
            kfree(mem);
            goto bad;
        }
    }
    return d;

bad:
    freevm(d);
    return 0;
}


//PAGEBREAK!
// Map user virtual address to kernel address.
char*
uva2ka(pde_t *pgdir, char *uva)
{
  pte_t *pte;

  pte = walkpgdir(pgdir, uva, 0);
  if((*pte & PTE_P) == 0)
    return 0;
  if((*pte & PTE_U) == 0)
    return 0;
  return (char*)P2V(PTE_ADDR(*pte));
}

// Copy len bytes from p to user address va in page table pgdir.
// Most useful when pgdir is not the current page table.
// uva2ka ensures this only works for PTE_U pages.
int
copyout(pde_t *pgdir, uint va, void *p, uint len)
{
  char *buf, *pa0;
  uint n, va0;

  buf = (char*)p;
  while(len > 0){
    va0 = (uint)PGROUNDDOWN(va);
    pa0 = uva2ka(pgdir, (char*)va0);
    if(pa0 == 0)
      return -1;
    n = PGSIZE - (va - va0);
    if(n > len)
      n = len;
    memmove(pa0 + (va - va0), buf, n);
    len -= n;
    buf += n;
    va = va0 + PGSIZE;
  }
  return 0;
}
// Translates a user virtual address to a physical address.
uint va2pa(uint va) {
    struct proc *p = myproc();
    pte_t *pte;
    uint pa;

    pte = walkpgdir(p->pgdir, (char*)va, 0);
    if(pte == 0)
        return (uint)-1;
    if((*pte & PTE_P) == 0)
        return (uint)-1;
    pa = PTE_ADDR(*pte) | (va & 0xFFF);
    return pa;
}

// Implement getwmapinfo
int getwmapinfo(struct wmapinfo *info) {
    struct proc *p = myproc();

    if(!p || !info)
        return FAILED;

    info->total_mmaps = p->num_mappings;
    for(int i = 0; i < p->num_mappings && i < MAX_WMMAP_INFO; i++) {
        info->addr[i] = p->mappings[i].addr;
        info->length[i] = p->mappings[i].length;
        info->n_loaded_pages[i] = p->mappings[i].n_loaded_pages;
    }

    return SUCCESS;
}

// Implement wmap
int wmap(uint addr, int length, int flags, int fd) {
    struct proc *p = myproc();

    // Essential debug statement
    cprintf("wmap: Entering with addr=0x%x, length=%d, flags=0x%x, fd=%d\n", addr, length, flags, fd);

    // 1. Validate Flags: MAP_FIXED and MAP_SHARED must be set
    if ((flags & MAP_FIXED) == 0) {
        cprintf("wmap ERROR: MAP_FIXED must be present in flags\n");
        return FAILED;
    }
    if ((flags & MAP_SHARED) == 0) {
        cprintf("wmap ERROR: MAP_SHARED must be present in flags\n");
        return FAILED;
    }

    // 2. Validate Address Range: Must be within user space (0x60000000 - 0x80000000)
    if (addr < 0x60000000 || addr >= 0x80000000) {
        cprintf("wmap ERROR: addr=0x%x not in user space\n", addr);
        return FAILED;
    }

    // 3. Validate Address Alignment: Must be page-aligned
    if (addr % PGSIZE != 0) {
        cprintf("wmap ERROR: addr=0x%x is not page-aligned\n", addr);
        return FAILED;
    }

    // 4. Validate Length: Must be greater than 0
    if (length <= 0) {
        cprintf("wmap ERROR: invalid length=%d\n", length);
        return FAILED;
    }

    // 5. Check if Maximum Number of Mappings is Reached
    if (p->num_mappings >= MAX_MAPPINGS) {
        cprintf("wmap ERROR: maximum number of mappings (%d) reached.\n", MAX_MAPPINGS);
        return FAILED;
    }

    // 6. Check for Overlapping Mappings
    for (int i = 0; i < p->num_mappings; i++) {
        uint start = p->mappings[i].addr;
        uint end = start + p->mappings[i].length;
        if (addr < end && (addr + length) > start) {
            cprintf("wmap ERROR: overlapping with existing mapping %d (0x%x - 0x%x).\n", 
                    i + 1, start, end);
            return FAILED; // Overlapping with existing mapping
        }
    }

    // 7. Handle MAP_ANONYMOUS Flag
    if (flags & MAP_ANONYMOUS) {
        // For anonymous mappings, ignore the passed fd and set it to -1 internally
        cprintf("wmap: Handling anonymous mapping. Ignoring passed fd=%d and setting fd to -1.\n", fd);
        fd = -1; // Treat as anonymous mapping regardless of passed fd
    } else {
        // For file-backed mappings, validate the file descriptor
        if (fd < 0 || fd >= NOFILE || p->ofile[fd] == 0) {
            cprintf("wmap ERROR: invalid file descriptor %d.\n", fd);
            return FAILED;
        }
        struct file *f = p->ofile[fd];
        if (f->type != FD_INODE) {
            cprintf("wmap ERROR: file descriptor %d is not FD_INODE.\n", fd);
            return FAILED;
        }
        filedup(f); // Increment the file reference count
        cprintf("wmap: Handling file-backed mapping with fd=%d.\n", fd);
    }

    // 8. Record the Mapping
    struct mapping *m = &p->mappings[p->num_mappings++];
    m->addr = addr;
    m->length = length;
    m->flags = flags;
    if (flags & MAP_ANONYMOUS) {
        m->fd = -1;    // No file descriptor for anonymous mappings
        m->file = 0;   // No file pointer
    } else {
        m->fd = fd;               // Assign the file descriptor
        m->file = p->ofile[fd];  // Assign the file pointer
    }
    m->n_loaded_pages = 0;

    cprintf("wmap: Successfully recorded mapping %d at addr=0x%x with length=%d bytes.\n", 
            p->num_mappings, addr, length);

    // 9. Set Up PTEs with present=0 for the Mapped Region (Lazy Allocation)
    uint end_addr = PGROUNDUP(addr + length);
    for (uint va = addr; va < end_addr; va += PGSIZE) {
        pte_t *pte = walkpgdir(p->pgdir, (const void*)va, 1); // Create page table entry if not present
        if (pte == 0) {
            cprintf("wmap ERROR: walkpgdir failed for va=0x%x\n", va);
            return FAILED;
        }
        // Set PTE to present=0, writable, user-accessible
        *pte = PTE_U | PTE_W; // PTE_P (present) is not set
        // Minimal logging to avoid timeouts
        if (va == addr) { // Log only the first PTE setup for brevity
            cprintf("wmap: Set PTE for va=0x%x to PTE_U | PTE_W (present=0).\n", va);
        }
    }

    // 10. Return the Starting Address of the Mapping
    return addr; // Return the starting address of the mapping
}
// Implement wunmap
int wunmap(uint addr) {
    struct proc *p = myproc();
    int index = -1;

    // Find the mapping corresponding to the provided address
    for (int i = 0; i < p->num_mappings; i++) {
        if (p->mappings[i].addr == addr) {
            index = i;
            break;
        }
    }

    if (index == -1) {
        cprintf("wunmap ERROR: No mapping found at address 0x%x\n", addr);
        return -1;
    }

    struct mapping *m = &p->mappings[index];

    // Check if the mapping is file-backed
    if (m->fd >= 0 && (m->flags & MAP_SHARED)) {
        // File-backed mapping

        struct file *f = m->file;
        if (f == 0 || f->type != FD_INODE) {
            cprintf("wunmap ERROR: Invalid file for mapping at 0x%x\n", addr);
            return -1;
        }

        uint map_start = m->addr;
        uint map_end = map_start + m->length;
        for (uint va = map_start; va < map_end; va += PGSIZE) {
            pte_t *pte = walkpgdir(p->pgdir, (void *)va, 0);
            if (pte && (*pte & PTE_P)) {
                // Begin a transaction for this page
                begin_op();

                // Get the physical address
                uint pa = PTE_ADDR(*pte);

                // Convert physical address to kernel virtual address
                char *vpa = P2V(pa);

                // Calculate the file offset
                uint file_offset = va - map_start;

                // Write the page back to the file
                if (writei(f->ip, vpa, file_offset, PGSIZE) != PGSIZE) {
                    cprintf("wunmap ERROR: Failed to write page at va=0x%x to file\n", va);
                    end_op(); // End the transaction before returning
                    return -1;
                }

                // End the transaction
                end_op();

                // Clear the PTE
                *pte = 0;
            }
        }

        // Close the file
        fileclose(f);
    } else {
        // Anonymous mapping

        uint map_start = m->addr;
        uint map_end = map_start + m->length;
        for (uint va = map_start; va < map_end; va += PGSIZE) {
            pte_t *pte = walkpgdir(p->pgdir, (void *)va, 0);
            if (pte && (*pte & PTE_P)) {
                // Get the physical address
                uint pa = PTE_ADDR(*pte);

                if (m->flags & MAP_SHARED) {
                    // For shared anonymous mappings, do not free the physical page
                    // Just clear the PTE to unmap it from this process
                    // Leave the physical page intact
                } else {
                    // For private mappings, free the physical page
                    kfree(P2V(pa));
                }

                // Clear the PTE
                *pte = 0;
            }
        }
    }

    // Remove the mapping from the process's mapping list
    for (int i = index; i < p->num_mappings - 1; i++) {
        p->mappings[i] = p->mappings[i + 1];
    }
    p->num_mappings--;

    return 0; // Success
}
