/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

#include <lib/elf/Elf32.h>
#include <kernel/Errors.h>
#include <arch/vmm/VMM.h>
#include <kernel/vmm/vmm.h>
#include <kernel/vmm/VMVirtualMgr.h>
#include <kernel/io/Channel.h>
#include <kernel/threads/Thread.h>
#include <kernel/util/Hexdump.h>

/**
 * C'tor
 */
Elf32::Elf32()
{
    globalCtorArray = 0;
    globalCtorCount = 0;
    globalDtorArray = 0;
    globalDtorCount = 0;

    shdrBase = null;
    symSection = null;
    strSection = null;
}

/**
 * D'tor
 */
Elf32::~Elf32()
{
}

/* See declaration  */
error_t Elf32::load(VMSpace& aspace, Node* node, OUT Elf32*& elf)
{
    assert(&aspace != &VMM::getKernelSpace());
    error_t res = E_OK;

    Channel* ch = (Channel*)node->getInterface(IIDChannel);
    if(ch == null)
        return E_NOT_FOUND;

    // Read the header
    Elf32_Ehdr* hdr = new Elf32_Ehdr();
    if(ch->read((uint8*)hdr, sizeof(Elf32_Ehdr)) != sizeof(Elf32_Ehdr))
        return E_INVALID;

    // Validate the header
    if(memcmp(hdr->e_ident, ELF_MAGIC, 4) != 0 ||
       hdr->e_ident[EI_VERSION] != EV_CURRENT ||
       hdr->e_phentsize < sizeof(Elf32_Phdr))
    {

        printf("Invalid ELF image\n");
        return E_INVALID;
    }

#ifdef LITTLE_ENDIAN
    if(hdr->e_ident[EI_DATA] != ELFDATA2LSB)
    {
        printf("Invalid ELF image. Little endian format expected.\n");
        return E_INVALID;
    }
#else
    if(hdr->e_ident[EI_DATA] != ELFDATA2MSB)
    {
        printf("Invalid ELF image. Big endian format expected.\n");
        return E_INVALID;
    }
#endif

    // Could be file, network or attached module
    printf("ELF Prog Headers: %u, type (1=reloc,2=exec,3=shared): %u\n",
            (int) hdr->e_phnum, (int)hdr->e_type);

    Elf32_Phdr* phdr = new Elf32_Phdr[hdr->e_phnum];
    ch->setPosition(hdr->e_phoff);
    if(ch->read((uint8*)phdr, sizeof(Elf32_Phdr)*hdr->e_phnum) != (int)sizeof(Elf32_Phdr)*hdr->e_phnum)
        return E_INVALID;

    // Spin through the program headers
    for (int i=0; res == E_OK && i < hdr->e_phnum; i++)
    {
        if(phdr->p_type == PT_LOAD)
        {
            if(phdr->p_vaddr < VM_LOADER_VMA || phdr->p_vaddr >= VM_KERNEL_VMA)
            {
                printf("INVALID vaddr WHILE LOADING ELF FILE\n");
                res =  E_INVALID;
            }
            else
            {
                printf("foff,va,fsz/msz,alig,flg,type: 0x%x, 0x%x, %u/%u, %u, %u, ",
                    phdr->p_offset,phdr->p_vaddr, phdr->p_filesz, phdr->p_memsz, phdr->p_align, phdr->p_type);

                if(phdr->p_flags & PF_R) printf("R");
                if(phdr->p_flags & PF_W) printf("W");
                if(phdr->p_flags & PF_X) printf("X");
                printf("\n");

                // Most of the time, only one segment has a bss but we handle the
                // rare cases too.
                uint32 bssSize = phdr->p_memsz - phdr->p_filesz;
                if(phdr->p_filesz < phdr->p_memsz)
                    kprintf("This segment has a BSS of %u bytes\n", bssSize);

                // Pages with real data
                uint32 realPages = Math::divup(phdr->p_filesz, VM_PAGE_SIZE);

                //uint32 bssPages = bssSize / VM_PAGE_SIZE;
                uint32 bssPages = (VM_PAGE_ALIGN(phdr->p_vaddr+phdr->p_memsz) -
                                   VM_PAGE_ALIGN(phdr->p_vaddr+phdr->p_filesz)) / VM_PAGE_SIZE;

                // Number of BSS bytes in the last demand-loaded page. May be
                // zero if the BSS is page aligned (accidentally or by means of
                // a linker scripts). We zero this part below.
                uint32 bssZeroBytes = VM_PAGE_SIZE - ((phdr->p_vaddr+phdr->p_filesz) % VM_PAGE_SIZE);

                //if(phdr->p_filesz % VM_PAGE_SIZE == 0)
                    //bssPages = Math::divup(bssSize, VM_PAGE_SIZE);

                // Provider for this segment
                VMNodeProvider* intervalProvider = new VMNodeProvider(node);

                // Set the node offset (where the data physically resides)
                intervalProvider->setOffset(phdr->p_offset);

                // What's going on here? We can't really trust the segments
                // to be page aligned (though the linker script probably does
                // it, but you never know), so have to inform the provider that
                // the data should be read into a particular offset relative to
                // the faulting page.
                vaddr start = VM_PAGE_ROUND(phdr->p_vaddr);
                vaddr rem = phdr->p_vaddr % VM_PAGE_SIZE;
                intervalProvider->setIntervalOffset(rem);

                int prot = (phdr->p_flags & PF_W) ? vm::ProtRW : vm::ProtRead;
                res = aspace.getVirtualMgr().alloc(
                    realPages, start, prot, vm::AllocFixed, intervalProvider);

                printf("realPages = %u, res = %u, start 0x%x, bss start 0x%x, len %u, offset: %u, rem: %u\n",
                       realPages, res, start, (phdr->p_vaddr + phdr->p_filesz),bssSize, intervalProvider->getOffset(), rem);

                // Zero BSS
                if(bssSize > 0)
                {
                    // The bss-only pages are zero-provided, backed by anon-
                    // storage.
                    if(bssPages)
                    {
                        VMZeroProvider* zp = new VMZeroProvider();

                        vaddr bssStart = start + realPages*VM_PAGE_SIZE;

                        printf("-------- %u BSS PAGES FROM 0x%x\n", bssPages, bssStart);

                        res = aspace.getVirtualMgr().alloc(
                            bssPages, bssStart, prot, vm::AllocFixed, zp);
                        printf("res = %u (realpages=%u)\n", res, realPages);
                    }
                    else
                        printf("NO BSS PAGES\n");

                    printf("memsetting %u bytes [0x%x..0x%x]... ", bssZeroBytes, (phdr->p_vaddr + phdr->p_filesz), (phdr->p_vaddr + phdr->p_filesz)+bssZeroBytes);
                    // Zero the BSS part of the last data page. This will cause
                    // that page to load.
                    memset((void*)(phdr->p_vaddr + phdr->p_filesz), 0, bssZeroBytes);
                    printf("Done\n");
                }

                if(res != E_OK)
                {
                    aspace.getVirtualMgr().dump();
                    printf("res=%u\n", res);
                    assert(false);
                }
            }
        }

        phdr++;
    }

    if(res == E_OK)
    {
        printf("READING SECTIONS\n");
        elf = new Elf32();

        // Copy
        elf->hdr = *hdr;

        // Loop through sections to get .ctors, .dtors, ,symtab, .init and .fini. The
        // userspace pre-main function must call these. The .ctors/.dtors
        // sections are arrays of (void)(function*)() ptrs, each element vaddr
        // in size. Size of arrays in bytes in section header. So a size of 8 on
        // x86_32 means two global c'tors. Call in order of appearance.

        // Read section headers
        Elf32_Shdr* shdr = new Elf32_Shdr[hdr->e_shnum];
        //printf("setting position: %u, reading %u\n", hdr->e_shoff, (int)sizeof(Elf32_Shdr)*hdr->e_shnum);
        ch->setPosition(hdr->e_shoff);
        int read = ch->read((uint8*)shdr, sizeof(Elf32_Shdr)*hdr->e_shnum);
        if(read != (int)sizeof(Elf32_Shdr)*hdr->e_shnum)
        {
            fprintf(stderr, "READ FAILED: %d != %d\n", read, sizeof(Elf32_Shdr)*hdr->e_shnum);
            return E_INVALID;
        }

        elf->shdrBase = shdr;

        printf("e_shoff: %u, sizeof=%u, size=%u, stringtbl: 0x%x\n",
               hdr->e_shoff, sizeof(Elf32_Shdr), hdr->e_shentsize, hdr->e_shstrndx);

        printf("SECTIONS:\n\n");
        for(uint16 i=0; i < hdr->e_shnum; i++)
        {
            // Section header index for section names OK?
            if(hdr->e_shstrndx != SHN_UNDEF)
            {
                char sname[128];

                // Note: e_shstrndx indexes a section containing section names
                ch->setPosition(elf->shdrBase[hdr->e_shstrndx].sh_offset + shdr->sh_name);
                for(int j=0; ; j++)
                {
                    sname[j] = ch->read();
                    if(sname[j] == '\0')
                        break;
                }

                printf("%s: ", sname);

                if(strcmp(sname, ".ctors") == 0)
                    elf->setGlobalCtorArray((vaddr)shdr->sh_addr, shdr->sh_size);
                else if(strcmp(sname, ".dtors") == 0)
                    elf->setGlobalDtorArray((vaddr)shdr->sh_addr, shdr->sh_size);
                else if(strcmp(sname, ".symtab") == 0)
                    elf->symSection = (Elf32_Shdr*)((char*)hdr + shdr->sh_offset);
                else if(strcmp(sname, ".strtab") == 0)
                    elf->strSection = (Elf32_Shdr*)((char*)hdr + shdr->sh_offset);
                else if(strcmp(sname, ".init") == 0)
                { NO_IMPL(); }
                else if(strcmp(sname, ".fini") == 0)
                { NO_IMPL(); }
            }

            printf("    addr: 0x%x, off: %u, type: %u, size: %u, align: %u, nameidx: %u\n",
                   shdr->sh_addr, shdr->sh_offset, shdr->sh_type,
                   shdr->sh_entsize, shdr->sh_addralign, shdr->sh_name);

            shdr++;
        }
        printf("\n");
    }
    else
        printf("ELF loading failed with %u\n", res);


    aspace.getVirtualMgr().dump();

    return res;
}

/* See declaration  */
char* Elf32::getSymbolName(vaddr addr)
{
    NO_IMPL();

    char* res = null;

    Elf32_Sym* sym;

    if(symSection != null)
    {
        sym = (Elf32_Sym*)symSection;
    }

    return res;
}
