/*
 * Apox Operating System
 * Copyright (c) 2006-2007, cryptocode
 */

#ifndef _APOX_ELF32_H_
#define _APOX_ELF32_H_

#include <kernel/libc/std.h>
#include <kernel/vmm/VMProvider.h>
#include <lib/elf/Elf.h>

/* The following definitions are straight off the ELF standard */

typedef uint32 Elf32_Addr;
typedef uint16 Elf32_Half;
typedef uint32 Elf32_Off;
typedef int32 Elf32_Sword;
typedef uint32 Elf32_Word;

struct Elf32_Ehdr
{
    unsigned char	e_ident[EI_NIDENT];
    Elf32_Half		e_type;
    Elf32_Half		e_machine;
    Elf32_Word		e_version;
    Elf32_Addr		e_entry;
    Elf32_Off		e_phoff;
    Elf32_Off		e_shoff;
    Elf32_Word		e_flags;
    Elf32_Half		e_ehsize;
    Elf32_Half		e_phentsize;
    Elf32_Half		e_phnum;
    Elf32_Half		e_shentsize;
    Elf32_Half		e_shnum;
    Elf32_Half		e_shstrndx;
}__attribute__((packed));

struct Elf32_Shdr
{
    Elf32_Word		sh_name;
    Elf32_Word		sh_type;
    Elf32_Word		sh_flags;
    Elf32_Addr		sh_addr;
    Elf32_Off		sh_offset;
    Elf32_Word		sh_size;
    Elf32_Word		sh_link;
    Elf32_Word		sh_info;
    Elf32_Word		sh_addralign;
    Elf32_Word		sh_entsize;
}__attribute__((packed));

struct Elf32_Phdr
{
    Elf32_Word		p_type;
    Elf32_Off		p_offset;
    Elf32_Addr		p_vaddr;
    Elf32_Addr		p_paddr;
    Elf32_Word		p_filesz;
    Elf32_Word		p_memsz;
    Elf32_Word		p_flags;
    Elf32_Word		p_align;
}__attribute__((packed));

struct Elf32_Sym
{
    Elf32_Word		st_name;
    Elf32_Addr		st_value;
    Elf32_Word		st_size;
    unsigned char	st_info;
    unsigned char 	st_other;
    Elf32_Half		st_shndx;
}__attribute__((packed));

#define ELF32_ST_BIND(i) ((i) >> 4)
#define ELF32_ST_TYPE(i) ((i) & 0xf)
#define ELF32_ST_INFO(b, t) (((b) << 4) + ((t) & 0xf))

/* Relocation entries */

struct Elf32_Rel
{
    Elf32_Addr r_offset;
    Elf32_Word r_info;
}__attribute__((packed));

struct Elf32_Rela
{
    Elf32_Addr r_offset;
    Elf32_Word r_info;
    Elf32_Sword r_addend;
}__attribute__((packed));

#define ELF32_R_SYM(i) ((i) >> 8)
#define ELF32_R_TYPE(i) ((unsigned char)(i))
#define ELF32_R_INFO(s, t) (((s) << 8) + (unsigned char)(t))

struct Elf32_Dyn
{
    Elf32_Sword d_tag;
    union
    {
        Elf32_Word d_val;
        Elf32_Addr d_ptr;
    } d_un;
}__attribute__((packed));

class VMSpace;

/**
 * Represents a 32-bit ELF executable or shared library
 */
class Elf32
{
    public:

        /**
         * Load ELF executable from a node into an address space
         *
         * @note Static
         */
        static error_t load(VMSpace& aspace, Node* node, OUT Elf32*& elf);

        /**
         * Load an ELF kernel, sets up the basic page tables and enables
         * paging. This is implemented only when compiled with the
         * BUILD_APOX_BUILDMODE_LOADER option.
         *
         * @param basepage  First free physical page. A few megabytes are
         *                  temporarily mapped over this for the frame mgr.
         * @param hdr       Kernel image address. Must be page aligned.
         * @param elf       OUT if successfull, contains the Elf32 object
         */
        static error_t loadkernel(paddr basepage, Elf32_Ehdr& hdr, OUT Elf32*& elf);

        /**
         * Copy master into this Elf32 object
         */
        void copyFrom(Elf32& master)
        {
            // A bitwise copy will do for now
            *this = master;
        }

        /** Get virtual entry point address as reported by the elf header */
        vaddr getEntryPoint()
        {
            return hdr.e_entry;
        }

        /**
         * Return symbol name associated with the address
         *
         * @param addr Address to look up
         * @return char* Symbol name, or null if not found
         */
        char* getSymbolName(vaddr addr);

        /**
         * Set address and size of global ctor section. This section is an
         * array of function pointers, and each function is called by the
         * kernel program loader before main() is called.
         *
         * @param va Virtual address of section.
         * @param size Section size, in bytes
         */
        void setGlobalCtorArray(vaddr va, int size)
        {
            globalCtorArray = va;
            globalCtorCount = size / sizeof(vaddr);
        }

        vaddr getGlobalCtorArray()
        {
            return globalCtorArray;
        }

        int getGlobalCtorCount()
        {
            return globalCtorCount;
        }

        /**
         * Set address and size of global dtor section. This section is an
         * array of function pointers, and each function is called by the
         * kernel program loader after main() has finished.
         *
         * @param va Virtual address of section.
         * @param size Section size, in bytes
         */
        void setGlobalDtorArray(vaddr va, int size)
        {
            globalDtorArray = va;
            globalDtorCount = size / sizeof(vaddr);
        }

        vaddr getGlobalDtorArray()
        {
            return globalDtorArray;
        }

        int getGlobalDtorCount()
        {
            return globalDtorCount;
        }

        /**
         * Last address
         *
         * @return vaddr
         */
        vaddr getImageEndPage()
        {
            return endpage;
        }

//    protected:

        /** C'tor */
        Elf32();

        /** D'tor */
        ~Elf32();

    private:

        vaddr globalCtorArray;
        int globalCtorCount;
        vaddr globalDtorArray;
        int globalDtorCount;

        /**
         * ELF header
         */
        Elf32_Ehdr hdr;

        /**
         * Section headers base
         */
        Elf32_Shdr* shdrBase;

        /**
         * Symbol table section; null if not found during load.
         */
        Elf32_Shdr* symSection;

        /**
         * String section
         */
        Elf32_Shdr* strSection;

        /**
         * The address of the page right after the last segment mapped in to
         * memory. This is useful for extending the image with adjacent
         * semi-dynamic allocations (for instance, this is used to put the
         * frame manager data structures right after the kernel image.)
         */
        vaddr endpage;
};

#endif // _APOX_ELF32_H_
