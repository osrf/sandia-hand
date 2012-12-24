OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
	rom (rx)  : ORIGIN = 0x00080000, LENGTH = 0x00080000 /* flash  , 512k */
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00010000 /* sram   ,  64k */
}

/* Section Definitions */ 
SECTIONS 
{ 
    .bl_text 0x80000 :
    {
      . = ALIGN(4);
      _bl_sfixed = .;
      KEEP(*bootloader_startup.S.o(*.text))
      KEEP(*(.bl_text))  /**bootloader.o(.bl_text))*/
      /*KEEP(*bl_text .bl_text.*)*/
      /*KEEP(*(.bl_id))*/
      . = 0x1ffc;
      _sid = .;
      KEEP(*(.bl_id))
    } > rom 
    .text    0x82000 : 
    { 
        /*. = ALIGN(4);*/
        _sfixed = .;
        KEEP(*(.vectors .vectors.*))
        *(.text .text.* .gnu.linkonce.t.*) 	      
        *(.glue_7t) *(.glue_7)		                
        *(.rodata .rodata* .gnu.linkonce.r.*)		    	                  
        *(.ARM.extab* .gnu.linkonce.armextab.*)

        /* Support C constructors, and C destructors in both user code
           and the C library. This also provides support for C++ code. */
        . = ALIGN(4);
        KEEP(*(.init))
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(0x4);
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))

        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))

        . = ALIGN(4);
        _efixed = .;            /* End of text section */
    } > rom

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    PROVIDE_HIDDEN (__exidx_start = .);
    .ARM.exidx :
    {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rom
    PROVIDE_HIDDEN (__exidx_end = .);

    . = ALIGN(4); 
    _etext = .;

    .relocate : AT (_etext)
    {
        . = ALIGN(4);
        _srelocate = .;
        *(.ramfunc .ramfunc.*);
        *(.data .data.*);
        . = ALIGN(4);
        _erelocate = .;
    } > ram

    _normal_start_bss = .;
    .bss (NOLOAD) : /* smash the bootloader bss with the application bss */
    { 
        . = ALIGN(4);
        _sbss = . ;
        _szero = .;
        *(.debug_bss .debug_bss.*)
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = . ;
        _ezero = .;
    } > ram
    _normal_end_bss = .;

    . = _normal_start_bss;
    _bl_szero = .;
    .bl_bss (NOLOAD) :  /* plunk down the bootloader bss */
    {
      KEEP(*(.bl_bss))
    } > ram
    _bl_ezero = .;

    . = _normal_end_bss;

    /* stack section */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.stack .stack.*)
    } > ram

    . = ALIGN(4); 
    _end = . ; 
}
