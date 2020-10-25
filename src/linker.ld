/*******************************************
   Memory Definitions für LPC 1768
*******************************************/
MEMORY
{
     flash  (RX) : ORIGIN = 0x00000000, LENGTH = 512k /* .text */
    isrram  (RW) : ORIGIN = 0x10000000, LENGTH = 256  /* RAM-Sprungtabelle für Interrupts */
       ram  (RW) : ORIGIN = 0x10000100, LENGTH = 16k-256	/* .data, .bss */
    ahbram  (RW) : ORIGIN = 0x2007C040, LENGTH = 32K-64-32	/* .ahb */
}

/*
 * IAP routine location. 1 makes the routine a thumb routine!
 */
IAP = 0x1FFF1FF1;

/*******************************************
   Section Definitions
*******************************************/
SECTIONS
{
    . = 0;
    /* Code-Speicher im Flash ***********/
    .text :
    {
        . = ALIGN(4);
        __code_start__ = .;

        KEEP(*( .isr_vector ));

        *(.text)
        *(.text.*)

        . = ALIGN(4);
        __code_end__ = .;

        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)

    } >flash

    . = ALIGN(4);

    /* contains unwinding information */
    .ARM.exidx : {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
        . = ALIGN(4);
    } >flash

    /* Konstanten im Flash ****************/
    .rodata . :
    {
        . = ALIGN(4);
        *(.rodata)
        
        . = ALIGN(4);   
        *(.rodata.*)

        PROVIDE(start_ctors = .);
        KEEP(*( .init_array ));
        KEEP(*(SORT_BY_INIT_PRIORITY( .init_array.* )));
        PROVIDE(end_ctors = .);

        PROVIDE(start_dtors = .);
        KEEP(*( .fini_array ));
        KEEP(*(SORT_BY_INIT_PRIORITY( .fini_array.* )));
        PROVIDE(end_dtors = .);
        . = ALIGN(4);
    } >flash

    . = ALIGN(4);
    _etext = . ;

    PROVIDE (__text__end = .);

    /************* RAM ******************************/
    isr_ramvector :
    {
        __isr_ramvector_start__ = . ;
        PROVIDE (__isr_ramvector__start = .);

        *(.isr_vector_ram)
        /*(.gnu.linkonce.b*)*/
        . = ALIGN(4);
        __isr_ramvector_end__ = . ;
        PROVIDE (__isr_ramvector__end = .);

    } >isrram

    stack 0x10000100 :
    {
        . = ALIGN(4);
        
        *(.stackarea)
        
        . = ALIGN(4);
    } >ram

    /* Initialisierte Variablen im Flash */
    .data : AT (_etext)
    {
        __data_start__ = . ;
        PROVIDE (__data__start = .) ;

        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d*)
        SORT(CONSTRUCTORS)
        __data_end__ = . ;
        PROVIDE (__data__end = .);
    } >ram

    . = ALIGN(4);
    _edata = . ;

   .bss :
   {
        __bss_start__ = . ;
        PROVIDE (__bss__start = .);

        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b*)
        . = ALIGN(4);
        __bss_end__ = . ;
        PROVIDE (__bss__end = .);
    } >ram

    _end = .;
    PROVIDE (end = .);

    /************** AHBRAM **************************/
    ahb_ram : AT (ORIGIN(ahbram)) {
    	*(.ahb)
    } >ahbram
}

