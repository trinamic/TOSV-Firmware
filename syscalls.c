/**
* @file syscalls.c
*
* @brief low level functions for the newlib (ARM clib)
*
* @author Michael Struwe <mis@port.de>
*
* This module contains low level functions that are used by the clib. Because
* these are target specific, the functions must be implemented explicitly.
*
* @copyright
* Copyright 2009-2014 port GmbH Halle/Saale.
* This software is protected Intellectual Property and may only be used
* according to the licence agreement.
*/

#include <stdlib.h>

/* constant definitions
---------------------------------------------------------------------------*/
#define HEAP_SIZE 5120

/* local defined data types
---------------------------------------------------------------------------*/

/* list of external used functions, if not in headers
---------------------------------------------------------------------------*/
//void _exit(int i);
void* _sbrk(int incr);

/* list of global defined functions
---------------------------------------------------------------------------*/

/* list of local defined functions
---------------------------------------------------------------------------*/

/* external variables
---------------------------------------------------------------------------*/

/* global variables
---------------------------------------------------------------------------*/

/* local defined variables
---------------------------------------------------------------------------*/

/**
* @brief dummy exit function
*
* The function simulates a shutdown of the application by disabling the
* interrupts and entering an endless loop.
*
* @return none
*/
//void _exit(
//    int i       /**< dummy process id */
//    )
//{
//    __disable_irq();
//
//    if (i <= 0) {
//        i = 1;
//    }
//
//    while(1);
//}

/**
* @brief allocate memmory blocks for malloc
*
* This function provides data memory from the heap for the malloc function.
*
* @return start address of allocated memory or NULL
*/
void* _sbrk(int incr)
{
    static unsigned char _heap[HEAP_SIZE];
    static void* pHeap = &_heap[0];
    static int heap_size = HEAP_SIZE;
    void * pHeap_tmp;

    if ((heap_size - incr) >= 0) {
        pHeap_tmp = pHeap;
        heap_size -= incr;
        pHeap += incr;
        return pHeap_tmp;
    }
    else {
        return NULL;
    }
}

