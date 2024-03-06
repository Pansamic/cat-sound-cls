/**
 * @file ppbuf.c
 * @author pansamic(pansamic@foxmail.com)
 * @brief ping pong buffer
 * @version 0.1
 * @date 2024-03-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stddef.h>
#include <string.h>
#include "ppbuf.h"

/**
 * @brief mount two buffers to a ping pong buffer controller.
 * 
 * @param ppbuf ppbuf controller type pointer
 * @param buf1 unsigned char type pointer
 * @param capacity1 whole capacity of buf1, unit:byte
 * @param buf2 unsigned char type pointer
 * @param capacity2 whole capacity of buf2, unit:byte
 * @return status 0:success 1:fail
 */
unsigned char ppbuf_init(ppbuf_t *ppbuf, unsigned char *buf1, unsigned int capacity1, unsigned char *buf2, unsigned int capacity2)
{
    if(buf1 == NULL || buf2 == NULL)
        return 1;
    ppbuf->bufs[0] = buf1;
    ppbuf->bufs[1] = buf2;
    ppbuf->size[0] = 0;
    ppbuf->size[1] = 0;
    ppbuf->busy[0] = 0;
    ppbuf->busy[1] = 0;
    ppbuf->input_index = 0;
    ppbuf->capacity[0] = capacity1;
    ppbuf->capacity[1] = capacity2;
    memset(buf1, 0, capacity1);
    memset(buf2, 0, capacity2);
    return 0;
}

/**
 * @brief write data to the input buffer of the ping pong buffer controller.
 * 
 * @param ppbuf ppbuf controller type pointer
 * @param data data to be written
 * @param size size of data, unit:byte
 * @return status 0:success 1:fail
 */
unsigned char ppbuf_write(ppbuf_t *ppbuf, unsigned char *data, unsigned int size)
{
    if(ppbuf->busy[ppbuf->input_index] || ppbuf->size[ppbuf->input_index] + size > ppbuf->capacity[ppbuf->input_index])
        return 1;
    for(unsigned int i = 0; i < size; i++)
    {
        ppbuf->bufs[ppbuf->input_index][ppbuf->size[ppbuf->input_index] + i] = data[i];
    }
    ppbuf->size[ppbuf->input_index] += size;
    return 0;
}

/**
 * @brief read data from the output buffer of the ping pong buffer controller.
 * 
 * @param ppbuf ppbuf controller type pointer
 * @param data data destination pointer, the data will be written to this address
 * @param size data size to be read, unit:byte
 * @return status 0:success 1:fail
 */
unsigned char ppbuf_read(ppbuf_t *ppbuf, unsigned char *data, unsigned int size)
{
    if(ppbuf->size[1 - ppbuf->input_index] < size)
        return 1;
    for(unsigned int i = 0; i < size; i++)
    {
        data[i] = ppbuf->bufs[1 - ppbuf->input_index][i];
    }
    ppbuf->size[1 - ppbuf->input_index] -= size;
    for(unsigned int i = 0; i < ppbuf->size[1 - ppbuf->input_index]; i++)
    {
        ppbuf->bufs[1 - ppbuf->input_index][i] = ppbuf->bufs[1 - ppbuf->input_index][i + size];
    }
    return 0;
}

/**
 * @brief increase the size of the input buffer of the ping pong buffer controller.
 *     this function is used to increase the size of the input buffer after writing 
 *     data to the input buffer.
 * 
 * @param ppbuf ppbuf controller type pointer
 * @param size increase size, unit:byte
 * @return status 0:success 1:fail
 */
unsigned char ppbuf_increase(ppbuf_t *ppbuf, unsigned int size)
{
    if(ppbuf->size[ppbuf->input_index] + size > ppbuf->capacity[ppbuf->input_index])
        return 1;
    ppbuf->size[ppbuf->input_index] += size;
    return 0;
}

/**
 * @brief decrease the size of the output buffer of the ping pong buffer controller.
 *     this function is used to decrease the size of the output buffer after reading
 *     data from the output buffer.
 * 
 * @param ppbuf ppbuf controller type pointer
 * @param size decrease size, unit:byte
 * @return status 0:success 1:fail
 */
unsigned char ppbuf_decrease(ppbuf_t *ppbuf, unsigned int size)
{
    if(ppbuf->size[1 - ppbuf->input_index] < size)
        return 1;
    ppbuf->size[1 - ppbuf->input_index] -= size;
    return 0;
}

/**
 * @brief switch the input and output buffer of the ping pong buffer controller.
 * 
 * @param ppbuf ppbuf controller type pointer
 * @return status 0:success 1:fail
 */
unsigned char ppbuf_switch(ppbuf_t *ppbuf)
{
    if(!ppbuf->busy[1 - ppbuf->input_index] && !ppbuf->busy[ppbuf->input_index])
    {
        ppbuf->input_index = 1 - ppbuf->input_index;
    }
    else
    {
        return 1;
    }
    return 0;
}

unsigned char ppbuf_empty_input(ppbuf_t *ppbuf)
{
    if(ppbuf->busy[ppbuf->input_index])
        return 1;
    ppbuf->size[ppbuf->input_index] = 0;
    return 0;
}

unsigned char ppbuf_empty_output(ppbuf_t *ppbuf)
{
    if(ppbuf->busy[1 - ppbuf->input_index])
        return 1;
    ppbuf->size[1 - ppbuf->input_index] = 0;
    return 0;
}

unsigned char* ppbuf_get_input(ppbuf_t *ppbuf)
{
    return ppbuf->bufs[ppbuf->input_index];
}
unsigned char* ppbuf_get_output(ppbuf_t *ppbuf)
{
    return ppbuf->bufs[1 - ppbuf->input_index];
}
unsigned int ppbuf_get_input_size(ppbuf_t *ppbuf)
{
    return ppbuf->size[ppbuf->input_index];
}
unsigned int ppbuf_get_output_size(ppbuf_t *ppbuf)
{
    return ppbuf->size[1 - ppbuf->input_index];
}
unsigned int ppbuf_get_input_capacity(ppbuf_t *ppbuf)
{
    return ppbuf->capacity[ppbuf->input_index];
}
unsigned int ppbuf_get_output_capacity(ppbuf_t *ppbuf)
{
    return ppbuf->capacity[1 - ppbuf->input_index];
}