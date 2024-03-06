#ifndef __PPBUF_H__
#define __PPBUF_H__

typedef struct ppbuf_t {
    unsigned char input_index;
    unsigned char busy[2];
    unsigned char *bufs[2];
    unsigned int size[2];
    unsigned int capacity[2];
} ppbuf_t;

unsigned char ppbuf_init(ppbuf_t *ppbuf, unsigned char *buf1, unsigned int capacity1, unsigned char *buf2, unsigned int capacity2);
unsigned char ppbuf_switch(ppbuf_t *ppbuf);
unsigned char ppbuf_write(ppbuf_t *ppbuf, unsigned char *data, unsigned int size);
unsigned char ppbuf_read(ppbuf_t *ppbuf, unsigned char *data, unsigned int size);
unsigned char ppbuf_increase(ppbuf_t *ppbuf, unsigned int size);
unsigned char ppbuf_decrease(ppbuf_t *ppbuf, unsigned int size);
unsigned char ppbuf_empty_input(ppbuf_t *ppbuf);
unsigned char ppbuf_empty_output(ppbuf_t *ppbuf);
unsigned char* ppbuf_get_input(ppbuf_t *ppbuf);
unsigned char* ppbuf_get_output(ppbuf_t *ppbuf);
unsigned int ppbuf_get_input_size(ppbuf_t *ppbuf);
unsigned int ppbuf_get_output_size(ppbuf_t *ppbuf);
unsigned int ppbuf_get_input_capacity(ppbuf_t *ppbuf);
unsigned int ppbuf_get_output_capacity(ppbuf_t *ppbuf);

#endif // __PPBUF_H__