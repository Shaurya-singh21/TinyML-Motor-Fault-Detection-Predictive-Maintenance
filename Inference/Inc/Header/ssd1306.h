#ifndef SSD1306_H
#define SSD1306_H

/*
 * ssd1306.h
 * SSD1306 128×64 OLED driver over I2C + DMA
 * Depends on peripheral_init.h for hardware layer
 */

#include "i2c.h"
#include "stdint.h"
#define OLED_W      128
#define OLED_H       64
#define OLED_PAGES    8    /* 64 pixels / 8 bits per page */
#define OLED_ADDR   0x78
/* ── Init ─────────────────────────────────────────────────────────────────── */
void oled_init(void);          /* send SSD1306 startup command sequence     */

/* ── Draw into framebuffer (does NOT update display until oled_flush) ─────── */
void oled_clear(void);                          /* fill fb with zeros        */
void oled_set_pixel(int x, int y, int on);      /* set/clear a single pixel  */
void oled_draw_char(int x, int page, char c);   /* 5×8 glyph at page row     */
void oled_print(int x, int page, const char *s);/* string at page row        */

/* ── Push framebuffer to display via DMA ─────────────────────────────────── */
void oled_flush(void);
uint8_t oled_is_busy(void);
void oled_dma_complete(void);
#endif
