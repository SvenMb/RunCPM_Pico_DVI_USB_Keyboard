// SPDX-FileCopyrightText: 2023 Mockba the Borg
// SPDX-FileCopyrightText: 2023 Jeff Epler for Adafruit Industries
// SPDX-FileCopyrightText: 2024 Hisashi Kato
//
// SPDX-License-Identifier: MIT

#include <SdFat.h>  // SDFat - Adafruit Fork
#include <PicoDVI.h>
#include <Adafruit_TinyUSB.h>
#include "../../console.h"
#include "../../arduino_hooks.h"

#include "keymapperUS.h"

#define USE_DISPLAY (1)
#define USE_KEYBOARD (1)


#ifndef USE_DISPLAY
#define USE_DISPLAY (0)
#endif

#ifndef USE_KEYBOARD
#define USE_KEYBOARD (0)
#endif

#ifndef USE_MSC
#define USE_MSC (0)
#endif


uint8_t getch_serial1(void) {
  while (true) {
    int r = Serial1.read();
    if (r != -1) {
      return r;
    }
  }
}

bool kbhit_serial1(void) {
  return Serial1.available();
}


#define USBH_KEY_BUFFER_SIZE 64

uint8_t usbhkbuf[USBH_KEY_BUFFER_SIZE];
uint8_t usbhkbufnum = 0;

bool usbhkbd_write(uint8_t code) {
  if (usbhkbufnum > USBH_KEY_BUFFER_SIZE) {
    return false;
  } else {
    usbhkbuf[usbhkbufnum] = code;
    usbhkbufnum++;
    return true;
  }
}

uint8_t usbhkbd_available(void) {
  if (usbhkbufnum == 0) return 0;
  else return usbhkbufnum;
}

int usbhkbd_read(void) {
  if (usbhkbufnum == 0) {
    return (-1);
  } else {
    usbhkbufnum--;
    uint8_t code = usbhkbuf[usbhkbufnum];
    return code;
  }
}

uint8_t getch_usbh(void) {
  while (true) {
    int r = usbhkbd_read();
    if (r != -1) {
      return r;
    }
  }
}

bool kbhit_usbh(void) {
  return usbhkbd_available();
}



// USB Keyboard
#if USE_KEYBOARD

#ifndef USE_TINYUSB_HOST
#error This sketch requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

#define KBD_INT_TIME 100  // USB HOST processing interval us

static repeating_timer_t rtimer;

#define LANGUAGE_ID 0x0409   // Language ID: English
Adafruit_USBH_Host USBHost;  // USB Host object

#define MAX_REPORT 4

static struct {  // Each HID instance can has multiple reports
  uint8_t report_count;
  tuh_hid_report_info_t report_info[MAX_REPORT];
} hid_info[CFG_TUH_HID];

static bool keyboard_mounted = false;
static uint8_t keyboard_dev_addr = 0;
static uint8_t keyboard_idx = 0;
static uint8_t keyboard_leds = 0;
static bool keyboard_leds_changed = false;

int old_ascii = -1;
uint32_t repeat_timeout;
// this matches Linux default of 500ms to first repeat, 1/20s thereafter
const uint32_t default_repeat_time = 50;
const uint32_t initial_repeat_time = 500;

void send_ascii(uint8_t code, uint32_t repeat_time = default_repeat_time) {
  old_ascii = code;
  repeat_timeout = millis() + repeat_time;
  usbhkbd_write(code);
}

void usb_host_task(void) {
  USBHost.task();
  uint32_t now = millis();
  uint32_t deadline = repeat_timeout - now;
  if (old_ascii >= 0 && deadline > INT32_MAX) {
    send_ascii(old_ascii);
    deadline = repeat_timeout - now;
  } else if (old_ascii < 0) {
    deadline = UINT32_MAX;
  }
  if (keyboard_leds_changed) {
    tuh_hid_set_report(keyboard_dev_addr, keyboard_idx, 0 /*report_id*/, HID_REPORT_TYPE_OUTPUT, &keyboard_leds, sizeof(keyboard_leds));
  }
}

bool timer_callback(repeating_timer_t *rtimer) {  // USB Host is executed by timer interrupt.
  usb_host_task();
  return true;
}

#endif


/*
#define SPI_CLOCK (20'000'000)
#define SD_CS_PIN (17)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
DedicatedSpiCard blockdevice;
*/

//FatFileSystem SD;
// =========================================================================================
// Define SdFat as alias for SD
// =========================================================================================
SdFat SD;

// =========================================================================================
// Define Board-Data
// GP25 green onboard LED
// =========================================================================================
#define LED 25  // GPIO25
#define LEDinv 0
#define board_pico
#define board_analog_io
#define board_digital_io
#define BOARD "Raspberry Pi Pico"

// =========================================================================================
// SPIINIT !!ONLY!! for ESP32-boards
// #define SPIINIT Clock, MISO, MOSI, Card-Select
// =========================================================================================
// #define SPIINIT 18,16,19,SS
#define SPIINIT 2, 4, 3, 5
// #define SPIINIT_TXT "18,16,19,17"
#define SPIINIT_TXT "2,4,3,5"

// =========================================================================================
// Pin Documentation
// =========================================================================================
// Normal RPi Pico
// MISO - Pin 21 - GPIO 16
// MOSI - Pin 25 - GPIO 19
// CS   - Pin 22 - GPIO 17
// SCK  - Pin 24 - GPIO 18

// MicroSD Pin Definition for RC2040 board
// Pin 6 - GPIO 4 MISO
// Pin 7 - GPIO 5 Chip/Card-Select (CS / SS)
// Pin 4 - GPIO 2 Clock (SCK)
// Pin 5 - GPIO 3 MOSI

// FUNCTIONS REQUIRED FOR USB MASS STORAGE ---------------------------------

#if USE_MSC
Adafruit_USBD_MSC usb_msc;       // USB mass storage object
static bool msc_changed = true;  // Is set true on filesystem changes

// Callback on READ10 command.
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  return blockdevice.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 command.
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  digitalWrite(LED_BUILTIN, HIGH);
  return blockdevice.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 completion.
void msc_flush_cb(void) {
  blockdevice.syncBlocks();  // Sync with blockdevice
  SD.cacheClear();           // Clear filesystem cache to force refresh
  digitalWrite(LED_BUILTIN, LOW);
  msc_changed = true;
}
#endif



// DVI Display
#if USE_DISPLAY
// DVItext1 display(DVI_RES_640x240p60, pimoroni_demo_hdmi_cfg);
DVItext1 display(DVI_RES_640x240p60, pico_sock_cfg);
// DVItext1 display(DVI_RES_800x240p30, pico_sock_cfg);

#define TS_NORMAL 0
#define TS_WAITBRACKET 1
#define TS_STARTCHAR 2
#define TS_READPARAM 3
#define TS_HASH 4
#define TS_READCHAR 5

#define CS_TEXT_US 0
#define CS_TEXT_UK 1
#define CS_GRAPHICS 2

static uint8_t terminal_state = TS_NORMAL;
static uint8_t color_fg, color_bg, attr = 0, cur_attr = 0;
static int cursor_col = 0, cursor_row = 0, saved_col = 0, saved_row = 0;
static int scroll_region_start, scroll_region_end;
static bool cursor_shown = true, origin_mode = false, cursor_eol = false, auto_wrap_mode = true, vt52_mode = false, localecho = false;
static bool saved_eol = false, saved_origin_mode = false, insert_mode = false;
static uint8_t saved_attr, saved_fg, saved_bg, saved_charset_G0, saved_charset_G1, *charset, charset_G0, charset_G1, tabs[255];

uint16_t underCursor = ' ';

void scroll_region(uint8_t start, uint8_t end, int8_t n) {

  if (n>0) {
    // scroll up
    if (n>end-start)
      n=end-start+1;
    else
      memmove(display.getBuffer() + start*display.width(), display.getBuffer() + (start+n)*display.width(), display.width() * (end-start-n+1)*2);
    for(int i=0;i<n;i++) {
      display.drawFastHLine(0, end-i , display.width(), ' '); // Clear bottom line
    }
  } else if (n<0) {
    // scroll down
    n=-n;
    if (n>end-start)
      n=end-start+1;
    else
      memmove(display.getBuffer() + (start+n)*display.width(), display.getBuffer() + start*display.width(), display.width() * (end-start-n+1)*2);
    for(int i=0;i<n;i++) {
      display.drawFastHLine(0, start+i , display.width(), ' '); // Clear top line
    }
  }
};

void fb_insert(uint8_t x, uint8_t y, uint8_t n) {
  if( y < display.height() && x < display.width()) {
    if (x+n < display.width()) { 
      memmove(display.getBuffer() + y*display.width()+x+n, display.getBuffer() + y*display.width() + x, n*2);
    }
    for (int i=0;i<n;i++) {
      display.getBuffer()[y*display.width()+x+n]=0xdb;
    }
  }
};

void fb_delete(uint8_t x, uint8_t y, uint8_t n) {
  if( y < display.height() && x < display.width()) {
    if (x+n < display.width()) { 
      memmove(display.getBuffer() + y*display.width()+x, display.getBuffer() + y*display.width() + x+n, n*2);
    }
    for (int i=0;i<n;i++) {
      if(x+n-i<display.width()) {
        display.getBuffer()[y*display.width()+x+n-i]=0xdb;
      }
    }
  }
};



static uint8_t get_charset(char c) {
  switch (c) {
    case 'A': return CS_TEXT_UK;
    case 'B': return CS_TEXT_US;
    case '0': return CS_GRAPHICS;
    case '1': return CS_TEXT_US;
    case '2': return CS_GRAPHICS;
  }

  return CS_TEXT_US;
}   

static void show_cursor(bool show) {
  if (show) {
    // show cursor
    underCursor = display.getBuffer()[cursor_row*display.width()+cursor_col];
    display.getBuffer()[cursor_row*display.width()+cursor_col] = 0xff00 | underCursor;
  } else {
    // show char below
    display.getBuffer()[cursor_row*display.width()+cursor_col] = underCursor & 0xff;
  }
}

static void move_cursor_wrap(int row, int col) {
  if (row != cursor_row || col != cursor_col) {
    int top_limit = scroll_region_start;
    int bottom_limit = scroll_region_end;

    if (cursor_shown && cursor_row >= 0 && cursor_col >= 0) show_cursor(false);

    while (col < 0) {
      col += display.width();
      row--;
    }
    while (row < top_limit) {
      row++;
      // framebuf_scroll_region(top_limit, bottom_limit, -1, color_fg, color_bg);
      scroll_region(top_limit, bottom_limit, -1);
    }
    while (col >= display.width()) {
      col -= display.width();
      row++;
    }
    while (row > bottom_limit) {
      row--;
      // framebuf_scroll_region(top_limit, bottom_limit, 1, color_fg, color_bg);
      scroll_region(top_limit, bottom_limit, 1);
    }

    cursor_row = row;
    cursor_col = col;
    cursor_eol = false;

    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);

    if (cursor_shown) show_cursor(true);
  }
};

static void move_cursor_within_region(int row, int col, int top_limit, int bottom_limit) {
  if (row != cursor_row || col != cursor_col) {
    if (cursor_shown && cursor_row >= 0 && cursor_col >= 0) show_cursor(false);

    if (col < 0)
      col = 0;
    else if (col >= display.width())
      col = display.width() - 1;

    if (row < top_limit)
      row = top_limit;
    else if (row > bottom_limit)
      row = bottom_limit;

    cursor_row = row;
    cursor_col = col;
    cursor_eol = false;

    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    if (cursor_shown) show_cursor(true);
  }
}

static void move_cursor_limited(int row, int col) {
  // only move if cursor is currently within scroll region, do not move
  // outside of scroll region
  if (cursor_row >= scroll_region_start && cursor_row <= scroll_region_end)
    move_cursor_within_region(row, col, scroll_region_start, scroll_region_end);
}

static void init_cursor(int row, int col) {
  cursor_row = -1;
  cursor_col = -1;
  move_cursor_within_region(row, col, 0, display.height() - 1);
}

static void print_char_vt(char c) {

  if (cursor_eol) {
    // cursor was already past the end of the line => move it to the next line now
    move_cursor_wrap(cursor_row + 1, 0);
    cursor_eol = false;
  }

  if (insert_mode) {
    show_cursor(false);
    // framebuf_insert(cursor_col, cursor_row, 1, color_fg, color_bg);
    fb_insert(cursor_col, cursor_row, 1);
  }

  // if (*charset == CS_TEXT_UK && c == 35)
  //   c = font_map_graphics_char(125, (attr & ATTR_BOLD) != 0);  // pound sterling symbol
  // else if (*charset == CS_GRAPHICS)
  //   c = font_map_graphics_char(c, (attr & ATTR_BOLD) != 0);

  // framebuf_set_color(cursor_col, cursor_row, color_fg, color_bg);
  // framebuf_set_attr(cursor_col, cursor_row, attr);
  // framebuf_set_char(cursor_col, cursor_row, c);
  // display.drawPixel(cursor_col, cursor_row, c);
  display.getBuffer()[cursor_row*display.width()+cursor_col] = c;
  if (auto_wrap_mode && cursor_col == display.width() - 1) {
    // cursor stays in last column but will wrap if another character is typed
    // cur_attr = attr;
    show_cursor(cursor_shown);
    cursor_eol = true;
  } else
    init_cursor(cursor_row, cursor_col + 1);
}

void terminal_reset() {
  saved_col = 0;
  saved_row = 0;
  cursor_shown = true;
  // color_fg = config_get_terminal_default_fg();
  // color_bg = config_get_terminal_default_bg();
  scroll_region_start = 0;
  scroll_region_end = display.height() - 1;
  origin_mode = false;
  cursor_eol = false;
  auto_wrap_mode = true;
  insert_mode = false;
  vt52_mode = false;
  // attr = config_get_terminal_default_attr();
  saved_attr = 0;
  charset_G0 = CS_TEXT_US;
  charset_G1 = CS_GRAPHICS;
  saved_charset_G0 = CS_TEXT_US;
  saved_charset_G1 = CS_GRAPHICS;
  charset = &charset_G0;
  memset(tabs, 0, display.width());
}


void terminal_clear_screen() {
  // framebuf_fill_screen(' ', color_fg, color_bg);
  display.fillRect(0, 0, display.width(), display.height(), ' ');
  init_cursor(0, 0);
  scroll_region_start = 0;
  scroll_region_end = display.height() - 1;
  origin_mode = false;
}

static void send_char(char c) {
  // serial_send_char(c);
  // if( localecho ) terminal_receive_char(c);
}

static void send_string(const char *s) {
  // serial_send_string(s);
  // if( localecho ) terminal_receive_string(s);
}

static void terminal_process_text(char c) {
  switch (c) {
    case 5:  // ENQ => send answer-back string
      // send_string(config_get_terminal_answerback());
      break;

    case 7:  // BEL => produce beep
      //sound_play_tone(config_get_audible_bell_frequency(),
      //                config_get_audible_bell_duration(),
      //                config_get_audible_bell_volume(),
      //                false);
      //framebuf_flash_screen(config_get_visual_bell_color(), config_get_visual_bell_duration());
      break;

    case 8:    // backspace
    case 127:  // delete
      {
        //uint8_t mode = c==8 ? config_get_terminal_bs() : config_get_terminal_del();
        //if( mode>0 )
        //  {
        int top_limit = origin_mode ? scroll_region_start : 0;
        if (cursor_row > top_limit)
          move_cursor_wrap(cursor_row, cursor_col - 1);
        else
          move_cursor_limited(cursor_row, cursor_col - 1);

        // framebuf_set_char(cursor_col, cursor_row, ' ');
        // display.drawPixel(cursor_col, cursor_row, ' ');
        display.getBuffer()[cursor_row*display.width()+cursor_col] = ' '; 
        // framebuf_set_attr(cursor_col, cursor_row, 0);
        // cur_attr = 0;
        show_cursor(cursor_shown);

        break;
      }

    case '\t':  // horizontal tab
      {
        int col = cursor_col + 1;
        while (col < display.width() - 1 && !tabs[col]) col++;
        move_cursor_limited(cursor_row, col);
        break;
      }

    case '\n':  // newline
    case 11:    // vertical tab (interpreted as newline)
    case 12:    // form feed (interpreted as newline)
      move_cursor_wrap(cursor_row + 1, cursor_col);
      break;
    case '\r':  // carriage return
      move_cursor_wrap(cursor_row, 0);
      break;

    case 14:  // SO
      // charset = &charset_G1;
      break;

    case 15:  // SI
      // charset = &charset_G0;
      break;

    default:  // regular character
      if (c >= 32) print_char_vt(c);
      break;
  }
}

static void terminal_process_command(char start_char, char final_char, uint8_t num_params, uint8_t *params) {
  // NOTE: num_params>=1 always holds, if no parameters were received then params[0]=0
  if (final_char == 'l' || final_char == 'h') {
    bool enabled = final_char == 'h';
    if (start_char == '?') {
      switch (params[0]) {
        case 2:
          if (!enabled) {
            terminal_reset();
            vt52_mode = true;
          }
          break;

        case 3:  // switch 80/132 columm mode - 132 columns not supported but we can clear the screen
          terminal_clear_screen();
          break;

        case 4:  // enable smooth scrolling (emulated via scroll delay)
          // framebuf_set_scroll_delay(enabled ? config_get_terminal_scrolldelay() : 0);
          break;

        case 5:  // invert screen
          // framebuf_set_screen_inverted(enabled);
          break;

        case 6:  // origin mode
          origin_mode = enabled;
          move_cursor_limited(scroll_region_start, 0);
          break;

        case 7:  // auto-wrap mode
          auto_wrap_mode = enabled;
          break;

        case 12:  // local echo (send-receive mode)
          localecho = !enabled;
          break;

        case 25:  // show/hide cursor
          cursor_shown = enabled;
          show_cursor(cursor_shown);
          break;
      }
    } else if (start_char == 0) {
      switch (params[0]) {
        case 4:  // insert mode
          insert_mode = enabled;
          break;
      }
    }
  } else if (final_char == 'J') {
    switch (params[0]) {
      case 0:
        // for (int i = cursor_row; i < display.height(); i++) framebuf_set_row_attr(i, 0);
        display.fillRect(cursor_col, cursor_row, display.width() - cursor_col, display.height() - cursor_row, ' ');
        break;

      case 1:
        // for (int i = 0; i < cursor_row; i++) framebuf_set_row_attr(i, 0);
        display.fillRect(0, 0, cursor_col, cursor_row, ' ');
        break;

      case 2:
        // for (int i = 0; i < framebuf_get_nrows(); i++) framebuf_set_row_attr(i, 0);
        display.fillRect(0, 0, display.width(), display.height(), ' ');
        break;
    }

    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    show_cursor(cursor_shown);
  } else if (final_char == 'K') {
    switch (params[0]) {
      case 0:
        display.fillRect(cursor_col, cursor_row, display.width(), display.height(), ' ');
        break;

      case 1:
        display.fillRect(0, cursor_row, cursor_col, 1, ' ');
        break;

      case 2:
        display.fillRect(0, cursor_row, display.width(), 1, ' ');
        break;
    }

    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    show_cursor(cursor_shown);
  } else if (final_char == 'A') {
    move_cursor_limited(cursor_row - MAX(1, params[0]), cursor_col);
  } else if (final_char == 'B') {
    move_cursor_limited(cursor_row + MAX(1, params[0]), cursor_col);
  } else if (final_char == 'C' || final_char == 'a') {
    move_cursor_limited(cursor_row, cursor_col + MAX(1, params[0]));
  } else if (final_char == 'D' || final_char == 'j') {
    move_cursor_limited(cursor_row, cursor_col - MAX(1, params[0]));
  } else if (final_char == 'E' || final_char == 'e') {
    move_cursor_limited(cursor_row + MAX(1, params[0]), 0);
  } else if (final_char == 'F' || final_char == 'k') {
    move_cursor_limited(cursor_row - MAX(1, params[0]), 0);
  } else if (final_char == 'd') {
    move_cursor_limited(MAX(1, params[0]), cursor_col);
  } else if (final_char == 'G' || final_char == '`') {
    move_cursor_limited(cursor_row, MAX(1, params[0]) - 1);
  } else if (final_char == 'H' || final_char == 'f') {
    int top_limit = origin_mode ? scroll_region_start : 0;
    int bottom_limit = origin_mode ? scroll_region_end : display.height() - 1;
    move_cursor_within_region(top_limit + MAX(params[0], 1) - 1, num_params < 2 ? 0 : MAX(params[1], 1) - 1, top_limit, bottom_limit);
  } else if (final_char == 'I') {
    int n = MAX(1, params[0]);
    int col = cursor_col + 1;
    while (n > 0 && col < display.width() - 1) {
      while (col < display.width() - 1 && !tabs[col]) col++;
      n--;
    }
    move_cursor_limited(cursor_row, col);
  } else if (final_char == 'Z') {
    int n = MAX(1, params[0]);
    int col = cursor_col - 1;
    while (n > 0 && col > 0) {
      while (col > 0 && !tabs[col]) col--;
      n--;
    }
    move_cursor_limited(cursor_row, col);
  } else if (final_char == 'L' || final_char == 'M') {
    int n = MAX(1, params[0]);
    int bottom_limit = origin_mode ? scroll_region_end : display.height() - 1;
    show_cursor(false);
    // framebuf_scroll_region(cursor_row, bottom_limit, final_char == 'M' ? n : -n, color_fg, color_bg);
    scroll_region(cursor_row, bottom_limit, final_char == 'M' ? n : -n);
    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    show_cursor(cursor_shown);
  } else if (final_char == '@') {
    int n = MAX(1, params[0]);
    show_cursor(false);
    // framebuf_insert(cursor_col, cursor_row, n, color_fg, color_bg);
    fb_insert(cursor_col, cursor_row, n);
    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    show_cursor(cursor_shown);
  } else if (final_char == 'P') {
    int n = MAX(1, params[0]);
    //framebuf_delete(cursor_col, cursor_row, n, color_fg, color_bg);
    fb_delete(cursor_col, cursor_row, n);
    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    show_cursor(cursor_shown);
  } else if (final_char == 'S' || final_char == 'T') {
    int top_limit = origin_mode ? scroll_region_start : 0;
    int bottom_limit = origin_mode ? scroll_region_end : display.height() - 1;
    int n = MAX(1, params[0]);
    show_cursor(false);
    while (n--) scroll_region(top_limit, bottom_limit, final_char == 'S' ? n : -n);
    // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
    show_cursor(cursor_shown);
  } else if (final_char == 'g') {
    int p = params[0];
    if (p == 0)
      tabs[cursor_col] = false;
    else if (p == 3)
      memset(tabs, 0, display.width());
  } else if (final_char == 'm') {
    /* unsigned int i;
    for (i = 0; i < num_params; i++) {
      int p = params[i];

      if (p == 0) {
        color_fg = config_get_terminal_default_fg();
        color_bg = config_get_terminal_default_bg();
        attr = config_get_terminal_default_attr();
        //cursor_shown = true;
        show_cursor(cursor_shown);
      } else if (p == 1)
        attr |= ATTR_BOLD;
      else if (p == 4)
        attr |= ATTR_UNDERLINE;
      else if (p == 5)
        attr |= ATTR_BLINK;
      else if (p == 7)
        attr |= ATTR_INVERSE;
      else if (p == 22)
        attr &= ~ATTR_BOLD;
      else if (p == 24)
        attr &= ~ATTR_UNDERLINE;
      else if (p == 25)
        attr &= ~ATTR_BLINK;
      else if (p == 27)
        attr &= ~ATTR_INVERSE;
      else if (p >= 30 && p <= 37)
        color_fg = p - 30;
      else if (p == 38 && num_params >= i + 2 && params[i + 1] == 5) {
        color_fg = params[i + 2] & 15;
        i += 2;
      } else if (p == 39)
        color_fg = config_get_terminal_default_fg();
      else if (p >= 40 && p <= 47)
        color_bg = p - 40;
      else if (p == 48 && num_params >= i + 2 && params[i + 1] == 5) {
        color_bg = params[i + 2] & 15;
        i += 2;
      } else if (p == 49)
        color_bg = config_get_terminal_default_bg();

      show_cursor(cursor_shown); */
  } else if (final_char == 'r') {
    if (num_params == 2 && params[1] > params[0]) {
      scroll_region_start = MAX(params[0], 1) - 1;
      scroll_region_end = MIN(params[1], display.height()) - 1;
    } else if (params[0] == 0) {
      scroll_region_start = 0;
      scroll_region_end = display.height() - 1;
    }

    move_cursor_within_region(scroll_region_start, 0, scroll_region_start, scroll_region_end);
  } else if (final_char == 's') {
    saved_row = cursor_row;
    saved_col = cursor_col;
    saved_eol = cursor_eol;
    saved_origin_mode = origin_mode;
    saved_fg = color_fg;
    saved_bg = color_bg;
    saved_attr = attr;
    saved_charset_G0 = charset_G0;
    saved_charset_G1 = charset_G1;
  } else if (final_char == 'u') {
    move_cursor_limited(saved_row, saved_col);
    origin_mode = saved_origin_mode;
    cursor_eol = saved_eol;
    color_fg = saved_fg;
    color_bg = saved_bg;
    attr = saved_attr;
    charset_G0 = saved_charset_G0;
    charset_G1 = saved_charset_G1;
  } else if (final_char == 'c') {
  // device attributes resport
  // https://www.vt100.net/docs/vt100-ug/chapter3.html#DA
  // https://invisible-island.net/xterm/ctlseqs/ctlseqs.html#h4-Functions-using-CSI-_-ordered-by-the-final-character-lparen-s-rparen:CSI-Ps-c.1CA3
  // "ESC [?1;0c" => base VT100, no options
  // "ESC [?6c"   => VT102
  // send_string("\033[?6c");
  } else if (final_char == 'n') {
  /* if (params[0] == 5) {
      // terminal status report
      send_string("\033[0n");
    } else if (params[0] == 6) {
      // cursor position report
      int top_limit = origin_mode ? scroll_region_start : 0;
      char buf[20];
      snprintf(buf, 20, "\033[%u;%uR", cursor_row - top_limit + 1, cursor_col + 1);
      send_string(buf); */
  }
}


void terminal_receive_char_vt102(char c) {
  static char start_char = 0;
  static uint8_t num_params = 0;
  static uint8_t params[16];

  if (terminal_state != TS_NORMAL) {
    if (c == 8 || c == 10 || c == 13) {
      // processe some cursor control characters within escape sequences
      // (otherwise we fail "vttest" cursor control tests)
      terminal_process_text(c);
      return;
    } else if (c == 11) {
      // ignore VT character plus the following character
      // (otherwise we fail "vttest" cursor control tests)
      terminal_state = TS_READCHAR;
      return;
    }
  }

  switch (terminal_state) {
    case TS_NORMAL:
      {
        if (c == 27)
          terminal_state = TS_WAITBRACKET;
        else
          terminal_process_text(c);

        break;
      }

    case TS_WAITBRACKET:
      {
        terminal_state = TS_NORMAL;

        switch (c) {
          case '[':
            start_char = 0;
            num_params = 1;
            params[0] = 0;
            terminal_state = TS_STARTCHAR;
            break;

          case '#':
            terminal_state = TS_HASH;
            break;

          case 27: print_char_vt(c); break;                               // escaped ESC
          case 'c': terminal_reset(); break;                              // reset
          case '7': terminal_process_command(0, 's', 0, NULL); break;     // save cursor position
          case '8': terminal_process_command(0, 'u', 0, NULL); break;     // restore cursor position
          case 'H': tabs[cursor_col] = true; break;                       // set tab
          case 'J': terminal_process_command(0, 'J', 0, NULL); break;     // clear to end of screen
          case 'K': terminal_process_command(0, 'K', 0, NULL); break;     // clear to end of row
          case 'D': move_cursor_wrap(cursor_row + 1, cursor_col); break;  // cursor down
          case 'E': move_cursor_wrap(cursor_row + 1, 0); break;           // cursor down and to first column
          case 'I': move_cursor_wrap(cursor_row - 1, 0); break;           // cursor up and to furst column
          case 'M': move_cursor_wrap(cursor_row - 1, cursor_col); break;  // cursor up
          case '(':
          case ')':
          case '+':
          case '*':
            start_char = c;
            terminal_state = TS_READCHAR;
            break;
        }

        break;
      }

    case TS_STARTCHAR:
    case TS_READPARAM:
      {
        if (c >= '0' && c <= '9') {
          params[num_params - 1] = params[num_params - 1] * 10 + (c - '0');
          terminal_state = TS_READPARAM;
        } else if (c == ';') {
          // next parameter (max 16 parameters)
          num_params++;
          if (num_params > 16)
            terminal_state = TS_NORMAL;
          else {
            params[num_params - 1] = 0;
            terminal_state = TS_READPARAM;
          }
        } else if (terminal_state == TS_STARTCHAR && (c == '?' || c == '#')) {
          start_char = c;
          terminal_state = TS_READPARAM;
        } else {
          // not a parameter value or startchar => command is done
          terminal_process_command(start_char, c, num_params, params);
          terminal_state = TS_NORMAL;
        }

        break;
      }

    case TS_HASH:
      {
        switch (c) {
          case '3':
            {
              // framebuf_set_row_attr(cursor_row, ROW_ATTR_DBL_WIDTH | ROW_ATTR_DBL_HEIGHT_TOP);
              break;
            }

          case '4':
            {
              // framebuf_set_row_attr(cursor_row, ROW_ATTR_DBL_WIDTH | ROW_ATTR_DBL_HEIGHT_BOT);
              break;
            }

          case '5':
            {
              // framebuf_set_row_attr(cursor_row, 0);
              break;
            }

          case '6':
            {
              // framebuf_set_row_attr(cursor_row, ROW_ATTR_DBL_WIDTH);
              break;
            }

          case '8':
            {
              // fill screen with 'E' characters (DEC test feature)
              int top_limit = origin_mode ? scroll_region_start : 0;
              int bottom_limit = origin_mode ? scroll_region_end : display.height();
              show_cursor(false);
              display.fillRect(0, top_limit, display.width(), bottom_limit - top_limit, 'E');
              // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
              show_cursor(cursor_shown);
              break;
            }
        }

        terminal_state = TS_NORMAL;
        break;
      }

    case TS_READCHAR:
      {
        if (start_char == '(')
          charset_G0 = get_charset(c);
        else if (start_char == ')')
          charset_G1 = get_charset(c);

        terminal_state = TS_NORMAL;
        break;
      }
  }
}

void terminal_receive_char_vt52(char c) {
  static char start_char, row;

  switch (terminal_state) {
    case TS_NORMAL:
      {
        if (c == 27)
          terminal_state = TS_STARTCHAR;
        else
          terminal_process_text(c);

        break;
      }

    case TS_STARTCHAR:
      {
        terminal_state = TS_NORMAL;

        switch (c) {
          case 'A':
            move_cursor_limited(cursor_row - 1, cursor_col);
            break;

          case 'B':
            move_cursor_limited(cursor_row + 1, cursor_col);
            break;

          case 'C':
            move_cursor_limited(cursor_row, cursor_col + 1);
            break;

          case 'D':
            move_cursor_limited(cursor_row, cursor_col - 1);
            break;

          case 'E':
            // framebuf_fill_screen(' ', color_fg, color_bg);
            display.fillRect(0,0,display.width(),display.height(),' ');
            // fall through

          case 'H':
            move_cursor_limited(0, 0);
            break;

          case 'I':
            move_cursor_wrap(cursor_row - 1, cursor_col);
            break;

          case 'J':
            show_cursor(false);
            display.fillRect(cursor_col, cursor_row, display.width()-cursor_col, display.height()-cursor_row, ' ');
            // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
            show_cursor(cursor_shown);
            break;

          case 'K':
            show_cursor(false);
            display.fillRect(cursor_col, cursor_row, display.width()-cursor_col, 1, ' ');
            // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
            show_cursor(cursor_shown);
            break;

          case 'L':
          case 'M':
            show_cursor(false);
            // framebuf_scroll_region(cursor_row, framebuf_get_nrows() - 1, c == 'M' ? 1 : -1, color_fg, color_bg);
            scroll_region(cursor_row, display.height()-1, c == 'M' ? 1 : -1);
            // cur_attr = framebuf_get_attr(cursor_col, cursor_row);
            show_cursor(cursor_shown);
            break;

          case 'Y':
            start_char = c;
            row = 0;
            terminal_state = TS_READPARAM;
            break;

          case 'Z':
            // send_string("\033/K");
            break;

          case 'b':
          case 'c':
            start_char = c;
            terminal_state = TS_READPARAM;
            break;

          case 'd':
            display.fillRect(0, 0, cursor_col+1, cursor_row+1, ' ');
            init_cursor(cursor_col, cursor_row);
            break;

          case 'e':
            show_cursor(true);
            break;

          case 'f':
            show_cursor(false);
            break;

          case 'j':
            saved_col = cursor_col;
            saved_row = cursor_row;
            break;

          case 'k':
            move_cursor_limited(saved_row, saved_col);
            break;

          case 'l':
            display.fillRect(0, cursor_row, display.width(), 1, ' ');
            init_cursor(0, cursor_row);
            break;

          case 'o':
            display.fillRect(0, cursor_row, cursor_col+1, 1, ' ');
            show_cursor(cursor_shown);
            break;

          case 'p':
            // framebuf_set_screen_inverted(true);
            break;

          case 'q':
            // framebuf_set_screen_inverted(false);
            break;

          case 'v':
            auto_wrap_mode = true;
            break;

          case 'w':
            auto_wrap_mode = false;
            break;

          case '<':
            terminal_reset();
            vt52_mode = false;
            break;
        }

        break;
      }

    case TS_READPARAM:
      {
        if (start_char == 'Y') {
          if (row == 0)
            row = c;
          else {
            if (row >= 32 && c >= 32) move_cursor_limited(row - 32, c - 32);
            terminal_state = TS_NORMAL;
          }
        } else if (start_char == 'b' && c >= 32) {
          // color_fg = (c - 32) & 15;
          show_cursor(cursor_shown);
        } else if (start_char == 'c' && c >= 32) {
          // color_bg = (c - 32) & 15;
          show_cursor(cursor_shown);
        }

        break;
      }
  }
}

void putch_display(uint8_t c)
{
  // if( config_get_terminal_clearBit7() ) c &= 0x7f;

  if( !vt52_mode )
    terminal_receive_char_vt102(c);
  else
    terminal_receive_char_vt52(c);
}
#endif



bool port_init_early() {
#if USE_DISPLAY
  //vreg_set_voltage(VREG_VOLTAGE_1_20);
  //delay(10);
  if (!display.begin()) {
    return false;
  }

  terminal_reset();
  terminal_clear_screen();
  _putch_hook = putch_display;
#endif

#if USE_KEYBOARD
  USBHost.begin(0);
  // USB Host is executed by timer interrupt.
  add_repeating_timer_us(KBD_INT_TIME /*us*/, timer_callback, NULL, &rtimer);

  _getch_hook = getch_usbh;
  _kbhit_hook = kbhit_usbh;
#endif


  // USB mass storage / filesystem setup (do BEFORE Serial init)
  /*
  if (!blockdevice.begin(SD_CONFIG)) { _puts("Failed to initialize SD card"); return false; }
#if USE_MSC
  // Set disk vendor id, product id and revision
  usb_msc.setID("Adafruit", "Internal Flash", "1.0");
  // Set disk size, block size is 512 regardless of blockdevice page size
  usb_msc.setCapacity(blockdevice.sectorCount(), 512);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setUnitReady(true); // MSC is ready for read/write
  if (!usb_msc.begin()) {
      _puts("Failed to initialize USB MSC"); return false;
  }
#endif
*/
  return true;
}

/*
bool port_flash_begin() {
  if (!SD.begin(&blockdevice, true, 1)) { // Start filesystem on the blockdevice
      _puts("!SD.begin()"); return false;
  }
  return true;
}
*/


#if defined(__cplusplus)
extern "C" {
#endif


#if USE_KEYBOARD

  hid_keyboard_report_t old_report;

  bool report_contains(const hid_keyboard_report_t &report, uint8_t key) {
    for (int i = 0; i < 6; i++) {
      if (report.keycode[i] == key) return true;
    }
    return false;
  }

  void process_boot_kbd_report(uint8_t dev_addr, uint8_t idx, const hid_keyboard_report_t &report) {

    bool alt = report.modifier & 0x44;
    bool shift = report.modifier & 0x22;
    bool ctrl = report.modifier & 0x11;

    bool num = old_report.reserved & 1;
    bool caps = old_report.reserved & 2;

    uint8_t code = 0;

    if (report.keycode[0] == 1 && report.keycode[1] == 1) {
      // keyboard says it has exceeded max kro
      return;
    }

    // something was pressed or release, so cancel any key repeat
    old_ascii = -1;

    for (auto keycode : report.keycode) {
      if (keycode == 0) continue;
      if (report_contains(old_report, keycode)) continue;

      /* key is newly pressed */
      if (keycode == HID_KEY_NUM_LOCK) {
        num = !num;
#ifdef USE_JP_KEYBOARD
      } else if ((keycode == HID_KEY_CAPS_LOCK) && shift) {
#else
      } else if (keycode == HID_KEY_CAPS_LOCK) {
#endif
        caps = !caps;
      } else {
        for (const auto &mapper : keycode_to_ascii) {
          if (!(keycode >= mapper.first && keycode <= mapper.last))
            continue;
          if (mapper.flags & FLAG_SHIFT && !shift)
            continue;
          if (mapper.flags & FLAG_NUMLOCK && !num)
            continue;
          if (mapper.flags & FLAG_CTRL && !ctrl)
            continue;
          if (mapper.flags & FLAG_LUT) {
            code = lut[mapper.code][keycode - mapper.first];
          } else {
            code = keycode - mapper.first + mapper.code;
          }
          if (mapper.flags & FLAG_ALPHABETIC) {
            if (shift ^ caps) {
              code ^= ('a' ^ 'A');
            }
          }
          if (ctrl) code &= 0x1f;
          if (alt) code ^= 0x80;
          send_ascii(code, initial_repeat_time);  // send code
          break;
        }
      }
    }

    //uint8_t leds = (caps | (num << 1));
    keyboard_leds = (num | (caps << 1));
    if (keyboard_leds != old_report.reserved) {
      keyboard_leds_changed = true;
      // no worky
      //auto r = tuh_hid_set_report(dev_addr, idx/*idx*/, 0/*report_id*/, HID_REPORT_TYPE_OUTPUT/*report_type*/, &leds, sizeof(leds));
      //tuh_hid_set_report(dev_addr, idx/*idx*/, 0/*report_id*/, HID_REPORT_TYPE_OUTPUT/*report_type*/, &leds, sizeof(leds));
    } else {
      keyboard_leds_changed = false;
    }
    old_report = report;
    old_report.reserved = keyboard_leds;
  }

  /*
//--------------------------------------------------------------------+
// Generic Report
//--------------------------------------------------------------------+
void process_generic_report(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len)
{
  uint8_t const rpt_count = hid_info[idx].report_count;
  tuh_hid_report_info_t* rpt_info_arr = hid_info[idx].report_info;
  tuh_hid_report_info_t* rpt_info = NULL;

  if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0)
  {
    // Simple report without report ID as 1st byte
    rpt_info = &rpt_info_arr[0];
  }else
  {
    // Composite report, 1st byte is report ID, data starts from 2nd byte
    uint8_t const rpt_id = report[0];

    // Find report id in the array
    for(uint8_t i=0; i<rpt_count; i++)
    {
      if (rpt_id == rpt_info_arr[i].report_id )
      {
        rpt_info = &rpt_info_arr[i];
        break;
      }
    }

    report++;
    len--;
  }

  // For complete list of Usage Page & Usage checkout src/class/hid/hid.h. For examples:
  // - Keyboard                     : Desktop, Keyboard
  // - Mouse                        : Desktop, Mouse
  // - Gamepad                      : Desktop, Gamepad
  // - Consumer Control (Media Key) : Consumer, Consumer Control
  // - System Control (Power key)   : Desktop, System Control
  // - Generic (vendor)             : 0xFFxx, xx

  if ( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP )
  {
    switch (rpt_info->usage)
    {
      case HID_USAGE_DESKTOP_KEYBOARD:
        {
          // Assume keyboard follow boot report layout
          process_boot_kbd_report(dev_addr, idx, *(hid_keyboard_report_t const*) report );
        }
        break;

      case HID_USAGE_DESKTOP_MOUSE:
        {
          // Assume mouse follow boot report layout
        }
        break;

      default:
        break;
    }
  }
  else //Other than HID_USAGE_PAGE_DESKTOP
  {
  }
}
*/

  //--------------------------------------------------------------------+
  // TinyUSB Host callbacks
  //--------------------------------------------------------------------+

  // Invoked when device is mounted (configured)
  void tuh_mount_cb(uint8_t dev_addr) {
  }

  // Invoked when device is unmounted (bus reset/unplugged)
  void tuh_umount_cb(uint8_t dev_addr) {
  }

  // Invoked when device with hid interface is mounted
  // Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
  // can be used to parse common/simple enough descriptor.
  // Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
  // therefore report_desc = NULL, desc_len = 0
  void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const *desc_report, uint16_t desc_len) {
    // By default host stack will use activate boot protocol on supported interface.
    // Therefore for this simple example, we only need to parse generic report descriptor (with built-in parser)
    hid_info[idx].report_count = tuh_hid_parse_report_descriptor(hid_info[idx].report_info, MAX_REPORT, desc_report, desc_len);
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, idx);

    switch (itf_protocol) {
      case HID_ITF_PROTOCOL_NONE:  //HID_PROTOCOL_BOOT:NONE
        break;

      case HID_ITF_PROTOCOL_KEYBOARD:  //HID_PROTOCOL_BOOT:KEYBOARD
        if (keyboard_mounted != true) {
          keyboard_dev_addr = dev_addr;
          keyboard_idx = idx;
          keyboard_mounted = true;
        }
        break;

      case HID_ITF_PROTOCOL_MOUSE:  //HID_PROTOCOL_BOOT:MOUSE
        break;
    }

    if (!tuh_hid_receive_report(dev_addr, idx)) {
    }
  }

  // Invoked when device with hid interface is un-mounted
  void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
    if (dev_addr == keyboard_dev_addr && idx == keyboard_idx) {
      keyboard_mounted = false;
      keyboard_dev_addr = 0;
      keyboard_idx = 0;
      keyboard_leds = 0;
      old_report = { 0 };
    }
  }

  // Invoked when received report from device via interrupt endpoint
  void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const *report, uint16_t len) {
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, idx);

    switch (itf_protocol) {
      case HID_ITF_PROTOCOL_KEYBOARD:
        if (keyboard_mounted == true) {
          process_boot_kbd_report(dev_addr, idx, *(hid_keyboard_report_t const *)report);
        }
        break;

      case HID_ITF_PROTOCOL_MOUSE:
        break;

      default:
        //process_generic_report(dev_addr, idx, report, len);
        break;
    }
    // continue to request to receive report
    if (!tuh_hid_receive_report(dev_addr, idx)) {
    }
  }

#endif

#if defined(__cplusplus)
}
#endif
