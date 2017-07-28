/**
 * Apox Operating System
 * Copyright (c) 2005-2006, cryptocode
 */

#ifndef _APOX_SCNTBL_NO_H_
#define _APOX_SCNTBL_NO_H_

#include <kernel/unicode/Unicode.h>

/**
 * Base scancode => Unicode mappings (NO_no)
 *
 * Where there are no mappings, -1 is used since this translates to "no mapping"
 * in the unichar_t definition.
 *
 * The first 128 entries are unshifted.
 * The next 128 entries are shifted.
 * The next 128 entries are AltG shifted.
 */
static unichar_t kbdBaseUnicodeMapping[3][256] =
{
    //
    // Unshifted state
    //
    {
        -1 /* 0 is not a scan code => indicates error */,
        -1 /* esc */,
        '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '+', '\\',
        0x08 /* backspace */,
        -1 /* tab */,
        'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '�', '"', '\n',
        -1 /* lctrl*/,
        'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '�', '�', '\'',
        -1 /* lshift */,
        '\'', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '-',
        -1 /* rshift */,
        '*'/* keypad star */,
        -1 /* lalt */,
        ' ',
    },


    //
    // Shifted state
    //
    {
        -1 /* 0 is not a scan code => indicates error */,
        -1 /* esc */,
        '!', '"', '#', '�', '%', '&', '/', '(', ')', '=', '?', '`',
        0x08 /* backspace */,
        -1 /* tab */,
        'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '�', '^', '\n',
        -1 /* lctrl*/,
        'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', '�', '�', '\'',
        -1 /* lshift */,
        '*', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', ';', ':', '_',
        -1 /* rshift */,
        '*'/* keypad star */,
        -1 /* lalt */,
        ' ',

    },


    //
    // Alt Gr state
    //
    {
        -1 /* 0 is not a scan code => indicates error */,
        -1 /* esc */,
        ' ', '@', '�', '$', '�', '�', '{', '[', ']', '}', '�', '\'',
        0x08 /* backspace */,
        -1 /* tab */,
        ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '~', '\n',
        -1 /* lctrl*/,
        ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\'',
        -1 /* lshift */,
        '�', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
        -1 /* rshift */,
        '*'/* keypad star */,
        -1 /* lalt */,
        ' ',
    },
};

#endif // _APOX_SCNTBL_NO_H_
