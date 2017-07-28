/**
 * Apox Operating System
 * Copyright (c) 2005-2006, cryptocode
 *
 * \defgroup ConsoleSystem
 * @{
 */

#ifndef _APOX_DRIVER_KEYBOARD_H_
#define _APOX_DRIVER_KEYBOARD_H_

#include <kernel/Object.h>
#include <kernel/Component.h>
#include <kernel/libc/std.h>
#include <kernel/io/Ports.h>
#include <kernel/drivers/Device.h>
#include <kernel/threads/Thread.h>
#include <kernel/adt/Queue.h>
#include <kernel/unicode/Unicode.h>
#include <kernel/drivers/keyboard/ScantableBase.h>

/**
 * \name Keyboard LED states
 * @{
 */
#define LED_SCR       0x01
#define LED_NUM       0x02
#define LED_CAP       0x04
/**@}*/

#define KBD_BREAK_MASK          0x80

#define KBD_SCAN_INVALID        0x00
#define KBD_SCAN_ESC            0x01
#define KBD_SCAN_ENTER          0x1C
#define KBD_SCAN_BACKSPACE      0x0E
#define KBD_SCAN_TAB            0x0F
#define KBD_SCAN_CTRL           0x1D // Right when extended
#define KBD_SCAN_SHIFT_LEFT     0x2A
#define KBD_SCAN_SHIFT_RIGHT    0x36
#define KBD_SCAN_ALT            0x38 // Right when extended (AltGr)
#define KBD_SCAN_SPACEBAR       0x39
#define KBD_SCAN_CAPSLOCK       0x3A
#define KBD_SCAN_F1             0x3B
#define KBD_SCAN_F2             0x3C
#define KBD_SCAN_F3             0x3D
#define KBD_SCAN_F4             0x3E
#define KBD_SCAN_F5             0x3F
#define KBD_SCAN_F6             0x40
#define KBD_SCAN_F7             0x41
#define KBD_SCAN_F8             0x41
#define KBD_SCAN_F9             0x43
#define KBD_SCAN_F10            0x44
#define KBD_SCAN_F11            0x85
#define KBD_SCAN_F12            0x86
#define KBD_SCAN_NUMLOCK        0x45
#define KBD_SCAN_SCROLLOCK      0x46

#define KBD_SCAN_KEYPAD7_HOME   0x47
#define KBD_SCAN_KEYPAD8_UP     0x48
#define KBD_SCAN_KEYPAD9_PGUP   0x49
#define KBD_SCAN_KEYPAD_MINUS   0x4A
#define KBD_SCAN_KEYPAD4_LEFT   0x4B
#define KBD_SCAN_KEYPAD5        0x4C
#define KBD_SCAN_KEYPAD6_RIGHT  0x4D
#define KBD_SCAN_KEYPAD1_END    0x4F
#define KBD_SCAN_KEYPAD2_DOWN   0x50
#define KBD_SCAN_KEYPAD3_PGDN   0x51
#define KBD_SCAN_KEYPAD0_INS    0x52
#define KBD_SCAN_KEYPAD_DEL     0x53

#define KBD_SCAN_ESCAPED        0xE0
#define KBD_SCAN_ESCAPED_BREAK  0xE1

#define KBD_MAP_UNSHIFTED       0x00
#define KBD_MAP_SHIFTED         0x01
#define KBD_MAP_ALTGR           0x02

/**
 * \name Shifted key states
 *
 * It just so happends that KEYSTATE_SHIFT and KEYSTATE_ALTGR are indexes
 * into the kbdBaseUnicodeMapping matrix => do NOT change these values unless
 * you know what you're doing.
 * @{
 */
    const int KEYSTATE_SHIFT=1;
    const int KEYSTATE_ALT=2;

    const int KEYSTATE_CTRL=4;
    //KEYSTATE_ALT=8,
    const int KEYSTATE_CAPSLOCK=16;
    const int KEYSTATE_NUMLOCK=32;
    const int KEYSTATE_SCROLLOCK=64;
    const int KEYSTATE_GRAY=128;

    /** If set, the *right* shift, alt or control was pressed */
    const int KEYSTATE_RIGHT=256;

/** @}*/

/** Event type (whether key was pressed or released) */
enum KeyEventType
{
    KEY_DOWN=1,
    KEY_UP=2,
};

class Char
{
    public:

        /** Returns true if the character is printable */
        static bool isPrintable(unichar ch)
        {
            return (ch > 31);
        }

        /** Returns true if the character is a control character */
        static bool isISOControl(unichar ch)
        {
            return (ch < 32);
        }

        /** Returns true if the characters is a 7-bit ASCII character */
        static bool isAscii(unichar ch)
        {
            return (ch < 128);
        }

        /** Returns true if the char is carriage return */
        static bool isCR(unichar ch)
        {
            return (ch == '\r');
        }

        /** True if the char is line feed */
        static bool isLF(unichar ch)
        {
            return (ch == '\n');
        }

        /** True if the char is either line feed or carriage return */
        static bool isCRorLF(unichar ch)
        {
            return (isCR(ch) || isLF(ch));
        }

        /** True if the char is backspace */
        static bool isBackspace(unichar ch)
        {
            return (ch == 0x08);
        }
};

/**
 * A keyboard event
 */
struct KeyEvent
{
    inline char getChar()
    {
        return (char)character;
    }

    /** Unicode character mapping, or -1 if there is no valid mapping */
    int character;

    /** Unicode character mapping, or -1 if there is no valid mapping */
    unichar_t unichar;

    /** The scan code, but without the make/break flag (check .type for KEY_DOWN and KEY_UP flags) */
    uint8_t scanvalue;

    /** Indicates whether the key was pressed or released */
    enum KeyEventType type;

    /** Various key states */
    int state;
};

/** Mouse listener interface */
class KeyListener
{
    public:

        static void addKeyListener(KeyListener* listener);
        static void removeKeyListener(KeyListener* listener);

        virtual bool onKeyEvent(KeyEvent* evt) { return true; }
        virtual bool onKeyDown(int x, int y) { return true; }
        virtual bool onKeyUp(int x, int y) { return true; }
};

/**
 * Keyboard device driver
 */
class Keyboard : public DeviceDriver
{
public:

    /**
     * Receiver thread
     */
    class Receiver : public Threadable, public Waitable
    {
        public:

            /** C'tor */
            Receiver(Keyboard* driver)
            {
                this->driver = driver;
            }

            /** @see Threadable#run */
            virtual void run(void* arg);

        private:

            Keyboard* driver;
    };

    Receiver receiver;

    /** C'tor */
    Keyboard() : receiver(this)
    {
        state = 0;
        lastScancode = 0;
        focusedComponent = null;
        eventQueue = null;
    }

    /** Initialize driver */
    static void initialize();

    /**
     * @see DeviceDriver#onInterrupt
     */
    virtual int onInterrupt(ContextState* ctx);

    /**
     *  Set focal component
     *  @returns Previous focus, or NULL
     */
    static Component* setFocus(Component* component);

    /** Returns current focal compoenent, or NULL */
    static Component* getFocus();

    /** Current shift states. This is copied into KeyEvent's as they occur */
    int state;

    /** Last scan code */
    uint8_t lastScancode;

    /** Only the focused component receives keyboard events */
    Component* focusedComponent;

    /** Keyboard event queue */
    QueuePtr eventQueue;

    /** Last key event */
    KeyEvent lastEvent;
};

/** Read a character.*/
int readchar(long timeout = -1);

/** Block on a key event. Caller must free return value */
KeyEvent* getKey(long timeout = -1);

/** Why reboot is here? Ask IBM */
void reboot();

void setLED();
void clearLED();

/**
 * Called when a keyboard interrupt goes off
 *
 * \returns TRUE if EOI was sent by this handler, else FALSE
 */
boolean onKeyboardInterrupt(ContextState *ctx);


#include <kernel/io/Channel.h>

/**
 * Keyboard input channel
 */
class KeyboardChannel : public InputChannel
{
    public:

        /** KeyboardChannel is not clonable, so this always returns null */
        virtual AbstractChannel* clone()
        {
            return null;
        }

        /**
         * Read a character; block forever
         *
         * @see InputChannel
         * @return Returns character, or negated error_t
         */
        virtual int read()
        {
            int res = readchar();

            // TODO: echo handling
            if(res > 0)
            {
                out << (char) res << eos;
            }
            else
                out << '?' << eos;

            return res;
        }

        /**
         * Always blocks until length characters is read
         *
         * @see InputChannel
         */
        virtual ssize_t read(uint8* bytes, size_t length, bool block = true)
        {
            ssize_t res = 0;
            for(int i=0; i < length; i++)
            {
                res = read();
                if(res < E_OK)
                    break;

                bytes[i] = res;
            }

            return res < E_OK ? res : length;
        }

    private:

        Keyboard* driver;

};

#endif // _APOX_DRIVER_KEYBOARD_H_

/**@}*/
