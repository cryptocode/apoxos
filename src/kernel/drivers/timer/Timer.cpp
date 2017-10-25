/*
 * The Apox Operating System
 * Copyright (c) 2005,2006,2007,2008 cryptocode
 *
 * Timer implementation
 */

#include <kernel/drivers/timer/Timer.h>
#include <kernel/drivers/video/Video.h>
#include <kernel/io/Ports.h>
#include <kernel/threads/ThreadScheduler.h>
#include <kernel/vmm/vmm.h>
#include <kernel/vmm/malloc.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/New.h>
#include <arch/Delay.h>

Timer* Timer::instance = null;

namespace
{
    #define USECS_PER_TICK      (1000000 / INITIAL_HZ)
    #define MSECS_PER_TICK      (1000 / INITIAL_HZ)
    #define TICKS_PER_MSEC      (1000 / INITIAL_HZ)

    volatile uint64 ticks = 0;
    volatile uint64 irqCounter = 0;
    volatile uint64 realticks = 0;
    volatile uint64 cycleMark = 0;

    /**
     * Alarm comparator
     */
    int cmpTimer(const void* a, const void* b)
    {
        Alarm* pA = (Alarm*)a;
        Alarm* pB = (Alarm*)b;

        if(unlikely(pA->dueTick == pB->dueTick))
            return 0;
        else if(pA->dueTick < pB->dueTick)
            return -1;
        else
            return 1;
    }
}

/* See declaration */
uint32 Timer::getTickCount32()
{
    return (volatile uint32)ticks;
}

/* See declaration */
uint64 Timer::getTickCount64()
{
    return ticks;
}

/* See declaration */
Timer* Timer::getInstance()
{
    if(Timer::instance == null)
    {
        Timer::instance = new Timer();
    }

    return Timer::instance;
}

/**
 * Create and initialize the timer
 */
Timer* Timer::initialize()
{
    Timer* timer = Timer::getInstance();

    // Register device
    Device* dev = new Device(timer, L"System Timer");
    DeviceManager::addDevice(dev);

    // Reprogram pit to fire on INITIAL_HZ frequency
    setHertz(INITIAL_HZ);

    // Timer alarms
    timer->alarms.setAllocator(timer);
    timer->alarms.setComparator((Comparator)cmpTimer);

    timer->nodeCache.initialize("AlarmNodeCache", null, sizeof(BinTreeNode<Alarm*>),
                                &VMM::getKernelSpace().getVirtualMgr());

    timer->alarmCache.initialize("AlarmCache", null, sizeof(Alarm),
                                 &VMM::getKernelSpace().getVirtualMgr());

    // Activate timer irq handler
    installIRQHandler(IRQ_SYSTEM_TIMER, timer);

    return timer;
}

typedef void (*alarmfunc)(void*);

/* See declaration */
int Timer::onInterrupt(ContextState* ctx)
{
    assert(irqEnabled() == 0);

    // Update tick count; this may get adjusted by CPU cycle measurements
    ticks++;

    // IRQ counter
    irqCounter++;

    // Update quantum
    Thread* t = Thread::current();
    assert(t != null);
    if(t->quantumRemaining > 0)
    {
        t->quantumRemaining--;
    }

    // Process due alarms; note that minItem is O(1)
    if(alarms.minItem() != null && ticks >= alarms.minItem()->dueTick)
    {
        Alarm* alarm = alarms.minItem();

        // Make a volatile dereference; another cpu may have cancelled the alarm.
        alarmfunc onAlarm = (alarmfunc)(volatile void*)alarm->onAlarm;
        void* param = alarm->param;

        // Remove item; this will cause Timer#deallocateObject to be called.
        alarms.remove(alarm);

        // onAlarm is null when the alarm is cancelled cancelled
        if(onAlarm)
            onAlarm(param);
    }

    if(alarms.minItem() == null)
        assert(alarms.count() == 0);

    // resched N times a sec at most
    if(ticks % (INITIAL_HZ / 100) == 0)
    {
        return IRQRES_RESCHEDULE;
    }

    return IRQRES_NORMAL;
}

/*
 * Called to allocate a BinTreeNode
 *
 * @param param Contains the parameter to the BinTreeNode constructor
 * @see Allocator#allocateObject
 */
void* Timer::allocateObject(void* param, long flags)
{
    BinTreeNode<Alarm*>* item = (BinTreeNode<Alarm*>*) nodeCache.pop();

    // Invoke c'tor with placement-new
    return new (item) BinTreeNode<Alarm*>((Alarm*)param);
}

/*
 * Called to deallocate a BinTreeNode
 *
 * @see Allocator#deallocateObject
 */
void Timer::deallocateObject(void* obj, long flags)
{
    assert(obj != null);

    BinTreeNode<Alarm*>* dealloced = (BinTreeNode<Alarm*>*) obj;

    // Free both the node object and the alarm
    alarmCache.push(dealloced->item);
    nodeCache.push((BinTreeNode<Alarm*>*) dealloced);
}

/**
 * Add a timer
 *
 * @param period Time in millisecond
 * @param onAlarm Function to call when the sleep period has expired
 */
Alarm* Timer::addTimer(uint64 period, void (*onAlarm)(void*), void* param)
{
    AutoSpin driverLock(Timer::getInstance());

    BinTree<Alarm*>& alarms = Timer::getInstance()->alarms;

    Alarm* alarm = Timer::getInstance()->alarmCache.pop();
    assert(alarm != null);

    // Note: This only works when INITIAL_HZ is 1000
    alarm->dueTick = ticks + period;

    // Prevent multiple alarms on the same tick. This keeps the irq latency
    // low when the tick fires (only one alarm). Also, the binary tree requires
    // a unique key and dueTick is the key.
    while(alarms.search(alarm))
    {
        alarm->dueTick++;
    }

    alarm->onAlarm = onAlarm;
    alarm->param = param;

    // Add alarm
    assert(alarms.add(alarm) == E_OK);

    return alarm;
}

/** Flag the alarm as cancelled so it'll be ignored */
void Timer::cancelTimer(Alarm* alarm)
{
    // It is not safe to remove the alarm here so we just mark it
    // as cancelled.
    alarm->onAlarm = 0;
}

/**
 * Change heartbeat
 */
void Timer::setHertz(int hertz)
{
    // Incredibly Blunt and Moronic
    int ratio = PIT_FREQUENCY / hertz;

    // Output the PIT command
    out8(0x43, 0x36);

    // Set low byte of ratio
    out8(0x40, ratio & 0xFF);

    // Set high byte of ratio
    out8(0x40, ratio >> 8);
}
