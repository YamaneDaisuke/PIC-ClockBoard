/**
 * IR transmitter (Version 0.0.4)
 *
 * Copyright (C) 2015 Daisuke Yamane
 * 
 */

#ifndef _TRANSMITTER_IR_H_
#define _TRANSMITTER_IR_H_

#include <mbed.h>

#include "RemoteIR.h"

/**
 * IR transmitter class.
 */
class TransmitterIR {
public:

    /**
     * Constructor.
     *
     * @param txpin Pin for transmit IR signal.
     */
    explicit TransmitterIR(PinName txpin);

    /**
     * Destructor.
     */
    ~TransmitterIR();

    typedef enum {
        Idle,
        Leader,
        Data,
        Trailer
    } State;

    /**
     * Get state.
     *
     * @return Current state.
     */
    State getState(void);

    /**
     * Set data.
     *
     * @param format Format.
     * @param buf Buffer of a data.
     * @param bitlength Bit length of the data.
     *
     * @return Data bit length.
     */
    int setData(RemoteIR::Format format, uint8_t *buf, int bitlength);

private:

    typedef struct {
        State state;
        unsigned int bitcount;
        unsigned int leader;
        unsigned int data;
        unsigned int trailer;
    } work_t;

    typedef struct {
        RemoteIR::Format format;
        int bitlength;
        uint8_t buffer[64];
    } data_t;

    PwmOut tx;
    Ticker ticker;
    data_t data;
    work_t work;
    int tmp;

    void tick();

};

#endif