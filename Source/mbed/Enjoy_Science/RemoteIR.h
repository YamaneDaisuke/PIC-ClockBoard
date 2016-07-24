/**
 * IR remote common class (Version 0.0.4)
 *
 * Copyright (C) 2015 Daisuke Yamane
 * 
 */

#ifndef _REMOTE_IR_H_
#define _REMOTE_IR_H_
#pragma O3
#pragma Otime

class RemoteIR {
public:

    typedef enum {
        UNKNOWN,
        NEC = 1,
        AEHA = 2,
        SONY = 3,
        HELI = 4,
        AEHA_REPEAT,
        NEC_REPEAT
    } Format;

    static const int TUS_NEC = 562;
    static const int TUS_AEHA = 425;
    static const int TUS_SONY = 600;
    static const int TUS_HELI = 330;

private:
    RemoteIR();
};

#endif
