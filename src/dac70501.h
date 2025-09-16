#ifndef DAC70501_H
#define DAC70501_H

/* Library headers */
#include "dacx0501.h"

/**
 * DAC70501 Class - 14-bit DAC
 * Inherits from dacx0501 base class
 */
class dac70501 : public dacx0501 {
   public:
    /**
     * Constructor
     */
    dac70501() : dacx0501(14) {}
};

#endif
