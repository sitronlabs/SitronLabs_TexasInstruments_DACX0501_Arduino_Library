#ifndef DAC80501_H
#define DAC80501_H

/* Library headers */
#include "dacx0501.h"

/**
 * DAC80501 Class - 16-bit DAC
 * Inherits from dacx0501 base class
 */
class dac80501 : public dacx0501 {
   public:
    /**
     * Constructor
     */
    dac80501() : dacx0501(16) {}
};

#endif
