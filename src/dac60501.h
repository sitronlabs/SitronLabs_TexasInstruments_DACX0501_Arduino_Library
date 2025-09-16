#ifndef DAC60501_H
#define DAC60501_H

/* Library headers */
#include "dacx0501.h"

/**
 * DAC60501 Class - 12-bit DAC
 * Inherits from dacx0501 base class
 */
class dac60501 : public dacx0501 {
   public:
    /**
     * Constructor
     */
    dac60501() : dacx0501(12) {}
};

#endif
