#ifndef SX127X_TEST_SX127X_AT_H
#define SX127X_TEST_SX127X_AT_H

#include "sx127x.h"
#include <stddef.h>

#define SX127X_CONTINUE 1

/**
 * @brief AT command interface to control the chip. All public functions can be controlled through this interface. For exact naming please refer to the source code.
 *
 * @param device Pointer to variable to hold the device handle
 * @param input Input buffer containing the command. Only first 256 bytes are actually used
 * @param output Pre-allocated output buffer for response. Success: "OK" Query response: "<data>\\r\\nOK" Error: "<error message>\\r\\nERROR".
 * @param output_len The length of the pre-allocated output buffer
 * @return
 *          SX127X_OK - when the input command was recognized and processed
 *          SX127X_CONTINUE - when input command was NOT recognized
 *          SX127X_ERR_INVALID_ARG - if any input parameters are invalid
 */
int sx127x_at_handler(sx127x *device, const char *input, char *output, size_t output_len);

#endif //SX127X_TEST_SX127X_AT_H
