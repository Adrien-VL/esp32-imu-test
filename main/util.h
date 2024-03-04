#ifndef util_h
#define util_h

/**
 * @brief Function to combine two 8-bit data into one 16-bit data
 */
int16_t util_combine_register_values(uint8_t high, uint8_t low) {
    return ((int16_t)high << 8) | low;
}

#endif