/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "py/mperrno.h"
#include "py/runtime.h"
#include "shared-bindings/audiobusio/PDMIn.h"
#include "shared-bindings/microcontroller/Pin.h"

// see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html#pdm-rx-usage
#include "driver/i2s_pdm.h"

i2s_chan_handle_t rx_handle;

// Caller validates that pins are free.
void common_hal_audiobusio_pdmin_construct(audiobusio_pdmin_obj_t *self,
    const mcu_pin_obj_t *clock_pin,
    const mcu_pin_obj_t *data_pin,
    uint32_t sample_rate,
    uint8_t bit_depth, // this is fixed to 16 bit
    bool mono, // below is currently set to mono, this is ignored
    uint8_t oversample // this has not been implemented
    ) {
    claim_pin(clock_pin);
    claim_pin(data_pin);

    self->sample_rate = sample_rate;
    self->mono = mono;
    self->clock_pin_number = clock_pin->number;
    self->data_pin_number = data_pin->number;

    if (bit_depth != 16) {
        mp_raise_ValueError(MP_ERROR_TEXT("only bit_depth=16 is supported"));
    }

    /* Allocate an I2S RX channel */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    /* Init the channel into PDM RX mode */
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = self->clock_pin_number,
            .din = self->data_pin_number,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
}

bool common_hal_audiobusio_pdmin_deinited(audiobusio_pdmin_obj_t *self) {
    return self->clock_pin_number == NO_PIN;
}

void common_hal_audiobusio_pdmin_deinit(audiobusio_pdmin_obj_t *self) {
    i2s_del_channel(rx_handle);

    reset_pin_number(self->clock_pin_number);
    self->clock_pin_number = NO_PIN;
    reset_pin_number(self->data_pin_number);
    self->data_pin_number = NO_PIN;
}

uint8_t common_hal_audiobusio_pdmin_get_bit_depth(audiobusio_pdmin_obj_t *self) {
    return 16;
}

uint32_t common_hal_audiobusio_pdmin_get_sample_rate(audiobusio_pdmin_obj_t *self) {
    return 16000;
}

uint32_t common_hal_audiobusio_pdmin_record_to_buffer(audiobusio_pdmin_obj_t *self,
    uint16_t *output_buffer, uint32_t output_buffer_length) {

    i2s_channel_read(rx_handle, output_buffer, output_buffer_length, NULL, 200);

    if (self->mono) {
        return (output_buffer_length / 2) * 2;
    } else {
        return (output_buffer_length / 4) * 4;
    }
}