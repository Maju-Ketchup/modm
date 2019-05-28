/**
 *  Copyright (c) 2017, Marten Junga (Github.com/Maju-Ketchup)
 * All Rights Reserved.
 *
 * The file is part of the modm library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// Set always control first
#include "dw1000.hpp"
#include <modm/board.hpp>

constexpr uint16_t modm::dw1000::lde_replicaCoeff[dw1000::PCODES];
constexpr uint32_t modm::dw1000::digital_bb_config[NUM_PRF][dw1000::NUM_PACS];
constexpr uint16_t modm::dw1000::dtune1[dw1000::NUM_PRF];
constexpr uint16_t modm::dw1000::sftsh[NUM_BR][dw1000::NUM_SFD];
constexpr uint8_t modm::dw1000::dwnsSFDlen[dw1000::NUM_BR];
constexpr uint8_t modm::dw1000::rx_config[dw1000::NUM_BW];
constexpr uint8_t modm::dw1000::fs_pll_tune[dw1000::NUM_CH];
constexpr uint32_t modm::dw1000::fs_pll_cfg[dw1000::NUM_CH];
constexpr uint32_t modm::dw1000::tx_config[dw1000::NUM_CH];
constexpr uint8_t modm::dw1000::chan_idx[dw1000::NUM_CH_SUPPORTED];
constexpr modm::dw1000::agc_cfg_struct modm::dw1000::agc_config;

