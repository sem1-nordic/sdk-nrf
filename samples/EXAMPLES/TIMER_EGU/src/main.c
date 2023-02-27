/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

#include "hal/nrf_timer.h"
#include "hal/nrf_ppi.h"
#include "hal/nrf_egu.h"
#include <nrfx_ppi.h>
#include <zephyr/irq.h>
#include <helpers/nrfx_gppi.h>

nrf_ppi_channel_t gppi;
#define EGU_CHANNEL 0
#define EGU_PRIO 1

#define MYTIMER NRF_TIMER2
#define MYTIMER_FREQ NRF_TIMER_FREQ_16MHz
#define TIMER_CC_0 0
#define TIMER_CC_1 1
#define TIMER_CC_2 2

static void m_handler(const void *context)
{
	nrf_egu_event_t egu_chan_triggered_evt =
		nrf_egu_triggered_event_get(EGU_CHANNEL);
	nrf_egu_event_clear(NRF_EGU0, egu_chan_triggered_evt);
	nrfx_gppi_channels_disable(BIT(gppi));
}

void main(void)
{
	volatile uint32_t runtime_us;
	volatile uint32_t runtime2_us;
	volatile uint32_t runtime_ticks;
	volatile uint32_t runtime2_ticks;

	/* Enable interrupt for EGU channel */
	nrf_egu_int_enable(NRF_EGU0, BIT(EGU_CHANNEL));

	/* Connect IRQ for EGU0 IRQ at priority EGU_PRIO, with callback m_handler and channel EGU_CHANNEL. */
	IRQ_CONNECT(SWI0_EGU0_IRQn, EGU_PRIO, m_handler, NULL, BIT(EGU_CHANNEL));
	/* Enable IRQ for EGU0 IRQ */
	irq_enable(SWI0_EGU0_IRQn);

	/* Allocate gppi channel */
	if (nrfx_gppi_channel_alloc(&gppi) != NRFX_SUCCESS) {
		return;
	}

	/* Set up endpoints of channel, event endpoint of NRF_TIMER_EVENT_COMPARE0,
	   so when MYTIMER count value equals CC0, this triggers event of task
	   NRF_EGU of channel EGU_CHANNEL */
	nrfx_gppi_channel_endpoints_setup(
		gppi,
		nrf_timer_event_address_get(
			MYTIMER,
			NRF_TIMER_EVENT_COMPARE0),
		nrf_egu_task_address_get(
			NRF_EGU0,
			nrf_egu_trigger_task_get(EGU_CHANNEL)));

	/* Enable gppi channel */
	nrfx_gppi_channels_enable(BIT(gppi));

	/* Configure timer instance */
	nrf_timer_bit_width_set(MYTIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_frequency_set(MYTIMER, MYTIMER_FREQ);
	nrf_timer_mode_set(MYTIMER, NRF_TIMER_MODE_TIMER);

	/* Clear timer */
	nrf_timer_task_trigger(MYTIMER, NRF_TIMER_TASK_CLEAR);

	/* Calculate number of ticks for comparison value */
	uint32_t comparison_ticks =
		nrf_timer_us_to_ticks(1000, MYTIMER_FREQ);
	/* Set CC0 value to comparison ticks */
	nrf_timer_cc_set(MYTIMER, TIMER_CC_0, comparison_ticks);

	/* Start timer */
	nrf_timer_task_trigger(MYTIMER, NRF_TIMER_TASK_START);

	k_sleep(K_MSEC(1000));

	/* Capture first time value to CC 1 */
	nrf_timer_task_trigger(MYTIMER, NRF_TIMER_TASK_CAPTURE1);

	k_sleep(K_MSEC(1000));

	/* Capture second time value to CC 2 */
	nrf_timer_task_trigger(MYTIMER, NRF_TIMER_TASK_CAPTURE2);

	runtime_ticks = nrf_timer_cc_get(MYTIMER, TIMER_CC_1);
	runtime2_ticks = nrf_timer_cc_get(MYTIMER, TIMER_CC_2);

	runtime_us = (runtime_ticks << MYTIMER_FREQ) / 16ULL;
	runtime2_us = (runtime2_ticks << MYTIMER_FREQ) / 16ULL;

	printk("%i", runtime_us);
	printk("%i", runtime2_us);
}
