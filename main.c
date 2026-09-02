/*
 * Copyright (c) 2023 Kanno System Labo
 *
 * Released under the MIT license.
 * see https://opensource.org/licenses/MIT
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "class/cdc/cdc_device.h"
#include "ssd1306.h"
#include "eeprom.h"
#include "image.h"
#include "pulse_ts.pio.h"

// Firmware id. Upstream uses "FWNX1R<nn>" and this fork's main used
// "FWNX1E01" -- both are `FWNX1` + one letter + two digits, so the letter
// space is shared and any id of that shape could collide with a future
// upstream release, or be mistaken for one. Use a shape upstream will not
// emit, and carry the commit so a deployed board is traceable to source.
// FW_BUILD_ID comes from git (or -DFW_BUILD_ID for the Docker build, which
// has no .git). "E" is for entropy, this fork's purpose. Dropping the
// release counter is deliberate: upstream emits sequence numbers, this
// emits a commit, so the two can never be confused. 15 chars fits the
// splash at x=30 (5px font + 1px gap, 98px from x=30).
#ifndef FW_BUILD_ID
#define FW_BUILD_ID "dev"
#endif
#define FW_VERSION "FWNX1E-" FW_BUILD_ID
#define MODEMAX 2
#define TOTAL_NUM 300

const uint LED_PIN = 25;
const float conversion_factor = 3.3f / ( 1 << 12) * 3;

// Written by the detection IRQ, read elsewhere: must not be cached in registers.
static volatile int tc = 0;
volatile uint32_t total_cnt;
volatile uint32_t ch1_cnt;
volatile uint32_t ch2_cnt;

static uint hvgPwmSlice;
static uint bzsPwmSlice;

uint16_t result;

static volatile bool df = false; // Detection Flag

bool isDual = false;
volatile bool is_sound_enabled = false;	// main loop/SerCmdExec write, detection IRQ reads
bool is_button_pressed = false;
volatile bool sout = true;      // main loop only, since the CPM write moved there
bool evt_mode = false;          // main loop only

volatile bool pd = false;		// main loop writes, timer IRQ reads and clears

uint8_t cptr;
volatile uint8_t dispMode;		// main loop writes, timer IRQ reads

volatile uint16_t gamma_sensitivity;	// SerCmdExec writes, timer IRQ reads
uint16_t alarm_trigger_cpm;
uint16_t hvg_pulsewidth;
// False when the stored HV pulse width was missing, erased or out of range,
// so the generator is parked rather than driven at a guessed bias.
bool hvg_trusted = true;
char CmdBuf[16];

uint32_t uptime;
uint32_t cpm;
uint32_t sma[4];
uint16_t cbuf[20];
volatile uint16_t cpx;
uint16_t lp;	// Long button press detection counter

uint16_t cnt = 0;
uint16_t fifocpm[TOTAL_NUM];
uint32_t sum = 0;
float avrg = 0;
float max_avrg = 0.00f;
float min_avrg = 1000.00f;

uint16_t tone;

uint8_t cbuf_idx;
uint8_t sec_tmr;
uint8_t sma_idx;

uint8_t gbx = 0;	// graph plotting queue index
uint16_t gpq[96];	// queue for graph plotting

int16_t cmax;
int16_t cmin;

ssd1306_t disp;

// ---------------------------------------------------------------------------
// Alarm
//
// atc was settable, saved, loaded and reported by `show`, but never compared
// against anything, so the threshold could be configured and could not fire.
// Hysteresis matters here because the reading is a 60 s window over Poisson
// counts: a bare threshold test chatters on and off around the setpoint.
// Clearing at 90% of the trigger costs nothing and makes the state stable.
// ---------------------------------------------------------------------------
static volatile bool alarm_on = false;

// ---------------------------------------------------------------------------
// No-signal supervision
//
// A dead tube or a failed HV supply reads as a confident 0 CPM, which is
// indistinguishable from "all clear" on the display -- the dangerous failure
// mode for an unattended instrument. Silence is evidence of a fault only in
// proportion to the count rate the detector normally sees, so the window is
// derived rather than fixed:
//
//     P(no counts in T) = exp(-lambda*T)   ->   T = ln(1/p) / lambda
//
// At a 25 CPM background a 30 s silence is already a one-in-a-million event.
// Shield the tube down to 1 CPM and the same 30 s window would cry failure
// roughly twice a day on perfectly good hardware, so it stretches to ~14 min
// instead. lambda is measured from this boot's own counts, biased low by two
// sigma so it is a lower bound rather than a best guess: underestimating the
// rate lengthens the window, which is the harmless direction.
//
// Deliberately NOT an input: the persisted baseline. A stored rate cannot be
// known to describe the environment the board woke up in, and if it is too
// high the window comes out too short, which is the direction that fabricates
// failure reports. It is kept as operator context only.
//
// The cost of that choice is honest: a board that powers up already dead has
// no observations to reason from, so it sits at the one-hour ceiling before
// saying anything. Waiting an hour to report a fault beats reporting one that
// is not there.
//
// The threshold stays at genuine zero and is never scaled off a baseline.
// Scaling it would fire every time the counter is moved off a source, which
// is normal use, not a fault.
//
// This is reported as a diagnostic, not a verdict: with no HV sense the
// firmware cannot tell a dead tube from a lead box, so it says "implausibly
// quiet", and the operator decides.
// ---------------------------------------------------------------------------
#define NSG_P_EXP        6u      // target false-positive rate, 1e-6
#define NSG_WIN_MIN_S    30u     // never claim a fault faster than this
#define NSG_WIN_MAX_S    3600u   // nor take longer than an hour

static bool     nsg_enabled = true;
static volatile bool     nsg_flag = false;
static volatile uint32_t silent_secs = 0;
static volatile uint32_t nsg_window_s = NSG_WIN_MAX_S;
static uint32_t last_seen_cnt = 0;

// Observed background floor, in tenths of a CPM so the 2-byte EEPROM slot
// keeps 0.1 CPM resolution up to 6553 CPM. Diagnostic context for the
// operator -- "this is what the board used to see" -- and explicitly not a
// threshold input; see the note above.
static uint16_t baseline_x10 = 0;
static uint16_t baseline_saved_x10 = 0;
static volatile bool baseline_save_req = false;

void SerCmdProc(char ch);
void SerCmdExec(void);
void sampleDisplay(void);
void drawScreen(void);

// ---------------------------------------------------------------------------
// Hardware pulse timestamping (PIO + DMA)
//
// Two state machines run pulse_ts.pio, one per detection input, started in
// sync so they share a single epoch. Each falling edge pushes a 32-bit
// timebase snapshot; a DMA channel per SM moves those words into a ring
// buffer. Nothing here needs the ARM core, so capture survives the 23 ms
// OLED refresh, USB traffic, and any other interrupt work.
// ---------------------------------------------------------------------------

#define TS_CH_COUNT   2
#define TS_RING_LOG2  10                       // 1024 entries per channel
#define TS_RING_LEN   (1u << TS_RING_LOG2)
#define TS_RING_BYTES (TS_RING_LEN * 4u)

// The DMA write-address wrap requires the buffer to be aligned to its size.
static uint32_t ts_ring[TS_CH_COUNT][TS_RING_LEN] __attribute__((aligned(TS_RING_BYTES)));

static const uint ts_gpio[TS_CH_COUNT] = { 22, 2 };   // CH1 = GP22, CH2 = GP2
static PIO       ts_pio = pio0;
static int       ts_dma[TS_CH_COUNT]      = { -1, -1 };
static uint64_t  ts_consumed[TS_CH_COUNT] = { 0, 0 };
static uint32_t  ts_lost[TS_CH_COUNT]     = { 0, 0 };
static uint32_t  ts_prev_x[TS_CH_COUNT]   = { 0, 0 };
static uint64_t  ts_ticks[TS_CH_COUNT]    = { 0, 0 };
static bool      ts_primed[TS_CH_COUNT]   = { false, false };
static uint32_t  ts_per_us = 6;   // SM ticks per microsecond, set in ts_start
static uint      ts_nch = 1;      // always 1: see the note in ts_start
static bool      ts_running = false;
// Output mode is one small scalar owned by SerCmdExec, which may run in
// UART1_IRQ. The main loop reconciles sout/evt_mode from it. Independent
// start/stop flags would race: a `stop` landing between the main loop's read
// and its action would be lost, and a `go` would leave CPM lines interleaved
// into a capture. Under the ARM EABI default (-fshort-enums) this compiles to
// a single byte, written with one strb, so the IRQ cannot tear it.
typedef enum { OUT_CPM = 0, OUT_QUIET, OUT_EVT } out_mode_t;
static volatile out_mode_t out_req = OUT_CPM;

// Work requested by interrupts and performed in the main loop.
//
// Nothing that can block belongs in an IRQ on this board. The OLED refresh is
// a 1025-byte I2C blit that takes ~23 ms, and any printf can stall for up to
// 500 ms waiting on USB plus 1000 ms on the stdio mutex. Running either from
// the 1 Hz timer callback stalled every other interrupt behind it, which is
// what made the detection interrupt miss counts in the first place. The timer
// now only computes and raises a flag; the main loop does the slow part with
// interrupts enabled, so the detector is never held off.
static volatile bool     ui_refresh_req = false;   // redraw the measurement screen
static volatile bool     cpm_out_pending = false;  // a CPM value is due on the wire
static volatile uint32_t cpm_out_value = 0;
static volatile uint32_t eeprom_err = 0;   // I2C failures, reported by `show`

// Silent-reboot telemetry: CONSECUTIVE watchdog resets since power-on, not a
// general reset count -- any non-watchdog reset (power cycle, BOOTSEL, DFU)
// correctly clears it back to zero.
// Watchdog scratch registers survive a watchdog reset
// but are cleared by a power cycle, which is exactly the distinction worth
// reporting: "has this board rebooted itself since it was plugged in?" Without
// it, a watchdog recovery is invisible -- counters restart and the host sees a
// healthy device, which is how the original lockup went unexplained. Scratch
// 0..3 are free; the SDK uses 4..7 for its own reboot magic. A deliberate
// `reboot` is tagged so only unexplained resets are counted. Note a fresh
// picotool load can still show 1, since it reboots outside our control.
static uint32_t wdt_reboots = 0;

// UART receive ring. The interrupt stores bytes and does nothing else, so no
// blocking work runs in interrupt context, but no input is dropped either:
// polling alone would lose data, because the RX FIFO holds 32 bytes (2.8 ms at
// 115200) while the main loop can be away for ~50 ms during a display redraw.
// Single producer in the IRQ, single consumer in the loop, power-of-two size,
// so it needs no locking on one core.
#define URX_LEN 256u
static volatile uint8_t  urx_buf[URX_LEN];
static volatile uint16_t urx_head = 0, urx_tail = 0;

static void on_uart_rx(void) {
	while (uart_is_readable(uart1)) {
		uint8_t ch = uart_getc(uart1);
		uint16_t next = (urx_head + 1u) & (URX_LEN - 1u);
		if (next != urx_tail) {          // full: drop, never block in an IRQ
			urx_buf[urx_head] = ch;
			urx_head = next;
		}
	}
}

// Total words the DMA has written since it was armed. transfer_count reads
// back as the remaining count, and we arm with 0xFFFFFFFF.
static uint32_t ts_produced(uint idx) {
	return 0xFFFFFFFFu - dma_hw->ch[ts_dma[idx]].transfer_count;
}

static void ts_start(void) {
	if (ts_running) {
		// Re-entering evt: drop the backlog so capture starts at "now", and
		// re-prime so the first event after the gap is consumed silently as a
		// new anchor. ts_ticks does not advance while stopped, so the clock
		// never jumps and the consumer's next delta is a real interarrival.
		for (uint i = 0; i < ts_nch; i++) {
			ts_consumed[i] = ts_produced(i);
			ts_primed[i] = false;
		}
		return;
	}

	// evt captures CH1 only, including on the Dual. Two independent blockers,
	// both of which must be solved together before ts_nch can become 2:
	//
	//  1. Ordering. ts_drain walks one ring to completion before starting the
	//     next, so a second channel would emit all of CH1's batch and then all
	//     of CH2's. Consumers take deltas across global line order, so
	//     interleaved channels yield out-of-order and negative intervals. The
	//     two rings must be merged by tick before output, using a cross-ring
	//     watermark, since a sample may not have landed in one ring when the
	//     other is snapshotted.
	//  2. Epoch. Each channel anchors its own tick total on its own first
	//     event, so even perfectly ordered output would mix two clocks.
	//
	// Either alone is insufficient. Shipping them half-done would feed
	// meaningless intervals to /dev/random as if they were decay timings.
	// The GPIO interrupt still counts CH2 for the display; only evt is CH1.
	ts_nch = 1;
	ts_per_us = clock_get_hz(clk_sys) / (1000000u * 8u);

	uint offset = pio_add_program(ts_pio, &pulse_ts_program);
	uint sm_mask = 0;

	for (uint i = 0; i < ts_nch; i++) {
		// Input only: do not claim the pad with pio_gpio_init, so the existing
		// GPIO interrupt, pull-up, LED and buzzer behaviour keep working.
		pio_sm_config c = pulse_ts_program_get_default_config(offset);
		sm_config_set_jmp_pin(&c, ts_gpio[i]);
		sm_config_set_clkdiv_int_frac(&c, 1, 0);

		// Enter at 'recov' so a line already low at boot cannot fake an edge.
		pio_sm_init(ts_pio, i, offset + pulse_ts_offset_recov, &c);
		pio_sm_exec(ts_pio, i, pio_encode_mov_not(pio_x, pio_null));
		sm_mask |= 1u << i;

		int ch = dma_claim_unused_channel(true);
		ts_dma[i] = ch;
		dma_channel_config dc = dma_channel_get_default_config(ch);
		channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
		channel_config_set_read_increment(&dc, false);
		channel_config_set_write_increment(&dc, true);
		channel_config_set_ring(&dc, true, TS_RING_LOG2 + 2);
		channel_config_set_dreq(&dc, pio_get_dreq(ts_pio, i, false));
		dma_channel_configure(ch, &dc, ts_ring[i], &ts_pio->rxf[i], 0xFFFFFFFFu, true);

		ts_consumed[i] = 0;
		ts_lost[i] = 0;
	}

	// Simultaneous start: both counters share an epoch from here on.
	pio_enable_sm_mask_in_sync(ts_pio, sm_mask);
	ts_running = true;
}

// Drain both rings to stdout. Runs in thread context, never in an interrupt.
static void ts_drain(void) {
	for (uint i = 0; i < ts_nch; i++) {
		if (ts_dma[i] < 0) continue;

		uint64_t produced = ts_produced(i);
		uint64_t avail = produced - ts_consumed[i];

		if (avail > TS_RING_LEN) {
			// DMA lapped the ring. Resuming here would hand the consumer one
			// fabricated interarrival spanning every lost event, because it
			// derives intervals from consecutive lines and cannot see a
			// comment marker. That is the same corruption this capture path
			// exists to remove, so fail closed instead of emitting it.
			// `evt` restarts the stream with a re-primed epoch.
			uint32_t lost = (uint32_t)(avail - TS_RING_LEN);
			ts_lost[i] += lost;
			ts_consumed[i] = produced;
			printf("# overrun ch%u lost=%lu - evt stopped\n",
			       i + 1, (unsigned long)lost);
			out_req = OUT_QUIET;
			return;
		}

		while (avail--) {
			uint32_t x = ts_ring[i][ts_consumed[i] & (TS_RING_LEN - 1)];
			ts_consumed[i]++;

			// X counts down. Modular subtraction accumulates into a 64-bit
			// tick total, so the 715 s hardware wrap never reaches the wire
			// and the reported clock is monotonic for the life of the boot.
			//
			// The priming sample is consumed silently. ts_ticks does not
			// advance while capture is stopped, so anchoring on this event and
			// printing from the next one means the consumer's first delta
			// after a restart is a genuine interarrival, rather than a span
			// covering the idle period or a degenerate zero.
			if (!ts_primed[i]) {
				ts_primed[i] = true;
				ts_prev_x[i] = x;
				continue;
			}
			ts_ticks[i] += (uint32_t)(ts_prev_x[i] - x);
			ts_prev_x[i] = x;

			// Wire format is exactly `E <microseconds> <channel>`, three
			// tokens. The downstream entropy feeder parses positionally and
			// rejects any line that is not exactly three fields, so this must
			// not gain a resolution field without updating that consumer in
			// lockstep. Capture itself stays at full 166.67 ns resolution.
			printf("E %llu %u\n",
			       (unsigned long long)(ts_ticks[i] / ts_per_us), i + 1);
		}
	}
}

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
	const uint CS_PIN_INDEX = 1;
	uint32_t flags = save_and_disable_interrupts();

	hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
			GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
			IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

	for (volatile int i = 0; i < 1000; ++i);

	bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

	hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
			GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
			IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

	restore_interrupts(flags);

	return button_state;
}

void gpio_callback(uint gpio, uint32_t events) {
	if (events & 0x04) {
		gpio_put(LED_PIN, 1);
		// The alarm owns the buzzer outright while it holds: a click landing
		// mid-alarm would drop the tone to click level, and the clear in
		// msecTimer_callback is barred during an alarm, so it would stay
		// there. One volatile byte read to keep the two paths disjoint.
		if (is_sound_enabled && !alarm_on) {
			pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, 1000/3);
		}

		df = true;
		cpx++;
		total_cnt++;

		if (gpio == 22) {
			ch1_cnt++;
		} else
		if (gpio == 2) {
			ch2_cnt++;
		}

		// Event timestamps are captured by PIO + DMA, not here. Formatting
		// them in this handler used to stall it for tens of microseconds and,
		// because the GPIO edge latch is one sticky bit per pin, any edges
		// arriving during the stall were merged away rather than queued.
	}
}

void SerCmdProc(char ch)
{
	switch(ch) {
		case '\b':
			if (cptr > 0) {
				cptr--;
				printf("\b \b");
			}
			break;
		case '\r':
			if (cptr < 16) {
				CmdBuf[cptr] = 0x00;
				printf("\r\n");
				SerCmdExec();
			}
			cptr = 0;
			break;
		case '\n':
			break;
		default:
			printf("%c", ch);
			if (cptr < (16 - 1)) {
				CmdBuf[cptr++] = ch;
			} else {
				cptr = 0;
				printf("?\r\n");
			}
			break;
	}
}

bool msecTimer_callback(repeating_timer_t *rt) {
	if (df == true) {
		tc++;
		if (tc > 20) {
			df = false;
			tc = 0;
			gpio_put(LED_PIN, 0);
			// The alarm owns the buzzer while it is active, so the per-count
			// click must not switch it back off mid-alarm.
			if (!alarm_on) {
				pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, 0);
			}
		}
	}

	// drawScreen blits the framebuffer over I2C, so it cannot run here. Leave
	// pd set and let the main loop service it; the redraw is idempotent and
	// coalescing repeats is correct, since drawScreen reads dispMode fresh.
	return true;
}

bool secTimer_callback(repeating_timer_t *rt) {
	uint8_t i;

	sec_tmr++;
	uptime++;	

	if (sec_tmr % 3 == 0) {
		// Read-and-clear must be atomic: the detection IRQ runs at a higher
		// priority than this timer callback and increments cpx.
		uint32_t irq_state = save_and_disable_interrupts();
		uint16_t counts = cpx;
		cpx = 0;
		restore_interrupts(irq_state);

		cbuf[cbuf_idx++] = counts;
		if (cbuf_idx == 20) cbuf_idx = 0;

		cpm = 0;
		for (i = 0; i < 20; i++) cpm += cbuf[i];
	}

	if (cnt == TOTAL_NUM) cnt = 0;

	sum -= fifocpm[cnt];
	fifocpm[cnt] = cpm;
	sum += fifocpm[cnt];
	cnt++;

	avrg = (float)sum / TOTAL_NUM;

	if (uptime > (300 + 60)) {
		if (avrg > max_avrg)
			max_avrg = avrg;

		if (avrg < min_avrg)
			min_avrg = avrg;

		// Persist the learned floor, but only when it has moved materially
		// and only via the main loop. eeprom_write_word holds interrupts off
		// for up to 10 ms, which at these rates loses a count, and avrg
		// updates every second -- writing as it learns would both drop counts
		// continuously and wear the part out in days. A 25% deadband plus
		// min_avrg being monotonic within a boot means this settles to a
		// handful of writes for the life of the board.
		uint16_t want = (uint16_t)(min_avrg * 10.0f + 0.5f);
		uint16_t ref  = baseline_saved_x10;
		if (want != ref && (want < (ref - ref / 4) || want > (ref + ref / 4))) {
			baseline_x10 = want;
			baseline_save_req = true;
		}
	}

	// Alarm, with hysteresis so it cannot chatter around the setpoint.
	// atc == 0 disables it, which is the shipped default.
	if (alarm_trigger_cpm > 0) {
		if (!alarm_on && cpm >= alarm_trigger_cpm) {
			alarm_on = true;
		} else if (alarm_on && cpm < (uint32_t)alarm_trigger_cpm * 9u / 10u) {
			alarm_on = false;
		}
	} else {
		alarm_on = false;
	}

	// Drive the buzzer on level changes only.
	//
	// The clear has to happen here and not lean on the per-count click path
	// to undo it. While the alarm holds, msecTimer_callback is barred from
	// switching the buzzer off, and that clear only runs after a count sets
	// df -- so if the alarm ends because counts stopped, no further click
	// would ever run and the tone would latch on forever. That is exactly
	// the case the no-signal supervision below exists to catch, which would
	// have made a dead tube sound like a screaming alarm.
	//
	// Comparing the wanted level rather than the alarm flag also covers
	// `set snd=off` arriving mid-alarm, where the flag never changes.
	{
		static uint16_t alarm_level_prev = 0;
		uint16_t want = (alarm_on && is_sound_enabled) ? (1000u / 2u) : 0u;
		if (want != alarm_level_prev) {
			pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, want);
			alarm_level_prev = want;
		}
	}

	// Silence supervision. Any count at all resets the run, so this measures
	// the current gap rather than a rate.
	if (total_cnt != last_seen_cnt) {
		last_seen_cnt = total_cnt;
		silent_secs = 0;
		nsg_flag = false;
	} else {
		silent_secs++;
	}

	// Size the window from what THIS boot has actually observed, and from
	// nothing else.
	//
	// The persisted baseline is deliberately not an input. Its error
	// direction is the dangerous one: a baseline learned in a hotter spot
	// overestimates lambda, an overestimate shortens the window, and a short
	// window is exactly what invents false failure reports. Concretely, a
	// baseline of 400 CPM captured next to a uranium-glass source pins the
	// window at the 30 s floor; move the counter somewhere with a 1 CPM
	// background and P(no counts in 30 s) is about 0.6, so it would cry
	// hardware failure almost continuously on a perfectly good tube.
	// Nothing at boot can tell us the environment did not change, so the
	// stored value informs the operator and never the threshold.
	//
	// Two session sources, lower wins, because a lower lambda means a longer
	// window means a claim that is harder to make:
	//
	//   counts so far   biased low on purpose, see below
	//   min_avrg        measured floor over 300 s, valid after the settle
	//                   gate; preferred when lower, since the running mean
	//                   includes any time spent near a source
	{
		float lam = 0.0f;
		uint32_t win = NSG_WIN_MAX_S;

		// Subtract 2 sigma from the observed count so this is a lower bound
		// on the rate rather than an estimate of it. 30 counts is the floor
		// for the bound to mean anything; below that there is no evidence
		// worth acting on and the window stays at the ceiling.
		if (uptime >= 60u && total_cnt >= 30u) {
			float n  = (float)total_cnt;
			float lo = n - 2.0f * sqrtf(n);
			if (lo > 0.0f) lam = lo / (float)uptime * 60.0f;
		}
		if (min_avrg < 1000.0f && (lam == 0.0f || min_avrg < lam)) {
			lam = min_avrg;
		}

		// Below 0.1 CPM there is not enough signal to justify any claim.
		if (lam >= 0.1f) {
			// T = ln(10^p)/lambda, lambda in CPM -> seconds
			float t = (float)NSG_P_EXP * 2.302585f / lam * 60.0f;
			if (t < (float)NSG_WIN_MIN_S) t = (float)NSG_WIN_MIN_S;
			if (t > (float)NSG_WIN_MAX_S) t = (float)NSG_WIN_MAX_S;
			win = (uint32_t)t;
		}
		nsg_window_s = win;

		if (nsg_enabled && silent_secs >= win) {
			nsg_flag = true;
		}
	}

	if (sec_tmr == 60) {
		sec_tmr = 0;
	}

	// The screen refresh and the serial write both block for tens of ms. Only
	// raise the request here; the main loop performs them.
	ui_refresh_req = true;

	return true;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// One render path for all three screens.
//
// There used to be two: prepareDisp for the button-driven redraw and dispCPM
// for the 1 Hz refresh, each carrying its own copy of all three screens. They
// had already drifted -- prepareDisp formatted VCC without ever reading the
// ADC, so the info screen showed 0.000000 until the next tick -- and an
// earlier divergence in the same pair is why dispMode needs a bounds check
// before use. Sampling is now separate from drawing: the 1 Hz tick samples
// then draws, a mode change only draws, so a redraw is idempotent and a mode
// change shows real values immediately instead of empty chrome.

static uint32_t disp_cpm;	// smoothed CPM for display, owned by sampleDisplay

// One status tag, highest priority first. [NV] means the averaging window is
// still filling, so the figure on screen is not yet meaningful.
static const char *statusTag(uint32_t settle) {
	if (nsg_flag)        return "[NS]";
	if (alarm_on)        return "[!!]";
	if (uptime < settle) return "[NV]";
	return NULL;
}

// Advances every per-second quantity. Must run exactly once per tick: the
// graph queue and the SMA ring are state, not derived values, so folding this
// into the renderer would fast-forward both on every button press.
void sampleDisplay(void) {
	uint8_t i;
	uint32_t tmp;

	sma[sma_idx++] = cpm;
	if (sma_idx == 4) sma_idx = 0;

	tmp = 0;
	for (i = 0; i < 4; i++) tmp += sma[i];
	tmp = ((tmp << 1) / 4 + 1) >> 1;

	disp_cpm = tmp;

	// Write at gbx then advance, which leaves gbx on the oldest slot -- where
	// the renderer starts its walk.
	gpq[gbx] = tmp;
	gbx++;
	if (gbx == 96) gbx = 0;

	// Handed to the main loop rather than written here, so the display path
	// and the serial path cannot stall each other.
	cpm_out_value = tmp;
	cpm_out_pending = true;
}

void drawScreen(void) {
	uint8_t i, j;
	char buf[24];
	const char *tag;

	// Guard the divisor once: gms is user-settable and a zero would put inf
	// or nan on the dose line.
	uint16_t gms = gamma_sensitivity ? gamma_sensitivity : 1;

	if (dispMode == 0) {
		ssd1306_clear(&disp);

		ssd1306_draw_char(&disp, 0, 0, 1, 'R');
		ssd1306_draw_char(&disp, 0, 8, 1, 'T');

		tag = statusTag(60);
		if (tag) ssd1306_draw_string(&disp, 8, 4, 1, (char *)tag);

		sprintf(buf, "%lu", (unsigned long)disp_cpm);
		ssd1306_draw_string(&disp, 96 - strlen(buf) * 16, 0, 2, buf);

		sprintf(buf, "CPM");
		ssd1306_draw_string(&disp, 98, 4, 1, buf);

		sprintf(buf, "%.4f", (float)disp_cpm / gms);
		if (strlen(buf) > 6) {
			sprintf(buf, "%.3f", (float)disp_cpm / gms);
		}
		ssd1306_draw_string(&disp, 0, 18, 2, buf);

		sprintf(buf, "uSv");
		ssd1306_draw_string(&disp, 98, 16, 1, buf);

		sprintf(buf, "/h");
		ssd1306_draw_string(&disp, 102, 24, 1, buf);

		cmax = 0;
		for (i = 0; i < 96; i++) {
			if (gpq[i] > cmax) cmax = gpq[i];
		}
		cmax = cmax + 8;
		cmax = ((cmax / 10) + 1) * 10;
		cmin = 0;

		// Oldest sample first. The three hand-unrolled wrap cases this
		// replaces all reduced to this single walk.
		j = 0;
		for (i = 0; i < 96; i++) {
			uint8_t idx = (uint8_t)((gbx + i) % 96);
			if (gpq[idx]) {
				int y = map(gpq[idx], cmin, cmax, 63, 34);
				ssd1306_draw_line(&disp, j, y, j, 62);
				j++;
			}
		}

		int k;
		for (k = 0; k < 98; k++) {
			if (k % 4 == 0) {
				ssd1306_draw_pixel(&disp, k, 35);
				ssd1306_draw_pixel(&disp, k, 63);
			}
		}
		for (k = 35; k < 64; k++) {
			if (k % 2 == 0) {
				ssd1306_draw_pixel(&disp, 0, k);
				ssd1306_draw_pixel(&disp, 100, k);
			}
		}

		ssd1306_draw_line(&disp, 98, 35, 102, 35);
		ssd1306_draw_line(&disp, 98, 63, 102, 63);

		sprintf(buf, "%d", (int)cmax);
		ssd1306_draw_string(&disp, 104, 36, 1, buf);

		sprintf(buf, "%d", (int)cmin);
		ssd1306_draw_string(&disp, 104, 56, 1, buf);

		ssd1306_show(&disp);
	} else
	if (dispMode == 1) {
		ssd1306_clear(&disp);

		ssd1306_draw_char(&disp, 0, 0, 1, 'M');
		ssd1306_draw_char(&disp, 0, 8, 1, 'A');

		tag = statusTag(300 + 60);
		if (tag) ssd1306_draw_string(&disp, 8, 4, 1, (char *)tag);

		sprintf(buf, "%.2f", avrg);
		ssd1306_draw_string(&disp, 96 - 24, 7, 1, buf + strlen(buf) - 3);
		buf[strlen(buf) - 3] = 0;
		ssd1306_draw_string(&disp, 96 - strlen(buf) * 16 - 24 + 4, 0, 2, buf);

		sprintf(buf, "CPM");
		ssd1306_draw_string(&disp, 98, 4, 1, buf);

		sprintf(buf, "%.4f", avrg / gms);
		if (strlen(buf) > 6) {
			sprintf(buf, "%.3f", avrg / gms);
		}
		ssd1306_draw_string(&disp, 0, 18, 2, buf);

		sprintf(buf, "uSv");
		ssd1306_draw_string(&disp, 98, 16, 1, buf);

		sprintf(buf, "/h");
		ssd1306_draw_string(&disp, 102, 24, 1, buf);

		int k;
		for (k = 0; k < 128; k++) {
			if (k % 4 == 0) {
				ssd1306_draw_pixel(&disp, k, 38);
			}
		}
		for (k = 38; k < 64; k++) {
			if (k % 3 == 0) {
				ssd1306_draw_pixel(&disp, 92, k);
			}
		}

		if (uptime < 300 + 60) {
			sprintf(buf, "MAX: UD");
		} else {
			sprintf(buf, "MAX:%.4f", max_avrg / gms);
		}
		ssd1306_draw_string(&disp, 0, 44, 1, buf);

		if (uptime < 300 + 60) {
			sprintf(buf, "MIN: UD");
		} else {
			sprintf(buf, "MIN:%.4f", min_avrg / gms);
		}
		ssd1306_draw_string(&disp, 0, 56, 1, buf);

		sprintf(buf, "N=");
		ssd1306_draw_string(&disp, 98, 44, 1, buf);

		sprintf(buf, "300");
		ssd1306_draw_string(&disp, 98, 56, 1, buf);

		ssd1306_show(&disp);
	} else
	if (dispMode == 2) {
		ssd1306_clear(&disp);

		sprintf(buf, "FWV:%s", FW_VERSION);
		ssd1306_draw_string(&disp, 0, 0, 1, buf);

		sprintf(buf, "TCN:%lu", total_cnt);
		ssd1306_draw_string(&disp, 0, 9, 1, buf);

		sprintf(buf, "CH1:%lu", ch1_cnt);
		ssd1306_draw_string(&disp, 0, 18, 1, buf);

		sprintf(buf, "CH2:%lu", ch2_cnt);
		ssd1306_draw_string(&disp, 0, 27, 1, buf);

		sprintf(buf, "OTS:%lu", uptime);
		ssd1306_draw_string(&disp, 0, 36, 1, buf);

		// Read immediately before formatting. The old button-driven path
		// printed a stale global here.
		result = adc_read();

		sprintf(buf, "VCC:%f", result * conversion_factor);
		ssd1306_draw_string(&disp, 0, 45, 1, buf);

		ssd1306_show(&disp);
	}
}

// The `reboot` command goes through the watchdog, so tag it or it would look
// like a fault. This only covers reboots we initiate: an external picotool
// reflash resets the chip outside our control and will still show up as one
// unexplained reset. Unplugging clears it.
#define WDT_INTENTIONAL 0x5245424fu   // 'REBO'

void software_reset() {
	watchdog_hw->scratch[1] = WDT_INTENTIONAL;
	watchdog_enable(1, 1);
	while(1);
}

void eeprom_write_byte (uint16_t addr, uint8_t val) {
	uint8_t src[1 + 1];
	uint32_t flags = save_and_disable_interrupts();

	src[0] = addr & 0xFF;
	src[1] = val;

	// No printf here: this runs with interrupts disabled, and the original
	// calls passed no argument for %s, so any bus error dereferenced a junk
	// pointer. Count the failure and let the main loop report it.
	if (i2c_write_timeout_us(i2c_default, 0x50, src, 2, false, 10000) < 0) eeprom_err++;
	restore_interrupts(flags);

	sleep_ms(5);
}

void eeprom_write_word (uint16_t addr, uint16_t val) {
	uint8_t src[1 + 2];
	uint32_t flags = save_and_disable_interrupts();

	src[0] = addr & 0xFF;
	src[1] = val  & 0xFF;
	src[2] = val >> 8;

	// No printf here: this runs with interrupts disabled, and the original
	// calls passed no argument for %s, so any bus error dereferenced a junk
	// pointer. Count the failure and let the main loop report it.
	if (i2c_write_timeout_us(i2c_default, 0x50, src, 3, false, 10000) < 0) eeprom_err++;
	restore_interrupts(flags);

	sleep_ms(5);
}

void eeprom_write_long (uint16_t addr, uint32_t val) {
	uint8_t src[1 + 4];
	uint32_t flags = save_and_disable_interrupts();

	src[0] = addr & 0xFF;
	src[1] = val  & 0xFF;
	src[2] = (val >>  8) & 0xFF;
	src[3] = (val >> 16) & 0xFF;
	src[4] = (val >> 24) & 0xFF;

	// No printf here: this runs with interrupts disabled, and the original
	// calls passed no argument for %s, so any bus error dereferenced a junk
	// pointer. Count the failure and let the main loop report it.
	if (i2c_write_timeout_us(i2c_default, 0x50, src, 5, false, 10000) < 0) eeprom_err++;
	restore_interrupts(flags);

	sleep_ms(5);
}

int eeprom_read_byte(int addr, uint8_t *p)
{
	uint8_t src;
    uint8_t rx;
    int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_timeout_us(i2c_default, 0x50, &src, 1, true, 10000);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

    ret = i2c_read_timeout_us(i2c_default, 0x50, &rx, 1, false, 10000);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

	*p = rx;

	restore_interrupts(flags);

	return PICO_OK;
}

int eeprom_read_word(int addr, uint16_t *p)
{
	uint8_t src;
    uint8_t rx[2];
    int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_timeout_us(i2c_default, 0x50, &src, 1, true, 10000);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

    ret = i2c_read_timeout_us(i2c_default, 0x50, rx, 2, false, 10000);
    if (ret != 2) {
		restore_interrupts(flags);
		return ret;
    }

	*p = *(uint16_t*)rx;

	restore_interrupts(flags);

	return PICO_OK;
}

int eeprom_read_long(int addr, uint32_t *p)
{
	uint8_t src;
    uint8_t rx[4];
    int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_timeout_us(i2c_default, 0x50, &src, 1, true, 10000);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

    ret = i2c_read_timeout_us(i2c_default, 0x50, rx, 4, false, 10000);
    if (ret != 4) {
		restore_interrupts(flags);
		return ret;
    }

	*p = *(uint32_t*)rx;

	restore_interrupts(flags);

	return PICO_OK;
}

int eeprom_read_page(int addr, uint8_t *p, size_t len)
{
	uint8_t src;
	uint8_t rx[16];
	int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_timeout_us(i2c_default, 0x50, &src, 1, true, 10000);
	if (ret != 1) {
		restore_interrupts(flags);
		return ret;
	}

	ret = i2c_read_timeout_us(i2c_default, 0x50, rx, len, false, 10000);
	if (ret < 0 || (size_t)ret != len) {
		restore_interrupts(flags);
		return ret;
	}

	memcpy(p, rx, len);

	restore_interrupts(flags);

	return PICO_OK;
}

void SerCmdExec(void) {
	char* ptr;
	char buf[24];

	// Match commands exactly, not by prefix. The fixed-length memcmp this
	// replaced also accepted any longer line that merely started with a
	// command, so `saved` ran `save`, `reboots` rebooted, `version` printed
	// `ver`, and -- worse -- a garbage line beginning "dfu" arriving on the
	// UART RX pin dropped the board into the USB bootloader (reset_usb_boot),
	// taking a deployed instrument offline until a physical reflash. Note the
	// hazard is trailing junk, not near-misses: `factory` never matched
	// `factry`, since the two differ at the fifth character.
	// CmdBuf is NUL-terminated
	// on every path that reaches here (SerCmdProc writes CmdBuf[cptr]=0 on CR,
	// and the over-length branch resets without dispatching), so strcmp is
	// safe and requires the command to end exactly where it should.
	//
	// `set ` stays a prefix match: it alone carries a key=value tail. With
	// exact matching everywhere else the dispatch order no longer matters, so
	// a plain strcmp chain says all there is to say without a table.
	if (strcmp((char*)CmdBuf, "stop") == 0) {
		// Last command wins: one strb, no read-modify-write. The main loop
		// owns sout/evt_mode and all PIO/DMA work, which must not run here.
		out_req = OUT_QUIET;
	} else
	if (strcmp((char*)CmdBuf, "go") == 0) {
		out_req = OUT_CPM;
	} else
	if (strcmp((char*)CmdBuf, "evt") == 0) {
		out_req = OUT_EVT;
	} else
	if (memcmp((char*)CmdBuf, "set ", 4) == 0) {
		ptr = (char*)CmdBuf + 4;
		// All three of these land in a uint16_t, and atoi returns int, so an
		// out-of-range entry used to wrap silently: `set atc=99999` stored
		// 34463 and reported it back as if accepted. Clamp instead, so the
		// value read back is the value in force.
		if (memcmp(ptr, "gms=", 4) == 0) {
			// Also the divisor on the dose line: a zero would render inf or
			// nan as a radiation reading.
			long v = atol(ptr + 4);
			if (v < 1)     v = 1;
			if (v > 65535) v = 65535;
			gamma_sensitivity = (uint16_t)v;
		} else
		if (memcmp(ptr, "atc=", 4) == 0) {
			long v = atol(ptr + 4);
			if (v < 0)     v = 0;
			if (v > 65535) v = 65535;
			alarm_trigger_cpm = (uint16_t)v;
		} else
		if (memcmp(ptr, "hvg=", 4) == 0) {
			// Bounded to the range the slice can actually express: 1..wrap.
			//
			// Above the wrap the comparator never trips and the channel
			// saturates, so a larger number does not mean a larger pulse; it
			// means the reported setting stops describing what drives the
			// tube. 0 is excluded because the boot guard treats it as the
			// marker for an unreadable cell, and honouring it here would make
			// that validation bypassable.
			//
			// No claim is made about which end of 1..2047 is safer. The
			// obvious reading of the inverted polarity turned out to be
			// wrong on this hardware -- 2047 was measured still counting
			// normally -- so within the expressible range the operator's
			// value stands as given.
			long v = atol(ptr + 4);
			if (v < 1)    v = 1;
			if (v > 2047) v = 2047;
			hvg_pulsewidth = (uint16_t)v;
			// The operator has asserted a value, so it is savable again, and
			// this is the way back from a generator parked at boot.
			hvg_trusted = true;
			pwm_set_chan_level(hvgPwmSlice, PWM_CHAN_A, hvg_pulsewidth);
			pwm_set_enabled(hvgPwmSlice, true);
		} else
		if (memcmp(ptr, "bps=", 4) == 0) {
			uint32_t bps;
			ptr += 4;
			bps = atol(ptr);
			uart_set_baudrate(uart1, bps);
		} else
		if (memcmp(ptr, "cfg=", 4) == 0) {
			ptr += 4;
			tone = atoi(ptr);
			pwm_set_clkdiv(bzsPwmSlice, tone);
		} else
		if (memcmp(ptr, "snd=", 4) == 0) {
			ptr += 4;
			// Exact on/off only: the old memcmp(ptr,"on",2) also accepted
			// `snd=onx` and silently enabled sound. An unrecognised value is
			// left to fall through and do nothing, as before.
			if (strcmp(ptr, "on") == 0) {
				is_sound_enabled = true;
			} else
			if (strcmp(ptr, "off") == 0) {
				is_sound_enabled = false;
			}
		} else
		if (memcmp(ptr, "dpd=", 4) == 0) {
			ptr += 4;
			if (strcmp(ptr, "on") == 0) {
				ssd1306_poweron(&disp);
			} else
			if (strcmp(ptr, "off") == 0) {
				ssd1306_poweroff(&disp);
			}
		} else
		if (memcmp(ptr, "nsg=", 4) == 0) {
			ptr += 4;
			if (strcmp(ptr, "on") == 0) {
				nsg_enabled = true;
			} else
			if (strcmp(ptr, "off") == 0) {
				nsg_enabled = false;
				nsg_flag = false;
			}
		} else
		if (memcmp(ptr, "dsp=", 4) == 0) {
			// The screen could previously only be changed by the on-board
			// button, which is no use on a board that lives in a rack, and
			// left the persistence path untestable without physical access.
			int v = atoi(ptr + 4);
			if (v >= 0 && v <= MODEMAX) {
				dispMode = (uint8_t)v;
				eeprom_write_byte(OFS_DISPMODE, dispMode);
				pd = true;
			}
		}
	} else
	if (strcmp((char*)CmdBuf, "save") == 0) {
		eeprom_write_word(OFS_GMS, gamma_sensitivity);
		eeprom_write_word(OFS_ALARM, alarm_trigger_cpm);
		// Never write the parked placeholder back: that would bake it into
		// storage as if it were calibration, and the next boot would trust
		// it. An explicit `set hvg` asserts a real value and makes it
		// savable again.
		if (hvg_trusted) eeprom_write_word(OFS_PWM_WIDTH, hvg_pulsewidth);
		eeprom_write_byte(OFS_SOUND, (is_sound_enabled == true)? 1 : 0);
		// The EEPROM has always had a slot for the selected screen and
		// nothing ever wrote it, so the choice was lost on every power cycle.
		eeprom_write_byte(OFS_DISPMODE, dispMode);
	} else
	if (strcmp((char*)CmdBuf, "show") == 0) {
		printf("ttc: %lu\r\n", (unsigned long)total_cnt);
		printf("gms: %u\r\n", gamma_sensitivity);
		printf("atc: %u\r\n", alarm_trigger_cpm);
		printf("hvg: %u\r\n", hvg_pulsewidth);
		// 1 = the pulse width came from valid storage or an explicit set.
		// 0 = storage was unusable, the generator is parked at the
		// minimum-energy end, and hvg above is a placeholder, not
		// calibration.
		printf("hvs: %u\r\n", hvg_trusted ? 1u : 0u);
		printf("dsp: %u\r\n", (unsigned)dispMode);
		// Every other persisted setting was reportable and this one was not,
		// so the mute state could only be confirmed by listening to it.
		printf("snd: %u\r\n", is_sound_enabled ? 1u : 0u);
		printf("alm: %u\r\n", alarm_on ? 1u : 0u);
		// enabled, length of the current silent run, window it is judged against
		printf("nsg: %u %lu %lu\r\n", nsg_enabled ? 1u : 0u,
		       (unsigned long)silent_secs, (unsigned long)nsg_window_s);
		// Background floor this board has recorded, as operator context. It
		// does not size the window above -- that comes from the current
		// session only.
		printf("bas: %u.%u\r\n", baseline_saved_x10 / 10u,
		       baseline_saved_x10 % 10u);
		printf("tsl: %lu %lu\r\n", (unsigned long)ts_lost[0], (unsigned long)ts_lost[1]);
		printf("eer: %lu\r\n", (unsigned long)eeprom_err);
		printf("ots: %lu\r\n", (unsigned long)uptime);
		printf("wdt: %lu\r\n", (unsigned long)wdt_reboots);
	} else
	if (strcmp((char*)CmdBuf, "reboot") == 0) {
		software_reset();
	} else
	if (strcmp((char*)CmdBuf, "factry") == 0) {
		// Operator preferences only.
		//
		// gms and hvg are per-board calibration: gms is this tube's
		// counts-per-uSv/h, hvg sets the tube's supply voltage. This
		// firmware holds no record of their factory values, so restoring
		// them would mean inventing numbers -- and an invented hvg
		// mis-biases the detector. They are left alone, and the command
		// reports which side of that line each setting fell on rather than
		// leaving the operator to guess how much was reset.
		alarm_trigger_cpm = 0;
		is_sound_enabled = true;
		dispMode = 0;
		nsg_enabled = true;
		nsg_flag = false;
		baseline_x10 = 0;
		baseline_saved_x10 = 0;
		min_avrg = 1000.00f;
		max_avrg = 0.00f;

		eeprom_write_word(OFS_ALARM, 0);
		eeprom_write_byte(OFS_SOUND, 1);
		eeprom_write_byte(OFS_DISPMODE, 0);
		eeprom_write_word(OFS_BASECPM, 0);

		pd = true;

		printf("reset: atc snd dsp bas\r\n");
		printf("kept: gms=%u hvg=%u (per-board calibration)\r\n",
		       gamma_sensitivity, hvg_pulsewidth);
	} else
	if (strcmp((char*)CmdBuf, "dfu") == 0) {
		ssd1306_clear(&disp);
		sprintf(buf, "DFU MODE");
		ssd1306_draw_string(&disp, 0, 0, 2, buf);
		ssd1306_show(&disp);

		reset_usb_boot(0, 0);
	} else
	if (strcmp((char*)CmdBuf, "ver") == 0) {
		uint8_t pmn[8 + 1];   // room for a terminator: the field may fill all 8
		memset(pmn, 0, sizeof(pmn));
		uint8_t rev[2];

		eeprom_read_page(OFS_MODELNAME, pmn, 8);   // pmn[8] stays NUL
		eeprom_read_word(OFS_BDREV, (uint16_t*)rev);

		printf("%s\n", pmn);
		printf("%02X%02X\n", rev[1],rev[0]);

		printf("%s\r\n", FW_VERSION);
	}
}

void lineart(void) {

	//60 67
	ssd1306_draw_line(&disp, 60, 0, 60, 63);
	ssd1306_draw_line(&disp, 67, 0, 67, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//56 71
	ssd1306_draw_line(&disp, 56, 0, 56, 63);
	ssd1306_draw_line(&disp, 71, 0, 71, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//y 28, 35  
	ssd1306_draw_line(&disp, 0, 28, 127, 28);
	ssd1306_draw_line(&disp, 0, 35, 127, 35);
	ssd1306_show(&disp);
	sleep_ms(2);

	//52 75
	ssd1306_draw_line(&disp, 52, 0, 52, 63);
	ssd1306_draw_line(&disp, 75, 0, 75, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//48 79
	ssd1306_draw_line(&disp, 48, 0, 48, 63);
	ssd1306_draw_line(&disp, 79, 0, 79, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//y 24  39
	ssd1306_draw_line(&disp, 0, 24, 127, 24);
	ssd1306_draw_line(&disp, 0, 39, 127, 39);
	ssd1306_show(&disp);
	sleep_ms(2);

	//44 83
	ssd1306_draw_line(&disp, 44, 0, 44, 63);
	ssd1306_draw_line(&disp, 83, 0, 83, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//40 87
	ssd1306_draw_line(&disp, 40, 0, 40, 63);
	ssd1306_draw_line(&disp, 87, 0, 87, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 20 43
	ssd1306_draw_line(&disp, 0, 20, 127, 20);
	ssd1306_draw_line(&disp, 0, 43, 127, 43);
	ssd1306_show(&disp);
	sleep_ms(2);

	//36 91
	ssd1306_draw_line(&disp, 36, 0, 36, 63);
	ssd1306_draw_line(&disp, 91, 0, 91, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//28 99
	ssd1306_draw_line(&disp, 28, 0, 28, 63);
	ssd1306_draw_line(&disp, 99, 0, 99, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//24 103
	ssd1306_draw_line(&disp, 24, 0, 24, 63);
	ssd1306_draw_line(&disp, 103, 0, 103, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 12 51
	ssd1306_draw_line(&disp, 0, 12, 127, 12);
	ssd1306_draw_line(&disp, 0, 51, 127, 51);
	ssd1306_show(&disp);
	sleep_ms(2);

	//20 107
	ssd1306_draw_line(&disp, 20, 0, 20, 63);
	ssd1306_draw_line(&disp, 107, 0, 107, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//16 111
	ssd1306_draw_line(&disp, 16, 0, 16, 63);
	ssd1306_draw_line(&disp, 111, 0, 111, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 8 55
	ssd1306_draw_line(&disp, 0, 8, 127, 8);
	ssd1306_draw_line(&disp, 0, 55, 127, 55);
	ssd1306_show(&disp);
	sleep_ms(2);

	//12 115
	ssd1306_draw_line(&disp, 12, 0, 12, 63);
	ssd1306_draw_line(&disp, 115, 0, 115, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//8 119
	ssd1306_draw_line(&disp, 8, 0, 8, 63);
	ssd1306_draw_line(&disp, 119, 0, 119, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 4 59
	ssd1306_draw_line(&disp, 0, 4, 127, 4);
	ssd1306_draw_line(&disp, 0, 59, 127, 59);
	ssd1306_show(&disp);
	sleep_ms(2);

	//4 123
	ssd1306_draw_line(&disp, 4, 0, 4, 63);
	ssd1306_draw_line(&disp, 123, 0, 123, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//0 127
	ssd1306_draw_line(&disp, 0, 0, 0, 63);
	ssd1306_draw_line(&disp, 127, 0, 127, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 0 63
	ssd1306_draw_line(&disp, 0, 0, 127, 0);
	ssd1306_draw_line(&disp, 0, 63, 127, 63);
	ssd1306_show(&disp);
}

void title() {
	ssd1306_clear(&disp);

	lineart();
	sleep_ms(180);

	ssd1306_clear(&disp);

	ssd1306_draw_string(&disp, 6, 0, 1, "NetIO Devices");

	ssd1306_bmp_show_image(&disp, gc10nx_bmp, gc10nx_bmp_len);

	if (isDual) {
		ssd1306_draw_string(&disp, 80, 20, 1, "<Dual>");
	}

	ssd1306_show(&disp);


	sleep_ms(220);

	ssd1306_draw_string(&disp, 6, 54, 1, "FW:");
	ssd1306_draw_string(&disp, 30, 54, 1, FW_VERSION);

	ssd1306_show(&disp);
	sleep_ms(740);

	ssd1306_clear(&disp);
}

int main() {
	static repeating_timer_t msecTimer;
	static repeating_timer_t secTimer;

	set_sys_clock_48mhz();

	if (!watchdog_caused_reboot()) {
		wdt_reboots = 0;                       // power-on clears the history
	} else if (watchdog_hw->scratch[1] == WDT_INTENTIONAL) {
		wdt_reboots = watchdog_hw->scratch[0]; // `reboot`/reflash: not a fault
	} else {
		wdt_reboots = watchdog_hw->scratch[0] + 1;
	}
	watchdog_hw->scratch[0] = wdt_reboots;
	watchdog_hw->scratch[1] = 0;

	stdio_init_all();

	// Armed before any I2C. The EEPROM and OLED helpers block, and the EEPROM
	// ones do so with interrupts disabled, so a stuck bus during init would hang
	// forever with no way back except physically holding BOOTSEL. The window is
	// generous because init legitimately takes a while; the main loop tightens
	// it to 3 s once running.
	watchdog_enable(8000, 1);

	// Futer Use (battery)
	adc_init();
	adc_gpio_init(29);
	adc_select_input(3);

	sleep_ms(400);

	i2c_init(i2c0, 400000); // FOR EEPROM
	gpio_set_function(20, GPIO_FUNC_I2C);
	gpio_set_function(21, GPIO_FUNC_I2C);
	gpio_pull_up(20);
	gpio_pull_up(21);
	bi_decl(bi_2pins_with_func(20, 21, GPIO_FUNC_I2C));

	// UART. The RX interrupt only fills urx_buf; commands are parsed in the main
	// loop, so the printf echo stays out of interrupt context and both transports
	// feed one parser instead of racing on CmdBuf.
	//uart_set_baudrate(uart1, 9600);
	irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
	irq_set_enabled(UART1_IRQ, true);
	uart_set_irq_enables(uart1, true, false);

	// OLED
	i2c_init(i2c1, 400000);
	gpio_set_function(14, GPIO_FUNC_I2C);
	gpio_set_function(15, GPIO_FUNC_I2C);
	gpio_pull_up(14);
	gpio_pull_up(15);
	bi_decl(bi_2pins_with_func(14, 15, GPIO_FUNC_I2C));
	disp.external_vcc = false;
	ssd1306_init(&disp, 128, 64, 0x3C, i2c1);

	watchdog_update();
	// eeprom_read_page fills exactly 8 bytes and guarantees no terminator, so a
	// string compare could run off the end if the field is corrupt or padded.
	// Compare a fixed length instead, and require the field to end there.
	uint8_t mn[8];
	memset(mn, 0, sizeof(mn));
	eeprom_read_page(OFS_MODELNAME, mn, sizeof(mn));
	if (memcmp(mn, "GC10nxd", 7) == 0 && (mn[7] == '\0' || mn[7] == 0xFF)) {
		isDual = true;
	}

	watchdog_update();
	title();

	uint8_t soundMode;
	eeprom_read_byte(OFS_SOUND, &soundMode);
	is_sound_enabled = (soundMode & 0x01) ? true : false;

	// Validate before this ever reaches the PWM.
	//
	// hvg_pulsewidth is a global initialised to 0, and eeprom_read_word
	// leaves its target untouched when the I2C transfer fails, so a stuck bus
	// left 0 here and sent it straight to the generator. An erased cell
	// (0xFFFF) exceeds the 2047 wrap, where the comparator never trips and
	// the channel saturates. Neither is a calibration and neither should be
	// applied.
	//
	// What this deliberately does NOT do is substitute a "safe" level. An
	// earlier version of this guard parked the slice at 2047 on the reasoning
	// that inverted polarity makes high levels the low-duty end. Measurement
	// disproved it: driven at 2047 the tube kept counting at its normal rate
	// for 80 s, so that level is not the quiet end and the substitution was
	// picking an unknown bias.
	//
	// Switching is disabled instead. With the slice stopped the converter
	// cannot pump at all, which is the low-energy state regardless of which
	// polarity or duty convention the hardware uses -- no assumption needed.
	// The board then reads zero counts, which the no-signal supervision
	// reports, and `set hvg` re-enables the generator.
	eeprom_read_word(OFS_PWM_WIDTH, &hvg_pulsewidth);
	if (hvg_pulsewidth == 0 || hvg_pulsewidth > 2047) {
		hvg_trusted = false;   // value kept as-is so `show` reveals what was stored
	}

	gpio_init(LED_PIN);		// DETECTION INDICATOR LED
	gpio_set_dir(LED_PIN, GPIO_OUT);

	gpio_set_function(10, GPIO_FUNC_PWM); // GPIO10
	gpio_set_function(16, GPIO_FUNC_PWM); // GPIO16

	// HV gen pulse
	hvgPwmSlice = pwm_gpio_to_slice_num(10);	// GPIO10 for HV gen pulse
	pwm_set_clkdiv(hvgPwmSlice, 6);
	pwm_set_wrap(hvgPwmSlice, 2047);
	pwm_set_chan_level(hvgPwmSlice, PWM_CHAN_A, hvg_trusted ? hvg_pulsewidth : 0);
	pwm_set_output_polarity(hvgPwmSlice, true, true);
	// Not enabled when the stored width was unusable: see above.
	pwm_set_enabled(hvgPwmSlice, hvg_trusted);

	// Buzzer
	bzsPwmSlice = pwm_gpio_to_slice_num(16);	// GPIO16 for BZ
	pwm_set_clkdiv(bzsPwmSlice, 25);
	pwm_set_wrap(bzsPwmSlice, 999);
	pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, 0);
	pwm_set_enabled(bzsPwmSlice, true);

	// DETECTION
	gpio_init(22);			// DETECTION PULSE INT CH1
	gpio_set_dir(22, GPIO_IN);
	gpio_pull_up(22);

	gpio_set_irq_enabled_with_callback(22, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

	gpio_init(2);			// DETECTION PULSE INT CH2
	gpio_set_dir(2, GPIO_IN);
	gpio_pull_up(2);

	// Only the Dual has a tube on CH2. On a single-tube board GP2 is an
	// unconnected header pin held by a weak internal pull-up, so enabling its
	// edge interrupt just invites noise into the count.
	if (isDual) {
		gpio_set_irq_enabled(2, GPIO_IRQ_EDGE_FALL, true);
	}

	// No interrupt priority juggling. The 1 Hz callback used to blit the whole
	// OLED framebuffer over I2C from inside the timer interrupt, blocking the
	// detection interrupt for ~23 ms and losing every edge but one in that
	// window. That work now runs in the main loop, so the alarm interrupt is
	// short and the detector needs no special priority to be serviced.
	add_repeating_timer_ms(-1, &msecTimer_callback, NULL, &msecTimer);
	add_repeating_timer_ms(-1000, &secTimer_callback, NULL, &secTimer);

	// LOAD EEPROM
	// Stage through plain locals: these globals are volatile because IRQs read
	// them, and eeprom_read_* takes a non-volatile pointer.
	//
	// Seed each local from its global. eeprom_read_* returns without writing
	// through the pointer when the I2C transfer fails, so an uninitialised
	// local would leave stack garbage in the global. dispMode in particular
	// must stay within MODEMAX: drawScreen tests it against each mode in turn
	// and draws nothing at all if none match, freezing the display.
	uint16_t gms_init = gamma_sensitivity;
	uint16_t alarm_init = alarm_trigger_cpm;
	uint8_t  dispmode_init = dispMode;
	uint16_t base_init = baseline_saved_x10;
	eeprom_read_word(OFS_GMS, &gms_init);
	eeprom_read_word(OFS_ALARM, &alarm_init);
	eeprom_read_byte(OFS_DISPMODE, &dispmode_init);
	eeprom_read_word(OFS_BASECPM, &base_init);
	// A blank cell reads as 0xFFFF, which as a background would be nonsense
	// and would size the silence window off a fictional 6553 CPM. Treat it,
	// and anything implausibly high for a background, as "not learned yet".
	if (base_init == 0xFFFF || base_init > 20000) base_init = 0;
	gamma_sensitivity = gms_init ? gms_init : 1;
	alarm_trigger_cpm = alarm_init;
	dispMode = (dispmode_init > MODEMAX) ? 0 : dispmode_init;
	baseline_saved_x10 = base_init;
	baseline_x10 = base_init;

	cmax = 0;
	cptr = 0;
	uptime = 0;

	int c;

	// Tighten the window now that init is done. The loop period is 25 ms plus a
	// display blit, so 3 s is far more headroom than any iteration needs.
	watchdog_enable(3000, 1);

	while(1) {

		if (get_bootsel_button()) {
			if (is_button_pressed) {
			}
			lp++;
			is_button_pressed = true;
		} else {
			if (lp > 20) {
				if (is_sound_enabled == true) {
					is_sound_enabled = false;
					eeprom_write_byte(OFS_SOUND, 0);
					lp = 0;
				} else {
					is_sound_enabled = true;
					eeprom_write_byte(OFS_SOUND, 1);
					lp = 0;
				}
			}

			if (lp <= 20 &&  lp > 0) {
				dispMode++;
				if (dispMode > MODEMAX) {
					dispMode = 0;
				}
				// Persist the choice. Button-driven, so this is rare enough
				// that the ~10 ms interrupts-off window in the write costs at
				// most a single count, and it matches what the long press
				// already does for the mute setting.
				eeprom_write_byte(OFS_DISPMODE, dispMode);
				pd = true;
			}

			is_button_pressed = false;
			lp = 0;
		}

		sleep_ms(25);

		// Reconcile output mode from one snapshot of the requested state, so a
		// command arriving mid-reconcile is applied whole on the next pass
		// rather than half-applied now. Converges within one loop iteration.
		out_mode_t want = out_req;

		if (want == OUT_EVT && !evt_mode) {
			// No banner: the downstream consumer rejects any line that is not
			// exactly three whitespace-separated tokens, so a header would be
			// dead weight at best. The wire format is unchanged from v1.
			ts_start();
		}

		sout     = (want == OUT_CPM);
		evt_mode = (want == OUT_EVT);
		// Deferred display work. Both paths blit the framebuffer over I2C and
		// block for ~23 ms, so they run here with interrupts enabled rather
		// than inside the timer callback that used to invoke them.
		//
		// A mode change only redraws; the 1 Hz tick samples first, because
		// the graph queue and SMA ring advance once per second and must not
		// be fast-forwarded by a button press.
		if (ui_refresh_req) {
			ui_refresh_req = false;
			sampleDisplay();
			pd = true;
		}
		if (pd) {
			pd = false;
			drawScreen();
		}
		if (cpm_out_pending) {
			cpm_out_pending = false;
			if (sout) printf("%lu\n", (unsigned long)cpm_out_value);
		}

		// Learned background, written here and never from the timer: the
		// EEPROM helpers hold interrupts off for up to 10 ms, which at these
		// rates loses a count. The deadband upstream keeps this to a handful
		// of writes over the life of the board.
		if (baseline_save_req) {
			baseline_save_req = false;
			eeprom_write_word(OFS_BASECPM, baseline_x10);
			baseline_saved_x10 = baseline_x10;
		}

		if (evt_mode) {
			ts_drain();
		}

		// Commands from both transports are parsed here, in thread context.
		// Previously UART bytes were parsed inside UART1_IRQ, which put printf
		// in an interrupt and let two transports write CmdBuf concurrently.
		while (urx_tail != urx_head) {
			uint8_t ch = urx_buf[urx_tail];
			urx_tail = (urx_tail + 1u) & (URX_LEN - 1u);
			SerCmdProc(ch);
		}
		if (tud_cdc_connected()) {
			while ((c = tud_cdc_read_char()) != -1) {
				SerCmdProc(c);
			}
		}

		watchdog_update();
	}

	return 0;
}
