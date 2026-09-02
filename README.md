# GC10next series firmware

Firmware source code for
GC10next and GC10nextDual

## How to build

Two build paths exist. The Docker path is the one that is regularly verified and
is self-contained: it clones its own Pico SDK and needs no local toolchain.

### Docker (verified)

```
docker build -f Dockerfile.build --build-arg FW_BUILD_ID=$(git rev-parse --short=7 HEAD) -o out .
```

This produces `out/gc10next.uf2`.

`FW_BUILD_ID` is **required**. The build container has no `.git`, so CMake cannot
derive the commit itself, and the firmware id carries the commit so a deployed
board is traceable to source. Rather than ship an unlabelled image, the build
fails by design when `FW_BUILD_ID` is missing.

### Native (Pico C SDK)

Prerequisites:

- an `arm-none-eabi` toolchain (`gcc-arm-none-eabi` and the matching newlib), and
- a Pico SDK reachable via `PICO_SDK_PATH`
  (https://www.raspberrypi.com/documentation/pico-sdk/).

The vendored `pico-sdk/` directory is gitignored, so a fresh clone does not
include it; supply your own and point `PICO_SDK_PATH` at it.

```
mkdir build && cd build
cmake -DFW_BUILD_ID=$(git rev-parse --short=7 HEAD) ..
make -j
```

The Docker path currently pins Pico SDK 1.5.1 (see `Dockerfile.build`), which may
be a different major version than a typical vendored `pico-sdk/` checkout, so the
two paths are not guaranteed to produce identical binaries. This is a known open
issue; which SDK version is authoritative has not been decided.

## Flashing

Hold the on-board BOOTSEL button while connecting USB. The board enumerates as an
`RPI-RP2` mass-storage device; copy the `.uf2` onto it and it reboots into the new
firmware. From a running board, the serial command `dfu` achieves the same thing
(see below).

## PreBuild image

`./latestFW/gc10nx_series.uf2` is upstream's stock image, not a build of this
fork. Download and flash it if you want the unmodified upstream firmware.

## Serial console

The board accepts commands on two transports in parallel:

- **USB CDC** — the virtual serial port the board presents over USB.
- **UART1** — GP4 (TX) and GP5 (RX) at 115200 baud
  (`PICO_DEFAULT_UART=1` in `CMakeLists.txt`).

Both feed one command parser. Each command is a single line terminated by
carriage return (CR). The input buffer holds **15 characters**; a longer line is
rejected with `?` and discarded, so no command needs more than that.

Commands are the **exact strings** listed below. Type them in full.

| Command             | Effect                                                                                                       | Does not                                                                            |
| ------------------- | ------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------- |
| `go`                | Stream one smoothed CPM value per second on the serial console. This is the default output mode at power-on. | Affect the display or counting.                                                     |
| `stop`              | Stop the per-second CPM stream.                                                                              | Stop counting; the tube and display keep running.                                   |
| `evt`               | Switch to per-pulse event capture (see **Event capture** below).                                             | Capture CH2, even on the Dual.                                                      |
| `save`              | Persist operator settings to EEPROM (see **Persistence**).                                                   | Persist `nsg`, `cfg`, `bps` or `dpd`.                                               |
| `show`              | Print the full status block (see **`show` field glossary**).                                                 | Change any state.                                                                   |
| `reboot`            | Restart the board through the watchdog. Tagged so it is not counted as an unexplained reset.                 | Erase any stored setting.                                                           |
| `factry`            | Reset operator preferences to defaults (see below).                                                          | Touch `gms` or `hvg` calibration.                                                   |
| `dfu`               | **Destructive to operation.** Drop into the USB bootloader (`RPI-RP2`).                                      | Come back on its own — counting stops until the board is reflashed or power-cycled. |
| `ver`               | Print the stored model name, board revision and firmware version.                                            | Change any state.                                                                   |
| `set <key>=<value>` | Change one setting (see the key table).                                                                      | Persist by itself, except `dsp` — use `save` to keep the rest across a power cycle. |

`set` keys:

| Key   | Value        | Range / form        | Effect                                                                                                                                                                                                                                                                                                |
| ----- | ------------ | ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `gms` | integer      | clamped to 1..65535 | Gamma sensitivity: counts per µSv/h, the divisor for the dose line. Per-board calibration. Zero is excluded because it would render the dose as inf/nan.                                                                                                                                              |
| `atc` | integer      | clamped to 0..65535 | Alarm threshold in CPM. `atc=0` disables the alarm (the shipped default).                                                                                                                                                                                                                             |
| `hvg` | integer      | clamped to 1..2047  | HV-generator pulse width. Per-board calibration. Values above the PWM wrap saturate the channel and stop describing the drive, so they are clamped out; `0` is reserved as the "unreadable cell" marker. A `set hvg` also marks the value trusted and re-enables a generator that was parked at boot. |
| `bps` | integer      | none                | Set the UART1 baud rate. Takes effect immediately; **not persisted**.                                                                                                                                                                                                                                 |
| `cfg` | integer      | none                | Set the buzzer PWM clock divider (click/alarm tone pitch). **Not persisted**.                                                                                                                                                                                                                         |
| `snd` | `on` / `off` | —                   | Enable or disable the per-count click and the alarm buzzer.                                                                                                                                                                                                                                           |
| `dpd` | `on` / `off` | —                   | Power the OLED panel on or off. **Not persisted**.                                                                                                                                                                                                                                                    |
| `nsg` | `on` / `off` | —                   | Enable or disable no-signal supervision. **Not persisted** (defaults on each boot); `off` also clears any current flag.                                                                                                                                                                               |
| `dsp` | integer      | 0..2                | Select the display screen. Written to EEPROM immediately, so this one setting sticks without `save`.                                                                                                                                                                                                  |

`factry` resets `atc`, `snd`, `dsp` and the learned background `bas` to defaults,
and re-enables `nsg`. It deliberately **keeps** `gms` and `hvg`: those are
per-board calibration (`gms` is this tube's counts-per-µSv/h, `hvg` sets the
tube's supply voltage), and the firmware holds no record of their factory values,
so restoring them would mean inventing numbers — and an invented `hvg`
mis-biases the detector. The command prints which settings it reset and which it
kept, so nothing is left to guess.

`dfu` and `factry` are the two commands with lasting consequences. `dfu` halts the
instrument; `factry` wipes operator preferences. Neither can be undone from the
console.

## `show` field glossary

`show` prints one field per line. None of the names is guessable and several are
specific to this fork.

| Field | Meaning                                                                                                                                                                                                                                                                           | Units               |
| ----- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------- |
| `ttc` | Total counts since boot (both channels).                                                                                                                                                                                                                                          | counts              |
| `gms` | Gamma sensitivity / dose divisor.                                                                                                                                                                                                                                                 | counts per µSv/h    |
| `atc` | Alarm threshold; `0` = alarm disabled.                                                                                                                                                                                                                                            | CPM                 |
| `hvg` | HV-generator pulse width. When `hvs` is 0 this is a stored placeholder, not calibration.                                                                                                                                                                                          | PWM level (0..2047) |
| `hvs` | HV-trusted flag. `1` = the pulse width came from valid storage or an explicit `set`. `0` = storage was unusable, so the generator is **parked** (switching disabled, no HV) and `hvg` shows a placeholder rather than calibration.                                                | 0/1                 |
| `dsp` | Currently selected display screen.                                                                                                                                                                                                                                                | 0..2                |
| `snd` | Click/alarm sound enabled.                                                                                                                                                                                                                                                        | 0/1                 |
| `alm` | Alarm currently firing.                                                                                                                                                                                                                                                           | 0/1                 |
| `nsg` | **Three values:** enabled flag, seconds since the last count, and the window the silence is currently judged against.                                                                                                                                                             | 0/1, s, s           |
| `bas` | Background floor this board has learned, printed as tenths (e.g. `bas: 12.3`). Operator context only; it does **not** size the no-signal window.                                                                                                                                  | CPM                 |
| `tsl` | Lost PIO timestamps, **per channel** (CH1 CH2).                                                                                                                                                                                                                                   | counts              |
| `eer` | EEPROM I2C failures since boot.                                                                                                                                                                                                                                                   | count               |
| `ots` | Uptime.                                                                                                                                                                                                                                                                           | seconds             |
| `wdt` | **Unexplained** self-reboots since power-on. Survives a watchdog reset and is cleared only by a power cycle. A `picotool` reflash resets the chip outside firmware control and so shows `1`. Any non-zero value after a clean power-up is a real self-reboot worth investigating. | count               |

## Display screens

The OLED cycles through three screens (short button press, or `set dsp=`).

- **Screen 0 — live rate (`RT`).** The current smoothed CPM, the derived dose in
  µSv/h, and a 96-sample trend graph of recent rate. Settle window 60 s.
- **Screen 1 — moving average (`MA`).** The 300-second moving-average CPM and its
  dose, plus the running MAX and MIN dose (shown as `UD`, undefined, until the
  window has filled). Settle window 300 + 60 s.
- **Screen 2 — info.** `FWV` firmware version, `TCN` total count, `CH1`/`CH2`
  per-channel counts, `OTS` uptime in seconds, `VCC` supply voltage.

A small status tag appears at the top-left of screens 0 and 1. Only the
highest-priority tag is shown, in this order:

1. `[NS]` — no signal: the tube has been silent longer than the supervision
   window (see below).
2. `[!!]` — alarm: the rate is at or above `atc`.
3. `[NV]` — not valid: the averaging window has not filled yet, so the figure on
   screen is not meaningful.

The settle windows are why `[NV]` exists. Screen 0's rate is an average over the
last 60 s and screen 1's is an average over 300 s (with a further 60 s before
MAX/MIN are tracked); before that time has elapsed the average is computed over a
partly empty window and does not yet describe the environment.

## Button (BOOTSEL)

The on-board BOOTSEL button doubles as the user interface while the firmware runs:

- **A brief press** cycles to the next display screen and persists the choice.
- **A sustained press** toggles the click sound and persists that too.

(The two are distinguished by how long the button is held; the exact threshold is
not a fixed wall-clock time and is not specified here.)

Holding BOOTSEL only enters the bootloader when done _at power-on / USB connect_,
as described under **Flashing**.

## Alarm

`set atc=<cpm>` arms a rate alarm; `atc=0` disables it, which is the shipped
default. The threshold is compared against the 60-second rate window, not an
instantaneous count.

The comparison has hysteresis: the alarm asserts at or above `atc` and clears only
once the rate falls below 90% of `atc`. The window is a 60 s average of Poisson
counts, so a bare threshold test would chatter on and off around the setpoint;
clearing at 90% costs nothing and keeps the state stable.

Whether the buzzer sounds during an alarm depends on `snd`: with `snd=off` the
alarm still asserts (and shows `[!!]` and `alm: 1`) but stays silent.

## No-signal supervision

A dead tube or a failed HV supply reads as a confident `0 CPM`, which on the
display is indistinguishable from "all clear" — the dangerous failure mode for an
unattended instrument. No-signal supervision exists to catch exactly that.

Silence is only evidence of a fault in proportion to the count rate the detector
normally sees, so the window is **derived from the count rate this boot has
actually observed**, not a fixed constant. At a normal background a 30-second
silence is already a one-in-a-million event; a shielded or low-background detector
legitimately sees far fewer counts, so its window stretches out (up to about an
hour) instead of crying wolf. The rate used is biased low on purpose, so the
window errs long — the harmless direction.

The persisted background (`bas`) is deliberately **not** an input to the window: a
rate learned in one environment cannot be assumed to describe the one the board
woke up in, and if it were too high the window would come out too short and
fabricate failure reports. It is kept as operator context only.

When the window elapses with no counts, the display shows `[NS]` and `show`
reports it via the `nsg` field. This is a **diagnostic, not a verdict**: with no HV
sense the firmware cannot tell a dead tube from a lead box, so it reports
"implausibly quiet" and leaves the judgement to the operator. `set nsg=off`
disables it; the setting is not persisted and defaults back on at each boot.

## Persistence

`save` writes operator settings to EEPROM. What survives a power cycle:

| Setting | Persisted by `save`?          | Notes                                                                                                                  |
| ------- | ----------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `gms`   | yes                           | Per-board calibration.                                                                                                 |
| `atc`   | yes                           | Alarm threshold.                                                                                                       |
| `hvg`   | yes, **only when `hvs` is 1** | A parked placeholder is deliberately never written, so an unusable value cannot be baked in as if it were calibration. |
| `snd`   | yes                           |                                                                                                                        |
| `dsp`   | yes                           | Also written immediately on `set dsp=` and on a button screen-change, so it sticks without `save`.                     |
| `nsg`   | **no**                        | Defaults on at each boot.                                                                                              |
| `cfg`   | **no**                        | Buzzer tone divider; volatile.                                                                                         |
| `bps`   | **no**                        | UART baud; volatile.                                                                                                   |
| `dpd`   | **no**                        | Display power; volatile.                                                                                               |

The learned background (`bas`) is maintained automatically and stored separately;
`factry` clears it. The four volatile `set` keys are called out explicitly because
a setting that looks like it sticks and does not is worse than one documented as
volatile.

## Event capture (`evt`)

`evt` streams one line per detected pulse, timestamped in hardware.

```
E 2496132 1
E 2617908 1
```

`E <microseconds> <channel>` — exactly three whitespace-separated tokens. The
microsecond value is a 64-bit monotonic counter from the start of capture and
does not wrap. This is byte-identical to the previous event format, because
downstream consumers parse it positionally and reject any line that is not
exactly three fields. Do not add a field without updating them in lockstep.

Capture runs on a PIO state machine (`pulse_ts.pio`) with a DMA channel feeding
a 1024-entry ring. The ARM core is not in the timing path, so timestamps are
unaffected by the 1 Hz OLED refresh, USB traffic, or any other interrupt work.
Every path through the PIO loop is exactly 8 cycles, which makes the counter
linear in real time rather than a clock that stalls while reporting an edge.

Internal resolution is 166.67 ns and the shortest resolvable pulse or gap is
8 cycles (0.167 us), against an SBM-20 dead time of roughly 190 us. The wire
format reports whole microseconds; the extra resolution keeps the microsecond
clock free of accumulated rounding.

`evt` captures CH1 only, including on the GC10nextDual. Enabling CH2 needs two
fixes together, not one: the drain walks a whole ring before starting the next,
so batched output would break the global line ordering consumers take deltas
from; and each channel anchors its own epoch, so even correctly ordered output
would mix two clocks. Either fix alone still yields wrong intervals. CH2 is
still counted for the display.

If the DMA laps the ring, the firmware prints `# overrun ch<N> lost=<count>`
and **stops the stream**. It cannot simply continue: the next line would carry
one fabricated interarrival spanning every lost event, and a consumer reading
intervals from consecutive lines has no way to see the marker. Send `evt` again
to restart; the first event is consumed silently as a new anchor so the next
delta is genuine. `show` reports the running total as `tsl:`.

## ToDo

1. Add support commands for GCXmonitor.
2. Trigger I/O function (for producing quantum cats).
</content>

</invoke>
