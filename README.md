# GC10next series firmware

Firmware source code for
GC10next and GC10nextDual

## How to build
Compile on Pico C SDK( https://www.raspberrypi.com/documentation/pico-sdk/ )

## PreBuild image
Download
./latestFW/gc10nx_series.uf2
and flash it.

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
1. Add tone generator for warning alarm.
2. Add support commands for GCXmonitor.
3. Trigger I/O function (for producing quantum cats).
4. Make a special mode which generates physical random number based on natural radioactive decay.
