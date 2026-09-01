# Wireless adapter for the Logitech G920

**This replaces the G920's USB cable with an ESP-NOW radio link.**

Two ESP32-S3 boards do the work. One stays with the wheel and reads it over
USB. The other plugs into the console or PC, where it looks like a real
G920. Force feedback travels back over the same link, so the wheel still
pushes against your hands.

You never open or modify the wheel. It connects with its own USB cable.

```
   Logitech G920 ──USB──▶ [ TX: ESP32-S3 ]  ))) ESP-NOW (((  [ RX: dongle ] ──USB──▶ Xbox / PC
                           USB Host                            USB Device
```

## Status

**It works.** The console accepts it as a real wheel, the Xbox security
check passes, and you can play with the wheel over the radio.

| Piece | State |
|---|---|
| Reading the wheel: connection, metadata, force feedback | done |
| The radio link itself | done — 250 updates/s with no losses, ~1.2 ms one way |
| Cloning the wheel's identity onto the dongle | done |
| Xbox security check (passed through to the wheel) | done |
| Dongle with screen and button | written, **not yet tested on real hardware** |
| Fail-safe: release the forces if the link breaks | not done |

Delay: our own code adds almost nothing — about 0.1 % of the total. Nearly
all of it is USB polling (1–4 ms on each side) and the radio (~1.2 ms).
Details in [`docs/LATENCY.md`](docs/LATENCY.md).

## Hardware

| Role | Board | Why |
|---|---|---|
| **TX** — at the wheel | ESP32-S3 (Supermini / DevKitC-1) | USB Host, small enough to tape to the wheel |
| **RX** — at the console | Pocket-Dongle-S3 / LilyGo T-Dongle-S3 | USB-stick form factor, screen, plugs straight into the console |

**One board can only do one job.** The ESP32-S3 has a single USB controller:
it can be a host or a device, never both, and it cannot switch while
running. That is how the chip is built — no amount of code gets around it.

### Wiring: TX to the wheel

The wheel's stock USB-A plug goes into a USB-A **female** socket that you
wire to the ESP32-S3:

```
        Logitech G920                       ESP32-S3 (TX)
        stock USB-A cable
              │
              ▼
     ┌─────────────────┐
     │  USB-A female   │
     │                 │
     │  1  VBUS  ──────┼──── +5 V through a high-side switch ◀── GPIO21
     │  2  D−    ──────┼──────────────────────────────────────── GPIO19
     │  3  D+    ──────┼──────────────────────────────────────── GPIO20
     │  4  GND   ──────┼──────────────────────────────────────── GND
     └─────────────────┘
              │
        1 kΩ bleeder from pin 1 to pin 4, right at the socket
```

**The D+ and D− pins cannot be changed.** GPIO19 is D−, GPIO20 is D+. They
run straight into the chip's USB hardware, so no other pins will work.

Wire colours inside USB cables (red/white/green/black) are only a
convention, not a guarantee. **Check them with a multimeter.**

### Wiring: power

The wheel runs from its own 24 V brick. The ESP32-S3 and the VBUS it feeds
to the wheel come from that same 24 V through an isolated DC-DC converter:

```
   G920 24 V PSU ──┬──▶ the wheel itself
                   │
                   └──▶ URB2405YMD (24 V → 5 V) ──┬──▶ ESP32-S3  5V pin
                                                  │
                                                  └──▶ high-side switch ──▶ socket pin 1 (VBUS)

   Star ground: DC-DC 5 V−, ESP32-S3 GND and socket pin 4 all meet at ONE point.
```

Three things that are easy to get wrong:

- **The 5 V on VBUS comes from the adapter, not from the wheel.** A device
  with its own power supply stays silent until it sees 5 V on VBUS — that
  is how it knows something was plugged into it.
- **Do not take that 5 V from the board's own `5V` pin.** On most dev boards
  that pin sits behind a diode, and feeding power back into it pushes
  current into the board's regulator from the wrong side.
- **Both sides must share a ground.** D+ and D− are measured against pin 4,
  so the wheel and the board need the same reference point. An isolated
  DC-DC converter does not change this: inside the wheel, the USB ground is
  already tied to the 24 V ground, so the isolation is bypassed through the
  wheel anyway. Either accept the shared ground, or add a real USB isolator
  (ADuM4160) on the data lines — an isolated power supply alone cannot do
  it.

### The VBUS switch

The firmware switches this 5 V itself, so it can power-cycle the wheel with
nobody touching anything (`-DG920_VBUS_GPIO=21`). The switch is an
optocoupler driving a PNP transistor:

```
              +5 V ──┬──────────────────── E ┐
                     │                       │  2N3906 (PNP)
                  [10 kΩ]                    │
                     ├───[1 kΩ]────┐     B ──┤
                     │             │         └── C ──▶ socket pin 1 (VBUS)
                     │      opto collector (4)
                     │      opto emitter  (3) ── GND
                     │
   GPIO21 ──[220 Ω]──▶ opto anode (1);   opto cathode (2) ── GND

   Plus a 1 kΩ bleeder from VBUS to GND at the socket.
```

GPIO21 high → the PNP conducts → the wheel gets a clean 5 V.
GPIO21 low or the board still booting → the wheel is unpowered.

Two things we learned the hard way:

- **An optocoupler on its own is not enough.** If the current flows through
  the optocoupler itself, it loses more than a volt (measured: 3.4–3.9 V
  instead of 5 V), and the wheel never sees enough voltage to wake up. Its
  job is to switch the transistor, not to carry the load.
- **Do not skip the 1 kΩ resistor.** The wheel draws only microamps from
  VBUS, so with nothing to drain the line, switching off does not actually
  turn it off: the voltage sits near 5 V for tens of seconds.

### RX: the dongle

Plugs straight into the console's USB port and is powered from it. No
wiring at all. On a T-Dongle-S3-class board the screen shows link quality,
wheel and console state, input rate and packet loss; the single button
steps the backlight (including off) and, held for 3 s, reboots into flash
mode.

## Building and flashing

[PlatformIO](https://platformio.org/) with the ESP-IDF framework.

```bash
# transmitter (at the wheel)
pio run -d firmware/tx -e tx-host   -t upload    # with logs, for bring-up
pio run -d firmware/tx -e tx-prod   -t upload    # silent, for actual use

# receiver (dongle)
pio run -d firmware/rx -e rx-dongle-gip -t upload   # with logs
pio run -d firmware/rx -e rx-prod       -t upload   # silent

# host-side unit tests, no hardware needed
pio test -d firmware/native
```

Flashing the dongle takes one extra step, and there is no way around it:
the board has a single USB port, and the firmware is using it to act as the
wheel. Either **hold the button for 3 seconds** and the running firmware
will reboot into the bootloader, or **hold the button while plugging it
in** — that one is handled by the chip's ROM and works even when the
firmware does not start at all.

## Layout

```
firmware/tx/        transmitter — USB Host, talks to the wheel
firmware/rx/        receiver — USB Device, talks to the console
firmware/common/    shared code: GIP protocol, radio link, identity, board support
firmware/native/    unit tests that run on the host machine
docs/               hardware notes, latency budget, host traces, roadmap
```

The shared code is genuinely shared — one copy, pulled into both firmwares
and into the host test build, so protocol logic is covered by tests that
need no hardware.

## Mistakes that cost us the most time

- **Do not touch GPIO19/20 before the USB stack starts.** We had diagnostic
  code that drove those pins as ordinary GPIOs during boot. Afterwards the
  USB hardware could no longer see a device that was *already plugged in*:
  the wheel was sitting right there holding its data line high, and the
  stack still reported an empty bus, forever. Plugging the wheel in later
  still worked — which is why "it only starts if I touch the connector" and
  "it works when I power it from the laptop" looked like grounding problems
  for weeks. They were not.
- **Never let two parts of the code number messages from the same pool.**
  This caused three separate bugs: the console and the transmitter both
  numbering messages to the wheel, the tunnel and the retry queue sharing a
  frame type, and the idle repeater resending a report with an old number.
- **Logging inside time-critical code changes the timing.** Printing 64
  bytes as hex over a 115200-baud serial port takes about 45 ms. That was
  enough to break the wheel's handshake once and to trip a watchdog another
  time. The production builds leave logging out completely.

## License

MIT for the code in this repository.

Microsoft's official GIP documentation is *not* included — it is partner
material. `docs/gip-official/` is gitignored; get the package yourself from
`aka.ms/gipdocs` if you need it.

Not affiliated with, endorsed by, or supported by Logitech or Microsoft.
