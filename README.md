# Wireless adapter for the Logitech G920

Two ESP32-S3 boards that put a radio link in the middle of a G920's USB
cable. One sits at the wheel and speaks USB **Host** to it; the other plugs
into the console or PC and pretends to **be** the wheel. Force feedback
crosses the link in the other direction, so the wheel still fights back.

The wheel is never opened or modified. Everything goes through its stock
USB-A cable.

```
   Logitech G920 ──USB──▶ [ TX: ESP32-S3 ]  ))) ESP-NOW (((  [ RX: dongle ] ──USB──▶ Xbox / PC
                           USB Host                            USB Device
```

## Status

**It works.** The console accepts the adapter as a genuine wheel, the
security handshake passes, and the wheel is playable over the radio.

| Piece | State |
|---|---|
| Wheel enumeration, metadata, force feedback | done |
| Radio transport (ESP-NOW) | done — 250 Hz with zero loss, ~1.2 ms one way |
| Cloning the wheel's identity onto the dongle | done |
| Xbox security passthrough | done — console accepts the wheel |
| Dongle with screen and button | written, **not yet tested on real hardware** |
| Watchdog / fail-safe (drop forces on link loss) | not done |

Measured latency budget: our own code is ~0.1 % of the path. The rest is
USB polling (1–4 ms each side) and the radio (~1.2 ms). See
[`docs/LATENCY.md`](docs/LATENCY.md).

## Hardware

| Role | Board | Why |
|---|---|---|
| **TX** — at the wheel | ESP32-S3 (Supermini / DevKitC-1) | USB Host, small enough to tape to the wheel |
| **RX** — at the console | Pocket-Dongle-S3 / LilyGo T-Dongle-S3 | USB-stick form factor, screen, plugs straight into the console |

**One board can only have one role.** The ESP32-S3 has a single USB-OTG
controller — Host or Device, never both, and it cannot switch at runtime.
That is silicon, not a software choice.

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

**D+ and D− pins are fixed by the silicon.** GPIO19 is D−, GPIO20 is D+.
They cannot be moved — there is no mux, it is the USB PHY itself.

Wire colours inside USB cables (red/white/green/black) are a convention,
not a guarantee. **Ring them out with a meter.**

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

- **The transmitter supplies VBUS, not the wheel.** A self-powered USB
  device will not start enumerating until it sees 5 V on VBUS — it is
  waiting to be told a host plugged in.
- **Do not feed VBUS from the board's own `5V` pin.** On most dev boards
  that pin sits behind a Schottky diode on the supply rail; back-feeding it
  drives the on-board regulator from its output.
- **The USB ground must be common.** D+/D− are measured against pin 4. An
  isolated DC-DC does *not* remove that requirement: the wheel ties its own
  USB ground to its 24 V ground internally, so the isolation is bridged
  through the wheel regardless. Either accept a common ground, or use a
  real USB isolator (ADuM4160) on the data lines — a DC-DC alone cannot do
  it.

### The VBUS switch

The firmware owns VBUS so it can power-cycle the wheel by itself
(`-DG920_VBUS_GPIO=21`). A high-side switch built from an optocoupler
driving a PNP transistor:

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

Two notes paid for the hard way:

- **An optocoupler alone will not do.** Used as the pass element it drops
  over a volt (measured: 3.4–3.9 V out of 5 V) and the wheel never crosses
  its VBUS threshold. It has to drive a transistor, not the load.
- **The bleeder is not optional.** The wheel draws microamps from VBUS, so
  without a resistor to discharge the node, "off" is not off — the node
  floats near 5 V for tens of seconds.

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

Flashing the dongle is awkward on purpose: it has one USB port and the
firmware takes it over as a device. Either **hold the button for 3 s** to
have the running firmware reboot into the ROM bootloader, or **hold the
button while plugging it in** — the latter is handled by the ROM and works
even when the firmware does not boot.

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

## Lessons that cost the most time

- **Do not touch GPIO19/20 before the USB stack starts.** Pre-stack
  diagnostics that drove those pins as ordinary GPIOs left the PHY unable
  to see a *statically connected* device: the wheel's pull-up was sitting
  right there and the stack reported an empty bus forever. Hot-plugging
  still worked, which is why "it only starts if I touch the connector" and
  "it works when powered from the laptop" both looked like grounding
  problems for weeks. They were not.
- **Two owners of one sequence-number pool is always a bug.** It bit three
  times: the console and the transmitter both numbering messages to the
  wheel, the tunnel and the reliable queue sharing a frame type, and the
  idle repeater re-sending a report with the same GIP sequence ID.
- **Logging in the hot path controls timing.** A 64-byte hexdump at 115200
  baud takes ~45 ms. It broke the wheel handshake once and tripped a
  watchdog another time. Production profiles compile logging out entirely.

## License

MIT for the code in this repository.

Microsoft's official GIP documentation is *not* included — it is partner
material. `docs/gip-official/` is gitignored; get the package yourself from
`aka.ms/gipdocs` if you need it.

Not affiliated with, endorsed by, or supported by Logitech or Microsoft.
