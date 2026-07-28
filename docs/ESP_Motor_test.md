# ESP_Motor_test — Motor Test Stand

## Overview

ESP32-based motor test stand controller. Receives commands from Android via Bluetooth Classic SPP (binary protocol). Controls ESC/motor via PWM (LEDC) or DShot (RMT). Supports bidirectional DShot with EDT v1 telemetry (eRPM, voltage, current, temperature).

Default protocol on boot: `ESC_PROTOCOL_PWM`. Can be changed via Bluetooth.

## Stack

ESP-IDF v5.5.3, C, FreeRTOS, RMT TX/RX (DShot), LEDC (PWM), esp_timer, Bluetooth Classic SPP, NVS.

---

## Entry Point

```
app_main()
  → esc_controller_init()
  → esc_protocol_select(ESC_PROTOCOL_PWM)   // default
  → bluetooth_spp_init()
```

`app_main()` does NOT call `esc_dshot_init()` directly — DShot init happens inside `esc_protocol_select()` when a DShot protocol is chosen via Bluetooth.

---

## Bluetooth SPP — Binary Protocol

Android app sends binary packets over Bluetooth Classic SPP. NOT text commands.

### Packet Format (4 bytes)

```
[SOF=0xAA] [CMD] [VALUE] [CRC]
```

CRC = `SOF ^ CMD ^ VALUE` (XOR checksum).

### Packet State Machine (RX)

```
RX_WAIT_SOF → RX_WAIT_COMMAND → RX_WAIT_VALUE → RX_WAIT_CRC
```

### Commands

| CMD byte | Name | Value | Description |
|----------|------|-------|-------------|
| 0x01 | CMD_ARM | — | Allow throttle |
| 0x02 | CMD_DISARM | — | Zero throttle, DISARMED |
| 0x03 | CMD_STOP | — | Zero throttle, stays ARMED |
| 0x04 | CMD_THROTTLE | 0..100 | Set throttle % (after arm) |
| 0x05 | CMD_PROTOCOL | 0..3 | Switch protocol (see below) |
| 0x06 | CMD_ESTOP | — | Emergency stop |
| 0x07 | CMD_RESET_ESTOP | — | Reset emergency stop |

### Protocol Values (CMD_PROTOCOL value byte)

| Value | Protocol |
|-------|----------|
| 0 | ESC_PROTOCOL_PWM |
| 1 | ESC_PROTOCOL_DSHOT300 |
| 2 | ESC_PROTOCOL_DSHOT600 |
| 3 | ESC_PROTOCOL_BIDIRECTIONAL_DSHOT |

### On Connect

ESP sends back:
1. Text: `"MotorTest_ESP32_is_connected\r\n"`
2. Binary: `[0xAA] [CMD_PROTOCOL] [current_protocol] [CRC]` — tells the app which protocol is active.

### On Disconnect

CMD_DISARM is automatically queued (motor stops).

---

## DShot Architecture

### Data Flow

```
bluetooth_spp.c (SPP callback)
  → protocol_rx_byte() state machine
  → xQueueSend(command_queue)
  → bt_command_task
  → command_handler_process(CMD_PROTOCOL, value)
  → command_select_protocol()
      → esc_controller_disarm()
      → esc_protocol_select(protocol)
          → esc_dshot_init(mode)       // if DShot/Bidir
          → esc_dshot_set_bidirectional(true)  // if Bidir
              → esc_dshot_enable_edt()  // sends DShot cmd 13 × 6
  → esc_dshot_start_stream()
```

### Timer → Frame Task

`esp_timer` fires every 500 μs (2 kHz), gives `taskNotify` to `dshot_frame_task`.

```
dshot_frame_task:
  → get current_throttle, bidirectional_mode
  → if bidir:
      → esc_dshot_make_packet(throttle, telemetry=true)
      → dshot_rmt_send_receive()
          → TX: rmt_transmit → rmt_tx_wait_all_done
          → TX→RX switch: rmt_disable TX, gpio_set_direction(INPUT), rmt_enable RX
          → RX: rmt_receive → xSemaphoreTake (callback-based)
          → RX→TX switch: rmt_disable RX, gpio_set_direction(OUTPUT), rmt_enable TX
      → dshot_rmt_decode_gcr() → decoded uint16
      → parse_edt_frame() → telemetry struct
      → ESP_LOGI: "T:XX eRPM:XXXX RPM:XXX V:XX.X I:XX.X T:XX°C OK/FAIL"
  → else:
      → dshot_rmt_send(esc_dshot_make_packet(throttle, false))
```

### DShot300 vs DShot600

RMT clocked at 10 MHz (DShot300, bit = 3.33 μs) or 20 MHz (DShot600, bit = 1.67 μs). Encoding: logic 1 = long HIGH + short LOW, logic 0 = inverse.

### Bidirectional DShot — How It Works (Simple)

Звичайний DShot — це однобічний зв'язок: ESP каже ESC "крутись на 50%", а ESC просто слухається. Bidirectional DShot додає зворотний зв'язок — ESC може відповісти ESP і сказати "я обертаюсь на 1200 eRPM, напруга 12.5V, температура 45°C".

**Як це працює по кроках:**

1. **ESP відправляє команду** — той самий DShot-пакет 16 біт на GPIO18, як і завжди. Але в біті telemetry flag стоїть "1" (telemetry request).

2. **ESC бачить запит** — він розуміє, що його попросили відповісти.

3. **ESP переключає GPIO на вхід** — одразу після передачі (за ~10 мкс) GPIO18 стає входом. RMT TX вимикається, RMT RX вмикається.

4. **ESC чекає ~30 мкс** — після отримання команди ESC готує відповідь і починає передавати назад тим самим дротом (GPIO18).

5. **ESP приймає 21 біт** — RMT RX ловить відповідь ESC. Це 21 біт, закодованих у GCR (Gray Code Recording) — спеціальний код, який гарантує достатньо перехресть для синхронізації.

6. **Декодування** — ESP зчитує біти, робить running XOR (зворотнє кодування GCR), перевіряє CRC (checksum), і якщо все вірно — отримує 16-бітне значення.

7. **EDT (Extended DShot Telemetry)** — це 16-бітне значення містить тип + значення. Тип вказує, що саме ESC повідомляє: eRPM, напругу, струм або температуру. ESC крутить усі типи по черзі (round-robin).

**Чому DShot300?** Bidir завжди використовує DShot300 (300 кбіт/с, 3.33 мкс на біт). Це стандарт для BLHeli_32. DShot600 занадто швидкий — ESC не встигає відповісти за 30 мкс.

**Чому одна лінія?** Весь обмін відбувається по одному дроту (GPIO18). Спочатку ESP "говорить", потім ESC "відповідає". Як дзвінок по телефону — хтось говорить, а потім замовкає і слухає відповідь.

### Bidirectional DShot — Technical Details

Uses DShot300. After TX, GPIO switches TX→RX. ESC responds with 21 bits of GCR-encoded data ~30 μs after TX. GCR decode: running XOR, then CRC (XOR of all 5 nibbles).

Each RMT symbol = one DShot bit. Bit value determined by `duration1 > duration0`.

`signal_range_max_ns = 100000` (100 μs) to allow ESC response delay.

---

## EDT v1 — Extended DShot Telemetry

### Enable

DShot command 13 sent 6 times while motor is stopped. Called automatically by `esc_dshot_set_bidirectional(true)`.

### Frame Format

```
16 bits: [EEE] [0] [DDDDDDDD] [CCCC]
  bits[15:13] = type (EEE)
  bit[12]     = 0 (marker)
  bits[11:4]  = value (DDDDDDDD)
  bits[3:0]   = CRC (CCCC)
```

### EDT Types

| Type | Meaning | Value Format |
|------|---------|--------------|
| 000 | eRPM low | 0–255 eRPM |
| 001 | Temperature | °C (0–255) |
| 010 | Voltage | ×0.25V per step |
| 011 | Current | ×1A per step |
| 100 | eRPM extended | 512 + value×2 |
| 101 | eRPM extended | 1024 + value×4 |
| 110 | eRPM extended | 2048 + value×8 |
| 111 | eRPM extended | 4096 + value×16 |

### Telemetry Struct

```c
typedef struct {
    uint16_t erpm;      // electrical RPM
    uint16_t rpm;       // mechanical RPM (eRPM × 2 / pole_count)
    uint8_t  temperature; // °C
    float    voltage;   // volts
    float    current;   // amps
    uint16_t raw_value; // raw 16-bit decoded GCR
    bool     valid;     // CRC passed
} dshot_telemetry_t;
```

### eRPM → RPM

`RPM = eRPM × 2 / pole_count`. Default `pole_count = 14`. Configured via `esc_dshot_set_pole_count()`.

---

## Timings

| Parameter | Value |
|-----------|-------|
| DShot frame | 500 μs (2 kHz) |
| DShot300 bit | 3.33 μs |
| DShot600 bit | 1.67 μs |
| TX→RX switch | ~10–20 μs |
| ESC response | ~30 μs after TX |
| RX timeout | 100 μs |
| EDT enable delay | 3 ms between cmd 13 |

---

## Modules

| File | Role |
|------|------|
| `main.c` | Entry point, init controller + BT |
| `esc_controller.c` | State machine (DISARMED/ARMED/RUNNING/E-STOP) |
| `esc_protocol.c` | Protocol selection, switches PWM↔DShot↔Bidir |
| `esc_dshot.c` | DShot frame task, timer, packet, bidir, EDT parse, telemetry |
| `dshot_rmt.c` | RMT TX/RX, GCR encode/decode, bidirectional send_receive |
| `esc_pwm.c` | PWM 50 Hz via LEDC |
| `bluetooth_spp.c` | Bluetooth Classic SPP server, binary packet RX/TX |
| `command_handler.c` | Binary command dispatch |

---

## Debug Logs

| Log | Meaning |
|-----|---------|
| `RMT RX init: rx_channel=0x...` | RX channel created |
| `Bidirectional DShot enabled` | Bidir mode activated |
| `Enabling EDT (6x cmd 13)...` | EDT enable sequence started |
| `EDT enabled` | EDT enable done |
| `T:20 eRPM:1200 RPM:171 V:12.5 I:2.3 T:45°C OK` | Telemetry received (all fields) |
| `T:50 eRPM:0 RPM:0 V:0.0 I:0.0 T:0°C FAIL` | No response or CRC error |
| `Parssing is OK` | BT packet parsed |
| `CRC is bad` | BT packet CRC mismatch |

---

## Testing Without Oscilloscope

1. `CMD_PROTOCOL = 0` (PWM), `CMD_ARM`, `CMD_THROTTLE = 5` — motor should spin (TX works).
2. `CMD_PROTOCOL = 1` (DSHOT300), `CMD_THROTTLE = 5` — motor should spin.
3. `CMD_PROTOCOL = 3` (Bidir DShot300), `CMD_THROTTLE = 20` — motor spins + telemetry logs with `OK`.
4. Check log for voltage, current, temperature values (EDT v1).

## Testing With Oscilloscope

1. Probe GPIO18 relative to GND.
2. `CMD_PROTOCOL = 2`, `CMD_ARM`, `CMD_THROTTLE = 10`.
3. Verify: 2 kHz frame rate (500 μs between packet starts), each frame = 16 pulses.

---

## Known Limitations

- Bidirectional DShot uses DShot300 only (BLHeli_32 standard).
- `signal_range_max_ns = 100 μs` — if ESC responds later, RMT misses it.
- No hardware propeller safety guard on protocol switch.
- `pole_count` hardcoded to 14 — change via `esc_dshot_set_pole_count()`, not Bluetooth.
- EDT parsing relies on ESC sending all telemetry types in round-robin; ESP captures whichever frame arrives during the RX window.
