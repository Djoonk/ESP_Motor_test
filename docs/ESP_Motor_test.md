# ESP_Motor_test — DShot

## Що це

ESP32-контролер стенду для перевірки ESC/безколекторного мотора. Керування через Bluetooth Classic SPP з Android, вихід на GPIO18 — PWM 50 Гц, DShot300, DShot600, Bidirectional DShot300.

## Стек

ESP-IDF v5.5.3, C, FreeRTOS, RMT TX/RX, LEDC (PWM), esp_timer, Bluetooth SPP, NVS.

---

## Архітектура DShot

### Точка входу

`app_main()` → `esc_controller_init()` → `esc_protocol_select()` → `esc_dshot_init()` створює RMT TX + RX канали та FreeRTOS-задачу `dshot_frame_task`.

### Генерація кадру

`esp_timer` стріляє кожні 500 мкс (2 кГц) і дає `taskNotify` задачі `dshot_frame_task`. Задача бере `current_throttle` і формує 16-бітний DShot-пакет: 11 біт газу + telemetry flag + 4-бітний XOR CRC.

### DShot300 vs DShot600

RMT тактована на 10 МГц (DShot300, біт = 3.3 мкс) або 20 МГц (DShot600, біт = 1.65 мкс). Кодування: логічна 1 = довгий HIGH + короткий LOW, логічна 0 = навпаки.

### Bidirectional DShot

Використовує DShot300. Після TX на GPIO18 відбувається перемикання TX→RX: RMT TX вимикається, GPIO переводиться на вхід, RMT RX вмикається. ESC відповідає 21 бітом GCR-закодованих даних (~30 мкс після TX). Декодування: running XOR (GCR), CRC — XOR усіх5 nibble.

### Потік даних (Bidir)

```
esp_timer → taskNotify → dshot_frame_task
  → esc_dshot_make_packet(throttle, telemetry=true)
  → dshot_rmt_send_receive()
      → TX: rmt_transmit → rmt_tx_wait_all_done
      → TX→RX switch: rmt_disable TX, gpio_input, rmt_enable RX
      → RX: rmt_receive → xSemaphoreTake (callback-семафор)
      → RX→TX switch: rmt_disable RX, gpio_output, rmt_enable TX
  → dshot_rmt_decode_gcr() → telemetry struct
  → ESP_LOGI: "T:%u eRPM:%u RPM:%u V:%s"
```

---

## Таймінги

| Параметр | Значення |
|----------|----------|
| Кадр DShot | 500 мкс (2 кГц) |
| DShot300 біт | 3.33 мкс |
| DShot600 біт | 1.67 мкс |
| TX→RX switch | ~10-20 мкс |
| Відповідь ESC | ~30 мкс після TX |
| RX timeout | 100 мкс |

---

## Модулі

| Файл | Роль |
|------|------|
| `main.c` | Точка входу, запуск |
| `esc_controller.c` | Автомат станів (DISARMED/ARMED/RUNNING/E-STOP) |
| `esc_protocol.c` | Вибір протоколу, перемикання PWM↔DShot |
| `esc_dshot.c` | DShot-задача, таймер, пакет, bidir, телеметрія |
| `dshot_rmt.c` | RMT TX/RX, кодування/декодування символів, GCR |
| `esc_pwm.c` | PWM 50 Гц через LEDC |
| `bluetooth_spp.c` | Bluetooth SPP сервер, прийом команд |
| `command_handler.c` | Парсинг бінарних команд |

---

## Команди (Bluetooth)

| Команда | Дія |
|---------|-----|
| `arm` | Дозволити газ |
| `disarm` | Нульовий газ, DISARMED |
| `throttle 0..100` | Встановити газ (після arm) |
| `protocol pwm/dshot300/dshot600/bdshot` | Перемкнути протокол |
| `stop` | Нульовий газ, залишає ARMED |
| `estop` | Екстренна зупинка |

---

## Як перевірити DShot

### Осцилограф

1. Підключи осцилограф до GPIO18 відносно GND ESP32.
2. Увімкни DShot через Bluetooth: `protocol dshot600` → `arm` → `throttle 10`.
3. Перевір: частота кадрів 2 кГц (500 мкс між початками пакетів), кожен кадр = 16 імпульсів.

### Bidirectional DShot

1. Підключи ESC, який підтримує bidir (BLHeli_32/AM32).
2. `protocol bdshot` → `arm` → `throttle 20`.
3. Дивись логи: `T:20 eRPM:xxxx RPM:xxx V:OK` — якщо `V:OK`, CRC проходить і ESC відповідає.

### Debug логи

| Лог | Що означає |
|-----|-----------|
| `RMT RX init: rx_channel=0x...` | RX канал створено |
| `Bidirectional DShot enabled` | Bidir увімкнено |
| `T:50 eRPM:1200 RPM:171 V:OK` | Телеметрія отримана |
| `T:50 eRPM:0 RPM:0 V:FAIL` | Відповідь не отримана або CRC помилка |
| `rmt_receive: ...` | Помилка прийому RMT |
| `TX done: ...` | Помилка передачі |

### Перевірка без осцилографа

1. Запусти `protocol dshot300`, `arm`, `throttle 5`.
2. Якщо мотор обертається — TX працює.
3. Перемкни на `protocol bdshot` з тим самим throttle.
4. Якщо мотор обертається і логи показують `V:OK` — bidir працює.

---

## Bidirectional DShot — деталі

### GCR декодування

ESC відповідає 21 бітом: [start=0][20 GCR біт]. Кожен RMT-символ = один DShot-біт. Біт визначається порівнянням `duration1 > duration0` (HIGH тривалість > LOW тривалість). Після зняття20 біт застосовується running XOR для GCR-декодування.

### eRPM → RPM

`RPM = eRPM * 2 / pole_count`. За замовчуванням `pole_count = 14`. Налаштовується через `esc_dshot_set_pole_count()`.

### Структура телеметрії

```c
typedef struct {
    uint16_t erpm;      // electrical RPM від ESC
    uint16_t rpm;       // mechanical RPM (eRPM * 2 / poles)
    uint16_t raw_value; // сире 16-бітне значення
    bool valid;         // CRC пройдено
} dshot_telemetry_t;
```

---

## Відомі обмеження

- Bidir використовує DShot300 (не DShot600) — стандарт для BLHeli_32.
- `signal_range_max_ns = 100 мкс` — якщо ESC відповідає пізніше, RMT пропустить.
- Немає hardware захисту від пропелеру при перемиканні протоколу.
- `pole_count` хардкоджений (14) — змінюється через API, не з Bluetooth.
