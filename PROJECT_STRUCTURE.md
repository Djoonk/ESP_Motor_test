# 🧩 Структура проекту ESP_Motor_test

> **Робоча гілка:** `feature/bidirectional-dshot__3` · **MCU:** ESP32 · **SDK:** ESP-IDF v5.x

Архітектура: **ESP32 ↔ Bluetooth Classic SPP ↔ Android**. ESP32 керує ESC на GPIO18 (PWM / DShot300 / DShot600 / Bidirectional DShot) і віддає сиру телеметрію по SPP. Це покрокове пояснення "що куди летить".

---

## 🗂 1. Склад файлів (main/)

| Файл | Роль |
| :--- | :--- |
| `main.c` | Точка входу `app_main()`, старт контролера + Bluetooth. |
| `esc_controller.c/.h` | Автомат станів стенду (disarm/arm/run/stop/e-stop), throttle. |
| `esc_protocol.c/.h` | Вибір протоколу (PWM/DShot300/DShot600/Bidir), масштаб газу. |
| `esc_pwm.c/.h` | LEDC PWM 50 Гц на GPIO18. |
| `esc_dshot.c/.h` | dshot-задача (esp_timer→RMT), EDT, сира телеметрія. |
| `dshot_rmt.c/.h` | RMT TX+RX (bidir), GCR-декодування EDT, EDP/dc. |
| `bluetooth_spp.c/.h` | SPP сервер, парсер пакетів, 2 задачі (команди+телеметрія). |
| `command_handler.c/.h` | Розподіл команд на контролер/протокол. |
| `dshot_rmt_encoder.h` | Кодування дshot-кадрів у RMT символи. |

---

## 2️⃣ Приклад потоку: команда з телефона

Той самий приклад для **CMD_ARM (0x01)** та будь-якої іншої команди.

```
Android
  │  4 байти: [0xAA SOF][cmd][value][CRC]
  ▼
[1] esp_spp_write() → ESP32 BT-стек
  ▼
[2] bluetooth_spp.c::spp_callback()  [ESP_SPP_DATA_IND_EVT]
      └─ для кожного байта → protocol_rx_byte(byte)
          - стан RX_WAIT_SOF → RX_WAIT_COMMAND → RX_WAIT_VALUE → RX_WAIT_CRC
          - перевірка CRC == (SOF ^ cmd ^ value)
          - ✅ створює bt_command_msg_t{command, value}
          - кладе в чергу command_queue (неблокуючий #0)
  ▼
[3] bluetooth_spp.c::bt_command_task   (FreeRTOS-задача, prio 5)
      └─ xQueueReceive(command_queue, &msg, portMAX_DELAY)
      └─ command_handler_process(msg.command, msg.value)
  ▼
[4] command_handler.c::command_handler_process()
      └─ switch(cmd): CMD_ARM → esc_controller_arm()
  ▼
[5] esc_controller.c::esc_controller_arm()
      └─ (для DShot: esc_dshot_rearm() + esc_protocol_stop())
      └─ стан → STAND_ARMED
  ▼
[6] (напр. throttle) esc_controller_set_throttle(value)
      └─ esc_protocol_set_throttle() → esc_dshot_set_throttle() / LEDC
  ▼
GPIO18 → ESC → мотор
```

### Відповідь на телефон
ESP32 **не відповідає окремим пакетом на кожну середню команду** — команди лише виконуються (логи). Відповідь з'являється у **двох випадках**:

1. **При підключенні** (`ESP_SPP_SRV_OPEN_EVT`):
   - текстовий `"MotorTest_ESP32_is_connected\r\n"` + 
   - пакет `[0xAA][0x05 = CMD_PROTOCOL][поточний протокол][CRC]`.
2. **Телеметрія** — окрема задача, кожні 500 мс.

---

## 3️⃣ Потік телеметрії (якщо вона ввімкнена)

Телеметрія йде **автоматично кожні 500 мс** без жодної команди з телефона, якщо BT-з'єднання відкрито.

```
[FreeRTOS] bluetooth_spp.c::bt_telemetry_task   (період 500 мс)
      │
      ├─ client_connected ? (якщо ні — пропуск)
      ▼
[1] esc_dshot.c::esc_dshot_get_raw_telemetry(&raw)
        portENTER_CRITICAL → читає telem_raw_erpm/temp/voltage/current
        (ці поля оновлюються в dshot-задачі з EDT-кадрів ESC)
      ▼
[2] збірка 8-байтного пакета:
   [0xAA][0x10=CMD_TELEMETRY][eRPM_lo][eRPM_hi][temp][voltage][current][CRC=XOR 0..6]
      ▼
[3] spp_send(pkt, 8) → esp_spp_write() → телефон
```

**Звідки сирі значення (в `dshot_rmt.c`):**
- RMT RX-канал приймає 20-бітний GCR-кадр відповіді ESC (bidirectional).
- `extract_telemetry_gcr()` → 20 біт.
- `convert_gcr_to_telemetry()` → 4×ріска GCR → 16 біт → CRC → тип (напруга `0x4`/струм `0x6`/температура `0x2`/eRPM) → масштабування.
- В AM32 2.20: напруга 0.25 В/LSB, струм 0.5 А/LSB, temp 1 °C/LSB.

---

## 4. Формат команд (телефон → ESP32)

| Поле | Байт | Коментар |
| :--- | :---: | :--- |
| SOF | `0xAA` | Start-фрейм, завжди `0xAA` |
| Command | 1 | див. таблицю нижче |
| Value | 1 | аргумент (напр. газ % / протокол) |
| CRC | 1 | `SOF ^ cmd ^ value` |

```c
// command_handler.h
CMD_ARM          = 0x01
CMD_DISARM       = 0x02
CMD_STOP         = 0x03
CMD_THROTTLE     = 0x04   // value = 0..100%
CMD_PROTOCOL     = 0x05   // value: 0=PWM 1=DShot300 2=DShot600 3=Bidir
CMD_ESTOP        = 0x06
CMD_RESET_ESTOP  = 0x07
```

**Протоколи (esc_protocol.h):** `ESC_PROTOCOL_PWM=0`, `ESC_PROTOCOL_DSHOT300=1`, `ESC_PROTOCOL_DSHOT600=2`, `ESC_PROTOCOL_BIDIRECTIONAL_DSHOT=3`.

---

## 5. Формат телеметриї (ESP32 → телефон, 8 байт)

| Поле | Байт | Коментар |
|------|:---:| :--- |
| SOF | `0xAA` | цеп |
| CMD | `0x10` | CMD_TELEMETRY (ESP → phone) |
| EРPM hi | 3 | старший байт eRPM |
| temp | 4 | температура, 1 °C/LSB |
| voltage | 5 | напруга (0.25 В/LSB, сирa) |
| current | 6 | струм (0.5 А/LSB, сирa) |
| CRC | 7 | XOR байтів 0..6 |

> temp/voltage/current — сирі (не помножені); телефон робить масштабування.

---

## 6. Дві FreeRTOS-задачі Bluetooth

| Задача | Пріо | Період | Робить |
|--------|:---:|:--:|--------|
| `bt_cmd_task` | 5 | по закінченню (блок. черга) | розбирає чергу команд → `command_handler_process()` |
| `bt_telem_task` | 3 | 500 мс | читає raw телеметрію → слає 8-байт пакет |

- **Безпека обробки:** `spp_callback()` (BT-потік) тільки кладе в чергу — ніколи не чекає, щоб не блокувати BT-стек.
- **Втрата з'єднанняя:** `ESP_SPP_CLOSE_EVT` → в чергу кладеть `CMD_DISARM` → мотор зупиняється.

---

## 7. Підказки для навігації

- **Початок ініціалізації:** `app_main()` → `esc_controller_init()` (PWM 50 Гц on GPIO18) → `bluetooth_spp_init()` (BT).
- **Диспетчер команд:** `command_handler.c` → `switch(command)`.
- **Джерело телеметрії:** `dshot_rmt.c` (GCR→EDT) → `esc_dshot.c` (raw-буфер) → `bluetooth_spp.c` (сира відправка).
- **Формат пакетів:** секції 4 і 5 вище.