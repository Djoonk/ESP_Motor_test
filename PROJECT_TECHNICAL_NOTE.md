---
tags:
- project-review
- dev-docs
---

# 📦 Проект: ESP_Motor_test

> **Суть:** ESP32-контролер стенду для перевірки одного ESC/безколекторного мотора. Керування надходить з Android через Bluetooth Classic SPP, а вихід на GPIO18 підтримує PWM, DShot300 і DShot600.

---

## 🎯 1. Короткий опис

- **Призначення:** Проєкт формує керувальний сигнал для ESC, обмежує газ до 0…100%, має стани `DISARMED`/`ARMED`/`RUNNING`/`EMERGENCY_STOP` і приймає текстові команди через Bluetooth.
- **Стек технологій:** C, ESP-IDF v5.x, FreeRTOS, ESP32, LEDC (PWM), RMT TX (DShot), `esp_timer`, Bluetooth Classic / SPP, NVS.
- **Фізичний інтерфейс:** GPIO18 — єдиний вихід керування ESC. Земля ESP32 та ESC має бути спільною.
- **Поточний стартовий режим:** `app_main()` після ініціалізації контролера перемикає вихід на DShot600. Bluetooth запускається після налаштування ESC.

---

## 🚪 2. Точка входу (Entry Point)

- **Головний файл:** `main/main.c`
- **Головний метод/функція:** `app_main()`
- **Логіка запуску:**
  1. `esc_controller_init()` викликає `esc_protocol_init()`, який запускає PWM 50 Гц з безпечним імпульсом 1000 мкс.
  2. `app_main()` викликає `esc_protocol_select(ESC_PROTOCOL_DSHOT600)`: PWM вимикається, RMT налаштовується на GPIO18, запускається DShot-потік 2 кГц.
  3. `bluetooth_spp_init()` запускає Bluetooth Classic SPP сервер `MotorTest_SPP` з ім’ям пристрою `MotorTest_ESP32`.
  4. Основна задача лише очікує у нескінченному циклі. Команди обробляє callback Bluetooth SPP, а DShot кадри відправляє окрема FreeRTOS-задача `dshot_frame`.

---

## 📚 3. Бібліотеки та залежності

| Бібліотека / Модуль | Для чого використовується в проекті |
| :--- | :--- |
| ESP-IDF | Базовий SDK, запуск `app_main()`, помилки `esp_err_t`, логування `ESP_LOG*`. |
| FreeRTOS | Задача DShot, task notifications, критичні секції, затримки переходу протоколів. |
| `driver/ledc.h` | Апаратний PWM 50 Гц для аналогового ESC-сигналу 1000…2000 мкс. |
| `driver/rmt_tx.h` | Відправка 16-бітних DShot кадрів через RMT TX. |
| `esp_timer.h` | Період 500 мкс, який дає частоту DShot кадрів 2 кГц. |
| Bluetooth Classic SPP | Текстові команди з Android і текстові відповіді. |
| NVS Flash | Ініціалізація, потрібна Bluetooth стеку. |
| `driver/gpio.h` | Примусовий `LOW` на GPIO18 на 300 мс при переході DShot300 ↔ DShot600. |

Залежності компонента описані у `main/CMakeLists.txt`: `bt`, `nvs_flash`, `esp_driver_ledc`, `esp_driver_rmt`.

---

## ⚙️ 4. Архітектура та ключові функції

### Структура модулів

- **`main/main.c`** — старт системи, вибір стартового протоколу, запуск Bluetooth.
- **`main/esc_controller.c`** — автомат станів стенду, arm/disarm/stop/e-stop, обмеження газу.
- **`main/esc_protocol.c`** — вибір PWM/DShot, масштабування газу для конкретного протоколу, безпечний перехід DShot300 ↔ DShot600.
- **`main/esc_pwm.c`** — генерація PWM через LEDC на GPIO18.
- **`main/esc_dshot.c`** — задача DShot, пакет DShot, CRC, таймер 2 кГц, поточне значення газу.
- **`main/dshot_rmt.c`** — RMT TX канал, кодування 16 біт у `rmt_symbol_word_t`, таймінги DShot300/600.
- **`main/bluetooth_spp.c`** — SPP сервер, приймання даних Android, відправка відповіді, disarm при втраті з’єднання.
- **`main/command_handler.c`** — парсинг текстових команд і виклик контролера/протоколу.
- **`main/* .h`** — публічні API модулів, типи стану та константи GPIO/PWM.

### Потік даних

```text
Android
  → Bluetooth SPP callback
  → command_handler_process()
  → esc_controller_*() / esc_protocol_select()
  → PWM (LEDC) або DShot (esp_timer → dshot_frame task → RMT)
  → GPIO18 → ESC → мотор
```

### Основні функції

#### `app_main()`

- **Файл:** `main/main.c`
- **Призначення:** Запускає контролер, вибирає DShot600, запускає SPP сервер.
- **Вхідні/вихідні дані:** `void` -> `void`.

#### `esc_controller_init()`

- **Файл:** `main/esc_controller.c`
- **Призначення:** Ініціалізує протокол за замовчуванням (PWM) і встановлює стан `STAND_DISARMED`.
- **Вхідні/вихідні дані:** `void` -> `esp_err_t`.

#### `esc_controller_arm()` / `esc_controller_disarm()`

- **Файл:** `main/esc_controller.c`
- **Призначення:** `arm` дозволяє газ, `disarm` одразу передає нульовий газ через активний протокол і переводить стенд у безпечний стан.
- **Вхідні/вихідні дані:** `void` -> `void`.

#### `esc_controller_set_throttle(uint16_t percent)`

- **Файл:** `main/esc_controller.c`
- **Призначення:** Приймає 0…100%. Працює тільки у `ARMED` або `RUNNING`; інші стани примусово зупиняють мотор.
- **Вхідні/вихідні дані:** `percent` -> `void`.
- **Масштабування:** PWM: 1000…2000 мкс. DShot: значення 48…2000 з 11-бітного діапазону.

#### `esc_protocol_select(esc_protocol_t protocol)`

- **Файл:** `main/esc_protocol.c`
- **Призначення:** Вмикає PWM, DShot300 або DShot600. Перед командами `protocol ...` `command_handler` викликає `esc_controller_disarm()`.
- **Вхідні/вихідні дані:** `ESC_PROTOCOL_*` -> `void`.
- **Особливість DShot300 ↔ DShot600:** DShot деініціалізується, GPIO18 утримується у `LOW` 300 мс, потім створюється новий RMT канал. Це допомагає ESC повторно визначити швидкість DShot.

#### `esc_dshot_init(dshot_mode_t mode)`

- **Файл:** `main/esc_dshot.c`
- **Призначення:** Налаштовує бітрейт RMT, створює RMT канал, FreeRTOS-задачу `dshot_frame` і `esp_timer`.
- **Вхідні/вихідні дані:** `DSHOT_MODE_300` або `DSHOT_MODE_600` -> `esp_err_t`.

#### `esc_dshot_start_stream()` / `esc_dshot_stop_stream()`

- **Файл:** `main/esc_dshot.c`
- **Призначення:** Запускають/зупиняють `esp_timer` з періодом 500 мкс. Це відповідає частоті кадрів 2 кГц.
- **Вхідні/вихідні дані:** `void` -> `void`.

#### `dshot_frame_task()`

- **Файл:** `main/esc_dshot.c`
- **Призначення:** Чекає notification від `esp_timer`, бере `current_throttle` під critical section, формує DShot пакет і передає його в RMT.
- **Вхідні/вихідні дані:** FreeRTOS task -> нескінченний цикл.

#### `esc_dshot_make_packet(uint16_t throttle, bool telemetry)`

- **Файл:** `main/esc_dshot.c`
- **Призначення:** Формує 16-бітний кадр: 11 біт газу, telemetry bit, 4-бітний XOR CRC.
- **Вхідні/вихідні дані:** `throttle`, `telemetry` -> `uint16_t` DShot packet.

#### `dshot_rmt_encode_packet(uint16_t packet, rmt_symbol_word_t symbols[16])`

- **Файл:** `main/dshot_rmt.c`
- **Призначення:** Перетворює кожен біт пакета на RMT high/low символ.
- **Вхідні/вихідні дані:** 16-бітний пакет -> масив 16 RMT символів.
- **Таймінги:**
  - DShot300: RMT 10 МГц, біт 3.3 мкс.
  - DShot600: RMT 20 МГц, біт 1.65 мкс, кадр ≈26.4 мкс.

#### `bluetooth_spp_init()` / `spp_callback()`

- **Файл:** `main/bluetooth_spp.c`
- **Призначення:** Запускають SPP сервер, приймають команди, відповідають текстом. При `ESP_SPP_CLOSE_EVT` викликається `esc_controller_disarm()`.
- **Вхідні/вихідні дані:** `void` -> `esp_err_t`; callback отримує подію Bluetooth.

#### `command_handler_process(const char *command, char *response, size_t response_size)`

- **Файл:** `main/command_handler.c`
- **Призначення:** Парсить текстові команди Android та формує відповідь.
- **Вхідні/вихідні дані:** рядок команди + буфер -> текстова відповідь.

### Команди Bluetooth

| Команда | Дія |
| :--- | :--- |
| `protocol pwm` | Disarm, вибрати PWM 50 Гц. |
| `protocol dshot300` | Disarm, вибрати DShot300. |
| `protocol dshot600` | Disarm, вибрати DShot600. |
| `protocol bdshot` | Disarm, зараз фактично запускає звичайний DShot300. |
| `arm` | Дозволити газ. |
| `disarm` | Нульовий газ і стан `DISARMED`. |
| `throttle 0..100` | Задати газ; команда дозволена лише після `arm`. |
| `stop` | Нульовий газ, але лишає `ARMED`. |
| `estop` | Нульовий газ і `EMERGENCY_STOP`. |
| `reset_estop` | Повернутися з `EMERGENCY_STOP` у `DISARMED`. |
| `status` | Повернути поточний стан контролера. |

---

## 🛠 5. Посібник розробника (Як працювати з кодом)

> [!TIP] Де шукати баги
> - **Немає сигналу GPIO18:** перевірити `esc_protocol_select()`, `esc_pwm_deinit()`, `dshot_rmt_init()` та осцилографом сам GPIO18 відносно GND ESP32.
> - **Є DShot-сигнал, але мотор не реагує:** перевірити бітрейт у `dshot_rmt_set_bitrate()`, фактичні таймінги у `dshot_rmt_encode_packet()`, поточний протокол та `arm` перед `throttle`.
> - **DShot300 → DShot600 не запускає ESC:** перевірити повідомлення `DShot switch: GPIO18 LOW for 300 ms`; не прибирати `esc_dshot_deinit()` і примусовий `LOW`.
> - **Мотор не зупиняється за timeout:** `esc_controller_check_timeout()` реалізована, але ніде не викликається. Додайте окрему періодичну задачу або `esp_timer` callback, який викликає її поза ISR.
> - **Bluetooth-команда губиться або склеюється:** `ESP_SPP_DATA_IND_EVT` зараз вважає один отриманий пакет однією повною командою. Для потокового протоколу потрібен буфер приймання та розбір `\r\n`.

> [!NOTE] Як додати новий функціонал
> 1. **Новий текстовий command:** додайте гілку в `command_handler_process()`; не керуйте RMT/LEDC напряму з `bluetooth_spp.c`.
> 2. **Новий режим ESC:** додайте значення в `esc_protocol_t`, визначте безпечний перехід у `esc_protocol_select()` і реалізацію генератора сигналу в окремому модулі.
> 3. **Новий DShot бітрейт:** додайте бітрейт у `dshot_rmt_set_bitrate()`, вибір `resolution_hz` і точні `T0H/T0L/T1H/T1L` у `dshot_rmt_encode_packet()`.
> 4. **Телеметрія або справжній bidirectional DShot:** потрібні TX→RX перемикання GPIO18, RMT RX, інверсія сигналу, декодування GCR та окремий стан приймання. Поточний `bdshot` цього не реалізує.

> [!WARNING] Важливі нюанси та підводні камені
> - **Безпека:** не перемикайте протокол та не прошивайте контролер з установленим пропелером. Після зміни протоколу потрібна окрема команда `arm`.
> - **Поточний `app_main()` примусово обирає DShot600.** Якщо потрібний старт у PWM, приберіть або замініть `esc_protocol_select(ESC_PROTOCOL_DSHOT600)`.
> - **`vTaskDelay(300 ms)` у `esc_protocol_select()` виконується з Bluetooth SPP callback.** Під час зміни DShot режиму Bluetooth callback блокується на 300 мс. Для production-версії перенесіть перемикання протоколу в окрему FreeRTOS-задачу через Queue.
> - **`ESP_ERROR_CHECK()` перезавантажує пристрій при помилці.** У production-коді для RMT/LEDC краще повертати `esp_err_t`, переводити контролер у `STAND_ERROR` і повідомляти Android.
> - **DShot task ігнорує return `dshot_rmt_send()`.** Помилка передавання логуватиметься у RMT модулі, але не переводить контролер у `STAND_ERROR`.
> - **`current_throttle` захищений critical section, а стан контролера й протокол — ні.** Після додавання кількох задач потрібен mutex або одна центральна задача керування ESC.
> - **`BIDIRECTIONAL_DSHOT` у переліку протоколів не є реалізованим bidirectional DShot.** Зараз він викликає DShot300 TX без телеметрії.

