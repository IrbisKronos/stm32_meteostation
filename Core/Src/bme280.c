/*
 * bme280.c — Реалізація драйвера BME280
 *
 * Весь код побудований на інформації з даташита BME280 (Bosch).
 * Компенсаційні формули — дослівно з розділу 4.2.3 даташита.
 */

#include "bme280.h"

/* ============================================================
 * ДОПОМІЖНІ ФУНКЦІЇ (ПРИВАТНІ)
 * ============================================================
 * Ці функції не оголошені в .h файлі — вони "внутрішні",
 * використовуються тільки всередині bme280.c.
 */

/*
 * Записати один байт у регістр BME280.
 *
 * Як це працює на рівні I2C:
 * STM32 (master) відправляє на шину:
 *   [Адреса пристрою + Write] → [Адреса регістру] → [Дані]
 *
 * HAL_I2C_Mem_Write робить це за нас:
 *   - DevAddress: 0x76 << 1 (I2C-адреса, зсунута для HAL)
 *   - MemAddress: адреса регістру всередині BME280
 *   - MemAddSize: I2C_MEMADD_SIZE_8BIT (регістри BME280 — 8-бітні)
 *   - pData: вказівник на байт, який записуємо
 *   - Size: 1 (один байт)
 *   - Timeout: 100 мс (якщо за цей час I2C не відповів — помилка)
 */
static HAL_StatusTypeDef BME280_WriteReg(BME280_Handle *dev, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(dev->hi2c, BME280_I2C_ADDR,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, 100);
}

/*
 * Прочитати один або кілька байтів з регістрів BME280.
 *
 * Як це працює на рівні I2C:
 * STM32 відправляє: [Адреса + Write] → [Адреса регістру]
 * Потім: [Адреса + Read] → BME280 відправляє [Дані байт 1] [Дані байт 2] ...
 *
 * Коли size > 1, BME280 автоматично збільшує адресу регістру
 * (auto-increment). Тобто якщо почав з 0x88 і читаєш 26 байт,
 * отримаєш вміст регістрів 0x88, 0x89, 0x8A, ..., 0xA1.
 * Це називається "burst read" і це ефективніше, ніж читати по одному.
 */
static HAL_StatusTypeDef BME280_ReadRegs(BME280_Handle *dev, uint8_t reg,
                                         uint8_t *buf, uint16_t size)
{
    return HAL_I2C_Mem_Read(dev->hi2c, BME280_I2C_ADDR,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            buf, size, 100);
}

/* ============================================================
 * КРОК 1: ЗЧИТУВАННЯ КАЛІБРУВАЛЬНИХ КОЕФІЦІЄНТІВ
 * ============================================================
 *
 * ЩО ТАКЕ КАЛІБРУВАЛЬНІ КОЕФІЦІЄНТИ?
 *
 * Кожен фізичний датчик має невеликі відхилення від ідеалу.
 * Два різних чіпи BME280 при однаковій температурі можуть
 * видавати різні "сирі" числа з АЦП.
 *
 * Тому на заводі кожен чіп тестують при відомих умовах
 * і записують у нього індивідуальні поправкові коефіцієнти.
 *
 * Ці коефіцієнти зберігаються в NVM (незмінна пам'ять) чіпа
 * і доступні через I2C-регістри. Зчитуємо їх ОДИН РАЗ при
 * ініціалізації і зберігаємо в структурі BME280_Handle.
 *
 * ДЕ ЛЕЖАТЬ КОЕФІЦІЄНТИ В ПАМ'ЯТІ BME280:
 *
 * Група 1 (регістри 0x88–0xA1, 26 байт):
 *   - dig_T1, dig_T2, dig_T3 (температура)
 *   - dig_P1 ... dig_P9 (тиск)
 *
 * Група 2 (регістри 0xA1 та 0xE1–0xE7):
 *   - dig_H1 (один байт за адресою 0xA1)
 *   - dig_H2 ... dig_H6 (7 байт, адреси 0xE1–0xE7)
 *
 * ЯК ЗІБРАТИ 16-БІТНЕ ЧИСЛО З ДВОХ 8-БІТНИХ РЕГІСТРІВ:
 *
 * Регістри BME280 — 8-бітні (один байт кожен).
 * Але коефіцієнти — 16-бітні (два байти).
 * Тому кожен коефіцієнт розбитий на два регістри: LSB і MSB.
 *
 * LSB = Least Significant Byte (молодший байт)
 * MSB = Most Significant Byte (старший байт)
 *
 * Щоб зібрати 16-бітне число:
 *   result = (MSB << 8) | LSB
 *
 * Приклад: dig_T1 зберігається в регістрах 0x88 (LSB) та 0x89 (MSB).
 * Якщо 0x88 = 0x6C і 0x89 = 0x6E, то:
 *   dig_T1 = (0x6E << 8) | 0x6C = 0x6E6C = 28268
 */
static HAL_StatusTypeDef BME280_ReadCalibration(BME280_Handle *dev)
{
    uint8_t buf[26];  /* Буфер для зчитування */
    HAL_StatusTypeDef status;

    /* ---- Група 1: Температура (T1-T3) та Тиск (P1-P9) ---- */
    /* Зчитуємо 26 байт починаючи з адреси 0x88 (burst read) */
    status = BME280_ReadRegs(dev, BME280_REG_CALIB_T_P, buf, BME280_CALIB_T_P_LEN);
    if (status != HAL_OK) return status;

    /*
     * Розбираємо буфер на окремі коефіцієнти.
     * Зверни увагу: buf[0] = регістр 0x88, buf[1] = регістр 0x89, і т.д.
     *
     * dig_T1: 0x88 (LSB) + 0x89 (MSB) — unsigned 16-bit
     * dig_T2: 0x8A (LSB) + 0x8B (MSB) — signed 16-bit
     * dig_T3: 0x8C (LSB) + 0x8D (MSB) — signed 16-bit
     */
    dev->calib.dig_T1 = (uint16_t)(buf[1] << 8) | buf[0];
    dev->calib.dig_T2 = (int16_t) ((buf[3] << 8) | buf[2]);
    dev->calib.dig_T3 = (int16_t) ((buf[5] << 8) | buf[4]);

    /*
     * dig_P1 ... dig_P9: 9 коефіцієнтів по 2 байти.
     * Починаються з buf[6] (регістр 0x8E).
     * P1 — unsigned, решта — signed.
     */
    dev->calib.dig_P1 = (uint16_t)(buf[7]  << 8) | buf[6];
    dev->calib.dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
    dev->calib.dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
    dev->calib.dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
    dev->calib.dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
    dev->calib.dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
    dev->calib.dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
    dev->calib.dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
    dev->calib.dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);

    /* ---- dig_H1: окремий байт за адресою 0xA1 ---- */
    status = BME280_ReadRegs(dev, BME280_REG_CALIB_H1, buf, 1);
    if (status != HAL_OK) return status;
    dev->calib.dig_H1 = buf[0];

    /* ---- Група 2: Вологість (H2-H6), регістри 0xE1–0xE7 ---- */
    status = BME280_ReadRegs(dev, BME280_REG_CALIB_H2, buf, BME280_CALIB_H2_LEN);
    if (status != HAL_OK) return status;

    /*
     * Коефіцієнти вологості розкладені хитрим чином (Bosch "заощадив" регістри):
     *
     * dig_H2: 0xE1 (LSB) + 0xE2 (MSB) — стандартно, signed 16-bit
     * dig_H3: 0xE3 — один байт, unsigned 8-bit
     * dig_H4: 0xE4 (MSB, біти [11:4]) + 0xE5 (LSB, біти [3:0]) — signed 16-bit
     * dig_H5: 0xE5 (LSB, біти [7:4]) + 0xE6 (MSB, біти [11:4]) — signed 16-bit
     * dig_H6: 0xE7 — один байт, signed 8-bit
     *
     * H4 та H5 ділять один регістр 0xE5 (buf[4]):
     *   - Молодші 4 біти 0xE5 → молодші 4 біти dig_H4
     *   - Старші 4 біти 0xE5 → молодші 4 біти dig_H5
     */
    dev->calib.dig_H2 = (int16_t) ((buf[1] << 8) | buf[0]);
    dev->calib.dig_H3 = buf[2];
    dev->calib.dig_H4 = (int16_t) ((buf[3] << 4) | (buf[4] & 0x0F));
    dev->calib.dig_H5 = (int16_t) ((buf[5] << 4) | ((buf[4] >> 4) & 0x0F));
    dev->calib.dig_H6 = (int8_t)  buf[6];

    return HAL_OK;
}

/* ============================================================
 * КРОК 2: КОНФІГУРАЦІЯ РЕЖИМУ ВИМІРЮВАННЯ
 * ============================================================
 *
 * BME280 має три режими роботи:
 *
 * SLEEP MODE (0b00):
 *   Датчик "спить", не вимірює. Споживання мінімальне (~0.1 мкА).
 *   Після ввімкнення живлення датчик завжди в цьому режимі.
 *
 * FORCED MODE (0b01 або 0b10):
 *   Датчик виконує ОДНЕ вимірювання і повертається в Sleep.
 *   Ми використовуємо цей режим — самі вирішуємо, КОЛИ вимірювати.
 *   Це оптимально для FreeRTOS: задача прокидається, запускає
 *   вимірювання, зчитує результат, засинає.
 *
 * NORMAL MODE (0b11):
 *   Датчик автоматично вимірює з заданим інтервалом.
 *   Зручно, але споживає більше.
 *
 * OVERSAMPLING (надлишкова дискретизація):
 *   Замість одного зчитування АЦП робить кілька і усереднює.
 *   Більше oversampling = точніший результат, але повільніше.
 *   x1 = одне зчитування (найшвидше)
 *   x2, x4, x8, x16 = усереднення кількох зчитувань
 *
 * IIR-ФІЛЬТР:
 *   Згладжує коливання тиску (наприклад, від грюкання дверима).
 *   Коефіцієнт 0 = вимкнений, 2, 4, 8, 16 = різна сила згладжування.
 */
static HAL_StatusTypeDef BME280_Configure(BME280_Handle *dev)
{
    HAL_StatusTypeDef status;

    /*
     * Регістр ctrl_hum (0xF2): oversampling вологості.
     * Біти [2:0] = osrs_h:
     *   000 = вимкнено
     *   001 = x1
     *   010 = x2
     *   011 = x4
     *   100 = x8
     * Ми ставимо 001 (x1) — достатньо для метеостанції.
     */
    status = BME280_WriteReg(dev, BME280_REG_CTRL_HUM, 0x01);
    if (status != HAL_OK) return status;

    /*
     * Регістр config (0xF5): фільтр та standby-час.
     * Біти [7:5] = t_sb (час між вимірюваннями в Normal Mode, нам не потрібно)
     * Біти [4:2] = filter (IIR-фільтр):
     *   000 = вимкнений
     *   001 = коефіцієнт 2
     *   010 = коефіцієнт 4
     *   011 = коефіцієнт 8
     *   100 = коефіцієнт 16
     * Біти [1:0] = spi3w_en (SPI 3-wire, нам не потрібно)
     *
     * Ми ставимо filter = 100 (коефіцієнт 16): 0b10010000 = 0x10
     * Фільтр потрібен для тиску — без нього значення "стрибатимуть".
     *
     * ВАЖЛИВО: config записується в Sleep Mode.
     * Оскільки після ввімкнення датчик вже в Sleep — все ОК.
     */
    status = BME280_WriteReg(dev, BME280_REG_CONFIG, 0x10);
    if (status != HAL_OK) return status;

    /*
     * Регістр ctrl_meas (0xF4): oversampling T та P + режим.
     * Біти [7:5] = osrs_t (oversampling температури):
     *   001 = x1
     * Біти [4:2] = osrs_p (oversampling тиску):
     *   001 = x1
     * Біти [1:0] = mode:
     *   00 = Sleep (ми НЕ запускаємо вимірювання тут)
     *
     * 0b00100100 = 0x24 (T: x1, P: x1, Mode: Sleep)
     *
     * Чому Sleep? Бо ми використовуємо Forced Mode —
     * вимірювання запускатимемо явно в BME280_ReadAll().
     */
    status = BME280_WriteReg(dev, BME280_REG_CTRL_MEAS, 0x24);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/* ============================================================
 * КРОК 3: ЗЧИТУВАННЯ "СИРИХ" ДАНИХ
 * ============================================================
 *
 * ЩО ТАКЕ "СИРІ" (RAW) ДАНІ?
 *
 * Всередині BME280 є три аналого-цифрових перетворювача (АЦП):
 * один для температури, один для тиску, один для вологості.
 *
 * АЦП перетворює фізичну величину (напругу від сенсора) в число.
 * Це число — НЕ температура в градусах. Це "сире" значення АЦП.
 *
 * Наприклад, "сира" температура може бути 519888.
 * Після компенсації це стане 22.35 °C.
 *
 * Температура і тиск — 20-бітні значення (розподілені по 3 регістрах).
 * Вологість — 16-бітне значення (2 регістри).
 *
 * ЯК ЗІБРАТИ 20-БІТНЕ ЧИСЛО З ТРЬОХ РЕГІСТРІВ:
 *
 * Приклад для температури:
 *   Регістр 0xFA (MSB):  біти [19:12]
 *   Регістр 0xFB (LSB):  біти [11:4]
 *   Регістр 0xFC (XLSB): біти [3:0] лежать у бітах [7:4] регістру
 *
 *   adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4)
 *
 *   buf[3] = 0xFA temp_msb  → зсуваємо на 12 біт вліво
 *   buf[4] = 0xFB temp_lsb  → зсуваємо на 4 біти вліво
 *   buf[5] = 0xFC temp_xlsb → зсуваємо на 4 біти вправо (бо дані в [7:4])
 *
 * BURST READ:
 *   Зчитуємо всі 8 байт за один I2C-запит, починаючи з 0xF7.
 *   Порядок: press_msb, press_lsb, press_xlsb,
 *            temp_msb,  temp_lsb,  temp_xlsb,
 *            hum_msb,   hum_lsb
 *
 *   Це швидше і надійніше, ніж читати кожен регістр окремо.
 *   BME280 гарантує, що дані в shadow-регістрах "заморожуються"
 *   на час burst-read, тому вони завжди консистентні.
 */

/* ============================================================
 * КРОК 4: КОМПЕНСАЦІЙНІ ФОРМУЛИ
 * ============================================================
 *
 * Формули нижче — дослівно з даташита BME280, розділ 4.2.3.
 * Bosch надає їх у двох варіантах:
 *   - Fixed-point (цілочисельна арифметика) — для мікроконтролерів
 *   - Floating-point — для ПК та мов високого рівня
 *
 * Ми використовуємо floating-point, бо STM32F411 має апаратний
 * FPU (Floating Point Unit) — операції з float виконуються за 1 такт.
 *
 * ЗМІННА t_fine:
 * Це ключова деталь архітектури BME280.
 * Формула температури обчислює проміжне значення t_fine,
 * яке потім використовується у формулах тиску та вологості.
 * Тому ЗАВЖДИ спочатку обчислюй температуру, потім тиск і вологість.
 */

/*
 * Компенсація температури.
 * Вхід: adc_T — "сире" 20-бітне значення з АЦП.
 * Побічний ефект: зберігає t_fine в dev->t_fine.
 * Вихід: температура в °C (наприклад, 22.35).
 */
static float BME280_CompensateTemperature(BME280_Handle *dev, int32_t adc_T)
{
    float var1, var2, T;
    BME280_CalibData *c = &dev->calib;

    var1 = (((float)adc_T) / 16384.0f - ((float)c->dig_T1) / 1024.0f) * ((float)c->dig_T2);
    var2 = ((((float)adc_T) / 131072.0f - ((float)c->dig_T1) / 8192.0f) *
            (((float)adc_T) / 131072.0f - ((float)c->dig_T1) / 8192.0f)) * ((float)c->dig_T3);

    dev->t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0f;

    return T;
}

/*
 * Компенсація тиску.
 * ВИМОГА: BME280_CompensateTemperature() мусить бути викликана раніше!
 * (бо використовує dev->t_fine)
 * Вихід: тиск у Паскалях (наприклад, 101325.0).
 * Ми потім ділитимемо на 100, щоб отримати гПа (гектопаскалі).
 */
static float BME280_CompensatePressure(BME280_Handle *dev, int32_t adc_P)
{
    float var1, var2, P;
    BME280_CalibData *c = &dev->calib;

    var1 = ((float)dev->t_fine / 2.0f) - 64000.0f;
    var2 = var1 * var1 * ((float)c->dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)c->dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)c->dig_P4) * 65536.0f);
    var1 = (((float)c->dig_P3) * var1 * var1 / 524288.0f + ((float)c->dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)c->dig_P1);

    /* Захист від ділення на нуль */
    if (var1 == 0.0f) return 0.0f;

    P = 1048576.0f - (float)adc_P;
    P = (P - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)c->dig_P9) * P * P / 2147483648.0f;
    var2 = P * ((float)c->dig_P8) / 32768.0f;
    P = P + (var1 + var2 + ((float)c->dig_P7)) / 16.0f;

    return P;
}

/*
 * Компенсація вологості.
 * ВИМОГА: BME280_CompensateTemperature() мусить бути викликана раніше!
 * Вихід: відносна вологість у % (наприклад, 45.2).
 */
static float BME280_CompensateHumidity(BME280_Handle *dev, int32_t adc_H)
{
    float var_H;
    BME280_CalibData *c = &dev->calib;

    var_H = ((float)dev->t_fine) - 76800.0f;

    /* Захист від ділення на нуль */
    if (var_H == 0.0f) return 0.0f;

    var_H = (((float)adc_H) - (((float)c->dig_H4) * 64.0f +
             ((float)c->dig_H5) / 16384.0f * var_H)) *
            (((float)c->dig_H2) / 65536.0f *
             (1.0f + ((float)c->dig_H6) / 67108864.0f * var_H *
              (1.0f + ((float)c->dig_H3) / 67108864.0f * var_H)));

    var_H = var_H * (1.0f - ((float)c->dig_H1) * var_H / 524288.0f);

    /* Обмеження діапазону: 0–100% */
    if (var_H > 100.0f) var_H = 100.0f;
    if (var_H < 0.0f)   var_H = 0.0f;

    return var_H;
}

/* ============================================================
 * ПУБЛІЧНІ ФУНКЦІЇ
 * ============================================================ */

/*
 * BME280_Init — Ініціалізація датчика.
 *
 * Що робить:
 * 1. Зберігає вказівник на I2C handle.
 * 2. Перевіряє Chip ID (чи це дійсно BME280).
 * 3. Робить софтверний скидання (щоб почати з чистого стану).
 * 4. Зчитує калібрувальні коефіцієнти (один раз на весь час роботи).
 * 5. Конфігурує режим вимірювання (oversampling, фільтр).
 *
 * Викликається один раз при старті програми (з main або з FreeRTOS-задачі).
 */
HAL_StatusTypeDef BME280_Init(BME280_Handle *dev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;

    dev->hi2c = hi2c;

    /* 1. Перевіряємо Chip ID */
    status = BME280_ReadRegs(dev, BME280_REG_CHIP_ID, &chip_id, 1);
    if (status != HAL_OK) return status;

    if (chip_id != BME280_CHIP_ID_VALUE) {
        /* Chip ID не збігається — або це не BME280,
         * або I2C-адреса неправильна, або проблема з проводами. */
        return HAL_ERROR;
    }

    /* 2. Софтверний скидання */
    status = BME280_WriteReg(dev, BME280_REG_RESET, BME280_RESET_VALUE);
    if (status != HAL_OK) return status;

    /* Чекаємо завершення скидання.
     * Після reset BME280 копіює калібрувальні дані з NVM у регістри.
     * Це займає ~2 мс. Чекаємо з запасом. */
    HAL_Delay(10);

    /* 3. Зчитуємо калібрувальні коефіцієнти */
    status = BME280_ReadCalibration(dev);
    if (status != HAL_OK) return status;

    /* 4. Конфігуруємо oversampling та фільтр */
    status = BME280_Configure(dev);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/*
 * BME280_ReadAll — Зчитати всі дані (температура, тиск, вологість).
 *
 * Що робить:
 * 1. Запускає вимірювання (Forced Mode).
 * 2. Чекає завершення.
 * 3. Зчитує 8 байт "сирих" даних одним burst-read.
 * 4. Збирає 20/16-бітні числа з окремих байтів.
 * 5. Застосовує компенсаційні формули.
 * 6. Записує результат у структуру BME280_Data.
 */
HAL_StatusTypeDef BME280_ReadAll(BME280_Handle *dev, BME280_Data *data)
{
    HAL_StatusTypeDef status;
    uint8_t buf[BME280_DATA_LEN];
    int32_t adc_T, adc_P, adc_H;

    /* ---- 1. Запустити вимірювання (Forced Mode) ---- */
    /*
     * Записуємо в ctrl_meas: T oversampling x1, P oversampling x1, Mode = Forced.
     * 0b00100101 = 0x25 (те саме, що 0x24, але mode = 01 замість 00).
     *
     * ВАЖЛИВО: щоразу при запуску Forced Mode треба записати ctrl_hum ПЕРЕД ctrl_meas,
     * бо значення ctrl_hum "фіксується" при записі ctrl_meas.
     */
    status = BME280_WriteReg(dev, BME280_REG_CTRL_HUM, 0x01);
    if (status != HAL_OK) return status;

    status = BME280_WriteReg(dev, BME280_REG_CTRL_MEAS, 0x25);
    if (status != HAL_OK) return status;

    /* ---- 2. Почекати завершення вимірювання ---- */
    /*
     * При oversampling x1 для всіх трьох параметрів
     * вимірювання займає ~8 мс. Чекаємо з запасом.
     *
     * Більш "правильний" спосіб — перевіряти біт "measuring"
     * в регістрі status (0xF3), але для простоти HAL_Delay достатньо.
     */
    HAL_Delay(10);

    /* ---- 3. Зчитати "сирі" дані (8 байт, burst read) ---- */
    status = BME280_ReadRegs(dev, BME280_REG_DATA_START, buf, BME280_DATA_LEN);
    if (status != HAL_OK) return status;

    /* ---- 4. Зібрати числа з байтів ---- */
    /*
     * buf[0] = 0xF7 = press_msb
     * buf[1] = 0xF8 = press_lsb
     * buf[2] = 0xF9 = press_xlsb (біти [7:4])
     * buf[3] = 0xFA = temp_msb
     * buf[4] = 0xFB = temp_lsb
     * buf[5] = 0xFC = temp_xlsb  (біти [7:4])
     * buf[6] = 0xFD = hum_msb
     * buf[7] = 0xFE = hum_lsb
     */
    adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);
    adc_H = ((int32_t)buf[6] << 8)  |  (int32_t)buf[7];

    /* ---- 5. Компенсація (ОБОВ'ЯЗКОВО: спочатку температура!) ---- */
    data->temperature = BME280_CompensateTemperature(dev, adc_T);
    data->pressure    = BME280_CompensatePressure(dev, adc_P) / 100.0f;  /* Па → гПа */
    data->humidity    = BME280_CompensateHumidity(dev, adc_H);

    return HAL_OK;
}
