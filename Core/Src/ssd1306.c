/*
 * ssd1306.c — Реалізація драйвера OLED-дисплея SSD1306
 *
 * Логіка роботи:
 * 1. Ініціалізуємо дисплей послідовністю команд (з даташита).
 * 2. Малюємо все в RAM-буфер (масив 1024 байти в STM32).
 * 3. Одним I2C-запитом відправляємо весь буфер на дисплей.
 *
 * Це називається "double buffering" — ми ніколи не малюємо
 * безпосередньо на дисплей, а тільки в буфер. Коли малюнок
 * готовий — відправляємо. Це уникає "мерехтіння" (flicker).
 */

#include "ssd1306.h"
#include "fonts.h"
#include <string.h>  /* Для memset() */

/* ============================================================
 * КОМАНДИ SSD1306 (з даташита)
 * ============================================================
 * Тут лише ті, що використовуються при ініціалізації.
 * Повний список — в даташиті SSD1306, розділ 10.
 */
#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_DISPLAY_CLK     0xD5
#define SSD1306_CMD_SET_MULTIPLEX       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_CHARGE_PUMP         0x8D
#define SSD1306_CMD_MEMORY_MODE         0x20
#define SSD1306_CMD_SEG_REMAP           0xA1
#define SSD1306_CMD_COM_SCAN_DEC        0xC8
#define SSD1306_CMD_SET_COM_PINS        0xDA
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_SET_PRECHARGE       0xD9
#define SSD1306_CMD_SET_VCOMH           0xDB
#define SSD1306_CMD_ENTIRE_DISPLAY_RAM  0xA4
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6
#define SSD1306_CMD_SET_COL_ADDR        0x21
#define SSD1306_CMD_SET_PAGE_ADDR       0x22

/* ============================================================
 * ПРИВАТНІ ФУНКЦІЇ
 * ============================================================ */

/*
 * Відправити одну команду в SSD1306.
 *
 * Протокол I2C для SSD1306:
 *   [I2C адреса] → [Керуючий байт] → [Байт даних]
 *
 * Керуючий байт:
 *   0x00 = наступний байт — команда
 *   0x40 = наступні байти — дані для GDDRAM (пікселі)
 *
 * Ми використовуємо HAL_I2C_Mem_Write, де:
 *   MemAddress = 0x00 (означає "команда")
 *   pData = вказівник на байт команди
 */
static HAL_StatusTypeDef SSD1306_WriteCommand(SSD1306_Handle *dev, uint8_t cmd)
{
    return HAL_I2C_Mem_Write(dev->hi2c, SSD1306_I2C_ADDR,
                             0x00, I2C_MEMADD_SIZE_8BIT,
                             &cmd, 1, 100);
}

/* ============================================================
 * ІНІЦІАЛІЗАЦІЯ ДИСПЛЕЯ
 * ============================================================
 *
 * Послідовність команд ініціалізації — з даташита SSD1306,
 * розділ "10.1.6 Application Example".
 *
 * Після подачі живлення дисплей у невизначеному стані.
 * Ці команди переводять його у робочий режим крок за кроком.
 *
 * Кожна команда пояснена нижче.
 */
HAL_StatusTypeDef SSD1306_Init(SSD1306_Handle *dev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef s;  /* Статус кожної команди */

    dev->hi2c = hi2c;
    dev->cursor_x = 0;
    dev->cursor_y = 0;

    /* Невелика затримка після ввімкнення живлення */
    HAL_Delay(100);

    /* 1. Вимкнути дисплей (поки налаштовуємо) */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_DISPLAY_OFF);     /* 0xAE */
    if (s != HAL_OK) return s;

    /*
     * 2. Встановити частоту оновлення дисплея.
     * 0xD5 — команда, 0x80 — значення (частота за замовчуванням).
     * Старші 4 біти = частота генератора, молодші 4 = дільник.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_DISPLAY_CLK);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x80);
    if (s != HAL_OK) return s;

    /*
     * 3. Мультиплексор — кількість рядків дисплея мінус один.
     * 128×64 → 64 - 1 = 63 (0x3F).
     * Якщо у тебе був би 128×32 — поставив би 31 (0x1F).
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_MULTIPLEX);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x3F);
    if (s != HAL_OK) return s;

    /*
     * 4. Зміщення дисплея (display offset) = 0.
     * Скільки рядків зсунути вгору. Нам зсув не потрібен.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_DISPLAY_OFFSET);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x00);
    if (s != HAL_OK) return s;

    /*
     * 5. Початковий рядок = 0.
     * Вказує, з якого рядка GDDRAM починати відображення.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_START_LINE);  /* 0x40 = рядок 0 */
    if (s != HAL_OK) return s;

    /*
     * 6. Charge Pump — КРИТИЧНО ВАЖЛИВО!
     * OLED-панелі потребують високої напруги (~7В) для роботи.
     * Charge Pump — це вбудований перетворювач напруги.
     * 0x14 = увімкнути charge pump.
     * Без цієї команди дисплей залишиться чорним!
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_CHARGE_PUMP);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x14);
    if (s != HAL_OK) return s;

    /*
     * 7. Режим адресації пам'яті = Horizontal (0x00).
     * Після запису байта автоматично переходить до наступного стовпця.
     * Коли досягне кінця рядка — перейде на наступну сторінку.
     * Це дозволяє відправити весь буфер (1024 байти) одним потоком.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_MEMORY_MODE);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x00);
    if (s != HAL_OK) return s;

    /*
     * 8. Segment Re-map: 0xA1 = стовпець 127 зліва.
     * Це дзеркалює зображення горизонтально.
     * Потрібно для правильної орієнтації конкретного модуля.
     * Якщо текст відображається дзеркально — зміни на 0xA0.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SEG_REMAP);       /* 0xA1 */
    if (s != HAL_OK) return s;

    /*
     * 9. COM Scan Direction: 0xC8 = знизу вгору.
     * Визначає порядок сканування рядків.
     * Якщо зображення перевернуте — зміни на 0xC0.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_COM_SCAN_DEC);    /* 0xC8 */
    if (s != HAL_OK) return s;

    /*
     * 10. Конфігурація COM-пінів.
     * 0x12 для 128×64, 0x02 для 128×32.
     * Це пов'язано з тим, як фізично з'єднані рядки OLED-панелі.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_COM_PINS);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x12);
    if (s != HAL_OK) return s;

    /*
     * 11. Контраст (яскравість): 0x00–0xFF.
     * 0xCF — помірно яскраво. Можна змінити пізніше.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_CONTRAST);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0xCF);
    if (s != HAL_OK) return s;

    /*
     * 12. Pre-charge period.
     * Час заряду/розряду пікселів. 0xF1 — стандарт для charge pump.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_PRECHARGE);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0xF1);
    if (s != HAL_OK) return s;

    /*
     * 13. VCOMH Deselect Level.
     * Напруга для вимкнених пікселів. 0x40 — стандартне значення.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_VCOMH);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x40);
    if (s != HAL_OK) return s;

    /*
     * 14. Відображати вміст RAM (а не все біле/чорне).
     * 0xA4 = показувати те, що в GDDRAM.
     * 0xA5 = увімкнути всі пікселі (тест).
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_ENTIRE_DISPLAY_RAM);  /* 0xA4 */
    if (s != HAL_OK) return s;

    /*
     * 15. Normal display (не інвертований).
     * 0xA6 = білий піксель = 1, чорний = 0.
     * 0xA7 = інвертовано.
     */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_NORMAL_DISPLAY);  /* 0xA6 */
    if (s != HAL_OK) return s;

    /* 16. Очищаємо екранний буфер (заповнюємо чорним) */
    SSD1306_Clear(dev);

    /* 17. Відправляємо чистий буфер на дисплей */
    s = SSD1306_UpdateScreen(dev);
    if (s != HAL_OK) return s;

    /* 18. Увімкнути дисплей! */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_DISPLAY_ON);      /* 0xAF */
    if (s != HAL_OK) return s;

    return HAL_OK;
}

/* ============================================================
 * ОНОВЛЕННЯ ЕКРАНУ
 * ============================================================
 * Відправляє весь буфер (1024 байти) з RAM STM32 у GDDRAM дисплея.
 *
 * Спочатку вказуємо діапазон стовпців (0–127) та сторінок (0–7),
 * потім відправляємо дані. SSD1306 у Horizontal Mode автоматично
 * переміщує "курсор запису" — нам достатньо просто відправити
 * 1024 байти підряд.
 */
HAL_StatusTypeDef SSD1306_UpdateScreen(SSD1306_Handle *dev)
{
    HAL_StatusTypeDef s;

    /* Встановити діапазон стовпців: від 0 до 127 */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_COL_ADDR);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x00);    /* Початковий стовпець */
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x7F);    /* Кінцевий стовпець (127) */
    if (s != HAL_OK) return s;

    /* Встановити діапазон сторінок: від 0 до 7 */
    s = SSD1306_WriteCommand(dev, SSD1306_CMD_SET_PAGE_ADDR);
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x00);    /* Початкова сторінка */
    if (s != HAL_OK) return s;
    s = SSD1306_WriteCommand(dev, 0x07);    /* Кінцева сторінка (7) */
    if (s != HAL_OK) return s;

    /*
     * Відправити 1024 байти екранного буфера.
     * 0x40 = "наступні байти — дані для GDDRAM".
     *
     * HAL_I2C_Mem_Write з MemAddress = 0x40 робить саме це:
     * [I2C addr + Write] → [0x40] → [buf[0]] [buf[1]] ... [buf[1023]]
     */
    s = HAL_I2C_Mem_Write(dev->hi2c, SSD1306_I2C_ADDR,
                           0x40, I2C_MEMADD_SIZE_8BIT,
                           dev->buffer, SSD1306_BUFFER_SIZE, 1000);

    return s;
}

/* ============================================================
 * ФУНКЦІЇ МАЛЮВАННЯ В БУФЕРІ
 * ============================================================
 * Ці функції НЕ відправляють нічого на дисплей.
 * Вони тільки змінюють екранний буфер в RAM.
 * Щоб побачити результат — виклич SSD1306_UpdateScreen().
 */

/*
 * Очистити весь буфер (всі пікселі = чорний).
 * memset заповнює масив нулями — це швидше, ніж цикл.
 */
void SSD1306_Clear(SSD1306_Handle *dev)
{
    memset(dev->buffer, 0x00, SSD1306_BUFFER_SIZE);
    dev->cursor_x = 0;
    dev->cursor_y = 0;
}

/*
 * Заповнити весь буфер (всі пікселі = білий).
 */
void SSD1306_Fill(SSD1306_Handle *dev)
{
    memset(dev->buffer, 0xFF, SSD1306_BUFFER_SIZE);
}

/*
 * Встановити один піксель.
 *
 * Як знайти потрібний байт і біт у буфері:
 *
 * Буфер організований так само, як GDDRAM дисплея:
 * 8 сторінок × 128 стовпців = 1024 байти.
 *
 * Для пікселя (x, y):
 *   Сторінка = y / 8   (ціле ділення — яка група з 8 рядків)
 *   Біт     = y % 8   (залишок — який рядок всередині групи)
 *   Індекс  = сторінка * 128 + x
 *
 * Приклад: піксель (10, 20)
 *   Сторінка = 20 / 8 = 2
 *   Біт     = 20 % 8 = 4
 *   Індекс  = 2 * 128 + 10 = 266
 *   → buffer[266], біт 4
 */
void SSD1306_DrawPixel(SSD1306_Handle *dev, uint16_t x, uint16_t y, SSD1306_Color color)
{
    /* Перевірка меж — не виходити за екран */
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    uint16_t index = (y / 8) * SSD1306_WIDTH + x;
    uint8_t  bit   = y % 8;

    if (color == SSD1306_COLOR_WHITE) {
        dev->buffer[index] |=  (1 << bit);   /* Встановити біт (піксель ON) */
    } else {
        dev->buffer[index] &= ~(1 << bit);   /* Скинути біт (піксель OFF) */
    }
}

/*
 * Горизонтальна лінія — для розділювачів між секціями екрану.
 */
void SSD1306_DrawHLine(SSD1306_Handle *dev, uint16_t x, uint16_t y,
                       uint16_t width, SSD1306_Color color)
{
    for (uint16_t i = 0; i < width && (x + i) < SSD1306_WIDTH; i++) {
        SSD1306_DrawPixel(dev, x + i, y, color);
    }
}

/* ============================================================
 * ФУНКЦІЇ ТЕКСТУ
 * ============================================================ */

/*
 * Встановити позицію курсору.
 * Наступний символ буде виведений у цій позиції.
 */
void SSD1306_SetCursor(SSD1306_Handle *dev, uint16_t x, uint16_t y)
{
    dev->cursor_x = x;
    dev->cursor_y = y;
}

/*
 * Вивести один ASCII-символ.
 *
 * Як це працює:
 * 1. Знаходимо символ у масиві шрифту: індекс = (ch - 32) * 5.
 * 2. Читаємо 5 байтів — це 5 стовпців символу.
 * 3. Для кожного стовпця перевіряємо 7 біт — це 7 рядків.
 * 4. Якщо біт = 1 → малюємо піксель.
 * 5. Зсуваємо курсор вправо на ширину символу + відступ.
 */
void SSD1306_WriteChar(SSD1306_Handle *dev, char ch, SSD1306_Color color)
{
    /* Перевірка: чи символ в діапазоні шрифту */
    if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR) {
        ch = '?';  /* Невідомий символ → виводимо знак питання */
    }

    /* Перевірка: чи вміститься символ на екрані */
    if (dev->cursor_x + FONT_WIDTH > SSD1306_WIDTH) {
        /* Не вміщується — перенос на новий рядок */
        dev->cursor_x = 0;
        dev->cursor_y += FONT_HEIGHT + 1;
    }

    if (dev->cursor_y + FONT_HEIGHT > SSD1306_HEIGHT) {
        /* Екран заповнений — більше не малюємо */
        return;
    }

    /* Знаходимо початок символу в масиві шрифту */
    uint16_t offset = (ch - FONT_FIRST_CHAR) * FONT_WIDTH;

    /* Малюємо 5 стовпців символу */
    for (uint8_t col = 0; col < FONT_WIDTH; col++) {
        uint8_t column_data = Font5x7[offset + col];

        /* Малюємо 7 рядків у стовпці */
        for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
            if (column_data & (1 << row)) {
                /* Біт = 1 → піксель символу */
                SSD1306_DrawPixel(dev,
                                  dev->cursor_x + col,
                                  dev->cursor_y + row,
                                  color);
            } else {
                /* Біт = 0 → фон (протилежний колір) */
                SSD1306_DrawPixel(dev,
                                  dev->cursor_x + col,
                                  dev->cursor_y + row,
                                  !color);
            }
        }
    }

    /* Малюємо стовпець відступу між символами (фон) */
    for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
        SSD1306_DrawPixel(dev,
                          dev->cursor_x + FONT_WIDTH,
                          dev->cursor_y + row,
                          !color);
    }

    /* Зсуваємо курсор вправо */
    dev->cursor_x += FONT_CHAR_W;
}

/*
 * Вивести рядок тексту.
 * Просто викликає WriteChar для кожного символу.
 */
void SSD1306_WriteString(SSD1306_Handle *dev, const char *str, SSD1306_Color color)
{
    while (*str != '\0') {
        SSD1306_WriteChar(dev, *str, color);
        str++;
    }
}
