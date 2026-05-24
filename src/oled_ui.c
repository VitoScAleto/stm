#include "../inc/oled_ui.h"


typedef enum {
    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_OK,
    KEY_BACK
} Key_t;

typedef enum {
    UI_MAIN = 0,
    UI_SELECT_FW,
    UI_BUSY,
    UI_MESSAGE
} UiState_t;

UiState_t ui_state = UI_MAIN;
uint32_t ui_menu_index = 0;
uint32_t ui_fw_index = 0;

Key_t Buttons_GetKey(void);

void OLED_DoFlashSelected(void)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(count == 0) {
        OLED_Status("Error", "No firmware");
        return;
    }

    if(ui_fw_index >= count)
        ui_fw_index = count - 1;

    OLED_Status("Flash selected", (char*)headers[ui_fw_index].name);

    int ok = FlashFirmwareFromAT25_Index(ui_fw_index, 1);

    if(ok)
        OLED_Status("Flash OK", "Target started");
    else
        OLED_Status("Flash ERROR", "Check SWD");
}

void OLED_DoVerifySelected(void)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(count == 0) {
        OLED_Status("Error", "No firmware");
        return;
    }

    if(ui_fw_index >= count)
        ui_fw_index = count - 1;

    OLED_Status("Verify CRC", "Working...");

    uint32_t crc = 0;
    bool ok = Firmware_Verify(addresses[ui_fw_index], &crc);

    char line[32];
    snprintf(line, sizeof(line), "%08lX", (unsigned long)crc);

    if(ok)
        OLED_Status("CRC OK", line);
    else
        OLED_Status("CRC ERROR", line);
}

void OLED_DoMemoryInfo(void)
{
    uint32_t total, used, free_space;
    Firmware_GetMemoryInfo(&total, &used, &free_space);

    char line1[32];
    char line2[32];

    snprintf(line1, sizeof(line1), "Used:%lu", (unsigned long)used);
    snprintf(line2, sizeof(line2), "Free:%lu", (unsigned long)free_space);

    OLED_Clear();
    OLED_DrawString(0, 0, "AT25 Memory", 1);
    OLED_DrawString(0, 16, line1, 1);
    OLED_DrawString(0, 26, line2, 1);
    OLED_DrawString(0, 56, "BACK", 1);
    OLED_Update();
}
void OLED_MenuTask(void)
{
    Key_t key = Buttons_GetKey();

    if(key == KEY_NONE)
        return;

    if(ui_state == UI_MAIN) {
        if(key == KEY_UP) {
            if(ui_menu_index == 0)
                ui_menu_index = 2;
            else
                ui_menu_index--;

            OLED_ShowMainMenu();
        }
        else if(key == KEY_DOWN) {
            ui_menu_index++;
            if(ui_menu_index > 2)
                ui_menu_index = 0;

            OLED_ShowMainMenu();
        }
        else if(key == KEY_OK) {
            if(ui_menu_index == 0) {
                ui_state = UI_SELECT_FW;
                OLED_ShowFirmwareSelect();
            }
            else if(ui_menu_index == 1) {
                ui_state = UI_SELECT_FW;
                OLED_ShowFirmwareSelect();
            }
            else if(ui_menu_index == 2) {
                ui_state = UI_MESSAGE;
                OLED_DoMemoryInfo();
            }
        }
    }
    else if(ui_state == UI_SELECT_FW) {
        FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
        uint32_t addresses[MAX_FIRMWARE_COUNT];
        uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

        if(key == KEY_BACK) {
            ui_state = UI_MAIN;
            OLED_ShowMainMenu();
            return;
        }

        if(count == 0) {
            OLED_ShowFirmwareSelect();
            return;
        }

        if(key == KEY_UP) {
            if(ui_fw_index == 0)
                ui_fw_index = count - 1;
            else
                ui_fw_index--;

            OLED_ShowFirmwareSelect();
        }
        else if(key == KEY_DOWN) {
            ui_fw_index++;
            if(ui_fw_index >= count)
                ui_fw_index = 0;

            OLED_ShowFirmwareSelect();
        }
        else if(key == KEY_OK) {
            ui_state = UI_BUSY;

            if(ui_menu_index == 0)
                OLED_DoFlashSelected();
            else if(ui_menu_index == 1)
                OLED_DoVerifySelected();

            ui_state = UI_MESSAGE;
        }
    }
    else if(ui_state == UI_MESSAGE) {
        if(key == KEY_BACK || key == KEY_OK) {
            ui_state = UI_MAIN;
            OLED_ShowMainMenu();
        }
    }
}

void Buttons_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // PB12 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    GPIOB->CRH |= GPIO_CRH_CNF12_1;
    GPIOB->ODR |= BTN_OK_MASK;

    // PB13 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOB->CRH |= GPIO_CRH_CNF13_1;
    GPIOB->ODR |= BTN_BACK_MASK;

    // PB10 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOB->CRH |= GPIO_CRH_CNF10_1;
    GPIOB->ODR |= BTN_UP_MASK;

    // PB11 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_CNF11_1;
    GPIOB->ODR |= BTN_DOWN_MASK;
}

uint8_t Button_IsPressed(uint32_t mask)
{
    return (GPIOB->IDR & mask) == 0;
}

Key_t Buttons_GetKey(void)
{
    static uint32_t lock = 0;

    if(lock) {
        lock--;
        return KEY_NONE;
    }

    if(Button_IsPressed(BTN_UP_MASK)) {
        lock = 70000;
        return KEY_UP;
    }

    if(Button_IsPressed(BTN_DOWN_MASK)) {
        lock = 70000;
        return KEY_DOWN;
    }

    if(Button_IsPressed(BTN_OK_MASK)) {
        lock = 70000;
        return KEY_OK;
    }

    if(Button_IsPressed(BTN_BACK_MASK)) {
        lock = 70000;
        return KEY_BACK;
    }

    return KEY_NONE;
}

void OLED_Status(const char* a, const char* b)
{
    OLED_Clear();
    OLED_DrawString(0, 0, a, 1);
    OLED_DrawString(0, 16, b, 1);
    OLED_Update();
}

void OLED_ShowMainMenu(void)
{
    OLED_Clear();

    OLED_DrawString(0, 0, "SWD PROGRAMMER", 1);

    OLED_DrawString(0, 16, ui_menu_index == 0 ? "> Flash AT25" : "  Flash AT25", 1);
    OLED_DrawString(0, 26, ui_menu_index == 1 ? "> Verify CRC" : "  Verify CRC", 1);
    OLED_DrawString(0, 36, ui_menu_index == 2 ? "> Memory Info" : "  Memory Info", 1);

    OLED_Update();
}

void OLED_ShowFirmwareSelect(void)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    OLED_Clear();

    if(count == 0) {
        OLED_DrawString(0, 0, "No firmware", 1);
        OLED_DrawString(0, 16, "BACK", 1);
        OLED_Update();
        return;
    }

    if(ui_fw_index >= count)
        ui_fw_index = count - 1;

    char line[32];

    OLED_DrawString(0, 0, "Select firmware", 1);

    snprintf(line, sizeof(line), "%lu/%lu",
             (unsigned long)(ui_fw_index + 1),
             (unsigned long)count);
    OLED_DrawString(96, 0, line, 1);

    OLED_DrawString(0, 16, ">", 1);
    OLED_DrawString(8, 16, (char*)headers[ui_fw_index].name, 1);

    snprintf(line, sizeof(line), "Size:%lu",
             (unsigned long)headers[ui_fw_index].size);
    OLED_DrawString(0, 32, line, 1);

    snprintf(line, sizeof(line), "CRC:%08lX",
             (unsigned long)headers[ui_fw_index].crc32);
    OLED_DrawString(0, 42, line, 1);

    OLED_DrawString(0, 56, "OK=Flash BACK=Exit", 1);

    OLED_Update();
}