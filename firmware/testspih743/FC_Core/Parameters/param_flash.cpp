#include "param_flash.h"
#include "main.h" 

// TÙY CHỌN: Ở Phase 5, bạn chọn một Sector rác ở cuối Flash để lưu trữ.
// Ví dụ: Sector 7 của Bank 2 trên chip STM32H743 (Dung lượng 128KB).
#define PARAM_FLASH_ADDR 0x081E0000

int param_flash_load(union param_value_u* active_array, bool* dirty_array, uint16_t count) {
    //TODO: Phase 5
    // Đọc từ PARAM_FLASH_ADDR.
    // Nếu Flash trống (0xFFFFFFFF), bỏ qua (giữ nguyên mặc định của RAM).
    // Nếu Flash có Data (BSON hoặc Struct lưu tạm), copy đè vào active_array.
    return 0; 
}

int param_flash_save(const union param_value_u* active_array, const bool* dirty_array, uint16_t count) {
    //TODO: Phase 5
    // 1. HAL_FLASH_Unlock();
    // 2. FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_BANK_2, ...);
    // 3. Quét vòng lặp từ 0 tới count:
    //      Nếu dirty_array[i] == true:
    //          Ghi giá trị active_array[i] xuống địa chỉ nhớ bằng HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, ...)
    // 4. HAL_FLASH_Lock();
    return 0;
}
