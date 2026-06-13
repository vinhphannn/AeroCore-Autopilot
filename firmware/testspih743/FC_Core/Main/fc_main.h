#ifndef FC_MAIN_H
#define FC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * Hàm này sẽ được gọi ở cuối hàm main() trong Core/Src/main.c
 * Mục đích: Nhường quyền điều khiển từ CubeIDE (C thuần) sang kiến trúc FC (C++)
 */
void FC_Run(void);

#ifdef __cplusplus
}
#endif

#endif // FC_MAIN_H
