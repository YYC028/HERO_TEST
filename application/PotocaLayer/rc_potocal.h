#ifndef RC_POTOCAL
#define RC_POTOCAL
  #include "struct_typedef.h"
  #include "drv_can.h"
  #include "main.h"


void USART3_rxDataHandler(uint8_t *rxBuf);


typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;








#endif