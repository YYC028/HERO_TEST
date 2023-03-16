  #include "rc_potocal.h"


  uint8_t temp_remote[8];
 RC_ctrl_t rc_ctrl;
 #define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
   void USART3_rxDataHandler(uint8_t *rxBuf)
{
    rc_ctrl.rc.ch[0] = (rxBuf[0] | (rxBuf[1] << 8)) & 0x07ff;        //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
    rc_ctrl.rc.ch[1] = (((rxBuf[1] >> 3)&0xff) | (rxBuf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = (((rxBuf[2] >> 6)&0xff) | (rxBuf[3] << 2) |          //!< Channel 2
                         (rxBuf[4] << 10)) &0x07ff;
    rc_ctrl.rc.ch[3] = (((rxBuf[4] >> 1)&0xff) | (rxBuf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((rxBuf[5] >> 4) & 0x0003);                  //!< Switch left！！！这尼玛是右
    rc_ctrl.rc.s[1] = ((rxBuf[5] >> 4) & 0x000C) >> 2;    		//!< Switch right！！！这才是左
    rc_ctrl.mouse.x = rxBuf[6] | (rxBuf[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = rxBuf[8] | (rxBuf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = rxBuf[10] | (rxBuf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse.press_l = rxBuf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = rxBuf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key.v = rxBuf[14] | (rxBuf[15] << 8);                    //!< KeyBoard value	
    rc_ctrl.rc.ch[4] = rxBuf[16] | (rxBuf[17] << 8);                 //NULL

    rc_ctrl.rc.ch[0]-=RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1]-=RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2]-=RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3]-=RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4]-=RC_CH_VALUE_OFFSET;
		for(int i=0;i<=7;i++)
		{
			temp_remote[i]=rxBuf[i];//volatile const uint8_t和uint8_t不一样不能直接带入can_remote这个函数
		}
		can_remote(temp_remote,0x33);
		
		for(int i=8;i<=15;i++)
		{
			temp_remote[i-8]=rxBuf[i];//volatile const uint8_t和uint8_t不一样不能直接带入can_remote这个函数
		}
		can_remote(temp_remote,0x34);
		
		temp_remote[0]=rxBuf[16];
		temp_remote[1]=rxBuf[17];
		temp_remote[2]=rxBuf[18];
		for(int i=3;i<=7;i++)
		{
			temp_remote[i]=0;
		}
		can_remote(temp_remote,0x35);
    
  // HAL_GPIO_TogglePin( GPIOH, LED_GREEN_Pin);





}