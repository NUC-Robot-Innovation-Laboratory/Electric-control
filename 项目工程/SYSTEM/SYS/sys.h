#ifndef _SYS_H_
#define _SYS_H_

#include "stm32f10x.h"
#include "led.h"

//公式参考: Cortex-M3权威指南92页
//完成位带地址的转换
//addr表示寄存器的基地址
//bitnum表示寄存器里的第几位
#define BITBAND(addr,bitnum) ((addr&0xF0000000)+0x2000000+((addr&0xFFFFF)<<5)+(bitnum<<2))
//将地址转换为指针类型
#define MEM_ADDR(addr) *((volatile u32 *)addr)

/*定义GPIO口输入输出寄存器的地址*/
#define GPIOA_IDR (0x40010800+0x8)
#define GPIOA_ODR (0x40010800+0xC)
#define GPIOB_IDR (0x40010C00+0x8)
#define GPIOB_ODR (0x40010C00+0xC)
#define GPIOC_IDR (0x40011000+0x8)
#define GPIOC_ODR (0x40011000+0xC)
#define GPIOD_IDR (0x40011400+0x8)
#define GPIOD_ODR (0x40011400+0xC)
#define GPIOE_IDR (0x40011800+0x8)
#define GPIOE_ODR (0x40011800+0xC)
#define GPIOF_IDR (0x40011C00+0x8)
#define GPIOF_ODR (0x40011C00+0xC)
#define GPIOG_IDR (0x40012000+0x8)
#define GPIOG_ODR (0x40012000+0xC)

/*定义GPIO口位操作宏*/
#define PAin(bitnum)  MEM_ADDR(BITBAND(GPIOA_IDR,bitnum))
#define PAout(bitnum) MEM_ADDR(BITBAND(GPIOA_ODR,bitnum))
#define PBin(bitnum)  MEM_ADDR(BITBAND(GPIOB_IDR,bitnum))
#define PBout(bitnum) MEM_ADDR(BITBAND(GPIOB_ODR,bitnum))
#define PCin(bitnum)  MEM_ADDR(BITBAND(GPIOC_IDR,bitnum))
#define PCout(bitnum) MEM_ADDR(BITBAND(GPIOC_ODR,bitnum))
#define PDin(bitnum)  MEM_ADDR(BITBAND(GPIOD_IDR,bitnum))
#define PDout(bitnum) MEM_ADDR(BITBAND(GPIOD_ODR,bitnum))
#define PEin(bitnum)  MEM_ADDR(BITBAND(GPIOE_IDR,bitnum))
#define PEout(bitnum) MEM_ADDR(BITBAND(GPIOE_ODR,bitnum))
#define PFin(bitnum)  MEM_ADDR(BITBAND(GPIOF_IDR,bitnum))
#define PFout(bitnum) MEM_ADDR(BITBAND(GPIOF_ODR,bitnum))
#define PGin(bitnum)  MEM_ADDR(BITBAND(GPIOG_IDR,bitnum))
#define PGout(bitnum) MEM_ADDR(BITBAND(GPIOG_ODR,bitnum))


/**
@code  
 The table below gives the allowed values of the pre-emption priority and subpriority according
 to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
  ============================================================================================================================
    NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority（抢占优先级） | NVIC_IRQChannelSubPriority(次优先级)  | Description
  ============================================================================================================================
   NVIC_PriorityGroup_0  |                0                  |            0-15             |   0 bits for pre-emption priority
                         |                                   |                             |   4 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------
   NVIC_PriorityGroup_1  |                0-1                |            0-7              |   1 bits for pre-emption priority
                         |                                   |                             |   3 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_2  |                0-3                |            0-3              |   2 bits for pre-emption priority
                         |                                   |                             |   2 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_3  |                0-7                |            0-1              |   3 bits for pre-emption priority
                         |                                   |                             |   1 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_4  |                0-15               |            0                |   4 bits for pre-emption priority
                         |                                   |                             |   0 bits for subpriority                       
  ============================================================================================================================
@endcode
*/
#define NVIC_PriorityGroup_0         ((uint32_t)0x7) /*!< 0 bits for pre-emption priority
                                                            4 bits for subpriority */
#define NVIC_PriorityGroup_1         ((uint32_t)0x6) /*!< 1 bits for pre-emption priority
                                                            3 bits for subpriority */
#define NVIC_PriorityGroup_2         ((uint32_t)0x5) /*!< 2 bits for pre-emption priority
                                                            2 bits for subpriority */
#define NVIC_PriorityGroup_3         ((uint32_t)0x4) /*!< 3 bits for pre-emption priority
                                                            1 bits for subpriority */
#define NVIC_PriorityGroup_4         ((uint32_t)0x3) /*!< 4 bits for pre-emption priority
                                                            0 bits for subpriority */
void STM32_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void SysTickInit(void);
#endif


