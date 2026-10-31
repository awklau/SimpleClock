/* SPDX-License-Identifier: MIT */
/* Test-only boundary for compiling the real M4 platform adapter on the host.
 * No register layout, electrical behavior, IRQ latency or hardware is simulated. */
#ifndef SIMPLECLOCK_TEST_HAL_H
#define SIMPLECLOCK_TEST_HAL_H
#include <stdint.h>

typedef struct
{
    uint32_t unused;
} I2C_HandleTypeDef;

typedef struct
{
    uint32_t BRR, BSRR, MODER, ODR, IDR;
} GPIO_TypeDef;

typedef enum
{
    HAL_OK,
    HAL_ERROR
} HAL_StatusTypeDef;

extern GPIO_TypeDef test_gpio_b;

typedef struct
{
    uint32_t ISR, ICR;
} I2C_TypeDef;

extern I2C_TypeDef test_i2c2;
#define I2C2 (&test_i2c2)
#define I2C_ISR_BUSY 0x8000U
#define I2C_ISR_BERR 0x100U
#define I2C_ICR_BERRCF 0x100U

/* Hybrid host tests only need the free-running timing probe counter. */
typedef struct
{
    uint32_t CNT;
} TIM_TypeDef;

extern TIM_TypeDef test_tim2;
#define TIM2 (&test_tim2)

#define GPIOB (&test_gpio_b)
#define GPIO_PIN_3 8U
#define GPIO_PIN_2 4U
extern GPIO_TypeDef test_gpio_a;
#define GPIOA (&test_gpio_a)

typedef struct
{
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

typedef struct
{
    uint32_t BRR, CR1, CR3, ISR, ICR, RDR, TDR;
} USART_TypeDef;

extern USART_TypeDef test_usart2;
#define USART2 (&test_usart2)
#define GPIO_MODE_AF_PP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF1_USART2 1U
#define USART2_IRQn 0U
#define USART_CR3_EIE 1U
#define USART_CR3_DMAR 2U
#define USART_CR1_UE 1U
#define USART_CR1_RE 2U
#define USART_CR1_TE 4U
#define USART_CR1_RXNEIE_RXFNEIE 8U
#define USART_ISR_TXE_TXFNF 1U
#define USART_ISR_RXNE_RXFNE 2U
#define USART_ISR_ORE 4U
#define USART_ISR_FE 8U
#define USART_ISR_NE 16U
#define USART_ISR_PE 32U
#define USART_ISR_TC 64U
#define USART_ICR_ORECF 4U
#define USART_ICR_FECF 8U
#define USART_ICR_NECF 16U
#define USART_ICR_PECF 32U
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART2_FORCE_RESET() ((void)0)
#define __HAL_RCC_USART2_RELEASE_RESET() ((void)0)

/* Register-level circular RX model. It models DMA progress while CPU IRQs
 * are masked, not target bus timing or peripheral electrical behavior. */
typedef struct
{
    uint32_t ISR, IFCR;
} DMA_TypeDef;

typedef struct
{
    uint32_t CCR, CNDTR;
    uintptr_t CPAR, CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
    uint32_t CCR;
} DMAMUX_Channel_TypeDef;

extern DMA_TypeDef test_dma1;
extern DMA_Channel_TypeDef test_dma1_channel1;
extern DMAMUX_Channel_TypeDef test_dmamux1_channel0;
#define DMA1 (&test_dma1)
#define DMA1_Channel1 (&test_dma1_channel1)
#define DMAMUX1_Channel0 (&test_dmamux1_channel0)
#define DMA1_Channel1_IRQn 1U
#define DMA_CCR_EN 1U
#define DMA_CCR_TCIE 2U
#define DMA_CCR_TEIE 8U
#define DMA_CCR_CIRC 32U
#define DMA_CCR_MINC 128U
#define DMA_CCR_PL_1 8192U
#define DMA_ISR_TCIF1 2U
#define DMA_ISR_TEIF1 8U
#define DMA_IFCR_CGIF1 1U
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
void HAL_GPIO_Init(GPIO_TypeDef *gpio, const GPIO_InitTypeDef *config);
void HAL_NVIC_SetPriority(unsigned irq, unsigned priority, unsigned subpriority);
void HAL_NVIC_EnableIRQ(unsigned irq);
#define HAL_BIND_TSSOP20_PIN20 0x30U
#define HAL_BIND_TSSOP20_PIN20_PB3 0x00300010U
uint32_t HAL_SYSCFG_GetPinBinding(uint32_t source);
#define I2C_MEMADD_SIZE_8BIT 1U
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address, uint16_t reg,
                                   uint16_t address_size, uint8_t *data, uint16_t size,
                                   uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address, uint16_t reg,
                                    uint16_t address_size, uint8_t *data, uint16_t size,
                                    uint32_t timeout);
#endif
