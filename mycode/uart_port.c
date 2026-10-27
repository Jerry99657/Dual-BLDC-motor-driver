#include "uart_port.h"
#include "main.h"
#include "protocol.h"
#include "stm32g4xx_ll_usart.h"
#include <string.h>

static uint8_t tx_buffer[128];
static const uint8_t *tx_next;
static uint16_t tx_left;
#define TX_INTERRUPTS (USART_CR1_TXEIE_TXFNFIE|USART_CR1_TCIE)
#define RX_ERRORS (USART_ISR_PE|USART_ISR_FE|USART_ISR_NE|USART_ISR_ORE)
_Static_assert(RX_ERRORS==(USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_ORECF),
               "USART error flags must map to W1C acknowledgements");

void Uart1_Init(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    LL_USART_DisableIT_TXE_TXFNF(USART1);
    LL_USART_DisableIT_TC(USART1);
    LL_USART_ClearFlag_TC(USART1);
    /* Retain any pending RX/error: the handler discards a damaged line.
     * RX remains armed through errors; no HAL rearm/state machine needed. */
    LL_USART_EnableIT_ERROR(USART1);
    LL_USART_EnableIT_RXNE_RXFNE(USART1);
    __set_PRIMASK(mask);
}

bool Uart1_TrySend(const uint8_t *data,uint16_t length)
{
    if(!data || !length || length>sizeof(tx_buffer)) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ready=LL_USART_IsEnabledIT_RXNE_RXFNE(USART1) &&
        !(LL_USART_ReadReg(USART1,CR1)&TX_INTERRUPTS);
    if(ready) {
        memcpy(tx_buffer,data,length);
        tx_next=tx_buffer; tx_left=length;
        LL_USART_ClearFlag_TC(USART1);
        LL_USART_EnableIT_TXE_TXFNF(USART1);
    }
    __set_PRIMASK(mask);
    return ready;
}

void Uart1_IRQHandler(void)
{
    uint32_t status=LL_USART_ReadReg(USART1,ISR);
    uint32_t enabled=LL_USART_ReadReg(USART1,CR1);
    if((enabled&(USART_CR1_RXNEIE_RXFNEIE|USART_CR1_PEIE)) ||
       (LL_USART_ReadReg(USART1,CR3)&USART_CR3_EIE)) {
        uint32_t errors=status&RX_ERRORS;
        if(errors) {
            /* ICR is W1C. Do not acknowledge TC/IDLE or consume a new byte
             * with a late flush. Drop until a subsequent clean newline. */
            LL_USART_WriteReg(USART1,ICR,errors);
            Protocol_DiscardLine(PROTO_SRC_USART);
        }
        if((status&USART_ISR_RXNE_RXFNE) && (errors || (enabled&USART_CR1_RXNEIE_RXFNEIE))) {
            uint8_t byte=LL_USART_ReceiveData8(USART1);
            if(!errors) Protocol_Feed(PROTO_SRC_USART,&byte,1U);
        }
    }
    if((status&USART_ISR_TXE_TXFNF) && (enabled&USART_CR1_TXEIE_TXFNFIE)) {
        LL_USART_TransmitData8(USART1,*tx_next++);
        if(--tx_left==0U) {
            LL_USART_DisableIT_TXE_TXFNF(USART1);
            LL_USART_EnableIT_TC(USART1);
        }
    } else if((status&USART_ISR_TC) && (enabled&USART_CR1_TCIE)) {
        LL_USART_ClearFlag_TC(USART1);
        LL_USART_DisableIT_TC(USART1);
    }
}
