#include "main.h"
#include "uart_port.h"
#include "protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

USART_TypeDef uart_regs;
uint32_t host_primask;
static uint8_t received[4096],wire[128];
static unsigned received_count,wire_count,errors,rdr_reads;
static bool inject_after_read,early_complete;
static const uint32_t rx_errors=USART_ISR_PE|USART_ISR_FE|USART_ISR_NE|USART_ISR_ORE;
static const uint32_t tx_interrupts=USART_CR1_TXEIE_TXFNFIE|USART_CR1_TCIE;
uint32_t HostRead(const uint32_t *reg)
{
    uint32_t value=*reg;
    if(reg==&uart_regs.RDR) {
        ++rdr_reads; uart_regs.ISR&=~USART_ISR_RXNE_RXFNE;
        if(inject_after_read) {
            inject_after_read=false;uart_regs.RDR='B';uart_regs.ISR|=USART_ISR_RXNE_RXFNE;
        }
    }
    return value;
}
void HostWrite(uint32_t *reg,uint32_t value)
{
    assert(reg==&uart_regs.ICR);
    assert(!(value&~(rx_errors|USART_ICR_TCCF)));
    uart_regs.ISR&=~value; /* Actual G431 W1C bit positions. */
}
void Protocol_Feed(uint8_t source,const uint8_t *p,uint32_t n)
{
    assert(source==PROTO_SRC_USART && n==1 && received_count<sizeof(received));
    received[received_count++]=*p;
}
void Protocol_DiscardLine(uint8_t source) { assert(source==PROTO_SRC_USART);++errors; }
static void irq(void)
{
    uart_regs.TDR=0x1000U;
    Uart1_IRQHandler();
    if(uart_regs.TDR!=0x1000U) {
        assert(wire_count<sizeof(wire));wire[wire_count++]=(uint8_t)uart_regs.TDR;
        uart_regs.ISR&=~(USART_ISR_TC|USART_ISR_TXE_TXFNF);
    }
}
static void drain(void)
{
    unsigned guard=0;
    while(uart_regs.CR1&USART_CR1_TXEIE_TXFNFIE) {
        assert(++guard<=128);uart_regs.ISR|=USART_ISR_TXE_TXFNF;irq();
    }
    assert(uart_regs.CR1&USART_CR1_TCIE);
    uart_regs.ISR|=USART_ISR_TC;irq();
    assert(!(uart_regs.CR1&tx_interrupts));
}
void __set_PRIMASK(uint32_t mask)
{
    host_primask=mask;
    if(early_complete && !mask) { early_complete=false;drain(); }
}
static void reset(void)
{
    uart_regs=(USART_TypeDef){0};host_primask=0;
    received_count=wire_count=errors=rdr_reads=0;inject_after_read=early_complete=false;
    Uart1_Init();
    assert(uart_regs.CR1==USART_CR1_RXNEIE_RXFNEIE && uart_regs.CR3==USART_CR3_EIE);
}
static void receive(uint8_t byte,uint32_t bad)
{
    uart_regs.RDR=byte;uart_regs.ISR|=USART_ISR_RXNE_RXFNE|bad;irq();
}
static void check_receive(void)
{
    reset();
    for(unsigned i=0;i<256;++i) receive((uint8_t)i,0);
    assert(received_count==256 && rdr_reads==256 && !errors);
    for(unsigned i=0;i<256;++i) assert(received[i]==i);
    /* A byte arriving after RDR consumption stays pending, no late flush. */
    inject_after_read=true;receive('A',0);
    assert(uart_regs.ISR&USART_ISR_RXNE_RXFNE);irq();
    assert(received[256]=='A' && received[257]=='B');
    const uint32_t bits[]={USART_ISR_PE,USART_ISR_FE,USART_ISR_NE,USART_ISR_ORE};
    for(unsigned mask=1;mask<16;++mask) {
        uint32_t bad=0;for(unsigned i=0;i<4;++i) if(mask&(1U<<i)) bad|=bits[i];
        unsigned before=received_count,old=errors;
        uart_regs.ISR|=USART_ISR_IDLE|USART_ISR_TC;
        receive('X',bad);
        assert(received_count==before && errors==old+1 && !(uart_regs.ISR&rx_errors));
        assert((uart_regs.ISR&(USART_ISR_IDLE|USART_ISR_TC))==(USART_ISR_IDLE|USART_ISR_TC));
        assert((uart_regs.CR1&USART_CR1_RXNEIE_RXFNEIE) && (uart_regs.CR3&USART_CR3_EIE));
        receive('Y',0);assert(received[received_count-1]=='Y');
    }
    /* Disabled sources cannot consume good RX or clear unrelated flags. */
    uart_regs.CR1=uart_regs.CR3=0;unsigned before=received_count,old=errors,reads=rdr_reads;
    receive('Z',rx_errors);
    assert(received_count==before && errors==old && rdr_reads==reads && (uart_regs.ISR&rx_errors)==rx_errors);
    /* Error-only source drains the damaged byte while RXNE is masked. */
    uart_regs.CR3=USART_CR3_EIE;irq();
    assert(errors==old+1 && !(uart_regs.ISR&(rx_errors|USART_ISR_RXNE_RXFNE)));
    reset();uart_regs.ISR=USART_ISR_ORE;irq();assert(errors==1 && !rdr_reads);
    receive('Q',0);assert(received_count==1);
    puts("PASS: real LL USART RX 0..255, all PE/FE/NE/ORE combinations, W1C/disabled-source semantics, continuous error recovery and next-byte preservation");
}
static void check_transmit(void)
{
    reset();
    uint8_t payload[129];for(unsigned i=0;i<sizeof(payload);++i) payload[i]=(uint8_t)i;
    assert(!Uart1_TrySend(NULL,1) && !Uart1_TrySend(payload,0) && !Uart1_TrySend(payload,129));
    for(unsigned length=1;length<=128;++length) {
        reset();uint8_t expected[128];memcpy(expected,payload,length);
        host_primask=1;assert(Uart1_TrySend(payload,(uint16_t)length) && host_primask==1);host_primask=0;
        memset(payload,0xA5,length); /* Caller storage may be immediately reused. */
        for(unsigned i=0;i<length;++i) {
            assert(!Uart1_TrySend(payload,1));
            uart_regs.ISR|=USART_ISR_TXE_TXFNF|USART_ISR_TC;
            /* Errors and RX can arrive while TX is active. */
            uart_regs.RDR=(uint8_t)i;uart_regs.ISR|=USART_ISR_RXNE_RXFNE;
            if(i==0) uart_regs.ISR|=USART_ISR_ORE;
            irq();assert(wire_count==i+1);
        }
        assert(!memcmp(wire,expected,length) && errors==1 && received_count==length-1);
        /* Last data register empty is not last stop bit complete. */
        uart_regs.ISR|=USART_ISR_TXE_TXFNF;irq();
        assert(wire_count==length && !Uart1_TrySend(payload,1));
        uart_regs.ISR|=USART_ISR_TC;irq();
        assert(Uart1_TrySend(payload,1));wire_count=0;drain();
        for(unsigned i=0;i<sizeof(payload);++i) payload[i]=(uint8_t)(i+length);
    }
    reset();early_complete=true;assert(Uart1_TrySend(payload,128));
    assert(!memcmp(wire,payload,128) && !host_primask && !(uart_regs.CR1&tx_interrupts));
    wire_count=0;assert(Uart1_TrySend(payload,1));drain();assert(wire_count==1);
    uart_regs.CR1=0;assert(!Uart1_TrySend(payload,1)); /* Before runtime start. */
    reset();host_primask=1;Uart1_Init();assert(host_primask==1);host_primask=0;
    puts("PASS: real LL USART copied TX lengths 1..128, full duplex/error during TX, BUSY immutability, TC-only release, immediate completion, invalid frames and IRQ-mask preservation");
}
int main(void) { check_receive();check_transmit();return 0; }
