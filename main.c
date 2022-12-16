/*!
    \file    main.c
    \brief   GPIO running LED demo

    \version 2016-01-15, V1.0.0, demo for GD32F1x0
    \version 2016-05-13, V2.0.0, demo for GD32F1x0
    \version 2019-11-20, V3.0.0, demo for GD32F1x0
    \version 2021-12-31, V3.1.0, demo for GD32F1x0
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f1x0.h"
#include "systick.h"

#define UART_OUT_TIME 50000

#define   R_CTRL  GPIO_PIN_14
#define   G_CTRL  GPIO_PIN_15

#define   R_OFF  GPIO_BOP(GPIOC)=R_CTRL 
#define   R_ON   GPIO_BC(GPIOC)=R_CTRL 
#define   G_OFF  GPIO_BOP(GPIOC)=G_CTRL 
#define   G_ON   GPIO_BC(GPIOC)=G_CTRL 

#define   MDUTY  20
#define   MDT  15

unsigned int u_timeout , hallstaA , hallstaB, hallstaF , result ;
unsigned char datin_buf[20] ;
unsigned char buf_add , trasnmark , trasnmarkdone , dat2s , dat2send , dat2sendcnt ;
unsigned char nummark ;

unsigned char pos[6],res[4] ;

unsigned char unicnt , openloadcnt , shortloadcnt , shortgndcnt ;

void RCC_Configuration(void)
{
  ErrStatus ErrStatusFlag;
  rcu_deinit();
  rcu_osci_on(RCU_IRC8M);
  rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M);
  ErrStatusFlag = ERROR ;
  while(ErrStatusFlag != SUCCESS)
  {
          ErrStatusFlag = rcu_osci_stab_wait(RCU_IRC8M);
          rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
          rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);
          rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);
          rcu_pll_config(RCU_PLLSRC_IRC8M_DIV2,RCU_PLL_MUL12);
          rcu_system_clock_source_config(RCU_CKSYSSRC_PLL);
          RCU_CTL0 |= RCU_CTL0_PLLEN;
  }
}

void uart_init(void)
{
  nvic_irq_enable(USART0_IRQn, 0, 1);
  
  rcu_periph_clock_enable(RCU_GPIOA);

  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART0);

  /* connect port to USARTx_Tx */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);

  /* connect port to USARTx_Rx */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);

  /* configure USART Tx as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);

  /* configure USART Rx as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);

  /* USART configure */
  usart_deinit(USART0);
  usart_baudrate_set(USART0, 9600U);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_enable(USART0);
  
  /* enable USART TBE interrupt */
  //usart_interrupt_enable(USART0, USART_INT_TBE);
  /* enable USART RBNE interrupt */
  usart_interrupt_enable(USART0, USART_INT_RBNE);
}

void USART0_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        /* receive data */
        //rx_buffer[rx_count++] = usart_data_receive(USART0);
        //usart_data_transmit(USART0, usart_data_receive(USART0));
        datin_buf[buf_add] = usart_data_receive(USART0) ;
        buf_add ++ ;
        u_timeout = 0 ;
        //if (rx_count >= rx_buffer_size){
        //    usart_interrupt_disable(USART0, USART_INT_RBNE);
        //}
    }

    //if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){
    //    /* transmit data */
    //    usart_data_transmit(USART0, tx_buffer[tx_count++]);
    //    if (tx_count >= tx_buffer_size){
    //        usart_interrupt_disable(USART0, USART_INT_TBE);
    //    }
    //}
}

void IOpin_init(void)
{
  rcu_periph_clock_enable(RCU_GPIOC);
  gpio_deinit(GPIOC);
  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_14|GPIO_PIN_15);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_14|GPIO_PIN_15);
  
  rcu_periph_clock_enable(RCU_GPIOA);
  gpio_deinit(GPIOA);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                                                        GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|
                                                        GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|
                                                        GPIO_PIN_12|GPIO_PIN_15
                );
  
  rcu_periph_clock_enable(RCU_GPIOF);
  gpio_deinit(GPIOF);
  gpio_mode_set(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7|GPIO_PIN_6);
  
  rcu_periph_clock_enable(RCU_GPIOB);
  gpio_deinit(GPIOB);
  gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_ALL);
}

void timer0_config(void)
{
    /* TIMER0 configuration: generate PWM signals with different duty cycles:
       TIMER0CLK = SystemCoreClock / 108 = 1MHz */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 23999;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_BOTH;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);

     /* CH0 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,100);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
    
    timer_channel_output_config(TIMER0,TIMER_CH_3,&timer_ocintpara);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,900);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
    
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,100);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    
    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,100);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    timer_primary_output_config(TIMER0,ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    
    /* TIMER0 channel control update interrupt enable */
    timer_interrupt_enable(TIMER0,TIMER_DMAINTEN_UPIE);
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(TIMER0_BRK_UP_TRG_COM_IRQn, 0, 1);
    
    //timer_enable(TIMER0);
    
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /*Configure PA9(TIMER0_CH1) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11);
}

void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
//void TIMER0_Channel_IRQHandler(void)
{
  timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
  
}

void timer1_config(void)
{
    /* TIMER1 configuration: generate PWM signals with different duty cycles:
       TIMER1CLK = SystemCoreClock / 108 = 1MHz */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_BOTH;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

     /* configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_3,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,999);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_3,100);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
    
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_2,750);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,250);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    timer_primary_output_config(TIMER1,ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    //timer_enable(TIMER1);
    
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /*Configure PA0-3(TIMER0_CH1) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

void timer2_config(void)
{
    /* TIMER1 configuration: generate PWM signals with different duty cycles:
       TIMER1CLK = SystemCoreClock / 108 = 1MHz */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);
    timer_deinit(TIMER2);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_BOTH;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

     /* configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
    timer_channel_output_config(TIMER2,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_3,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,999);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_3,100);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
    
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER2,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_0,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_2,750);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,250);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    timer_primary_output_config(TIMER2,ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    //timer_enable(TIMER1);
    
    rcu_periph_clock_enable(RCU_GPIOB);
    
    /*Configure PA0-3(TIMER0_CH1) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6|GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6|GPIO_PIN_7);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_6|GPIO_PIN_7);
    
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0|GPIO_PIN_1);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_0|GPIO_PIN_1);
}

unsigned char rev(unsigned char innum)
{
  unsigned char shift,out,recnt;
  shift = innum ;
  out   = 0 ;
  for(recnt=4;recnt;recnt--)
  {
    out = out << 1 ;
    if(shift & 1) out += 1 ;
    shift = shift >> 1 ;
  }
  return out ;
}

int main(void)
{
    
    RCC_Configuration();
      
    IOpin_init();
    
    //timer0_config();
    
    //timer_enable(TIMER0);
    
    while(1){
      hallstaB = GPIO_ISTAT(GPIOB) ;
      hallstaA = GPIO_ISTAT(GPIOA) ;
      hallstaF = GPIO_ISTAT(GPIOF) ;
      pos[1] = hallstaA & 0xf;
      pos[2] = (hallstaA & 0xf0) >> 4 ;
      pos[3] = (hallstaB & 0x7) + ((hallstaB & 0x400) >> 7) ;
      pos[4] = (hallstaB & 0x3c0) >> 6 ;
      pos[5] = ((hallstaB & 0x38) >> 2) + ((hallstaA & 0x8000) >> 15) ;
      res[1] = ((hallstaF & 0xc0) >> 4) + ((hallstaA & 0x1800) >> 11) ;
      res[2] = ((hallstaA & 0x700) >> 8) + ((hallstaB & 0x8000) >> 12) ;
      res[3] = (hallstaB & 0x7800) >> 11 ;
      
      pos[4] = rev(pos[4]);
      pos[5] = rev(pos[5]);
      res[1] = rev(res[1]);
      res[3] = rev(res[3]);
      
      res[2] = res[2] << 1 ;
      if(res[2] & 0x10) res[2] += 1 ;
      res[2] = res[2] & 0x0f ;
      res[2] = rev(res[2]);
      
      // 0~9 0000~1001,+-*/ 0xa,0xb,0xc,0xd
      // pos[3] is cal type,pos[1]pos[2] is 1st num,pos[4]pos[5] is 2nd num,res num is fixed
      nummark = 0 ;
      if(pos[2] < 10)
      {
        if(pos[1] < 10) pos[2] = pos[2] + pos[1] * 10 ;
        nummark += 1 ;
      }
      if(pos[5] < 10)
      {
        if(pos[4] < 10) pos[5] = pos[5] + pos[4] * 10 ;
        nummark += 2 ;
      }
      if((pos[3] > 9)&&(pos[3] < 14))
      {
        if(pos[3] == 10) result = pos[2] + pos[5] ;
        else if(pos[3] == 11) result = pos[2] - pos[5] ;
        else if(pos[3] == 12) result = pos[2] * pos[5] ;
        else if(pos[3] == 13) result = pos[2] / pos[5] ;
        nummark += 4 ;
      }
      if(res[3] < 10)
      {
        result = result - res[3] ;
        if(res[2] < 10) result = result - ( res[2] * 10 ) ;
        if(res[1] < 10) result = result - ( res[1] * 100) ;
        nummark += 8 ;
      }
      
      if(nummark == 0x0f){
        if(result) { R_ON ; G_OFF; }
        else       { G_ON ; R_OFF; }
      }
      else { G_OFF ; R_OFF; }
      //if( res[2] == 0xf ){ R_ON ; G_OFF; }
      //else if( res[1] == 0xf ){ G_ON ; R_OFF; }
      //else { G_OFF ; R_OFF; }
    }
}
