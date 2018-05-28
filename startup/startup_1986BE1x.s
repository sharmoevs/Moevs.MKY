;/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
;* File Name          : startup_1986BE1x.s
;* Author             : Гаврилин Д.А.
;* Version            : V1.0
;* Date               : 22-03-2012
;* Description        : 1986BE1x devices vector table for EWARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == __iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR 
;*                        address.
;*                      After Reset the Cortex-M1 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;********************************************************************************
;* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
;* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
;* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
;* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
;* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
;* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
;*******************************************************************************/
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;
; Vector Table Mapped to Address 0 at Reset

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit     
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler             ; Reset Handler
                
        DCD     NMI_Handler               ; NMI Handler
        DCD     HardFault_Handler         ; Hard Fault Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SVC_Handler               ; SVCall Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     PendSV_Handler            ; PendSV Handler
        DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
				DCD     MIL_STD_1553B2_IRQHandler	;IRQ0
				DCD     MIL_STD_1553B1_IRQHandler	;IRQ1
				DCD     USB_IRQHandler					  ;IRQ2
				DCD     CAN1_IRQHandler				    ;IRQ3
				DCD     CAN2_IRQHandler				    ;IRQ4
				DCD     DMA_IRQHandler      			;IRQ5
				DCD     UART1_IRQHandler				  ;IRQ6
				DCD     UART2_IRQHandler				  ;IRQ7
				DCD     SSP1_IRQHandler				    ;IRQ8
				DCD     BUSY_IRQHandler				    ;IRQ9
				DCD     ARINC429R_IRQHandler			;IRQ10
				DCD     POWER_IRQHandler				  ;IRQ11
				DCD     WWDG_IRQHandler				    ;IRQ12
				DCD     TIMER4_IRQHandler   			;IRQ13
				DCD     TIMER1_IRQHandler				  ;IRQ14
				DCD     TIMER2_IRQHandler				  ;IRQ15
				DCD     TIMER3_IRQHandler				  ;IRQ16
				DCD     ADC_IRQHandler					  ;IRQ17
				DCD     ETHERNET_IRQHandler			  ;IRQ18
				DCD     SSP3_IRQHandler				    ;IRQ19
				DCD     SSP2_IRQHandler				    ;IRQ20
				DCD     ARINC429T1_IRQHandler			;IRQ21
				DCD     ARINC429T2_IRQHandler			;IRQ22
				DCD     ARINC429T3_IRQHandler			;IRQ23
				DCD     ARINC429T4_IRQHandler			;IRQ24
				DCD     0							            ;IRQ25
				DCD     0							            ;IRQ26
				DCD     BKP_IRQHandler					  ;IRQ27
				DCD     EXT_INT1_IRQHandler			  ;IRQ28
				DCD     EXT_INT2_IRQHandler			  ;IRQ29
				DCD     EXT_INT3_IRQHandler			  ;IRQ30
				DCD     EXT_INT4_IRQHandler			  ;IRQ31

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;


        THUMB
        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, = SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0


        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler
        
        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler        

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler 

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler 

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler 


; External Interrupts
        PUBWEAK MIL_STD_1553B2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MIL_STD_1553B2_IRQHandler
        B MIL_STD_1553B2_IRQHandler

        PUBWEAK MIL_STD_1553B1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MIL_STD_1553B1_IRQHandler
        B MIL_STD_1553B1_IRQHandler

        PUBWEAK USB_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USB_IRQHandler
        B USB_IRQHandler

        PUBWEAK CAN1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN1_IRQHandler
        B CAN1_IRQHandler
                
        PUBWEAK CAN2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN2_IRQHandler
        B CAN2_IRQHandler

        PUBWEAK DMA_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_IRQHandler
        B DMA_IRQHandler

        PUBWEAK UART1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_IRQHandler
        B UART1_IRQHandler

        PUBWEAK UART2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_IRQHandler
        B UART2_IRQHandler

        PUBWEAK SSP1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSP1_IRQHandler
        B SSP1_IRQHandler

        PUBWEAK BUSY_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BUSY_IRQHandler
        B BUSY_IRQHandler

        PUBWEAK ARINC429R_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ARINC429R_IRQHandler
        B ARINC429R_IRQHandler

        PUBWEAK POWER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
POWER_IRQHandler
        B POWER_IRQHandler

        PUBWEAK WWDG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WWDG_IRQHandler
        B WWDG_IRQHandler

        PUBWEAK TIMER4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER4_IRQHandler
        B TIMER4_IRQHandler

        PUBWEAK TIMER1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER1_IRQHandler
        B TIMER1_IRQHandler

        PUBWEAK TIMER2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER2_IRQHandler
        B TIMER2_IRQHandler

        PUBWEAK TIMER3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER3_IRQHandler
        B TIMER3_IRQHandler

        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC_IRQHandler
        B ADC_IRQHandler

        PUBWEAK ETHERNET_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ETHERNET_IRQHandler
        B ETHERNET_IRQHandler

        PUBWEAK SSP3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSP3_IRQHandler
        B SSP3_IRQHandler

        PUBWEAK SSP2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSP2_IRQHandler
        B SSP2_IRQHandler

        PUBWEAK ARINC429T1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ARINC429T1_IRQHandler
        B ARINC429T1_IRQHandler

        PUBWEAK ARINC429T2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ARINC429T2_IRQHandler
        B ARINC429T2_IRQHandler
                
        PUBWEAK ARINC429T3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ARINC429T3_IRQHandler
        B ARINC429T3_IRQHandler

        PUBWEAK ARINC429T4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ARINC429T4_IRQHandler
        B ARINC429T4_IRQHandler

        PUBWEAK BKP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BKP_IRQHandler
        B BKP_IRQHandler

        PUBWEAK EXT_INT1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT_INT1_IRQHandler
        B EXT_INT1_IRQHandler

        PUBWEAK EXT_INT2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT_INT2_IRQHandler
        B EXT_INT2_IRQHandler

        PUBWEAK EXT_INT3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT_INT3_IRQHandler
        B EXT_INT3_IRQHandler

        PUBWEAK EXT_INT4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXT_INT4_IRQHandler
        B EXT_INT4_IRQHandler


        END
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

