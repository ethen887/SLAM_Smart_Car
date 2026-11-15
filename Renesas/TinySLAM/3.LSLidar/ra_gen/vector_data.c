/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_b_uart_rxi_isr, /* SCI2 RXI (Receive data full) */
            [1] = sci_b_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [2] = sci_b_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [3] = sci_b_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [4] = sci_b_uart_rxi_isr, /* SCI1 RXI (Receive data full) */
            [5] = sci_b_uart_txi_isr, /* SCI1 TXI (Transmit data empty) */
            [6] = sci_b_uart_tei_isr, /* SCI1 TEI (Transmit end) */
            [7] = sci_b_uart_eri_isr, /* SCI1 ERI (Receive error) */
            [8] = dmac_int_isr, /* DMAC1 INT (DMAC1 transfer end) */
            [9] = sci_b_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [10] = sci_b_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [11] = sci_b_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [12] = sci_b_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [13] = dmac_int_isr, /* DMAC2 INT (DMAC2 transfer end) */
            [14] = usbfs_interrupt_handler, /* USBFS INT (USBFS interrupt) */
            [15] = usbfs_resume_handler, /* USBFS RESUME (USBFS resume interrupt) */
            [16] = usbfs_d0fifo_handler, /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [17] = usbfs_d1fifo_handler, /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
            [18] = usbhs_interrupt_handler, /* USBHS USB INT RESUME (USBHS interrupt) */
            [19] = usbhs_d0fifo_handler, /* USBHS FIFO 0 (DMA transfer request 0) */
            [20] = usbhs_d1fifo_handler, /* USBHS FIFO 1 (DMA transfer request 1) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI2_RXI,GROUP0), /* SCI2 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI2_TXI,GROUP1), /* SCI2 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_SCI2_TEI,GROUP2), /* SCI2 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI2_ERI,GROUP3), /* SCI2 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SCI1_RXI,GROUP4), /* SCI1 RXI (Receive data full) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI1_TXI,GROUP5), /* SCI1 TXI (Transmit data empty) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI1_TEI,GROUP6), /* SCI1 TEI (Transmit end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI1_ERI,GROUP7), /* SCI1 ERI (Receive error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_DMAC1_INT,GROUP0), /* DMAC1 INT (DMAC1 transfer end) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI0_RXI,GROUP1), /* SCI0 RXI (Receive data full) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TXI,GROUP2), /* SCI0 TXI (Transmit data empty) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TEI,GROUP3), /* SCI0 TEI (Transmit end) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI0_ERI,GROUP4), /* SCI0 ERI (Receive error) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_DMAC2_INT,GROUP5), /* DMAC2 INT (DMAC2 transfer end) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_USBFS_INT,GROUP6), /* USBFS INT (USBFS interrupt) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_USBFS_RESUME,GROUP7), /* USBFS RESUME (USBFS resume interrupt) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_0,GROUP0), /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_1,GROUP1), /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_USBHS_USB_INT_RESUME,GROUP2), /* USBHS USB INT RESUME (USBHS interrupt) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_USBHS_FIFO_0,GROUP3), /* USBHS FIFO 0 (DMA transfer request 0) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_USBHS_FIFO_1,GROUP4), /* USBHS FIFO 1 (DMA transfer request 1) */
        };
        #endif
        #endif