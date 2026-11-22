/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_b_uart_rxi_isr, /* SCI1 RXI (Receive data full) */
            [1] = sci_b_uart_txi_isr, /* SCI1 TXI (Transmit data empty) */
            [2] = sci_b_uart_tei_isr, /* SCI1 TEI (Transmit end) */
            [3] = sci_b_uart_eri_isr, /* SCI1 ERI (Receive error) */
            [4] = dmac_int_isr, /* DMAC1 INT (DMAC1 transfer end) */
            [5] = sci_b_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [6] = sci_b_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [7] = sci_b_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [8] = sci_b_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [9] = dmac_int_isr, /* DMAC2 INT (DMAC2 transfer end) */
            [10] = sci_b_uart_rxi_isr, /* SCI4 RXI (Receive data full) */
            [11] = sci_b_uart_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [12] = sci_b_uart_tei_isr, /* SCI4 TEI (Transmit end) */
            [13] = sci_b_uart_eri_isr, /* SCI4 ERI (Receive error) */
            [14] = dmac_int_isr, /* DMAC3 INT (DMAC3 transfer end) */
            [15] = usbfs_interrupt_handler, /* USBFS INT (USBFS interrupt) */
            [16] = usbfs_resume_handler, /* USBFS RESUME (USBFS resume interrupt) */
            [17] = usbfs_d0fifo_handler, /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [18] = usbfs_d1fifo_handler, /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
            [19] = usbhs_interrupt_handler, /* USBHS USB INT RESUME (USBHS interrupt) */
            [20] = usbhs_d0fifo_handler, /* USBHS FIFO 0 (DMA transfer request 0) */
            [21] = usbhs_d1fifo_handler, /* USBHS FIFO 1 (DMA transfer request 1) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI1_RXI,GROUP0), /* SCI1 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI1_TXI,GROUP1), /* SCI1 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_SCI1_TEI,GROUP2), /* SCI1 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI1_ERI,GROUP3), /* SCI1 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_DMAC1_INT,GROUP4), /* DMAC1 INT (DMAC1 transfer end) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI0_RXI,GROUP5), /* SCI0 RXI (Receive data full) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TXI,GROUP6), /* SCI0 TXI (Transmit data empty) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TEI,GROUP7), /* SCI0 TEI (Transmit end) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI0_ERI,GROUP0), /* SCI0 ERI (Receive error) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_DMAC2_INT,GROUP1), /* DMAC2 INT (DMAC2 transfer end) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI4_RXI,GROUP2), /* SCI4 RXI (Receive data full) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TXI,GROUP3), /* SCI4 TXI (Transmit data empty) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TEI,GROUP4), /* SCI4 TEI (Transmit end) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_SCI4_ERI,GROUP5), /* SCI4 ERI (Receive error) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_DMAC3_INT,GROUP6), /* DMAC3 INT (DMAC3 transfer end) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_USBFS_INT,GROUP7), /* USBFS INT (USBFS interrupt) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_USBFS_RESUME,GROUP0), /* USBFS RESUME (USBFS resume interrupt) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_0,GROUP1), /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_1,GROUP2), /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_USBHS_USB_INT_RESUME,GROUP3), /* USBHS USB INT RESUME (USBHS interrupt) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_USBHS_FIFO_0,GROUP4), /* USBHS FIFO 0 (DMA transfer request 0) */
            [21] = BSP_PRV_VECT_ENUM(EVENT_USBHS_FIFO_1,GROUP5), /* USBHS FIFO 1 (DMA transfer request 1) */
        };
        #endif
        #endif