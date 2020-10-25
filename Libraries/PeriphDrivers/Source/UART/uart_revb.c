/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *************************************************************************** */

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "uart.h"
#include "dma.h"

/* **** Definitions **** */
#define MXC_UART_ERRINT_EN (MXC_F_UART_INT_EN_RX_FERR | \
                            MXC_F_UART_INT_EN_RX_PAR | \
                            MXC_F_UART_INT_EN_RX_OV)

#define MXC_UART_ERRINT_FL (MXC_F_UART_INT_FL_RX_FERR | \
                            MXC_F_UART_INT_FL_RX_PAR | \
                            MXC_F_UART_INT_FL_RX_OV)

/* **** Variable Declaration **** */
static void* AsyncTxRequests[MXC_UART_INSTANCES];
static void* AsyncRxRequests[MXC_UART_INSTANCES];

typedef struct {
    mxc_uart_req_t* req;
    int channelTx;
    int channelRx;
} uart_req_state_t;

uart_req_state_t states[MXC_UART_INSTANCES];
/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_UART_RevB_Init(mxc_uart_regs_t* uart, unsigned int baud, mxc_uart_clock_t clock)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    // Initialize UART
    MXC_UART_SetRXThreshold(uart, 1);       // Set RX threshold to 1 byte

    MXC_UART_SetDataSize(uart, 8);          // Set Datasize to 8 bits

    MXC_UART_SetParity(uart, MXC_UART_PARITY_DISABLE);
    MXC_UART_SetStopBits(uart, MXC_UART_STOP_1);

    return MXC_UART_SetFrequency(uart, baud, clock);
}

int MXC_UART_RevB_ReadyForSleep(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (AsyncTxRequests[MXC_UART_GET_IDX(uart)] != NULL) {
        return E_BUSY;
    }

    if (AsyncRxRequests[MXC_UART_GET_IDX(uart)] != NULL) {
        return E_BUSY;
    }

    return MXC_UART_GetActive(uart);
}

int MXC_UART_RevB_SetFrequency(mxc_uart_regs_t* uart, unsigned int baud, mxc_uart_clock_t clock)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    // Enable baud clock and wait for it to become ready.
    uart->ctrl |= MXC_F_UART_CTRL_BCLKEN;

    while (((uart->ctrl & MXC_F_UART_CTRL_BCLKRDY) >> MXC_F_UART_CTRL_BCLKRDY_POS) == 0);

    // OSR default value
    uart->osr = 5;

    // Check if the UART is LPUART
    if (uart == MXC_UART3) {
        switch (clock) {
        case MXC_UART_APB_CLK:
            uart->clkdiv = ((IBRO_FREQ) / baud);
            break;

        case MXC_UART_32K_CLK:
            uart->ctrl |= MXC_S_UART_CTRL_BCLKSRC_CLK2;
            uart->ctrl |= MXC_F_UART_CTRL_FDM;

            if (baud == 9600) {
                uart->clkdiv = 7;
            }
            else {
                uart->clkdiv = ((ERTCO_FREQ * 2) / baud);
            }

            if (baud > 2400) {
                uart->osr = 0;
            }
            else {
                uart->osr = 1;
            }

            break;

        default:
            return E_BAD_PARAM;
        }
    }
    else {
        switch (clock) {
        case MXC_UART_APB_CLK:
            uart->clkdiv = (PeripheralClock / baud);
            break;

        case MXC_UART_8M_CLK:
            uart->clkdiv = (IBRO_FREQ / baud);
            uart->ctrl |= MXC_S_UART_CTRL_BCLKSRC_CLK2;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    return MXC_UART_GetFrequency(uart);
}

int MXC_UART_RevB_GetFrequency(mxc_uart_regs_t* uart)
{
    int periphClock = 0;

    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (uart == MXC_UART3) {
        if ((uart->ctrl & MXC_F_UART_CTRL_BCLKSRC) == MXC_S_UART_CTRL_BCLKSRC_CLK2) {
            periphClock = ERTCO_FREQ * 2;
        }
        else if ((uart->ctrl & MXC_F_UART_CTRL_BCLKSRC) == MXC_S_UART_CTRL_BCLKSRC_PERIPHERAL_CLOCK) {
            periphClock = IBRO_FREQ;
        }
        else {
            return E_BAD_PARAM;
        }
    }
    else {
        if ((uart->ctrl & MXC_F_UART_CTRL_BCLKSRC) == MXC_S_UART_CTRL_BCLKSRC_CLK2) {
            periphClock = IBRO_FREQ;
        }
        else if ((uart->ctrl & MXC_F_UART_CTRL_BCLKSRC) == MXC_S_UART_CTRL_BCLKSRC_PERIPHERAL_CLOCK) {
            periphClock = PeripheralClock;
        }
        else {
            return E_BAD_PARAM;
        }
    }

    return (periphClock / uart->clkdiv);
}

int MXC_UART_RevB_SetDataSize(mxc_uart_regs_t* uart, int dataSize)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (dataSize < 5 || dataSize > 8) {
        return E_BAD_PARAM;
    }

    dataSize = (dataSize - 5) << MXC_F_UART_CTRL_CHAR_SIZE_POS;

    MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_CHAR_SIZE, dataSize);

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetStopBits(mxc_uart_regs_t* uart, mxc_uart_stop_t stopBits)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    switch (stopBits) {
    case MXC_UART_STOP_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_STOPBITS, 0 << MXC_F_UART_CTRL_STOPBITS_POS);
        break;

    case MXC_UART_STOP_2:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_STOPBITS, 1 << MXC_F_UART_CTRL_STOPBITS_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetParity(mxc_uart_regs_t* uart, mxc_uart_parity_t parity)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    switch (parity) {
    case MXC_UART_PARITY_DISABLE:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EN, 0 << MXC_F_UART_CTRL_PAR_EN_POS);
        break;

    case MXC_UART_PARITY_EVEN_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EN, 1 << MXC_F_UART_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EO, 0 << MXC_F_UART_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_MD, 0 << MXC_F_UART_CTRL_PAR_MD_POS);
        break;

    case MXC_UART_PARITY_EVEN_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EN, 1 << MXC_F_UART_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EO, 0 << MXC_F_UART_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_MD, 1 << MXC_F_UART_CTRL_PAR_MD_POS);
        break;

    case MXC_UART_PARITY_ODD_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EN, 1 << MXC_F_UART_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EO, 1 << MXC_F_UART_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_MD, 0 << MXC_F_UART_CTRL_PAR_MD_POS);
        break;

    case MXC_UART_PARITY_ODD_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EN, 1 << MXC_F_UART_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_EO, 1 << MXC_F_UART_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_PAR_MD, 1 << MXC_F_UART_CTRL_PAR_MD_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetFlowCtrl(mxc_uart_regs_t* uart, mxc_uart_flow_t flowCtrl, int rtsThreshold)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    switch (flowCtrl) {
    case MXC_UART_FLOW_DIS:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_HFC_EN, 0 << MXC_F_UART_CTRL_HFC_EN_POS);
        break;

    case MXC_UART_FLOW_EN:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_HFC_EN, 1 << MXC_F_UART_CTRL_HFC_EN_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    //FIXME: Fix missing code for CTS threshhold.

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetClockSource(mxc_uart_regs_t* uart, mxc_uart_clock_t clock)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    switch (clock) {
    case MXC_UART_APB_CLK:
        break;

    case MXC_UART_8M_CLK:
        uart->ctrl |= MXC_S_UART_CTRL_BCLKSRC_CLK2;
        break;

    case MXC_UART_32K_CLK:
        uart->ctrl |= MXC_S_UART_CTRL_BCLKSRC_CLK2;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_GetActive(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (uart->status & (MXC_F_UART_STATUS_TX_BUSY | MXC_F_UART_STATUS_RX_BUSY)) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AbortTransmission(mxc_uart_regs_t* uart)
{
    MXC_UART_ClearTXFIFO(uart);
    return E_NO_ERROR;
}

int MXC_UART_RevB_ReadCharacterRaw(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (uart->status & MXC_F_UART_STATUS_RX_EM) {
        return E_UNDERFLOW;
    }

    return uart->fifo;
}

int MXC_UART_RevB_WriteCharacterRaw(mxc_uart_regs_t* uart, uint8_t character)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    // Require the TX FIFO to be empty, so that we write out the expected character
    // Return error if the FIFO is full
    if (uart->status & MXC_F_UART_STATUS_TX_FULL) {
        return E_OVERFLOW;
    }

    uart->fifo = character;

    return E_NO_ERROR;
}

int MXC_UART_RevB_Read(mxc_uart_regs_t* uart, uint8_t* buffer, int* len)
{
    int read = 0;
    int retVal;

    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (buffer == NULL) {
        return E_NULL_PTR;
    }

    if (len == NULL) {
        return E_NULL_PTR;
    }

    for (; read < *len; read++) {
        retVal = MXC_UART_ReadCharacter(uart);

        if (retVal < 0) {
            *len = read;
            return retVal;
        }
        else {
            buffer[read] = retVal;
        }
    }

    *len = read;
    return E_NO_ERROR;
}

int MXC_UART_RevB_Write(mxc_uart_regs_t* uart, const uint8_t* byte, int* len)
{
    int written = 0;
    int retVal;

    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (byte == NULL) {
        return E_NULL_PTR;
    }

    if (len == NULL) {
        return E_NULL_PTR;
    }

    for (; written < *len; written++) {
        retVal = MXC_UART_WriteCharacter(uart, byte[written]);

        if (retVal != E_NO_ERROR) {
            *len = written;
            return retVal;
        }
    }

    *len = written;
    return E_NO_ERROR;
}

unsigned int MXC_UART_RevB_ReadRXFIFO(mxc_uart_regs_t* uart, unsigned char* bytes, unsigned int len)
{
    unsigned read = 0;

    for (; read < len; read++) {
        if (uart->status & MXC_F_UART_STATUS_RX_EM) {
            break;
        }

        bytes[read] = uart->fifo;
    }

    return read;
}

unsigned int MXC_UART_RevB_GetRXFIFOAvailable(mxc_uart_regs_t* uart)
{
    return (uart->status & MXC_F_UART_STATUS_RX_LVL) >> MXC_F_UART_STATUS_RX_LVL_POS;
}

unsigned int MXC_UART_RevB_WriteTXFIFO(mxc_uart_regs_t* uart, const unsigned char* bytes, unsigned int len)
{
    unsigned written = 0;

    for (; written < len; written++) {
        if (uart->status & MXC_F_UART_STATUS_TX_FULL) {
            break;
        }

        uart->fifo = bytes[written];
    }

    return written;
}

unsigned int MXC_UART_RevB_GetTXFIFOAvailable(mxc_uart_regs_t* uart)
{
    int txCnt = (uart->status & MXC_F_UART_STATUS_TX_LVL) >> MXC_F_UART_STATUS_TX_LVL_POS;
    return MXC_UART_FIFO_DEPTH - txCnt;
}

int MXC_UART_RevB_ClearRXFIFO(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    uart->ctrl |= MXC_F_UART_CTRL_RX_FLUSH;

    while ((uart->status & MXC_F_UART_STATUS_RX_EM) == 0);

    return E_NO_ERROR;
}

int MXC_UART_RevB_ClearTXFIFO(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    uart->ctrl |= MXC_F_UART_CTRL_TX_FLUSH;

    while ((uart->status & MXC_F_UART_STATUS_TX_EM) == 0);

    return E_NO_ERROR;
}


int MXC_UART_RevB_SetRXThreshold(mxc_uart_regs_t* uart, unsigned int numBytes)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    if (numBytes < 1 || numBytes > MXC_UART_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    numBytes <<= MXC_F_UART_CTRL_RX_THD_VAL_POS;
    MXC_SETFIELD(uart->ctrl, MXC_F_UART_CTRL_RX_THD_VAL, numBytes);

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevB_GetRXThreshold(mxc_uart_regs_t* uart)
{
    return ((uart->ctrl & MXC_F_UART_CTRL_RX_THD_VAL) >> MXC_F_UART_CTRL_RX_THD_VAL_POS);
}

unsigned int MXC_UART_RevB_GetFlags(mxc_uart_regs_t* uart)
{
    return uart->int_fl;
}

int MXC_UART_RevB_ClearFlags(mxc_uart_regs_t* uart, int flags)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    uart->int_fl = flags;

    return E_NO_ERROR;
}

int MXC_UART_RevB_EnableInt(mxc_uart_regs_t* uart, unsigned int intEn)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    uart->int_en |= intEn;

    return E_NO_ERROR;
}

int MXC_UART_RevB_DisableInt(mxc_uart_regs_t* uart, unsigned int intDis)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    uart->int_en &= ~intDis;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevB_GetStatus(mxc_uart_regs_t* uart)
{
    return uart->status;
}

int MXC_UART_RevB_Transaction(mxc_uart_req_t* req)
{
    uint32_t numToWrite, numToRead;

    if (MXC_UART_GET_IDX(req->uart) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt(req->uart, 0xFFFFFFFF);
    MXC_UART_ClearFlags(req->uart, 0xFFFFFFFF);

    req->txCnt = 0;
    req->rxCnt = 0;

    if (req->rxLen) {
        if (req->rxData == NULL) {
            return E_BAD_PARAM;
        }
    }

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }

        numToWrite = MXC_UART_GetTXFIFOAvailable(req->uart);
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO(req->uart, &req->txData[req->txCnt], numToWrite);

        while (req->txCnt < req->txLen) {
            while (!(MXC_UART_GetFlags(req->uart) & MXC_F_UART_INT_FL_TX_HE));

            numToWrite = MXC_UART_GetTXFIFOAvailable(req->uart);
            numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
            req->txCnt += MXC_UART_WriteTXFIFO(req->uart, &req->txData[req->txCnt], numToWrite);
            MXC_UART_ClearFlags(req->uart, MXC_F_UART_INT_FL_TX_HE);
        }
    }

    if (req->rxLen) {
        numToRead = MXC_UART_GetRXFIFOAvailable(req->uart);
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO(req->uart, &req->rxData[req->rxCnt], numToRead);

        while (req->rxCnt < req->rxLen) {
            while (!(MXC_UART_GetFlags(req->uart) & MXC_F_UART_INT_FL_RX_THD));

            numToRead = MXC_UART_GetRXFIFOAvailable(req->uart);
            numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
            req->rxCnt += MXC_UART_ReadRXFIFO(req->uart, &req->rxData[req->rxCnt], numToRead);
            MXC_UART_ClearFlags(req->uart, MXC_F_UART_INT_FL_RX_THD);
        }
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_TransactionAsync(mxc_uart_req_t* req)
{
    uint32_t numToWrite, numToRead;
    int uartNum = MXC_UART_GET_IDX(req->uart);

    if (uartNum < 0) {
        return E_INVALID;
    }

    if(!AsyncTxRequests[uartNum] && !AsyncRxRequests[uartNum]) {
        /* No requests pending, clear the interrupt state */
        MXC_UART_DisableInt(req->uart, 0xFFFFFFFF);
        MXC_UART_ClearFlags(req->uart, 0xFFFFFFFF);

    } else if (AsyncRxRequests[uartNum] && req->rxLen) {
        /* RX request pending */
        return E_BUSY;
    } else if (AsyncTxRequests[uartNum] && req->txLen) {
        /* TX request pending */
        return E_BUSY;
    }

    req->txCnt = 0;
    req->rxCnt = 0;

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }

        MXC_UART_EnableInt(req->uart, MXC_F_UART_INT_EN_TX_HE);
        numToWrite = MXC_UART_GetTXFIFOAvailable(req->uart);
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO(req->uart, &req->txData[req->txCnt], numToWrite);

        /* If we're finished writing to the TX FIFO and it's less than half+1 full, pend the interrupt */
        if((MXC_UART_GetTXFIFOAvailable(req->uart) >= (MXC_UART_FIFO_DEPTH/2))
         && (req->txCnt == req->txLen)) {
            NVIC_SetPendingIRQ(MXC_UART_GET_IRQ(uartNum));
        }

        AsyncTxRequests[MXC_UART_GET_IDX(req->uart)] = (void*) req;
    }

    if (req->rxLen) {
        // All error interrupts are related to RX
        MXC_UART_EnableInt(req->uart, MXC_UART_ERRINT_EN);

        if (req->rxData == NULL) {
            MXC_UART_DisableInt(req->uart, 0xFFFFFFFF);
            MXC_UART_ClearTXFIFO(req->uart);
            return E_BAD_PARAM;
        }

        MXC_UART_EnableInt(req->uart, MXC_F_UART_INT_EN_RX_THD);
        numToRead = MXC_UART_GetRXFIFOAvailable(req->uart);
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO(req->uart, &req->rxData[req->rxCnt], numToRead);
        MXC_UART_ClearFlags(req->uart, MXC_F_UART_INT_FL_RX_THD);

        AsyncRxRequests[MXC_UART_GET_IDX(req->uart)] = (void*) req;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncTxCallback(mxc_uart_regs_t* uart, int retVal)
{
    int uartNum = MXC_UART_GET_IDX(uart);
    if (uartNum < 0) {
        return E_BAD_PARAM;
    }

    mxc_uart_req_t* req = (mxc_uart_req_t*) AsyncTxRequests[uartNum];
    if ((req != NULL) && (req->callback != NULL)) {
        AsyncTxRequests[uartNum] = NULL;
        req->callback(req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncRxCallback(mxc_uart_regs_t* uart, int retVal)
{
    int uartNum = MXC_UART_GET_IDX(uart);
    if (uartNum < 0) {
        return E_BAD_PARAM;
    }

    mxc_uart_req_t* req = (mxc_uart_req_t*) AsyncRxRequests[uartNum];
    if ((req != NULL) && (req->callback != NULL)) {
        AsyncRxRequests[uartNum] = NULL;
        req->callback(req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncCallback(mxc_uart_regs_t* uart, int retVal)
{
    int uartNum = MXC_UART_GET_IDX(uart);
    if (uartNum < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_RevB_AsyncTxCallback(uart, retVal);

    /* Only call the callback once if it's for the same request */
    if(AsyncRxRequests[uartNum] != AsyncTxRequests[uartNum]) {
        MXC_UART_RevB_AsyncRxCallback(uart, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncStopTx(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt(uart, MXC_F_UART_INT_EN_TX_HE);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncStopRx(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt(uart, MXC_UART_ERRINT_EN);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncStop(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt(uart, 0xFFFFFFFF);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AbortAsync(mxc_uart_regs_t* uart)
{
    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_AsyncStop(uart);
    MXC_UART_AsyncCallback(uart, E_ABORT);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncHandler(mxc_uart_regs_t* uart)
{
    uint32_t numToWrite, numToRead, flags;
    mxc_uart_req_t* req;

    int uartNum = MXC_UART_GET_IDX(uart);

    if (uartNum < 0) {
        return E_INVALID;
    }

    flags = MXC_UART_GetFlags(uart);

    /* Unexpected interrupt */
    if (!AsyncTxRequests[uartNum] && !AsyncRxRequests[uartNum]) {
        MXC_UART_ClearFlags(uart, uart->int_fl);
        return E_INVALID;
    }

    if (flags & MXC_UART_ERRINT_FL & uart->int_en) {
        MXC_UART_AsyncStop(uart);
        MXC_UART_AsyncCallback(uart, E_COMM_ERR);
        return E_INVALID;
    }

    req = (mxc_uart_req_t*) AsyncTxRequests[uartNum];
    if ((req != NULL) && (req->txLen)) {
        numToWrite = MXC_UART_GetTXFIFOAvailable(req->uart);
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO(req->uart, &req->txData[req->txCnt], numToWrite);
        MXC_UART_ClearFlags(req->uart, MXC_F_UART_INT_FL_TX_HE);
    }

    req = (mxc_uart_req_t*) AsyncRxRequests[uartNum];
    if ((req != NULL) && (flags & MXC_F_UART_INT_FL_RX_THD) && (req->rxLen)) {
        numToRead = MXC_UART_GetRXFIFOAvailable(req->uart);
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO(req->uart, &req->rxData[req->rxCnt], numToRead);

        if ((req->rxLen - req->rxCnt) < MXC_UART_GetRXThreshold(req->uart)) {
            MXC_UART_SetRXThreshold(req->uart, req->rxLen - req->rxCnt);
        }

        MXC_UART_ClearFlags(req->uart, MXC_F_UART_INT_FL_RX_THD);
    }

    if(AsyncRxRequests[uartNum] == AsyncTxRequests[uartNum]) {
        if ((req != NULL) && (req->rxCnt == req->rxLen) && (req->txCnt == req->txLen)) {
            MXC_UART_AsyncStop(uart);
            MXC_UART_AsyncCallback(uart, E_NO_ERROR);
        }
        return E_NO_ERROR;
    }

    req = (mxc_uart_req_t*) AsyncRxRequests[uartNum];
    if ((req != NULL) && (req->rxCnt == req->rxLen)) {
        MXC_UART_RevB_AsyncStopRx(uart);
        MXC_UART_RevB_AsyncRxCallback(uart, E_NO_ERROR);
        return E_NO_ERROR;
    }

    req = (mxc_uart_req_t*) AsyncTxRequests[uartNum];
    if((req != NULL) && (req->txCnt == req->txLen)) {
        MXC_UART_RevB_AsyncStopTx(uart);
        MXC_UART_RevB_AsyncTxCallback(uart, E_NO_ERROR);
        return E_NO_ERROR;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_ReadRXFIFODMA(mxc_uart_regs_t* uart, unsigned char* bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX(uart);

    if (uart_num < 0) {
        return E_INVALID;
    }

    if (bytes == NULL) {
        return E_NULL_PTR;
    }

    channel = MXC_DMA_AcquireChannel();

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = channel;
    srcdst.dest = bytes;
    srcdst.len = len;


    states[uart_num].channelRx = channel;
    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);
    MXC_DMA_EnableInt(channel);
    MXC_DMA_Start(channel);
    MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

    uart->dma |= MXC_F_UART_DMA_RX_EN;

    return E_NO_ERROR;
}

int MXC_UART_RevB_WriteTXFIFODMA(mxc_uart_regs_t* uart, const unsigned char* bytes, unsigned int len,
                                 mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX(uart);

    if (uart_num < 0) {
        return E_INVALID;
    }

    if (bytes == NULL) {
        return E_NULL_PTR;
    }

    channel = MXC_DMA_AcquireChannel();

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = channel;
    srcdst.source = (void*)bytes;
    srcdst.len = len;


    states[uart_num].channelTx = channel;
    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);
    MXC_DMA_EnableInt(channel);
    MXC_DMA_Start(channel);
    MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

    uart->dma |= MXC_F_UART_DMA_TX_EN;

    return E_NO_ERROR;
}

int MXC_UART_RevB_TransactionDMA(mxc_uart_req_t* req)
{
    int uart_num = MXC_UART_GET_IDX(req->uart);

    if (uart_num < 0) {
        return E_BAD_PARAM;
    }

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }
    }

    if (req->rxLen) {
        if (req->rxData == NULL) {
            return E_BAD_PARAM;
        }
    }

    MXC_UART_DisableInt(req->uart, 0xFFFFFFFF);
    MXC_UART_ClearFlags(req->uart, 0xFFFFFFFF);

    //Set DMA FIFO threshold
    (req->uart)->dma |= (1 << MXC_F_UART_DMA_RX_THD_VAL_POS);
    (req->uart)->dma |= (2 << MXC_F_UART_DMA_TX_THD_VAL_POS);

    MXC_DMA_Init();

    //tx
    if ((req->txData != NULL) && (req->txLen)) {
        if (MXC_UART_WriteTXFIFODMA(req->uart, req->txData, req->txLen, NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
    }

    //rx
    if ((req->rxData != NULL) && (req->rxLen)) {
        if (MXC_UART_ReadRXFIFODMA(req->uart, req->rxData, req->rxLen, NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
    }

    return E_NO_ERROR;
}

void MXC_UART_RevB_DMACallback(int ch, int error)
{
}
