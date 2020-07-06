//
// dmachannel.cpp
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2014-2020  R. Stange <rsta2@o2online.de>
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <circle/dmachannel.h>
#include <circle/bcm2835.h>
#include <circle/bcm2835int.h>
#include <circle/memio.h>
#include <circle/timer.h>
#include <circle/synchronize.h>
#include <circle/new.h>
#include <assert.h>
#include <string.h>

#define DMA_CHANNELS			(DMA_CHANNEL_MAX + 1)

#define ARM_DMACHAN_CS(chan)		(ARM_DMA_BASE + ((chan) * 0x100) + 0x00)
	#define CS_RESET			(1 << 31)
	#define CS_ABORT			(1 << 30)
	#define CS_WAIT_FOR_OUTSTANDING_WRITES	(1 << 28)
	#define CS_PANIC_PRIORITY_SHIFT		20
		#define DEFAULT_PANIC_PRIORITY		15
	#define CS_PRIORITY_SHIFT		16
		#define DEFAULT_PRIORITY		1
	#define CS_ERROR			(1 << 8)
	#define CS_INT				(1 << 2)
	#define CS_END				(1 << 1)
	#define CS_ACTIVE			(1 << 0)
#define ARM_DMACHAN_CONBLK_AD(chan)	(ARM_DMA_BASE + ((chan) * 0x100) + 0x04)
#define ARM_DMACHAN_TI(chan)		(ARM_DMA_BASE + ((chan) * 0x100) + 0x08)
	#define TI_PERMAP_SHIFT			16
	#define TI_BURST_LENGTH_SHIFT		12
		#define DEFAULT_BURST_LENGTH		0
	#define TI_SRC_IGNORE			(1 << 11)
	#define TI_SRC_DREQ			(1 << 10)
	#define TI_SRC_WIDTH			(1 << 9)
	#define TI_SRC_INC			(1 << 8)
	#define TI_DEST_DREQ			(1 << 6)
	#define TI_DEST_WIDTH			(1 << 5)
	#define TI_DEST_INC			(1 << 4)
	#define TI_WAIT_RESP			(1 << 3)
	#define TI_TDMODE			(1 << 1)
	#define TI_INTEN			(1 << 0)
#define ARM_DMACHAN_SOURCE_AD(chan)	(ARM_DMA_BASE + ((chan) * 0x100) + 0x0C)
#define ARM_DMACHAN_DEST_AD(chan)	(ARM_DMA_BASE + ((chan) * 0x100) + 0x10)
#define ARM_DMACHAN_TXFR_LEN(chan)	(ARM_DMA_BASE + ((chan) * 0x100) + 0x14)
	#define TXFR_LEN_XLENGTH_SHIFT		0
	#define TXFR_LEN_YLENGTH_SHIFT		16
	#define TXFR_LEN_MAX			0x3FFFFFFF
	#define TXFR_LEN_MAX_LITE		0xFFFF
#define ARM_DMACHAN_STRIDE(chan)	(ARM_DMA_BASE + ((chan) * 0x100) + 0x18)
	#define STRIDE_SRC_SHIFT		0
	#define STRIDE_DEST_SHIFT		16
#define ARM_DMACHAN_NEXTCONBK(chan)	(ARM_DMA_BASE + ((chan) * 0x100) + 0x1C)
#define ARM_DMACHAN_DEBUG(chan)		(ARM_DMA_BASE + ((chan) * 0x100) + 0x20)
	#define DEBUG_LITE			(1 << 28)
#define ARM_DMA_INT_STATUS		(ARM_DMA_BASE + 0xFE0)
#define ARM_DMA_ENABLE			(ARM_DMA_BASE + 0xFF0)


#if RASPPI >= 4

// The DMA4 address registers (control block, source, and destination) work in
// terms of the "full 35-bit address space", NOT the usual bus address that the
// other DMA channels use.  This is the address AFTER mapping through the page
// registers.
//
// TODO: This works for 32-bit addresses in RAM and the frame buffer, but I'm
// not sure it's correct in general.  Ideally this would go to the MMU page
// registers to get the physical address translation.
#define FULL_ADDRESS(p) (reinterpret_cast<uintptr>(p) & ~0xC0000000)

#define CS4_ERROR				(1 << 10)

#define TI4_D_WAITS_SHIFT			24
#define TI4_S_WAITS_SHIFT			16
#define TI4_D_DREQ				(1 << 15)
#define TI4_S_DREQ				(1 << 14)
#define TI4_PERMAP_SHIFT			9
#define TI4_WAIT_RD_RESP			(1 << 3)
#define TI4_WAIT_RESP				(1 << 2)
#define TI4_TDMODE				(1 << 1)
#define TI4_INTEN				(1 << 0)

#define TI4_SRC_STRIDE_SHIFT			16
#define TI4_SRCI_IGNORE				(1 << 15)
#define TI4_SRCI_WIDTH_32			(0 << 13)
#define TI4_SRCI_WIDTH_64			(1 << 13)
#define TI4_SRCI_WIDTH_128			(2 << 13)
#define TI4_SRCI_WIDTH_256			(3 << 13)  // not supported on BCM2711 (Pi 4)
#define TI4_SRCI_INC				(1 << 12)
#define TI4_SRCI_BURST_LENGTH_SHIFT		8
#define TI4_SRCI_ADDR_HI_SHIFT			0
#define TI4_SRCI_ADDR_HI_MASK			0xff

#define TI4_DSTI_STRIDE_SHIFT			16
#define TI4_DSTI_IGNORE				(1 << 15)
#define TI4_DSTI_WIDTH_32			(0 << 13)
#define TI4_DSTI_WIDTH_64			(1 << 13)
#define TI4_DSTI_WIDTH_128			(2 << 13)
#define TI4_DSTI_WIDTH_256			(3 << 13)  // not supported on BCM2711 (Pi 4)
#define TI4_DSTI_INC				(1 << 12)
#define TI4_DSTI_BURST_LENGTH_SHIFT		8
#define TI4_DSTI_ADDR_HI_SHIFT			0
#define TI4_DSTI_ADDR_HI_MASK			0xff

#if AARCH == 64
#define TI4_SRCI_ADDR_HI(ptr) \
	(((FULL_ADDRESS(ptr) >> 32) & TI4_SRCI_ADDR_HI_MASK) << TI4_SRCI_ADDR_HI_SHIFT)
#define TI4_DSTI_ADDR_HI(ptr) \
	(((FULL_ADDRESS(ptr) >> 32) & TI4_DSTI_ADDR_HI_MASK) << TI4_DSTI_ADDR_HI_SHIFT)
#else // AARCH == 64
#define TI4_SRCI_ADDR_HI(ptr) 0
#define TI4_DSTI_ADDR_HI(ptr) 0
#endif // AARCH == 64

#endif // RASPPI >= 4


CDMAChannel::CDMAChannel (unsigned nChannel, CInterruptSystem *pInterruptSystem)
:	m_nChannel (CMachineInfo::Get ()->AllocateDMAChannel (nChannel)),
	m_pControlBlockBuffer (0),
	m_pControlBlock (0),
	m_pInterruptSystem (pInterruptSystem),
	m_bIRQConnected (FALSE),
	m_pCompletionRoutine (0),
	m_pCompletionParam (0),
	m_bStatus (FALSE)
{
	PeripheralEntry ();

	assert (m_nChannel != DMA_CHANNEL_NONE);
	assert (m_nChannel < DMA_CHANNELS);

#if RASPPI >= 4
	m_cControlBlockSize = m_nChannel >= 11 ? sizeof (TDMA4ControlBlock) : sizeof(TDMAControlBlock);
#else
	m_cControlBlockSize = sizeof(TDMAControlBlock);
#endif

	size_t bufsize = m_cControlBlockSize + 31;
	m_pControlBlockBuffer = new (HEAP_DMA30) u8[bufsize];
	assert (m_pControlBlockBuffer != 0);

	memset(m_pControlBlockBuffer, 0, bufsize);

	m_pControlBlock.ui = ((reinterpret_cast<uintptr>(m_pControlBlockBuffer) + 31) & ~31);

	write32 (ARM_DMACHAN_CONBLK_AD (m_nChannel), 0);
	write32 (ARM_DMA_ENABLE, read32 (ARM_DMA_ENABLE) | (1 << m_nChannel));
	CTimer::SimpleusDelay (1000);

	write32 (ARM_DMACHAN_CS (m_nChannel), CS_RESET);
	while (read32 (ARM_DMACHAN_CS (m_nChannel)) & CS_ACTIVE)
	{
		// do nothing
	}

	PeripheralExit ();
}
	

CDMAChannel::~CDMAChannel (void)
{
	PeripheralEntry ();

	assert (m_nChannel < DMA_CHANNELS);

	write32 (ARM_DMACHAN_CS (m_nChannel), CS_RESET);
	while (read32 (ARM_DMACHAN_CS (m_nChannel)) & CS_RESET)
	{
		// do nothing
	}

	write32 (ARM_DMA_ENABLE, read32 (ARM_DMA_ENABLE) & ~(1 << m_nChannel));

	PeripheralExit ();

	m_pCompletionRoutine = 0;

	if (m_pInterruptSystem != 0)
	{
		if (m_bIRQConnected)
		{
			assert (m_nChannel <= 12);
			m_pInterruptSystem->DisconnectIRQ (CMachineInfo::GetIRQForDMA (m_nChannel));
		}

		m_pInterruptSystem = 0;
	}

	CMachineInfo::Get ()->FreeDMAChannel (m_nChannel);

	m_pControlBlock.ui = 0;

	delete [] m_pControlBlockBuffer;
	m_pControlBlockBuffer = nullptr;
}

void CDMAChannel::SetupMemCopy (void *pDestination, const void *pSource, size_t nLength,
				unsigned nBurstLength, boolean bCached)
{
	assert (pDestination != 0);
	assert (pSource != 0);
	assert (nLength > 0);
	assert (nBurstLength <= 15);

	assert (m_pControlBlock.ui != 0);
	assert (nLength <= TXFR_LEN_MAX);

#if RASPPI >= 4
	if (m_nChannel >= 11) // DMA4
	{
		// DMA4 channel
		m_pControlBlock.dma4->nTransferInformation =	0;
		m_pControlBlock.dma4->nSourceAddress           = FULL_ADDRESS(pSource);
		m_pControlBlock.dma4->nSourceInformation =
			TI4_SRCI_WIDTH_128
			| TI4_SRCI_INC
			| (nBurstLength << TI4_SRCI_BURST_LENGTH_SHIFT)
			| TI4_SRCI_ADDR_HI(BUS_ADDRESS(pSource));
		m_pControlBlock.dma4->nDestinationAddress      = FULL_ADDRESS(pDestination);
		m_pControlBlock.dma4->nDestinationInformation =
			TI4_DSTI_WIDTH_128
			| TI4_DSTI_INC
			| (nBurstLength << TI4_DSTI_BURST_LENGTH_SHIFT)
			| TI4_DSTI_ADDR_HI(BUS_ADDRESS(pDestination));
		m_pControlBlock.dma4->nTransferLength          = nLength;
		m_pControlBlock.dma4->nNextControlBlockAddress = 0;
	}
	else
#endif
	{
		// DMA or DMA LITE channel
		assert (   !(read32 (ARM_DMACHAN_DEBUG (m_nChannel)) & DEBUG_LITE)
			|| nLength <= TXFR_LEN_MAX_LITE);

		m_pControlBlock.dma->nTransferInformation =
			(nBurstLength << TI_BURST_LENGTH_SHIFT)
			| TI_SRC_WIDTH
			| TI_SRC_INC
			| TI_DEST_WIDTH
			| TI_DEST_INC;
		m_pControlBlock.dma->nSourceAddress           = BUS_ADDRESS ((uintptr) pSource);
		m_pControlBlock.dma->nDestinationAddress      = BUS_ADDRESS ((uintptr) pDestination);
		m_pControlBlock.dma->nTransferLength          = nLength;
		m_pControlBlock.dma->n2DModeStride            = 0;
		m_pControlBlock.dma->nNextControlBlockAddress = 0;
	}

	if (bCached)
	{
		m_nDestinationAddress = (uintptr) pDestination;
		m_nBufferLength = nLength;

		CleanAndInvalidateDataCacheRange (reinterpret_cast<uintptr>(pSource), nLength);
		CleanAndInvalidateDataCacheRange (reinterpret_cast<uintptr>(pDestination), nLength);
	}
	else
	{
		m_nDestinationAddress = 0;
	}
}

void CDMAChannel::SetupIORead (void *pDestination, u32 nIOAddress, size_t nLength, TDREQ DREQ)
{
	assert (pDestination != 0);
	assert (nLength > 0);
	assert (nLength <= TXFR_LEN_MAX);
	assert (   !(read32 (ARM_DMACHAN_DEBUG (m_nChannel)) & DEBUG_LITE)
		|| nLength <= TXFR_LEN_MAX_LITE);
#if RASPPI >= 4
	// TODO: add DMA4 support
	assert (m_nChannel <= 10);
#endif

	nIOAddress &= 0xFFFFFF;
	assert (nIOAddress != 0);
	nIOAddress += GPU_IO_BASE;

	assert (m_pControlBlock.ui != 0);
	m_pControlBlock.dma->nTransferInformation     =   (DREQ << TI_PERMAP_SHIFT)
						    | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
						    | TI_SRC_DREQ
						    | TI_DEST_WIDTH
						    | TI_DEST_INC
						    | TI_WAIT_RESP;
	m_pControlBlock.dma->nSourceAddress           = nIOAddress;
	m_pControlBlock.dma->nDestinationAddress      = BUS_ADDRESS ((uintptr) pDestination);
	m_pControlBlock.dma->nTransferLength          = nLength;
	m_pControlBlock.dma->n2DModeStride            = 0;
	m_pControlBlock.dma->nNextControlBlockAddress = 0;

	m_nDestinationAddress = (uintptr) pDestination;
	m_nBufferLength = nLength;

	CleanAndInvalidateDataCacheRange ((uintptr) pDestination, nLength);
}

void CDMAChannel::SetupIOWrite (u32 nIOAddress, const void *pSource, size_t nLength, TDREQ DREQ)
{
	assert (pSource != 0);
	assert (nLength > 0);
	assert (nLength <= TXFR_LEN_MAX);
	assert (   !(read32 (ARM_DMACHAN_DEBUG (m_nChannel)) & DEBUG_LITE)
		|| nLength <= TXFR_LEN_MAX_LITE);
#if RASPPI >= 4
	// TODO: add DMA4 support
	assert (m_nChannel <= 10);
#endif

	nIOAddress &= 0xFFFFFF;
	assert (nIOAddress != 0);
	nIOAddress += GPU_IO_BASE;

	assert (m_pControlBlock.ui != 0);
	m_pControlBlock.dma->nTransferInformation     =   (DREQ << TI_PERMAP_SHIFT)
						    | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
						    | TI_SRC_WIDTH
						    | TI_SRC_INC
						    | TI_DEST_DREQ
						    | TI_WAIT_RESP;
	m_pControlBlock.dma->nSourceAddress           = BUS_ADDRESS ((uintptr) pSource);
	m_pControlBlock.dma->nDestinationAddress      = nIOAddress;
	m_pControlBlock.dma->nTransferLength          = nLength;
	m_pControlBlock.dma->n2DModeStride            = 0;
	m_pControlBlock.dma->nNextControlBlockAddress = 0;

	m_nDestinationAddress = 0;

	CleanAndInvalidateDataCacheRange ((uintptr) pSource, nLength);
}

void CDMAChannel::SetupMemCopy2D (void *pDestination, const void *pSource,
				  size_t nBlockLength, unsigned nBlockCount, size_t nBlockStride,
				  unsigned nBurstLength)
{
	assert (pDestination != 0);
	assert (pSource != 0);
	assert (nBlockLength > 0);
	assert (nBlockLength <= 0xFFFF);
	assert (nBlockCount > 0);
	assert (nBlockCount <= 0x3FFF);
	assert (nBlockStride <= 0xFFFF);
	assert (nBurstLength <= 15);

	assert (!(read32 (ARM_DMACHAN_DEBUG (m_nChannel)) & DEBUG_LITE));
#if RASPPI >= 4
	// TODO: add DMA4 support
	assert (m_nChannel <= 10);
#endif

	assert (m_pControlBlock.ui != 0);

	m_pControlBlock.dma->nTransferInformation     =   (nBurstLength << TI_BURST_LENGTH_SHIFT)
						    | TI_SRC_WIDTH
						    | TI_SRC_INC
						    | TI_DEST_WIDTH
						    | TI_DEST_INC
						    | TI_TDMODE;
	m_pControlBlock.dma->nSourceAddress           = BUS_ADDRESS ((uintptr) pSource);
	m_pControlBlock.dma->nDestinationAddress      = BUS_ADDRESS ((uintptr) pDestination);
	m_pControlBlock.dma->nTransferLength          =   ((nBlockCount-1) << TXFR_LEN_YLENGTH_SHIFT)
						    | (nBlockLength << TXFR_LEN_XLENGTH_SHIFT);
	m_pControlBlock.dma->n2DModeStride            = nBlockStride << STRIDE_DEST_SHIFT;
	m_pControlBlock.dma->nNextControlBlockAddress = 0;

	m_nDestinationAddress = 0;

	CleanAndInvalidateDataCacheRange ((uintptr) pSource, nBlockLength*nBlockCount);
}

void CDMAChannel::SetCompletionRoutine (TDMACompletionRoutine *pRoutine, void *pParam)
{
	assert (m_nChannel <= 12);
	assert (m_pInterruptSystem != 0);

#if RASPPI >= 4
	// TO DO: On Pi 4, DMA 7 & 8 and DMA 9 & 10 share IRQs, so we need to share
	// the completion handlers for these.  Each needs a common static stub
	// that maintains a pair of user handler callback pointers, and calls the
	// appropriate one (or both) according to the interrupt status bits.
	//
	// Until that's implemented, you can still use a completion routine for ONE
	// channel in each pair at any given time, but NOT BOTH.  If you try to use
	// both channels with completion routines at the same time, the interrupt
	// system will halt with an assert failure when you try to set up the second
	// completion routine, because it'll see that the shared IRQ slot is already
	// in use.
#endif

	if (!m_bIRQConnected)
	{
		m_pInterruptSystem->ConnectIRQ (CMachineInfo::GetIRQForDMA (m_nChannel), InterruptStub, this);

		m_bIRQConnected = TRUE;
	}

	m_pCompletionRoutine = pRoutine;
	assert (m_pCompletionRoutine != 0);

	m_pCompletionParam = pParam;
}

void CDMAChannel::Start (void)
{
	assert (m_nChannel < DMA_CHANNELS);
	assert (m_pControlBlock.ui != 0);

	if (m_pCompletionRoutine != 0)
	{
		assert (m_pInterruptSystem != 0);
		assert (m_bIRQConnected);
		m_pControlBlock.dma->nTransferInformation |= TI_INTEN;
	}

	PeripheralEntry ();

	assert (!(read32 (ARM_DMACHAN_CS (m_nChannel)) & CS_INT));
	assert (!(read32 (ARM_DMA_INT_STATUS) & (1 << m_nChannel)));

	uintptr cbRegValue = BUS_ADDRESS (m_pControlBlock.ui);
#if RASPPI >= 4
	if (m_nChannel >= 11)
	{
		// DMA4 channel - use physical address, bits 36:5 only (shifted left 5)
		cbRegValue = FULL_ADDRESS(m_pControlBlock.ui) >> 5;
	}
#endif
	write32 (ARM_DMACHAN_CONBLK_AD (m_nChannel), cbRegValue);

	CleanAndInvalidateDataCacheRange (m_pControlBlock.ui, m_cControlBlockSize);

	write32 (ARM_DMACHAN_CS (m_nChannel),   CS_WAIT_FOR_OUTSTANDING_WRITES
					      | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
					      | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)
					      | CS_ACTIVE);

	PeripheralExit ();
}

boolean CDMAChannel::Wait (void)
{
	assert (m_nChannel < DMA_CHANNELS);
	assert (m_pCompletionRoutine == 0);

	PeripheralEntry ();

	u32 nCS;
	while ((nCS = read32 (ARM_DMACHAN_CS (m_nChannel))) & CS_ACTIVE)
	{
		// do nothing
	}

#if RASPPI >= 4
	const unsigned errorBit = (m_nChannel < 11 ? CS_ERROR : CS4_ERROR);
#else
	const unsigned errorBit = CS_ERROR;
#endif

	m_bStatus = (nCS & errorBit) ? FALSE : TRUE;

	if (m_nDestinationAddress != 0)
	{
		CleanAndInvalidateDataCacheRange (m_nDestinationAddress, m_nBufferLength);
	}

	PeripheralExit ();

	return m_bStatus;
}

boolean CDMAChannel::GetStatus (void)
{
	assert (m_nChannel < DMA_CHANNELS);
	assert (!(read32 (ARM_DMACHAN_CS (m_nChannel)) & CS_ACTIVE));

	return m_bStatus;
}

void CDMAChannel::InterruptHandler (void)
{
	if (m_nDestinationAddress != 0)
	{
		CleanAndInvalidateDataCacheRange (m_nDestinationAddress, m_nBufferLength);
	}

	PeripheralEntry ();

	assert (m_nChannel < DMA_CHANNELS);

#ifndef NDEBUG
	u32 nIntStatus = read32 (ARM_DMA_INT_STATUS);
#endif
	u32 nIntMask = 1 << m_nChannel;
	assert (nIntStatus & nIntMask);
	write32 (ARM_DMA_INT_STATUS, nIntMask);

	u32 nCS = read32 (ARM_DMACHAN_CS (m_nChannel));
	assert (nCS & CS_INT);
	assert (!(nCS & CS_ACTIVE));
	write32 (ARM_DMACHAN_CS (m_nChannel), CS_INT); 

	PeripheralExit ();

	m_bStatus = nCS & CS_ERROR ? FALSE : TRUE;

	assert (m_pCompletionRoutine != 0);
	(*m_pCompletionRoutine) (m_nChannel, m_bStatus, m_pCompletionParam);
}

void CDMAChannel::InterruptStub (void *pParam)
{
	CDMAChannel *pThis = (CDMAChannel *) pParam;
	assert (pThis != 0);

	pThis->InterruptHandler ();
}
