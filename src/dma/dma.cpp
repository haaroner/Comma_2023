#include "dma.h"

Dma::Dma(uint32_t RCC_AHB1Periph_DMAx,Adc &adcx):m_adcx(adcx)
{
  m_RCC_AHB1Periph_DMAx = RCC_AHB1Periph_DMAx;
}

void Dma::adcInitInDma(uint8_t _num_of_adc_cycles)
{
	m_adcx.adcInit(_num_of_adc_cycles);
	m_adcx.startAdc();
	m_adcx.setChannel();
	m_adcx.adcDmaInit();
}
	void Dma::dmaInit(DMA_Stream_TypeDef* dmax_streamx, uint32_t dma_channelx, uint32_t bufferSize)
{
	RCC_AHB1PeriphClockCmd(m_RCC_AHB1Periph_DMAx, ENABLE);
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(dmax_streamx);
	
	DMA_InitStructure.DMA_Channel = dma_channelx;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(m_adcx.getAdc()->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&m_adcData[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = bufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(dmax_streamx, &DMA_InitStructure);
	DMA_Cmd(dmax_streamx, ENABLE);
}

uint16_t Dma::dataReturn(uint8_t n)
{
	return m_adcData[n];
}