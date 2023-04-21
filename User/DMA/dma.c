#include "lpc177x_8x_gpdma.h"
#include "debug_frmwrk.h"
#include "lpc177x_8x_pinsel.h"
#include "stdarg.h"
#include "Globalvalue/GlobalValue.h"

#define DMA_SIZE        11
#define DMASEND_SIZE    21
uint8_t DMADest_Buffer[DMA_SIZE];
uint8_t DMAsend_Buffer[DMASEND_SIZE];
GPDMA_Channel_CFG_Type GPDMACfg;
GPDMA_Channel_CFG_Type GPDMACfg1;
GPDMA_Channel_CFG_Type GPDMACfg2;
uint8_t missflag;
void lpc1788_DMA_Init(void);
void lpc1788_DMA_SetInit(void);
void DMA_IRQHandler (void)
{
		
    uint8_t i,ch,j;
		for(ch = 0;ch < 2;ch++)
		{
//			if (GPDMA_IntGetStatus(GPDMA_STAT_INT, ch))
//			{
	//                GPDMA_ChannelCmd(0, DISABLE);
					if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, ch))
					{                       
							GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, ch);      
	//                        for(i=0;i<DMA_SIZE;i++){printf("%d",DMADest_Buffer[i]);}
	//                        printf("\r\n");
							switch(ch)
							{
								case 0:
								{
									switch(DMADest_Buffer[1])
									{
										case FRAME_READ_RESULT:
										{
											if(DMADest_Buffer[READLEN-1] == SUMCK(DMADest_Buffer,READLEN-1))
											{
												for(i = 0;i < READLEN;i ++)
												{
													ComBuf.rec.buf[i] = DMADest_Buffer[i];
												}
												ComBuf.rec.end = 1;
												Uart_Process();
											}
										}break;
										case FRAME_RANGE_SET:
										{
											if(DMADest_Buffer[SETRANGELEN-1] == SUMCK(DMADest_Buffer,SETRANGELEN-1))
											{
												for(i = 0;i < SETRANGELEN;i ++)
												{
													ComBuf.rec.buf[i] = DMADest_Buffer[i];
												}
												ComBuf.rec.end = 1;
												Uart_Process();
											}
										}break;
										case FRAME_CLEAR:
										{
											
										}break;
										default:break;
									}
//									if(DMADest_Buffer[0] != UART_REC_BEGIN && DMADest_Buffer[1] != UART_REC_END)
//									{
////										debug_frmwrk_init();//´®¿Ú3³õÊ¼»¯
//										lpc1788_DMA_Init();
//									}else{
//											for(i = 0;i < DMA_SIZE;i ++)
//											{
//												ComBuf.rec.buf[i] = DMADest_Buffer[i];
//											}
//											ComBuf.rec.end = 1;
//									}
									LPC_GPDMACH0->CDestAddr = GPDMACfg.DstMemAddr;// Assign memory destination address
									LPC_GPDMACH0->CControl= GPDMA_DMACCxControl_TransferSize((uint32_t)GPDMACfg.TransferSize) \
									| GPDMA_DMACCxControl_SBSize((uint32_t)GPDMA_LUTPerBurst[GPDMACfg.SrcConn]) \
									| GPDMA_DMACCxControl_DBSize((uint32_t)GPDMA_LUTPerBurst[GPDMACfg.SrcConn]) \
									| GPDMA_DMACCxControl_SWidth((uint32_t)GPDMA_LUTPerWid[GPDMACfg.SrcConn]) \
									| GPDMA_DMACCxControl_DWidth((uint32_t)GPDMA_LUTPerWid[GPDMACfg.SrcConn]) \
									| GPDMA_DMACCxControl_DI \
									| GPDMA_DMACCxControl_I;
									GPDMA_ChannelCmd(0, ENABLE);
									
									if(GetSystemStatus()==SYS_STATUS_SETUPTEST && ComBuf.rec.buf[1] != 0X20)
									{
										missflag = 1;
									}
								}break;
								case 1:
								{
									GPDMA_ChannelCmd(1, DISABLE);
									GPDMA_ChannelCmd(0, ENABLE);
								}break;
								case 2:
								{
									GPDMA_ChannelCmd(2, DISABLE);
								}break;
							}
							     
					}
					if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, ch))                /* ?¨¬2¨¦DMA¨ª¡§¦Ì¨¤0?D??¡ä¨ª?¨®¡Á¡ä¨¬? */
					{
							GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, ch);//Channel0_Err++;        /* ??3yDMA¨ª¡§¦Ì¨¤0?D??¡ä¨ª?¨®???¨® */
					}
					
//			}
		}
}

void lpc1788_DMA_Init(void)
{
//        GPDMA_Channel_CFG_Type GPDMACfg;

	GPDMA_Init();  
	NVIC_DisableIRQ(DMA_IRQn);               
	NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));
			
//DMA USART RX CONFIG	
	GPDMACfg.ChannelNum = 0;
	GPDMACfg.SrcMemAddr =0;       
	GPDMACfg.DstMemAddr = (uint32_t)&DMADest_Buffer;       
	GPDMACfg.TransferSize = sizeof(DMADest_Buffer);
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;       
	GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;       
	GPDMACfg.DstConn = 0;       
	GPDMACfg.DMALLI = 0;       

	GPDMA_Setup(&GPDMACfg);
 
//		//DMA USART3 TX CONFIG	
//	GPDMACfg2.ChannelNum = 2;
//	GPDMACfg2.SrcMemAddr =(uint32_t)&ComBuf.rec.buf[5];       
//	GPDMACfg2.DstMemAddr = 0;       
//	GPDMACfg2.TransferSize = 1;
//	GPDMACfg2.TransferWidth = 0;
//	GPDMACfg2.TransferType = GPDMA_TRANSFERTYPE_M2P;       
//	GPDMACfg2.SrcConn = 0;       
//	GPDMACfg2.DstConn = GPDMA_CONN_UART3_Tx;       
//	GPDMACfg2.DMALLI = 0;       
//	GPDMA_Setup(&GPDMACfg2);
	
//DMA USART TX CONFIG	
	GPDMACfg1.ChannelNum = 1;
	GPDMACfg1.SrcMemAddr =(uint32_t)&ComBuf.send.buf;       
	GPDMACfg1.DstMemAddr = 0;       
	GPDMACfg1.TransferSize = 4;
	GPDMACfg1.TransferWidth = 0;
	GPDMACfg1.TransferType = GPDMA_TRANSFERTYPE_M2P;       
	GPDMACfg1.SrcConn = 0;       
	GPDMACfg1.DstConn = GPDMA_CONN_UART0_Tx;       
	GPDMACfg1.DMALLI = 0;       
	GPDMA_Setup(&GPDMACfg1);
	

//        LPC_SSP0->DMACR |=0x11;//SSP_DMACmd (0, SSP_DMA_RXDMA_EN, ENABLE);
 
	NVIC_EnableIRQ(DMA_IRQn);
	ComBuf.send.buf[0] = 0xAB;
	ComBuf.send.buf[1] = 0x00;
	ComBuf.send.buf[2] = 0xbf;
	GPDMA_ChannelCmd(0, DISABLE);
//	GPDMA_ChannelCmd(2, DISABLE);
	GPDMA_ChannelCmd(1, ENABLE);
//	GPDMA_ChannelCmd(2, ENABLE);
//	GPDMA_ChannelCmd(0, ENABLE);
//	GPDMA_ChannelCmd(1, DISABLE);
}

void lpc1788_DMA_SetInit(void)
{
//        GPDMA_Channel_CFG_Type GPDMACfg;

	GPDMA_Init();  
	NVIC_DisableIRQ(DMA_IRQn);               
	NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));
			
//DMA USART RX CONFIG	
	GPDMACfg.ChannelNum = 0;
	GPDMACfg.SrcMemAddr =0;       
	GPDMACfg.DstMemAddr = (uint32_t)&DMADest_Buffer;       
	GPDMACfg.TransferSize = 4;
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;       
	GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;       
	GPDMACfg.DstConn = 0;       
	GPDMACfg.DMALLI = 0;       

	GPDMA_Setup(&GPDMACfg);
 
//DMA USART TX CONFIG	
	GPDMACfg1.ChannelNum = 1;
	GPDMACfg1.SrcMemAddr =(uint32_t)&ComBuf.send.buf;       
	GPDMACfg1.DstMemAddr = 0;       
	GPDMACfg1.TransferSize = 4;
	GPDMACfg1.TransferWidth = 0;
	GPDMACfg1.TransferType = GPDMA_TRANSFERTYPE_M2P;       
	GPDMACfg1.SrcConn = 0;       
	GPDMACfg1.DstConn = GPDMA_CONN_UART0_Tx;       
	GPDMACfg1.DMALLI = 0;       
	GPDMA_Setup(&GPDMACfg1);
//        LPC_SSP0->DMACR |=0x11;//SSP_DMACmd (0, SSP_DMA_RXDMA_EN, ENABLE);
 
	ComBuf.send.buf[0] = 0xAB;
	ComBuf.send.buf[1] = 0x52;
	ComBuf.send.buf[2] = Save_Res.Set_Data.Range;
	ComBuf.send.buf[3] = 0xbf;
	NVIC_EnableIRQ(DMA_IRQn);
	GPDMA_ChannelCmd(0, DISABLE);
	GPDMA_ChannelCmd(1, ENABLE);
//        LPC_SSP0->DMACR |=0x11;//SSP_DMACmd (0, SSP_DMA_RXDMA_EN, ENABLE);
 
//	GPDMA_ChannelCmd(0, ENABLE);
//	GPDMA_ChannelCmd(1, DISABLE);
}

void DMASendReadInit(void)
{
	GPDMACfg1.ChannelNum = 1;
	GPDMACfg1.SrcMemAddr =(uint32_t)&ComBuf.send.buf;       
	GPDMACfg1.DstMemAddr = 0;       
	GPDMACfg1.TransferSize = 4;
	GPDMACfg1.TransferWidth = 0;
	GPDMACfg1.TransferType = GPDMA_TRANSFERTYPE_M2P;       
	GPDMACfg1.SrcConn = 0;       
	GPDMACfg1.DstConn = GPDMA_CONN_UART0_Tx;       
	GPDMACfg1.DMALLI = 0;       
	GPDMA_Setup(&GPDMACfg1);
		
	GPDMA_ChannelCmd(1, ENABLE);
}

void DMASendRangeInit(void)
{
//	GPDMACfg.ChannelNum = 0;
//	GPDMACfg.SrcMemAddr =0;       
//	GPDMACfg.DstMemAddr = (uint32_t)&DMADest_Buffer;       
//	GPDMACfg.TransferSize = 4;
//	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;       
//	GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;       
//	GPDMACfg.DstConn = 0;       
//	GPDMACfg.DMALLI = 0;       
//	GPDMA_Setup(&GPDMACfg);
	
	GPDMACfg1.ChannelNum = 1;
	GPDMACfg1.SrcMemAddr =(uint32_t)&ComBuf.send.buf;       
	GPDMACfg1.DstMemAddr = 0;       
	GPDMACfg1.TransferSize = 4;
	GPDMACfg1.TransferWidth = 0;
	GPDMACfg1.TransferType = GPDMA_TRANSFERTYPE_M2P;       
	GPDMACfg1.SrcConn = 0;       
	GPDMACfg1.DstConn = GPDMA_CONN_UART0_Tx;       
	GPDMACfg1.DMALLI = 0;       
	GPDMA_Setup(&GPDMACfg1);
		
	GPDMA_ChannelCmd(1, ENABLE);
}

void DMASendToPC(void)
{
//	GPDMACfg.ChannelNum = 0;
//	GPDMACfg.SrcMemAddr =0;       
//	GPDMACfg.DstMemAddr = (uint32_t)&DMADest_Buffer;       
//	GPDMACfg.TransferSize = 4;
//	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;       
//	GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;       
//	GPDMACfg.DstConn = 0;       
//	GPDMACfg.DMALLI = 0;       
//	GPDMA_Setup(&GPDMACfg);
	GPDMACfg2.ChannelNum = 2;
	GPDMACfg2.SrcMemAddr =(uint32_t)&ComBuf.rec.buf[5];       
	GPDMACfg2.DstMemAddr = 0;       
	GPDMACfg2.TransferSize = 1;
	GPDMACfg2.TransferWidth = 0;
	GPDMACfg2.TransferType = GPDMA_TRANSFERTYPE_M2P;       
	GPDMACfg2.SrcConn = 0;       
	GPDMACfg2.DstConn = GPDMA_CONN_UART3_Tx;       
	GPDMACfg2.DMALLI = 0;       
	GPDMA_Setup(&GPDMACfg2);
		
	GPDMA_ChannelCmd(2, ENABLE);
}