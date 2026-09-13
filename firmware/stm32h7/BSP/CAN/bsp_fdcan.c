/* BSP: FDCAN peripheral adaptation. */
#include "bsp_fdcan.h"
#include "QD4310.h"
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
	HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter);
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	/* FDCAN HAL expects the DLC field encoding, not a raw byte count.  A raw
	 * value of 3 made short QD4310 frames carry an invalid/zero DLC on H7. */
	if(len == 0)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_0;
	if(len == 1)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_1;
	if(len == 2)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_2;
	if(len == 3)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_3;
	if(len == 4)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_4;
	if(len == 5)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_5;
	if(len == 6)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_6;
	if(len == 7)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_7;
	if(len == 8)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_8;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len = 0;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		/* This HAL returns the raw DLC code (0..15), not FDCAN_DLC_BYTES_x. */
		if(pRxHeader.DataLength <= 8U)
			len = (uint8_t)pRxHeader.DataLength;
		
		return len;//接收数据
	}
	return 0;	
}



uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
volatile uint32_t qd4310_feedback_count = 0;
volatile uint32_t qd4310_feedback_count_motor0 = 0;
volatile uint32_t qd4310_feedback_count_motor1 = 0;
volatile uint8_t qd4310_active_can = 0;
volatile uint32_t qd4310_feedback_tick_motor0 = 0;
volatile uint32_t qd4310_feedback_tick_motor1 = 0;
void fdcan1_rx_callback(void)
{
	if(fdcanx_receive(&hfdcan1, &rec_id1, rx_data1) == 8U &&
	   Motor_0.hcan == &hfdcan1 &&
	   rec_id1 == (uint16_t)(0x500U + Motor_0.id))
	{
		QD4310_Update(&Motor_0, rx_data1);
		qd4310_active_can = 1U;
		qd4310_feedback_count++;
		qd4310_feedback_count_motor0++;
		qd4310_feedback_tick_motor0 = HAL_GetTick();
	}
}
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{
	if(fdcanx_receive(&hfdcan2, &rec_id2, rx_data2) == 8U &&
	   Motor_1.hcan == &hfdcan2 &&
	   rec_id2 == (uint16_t)(0x500U + Motor_1.id))
	{
		QD4310_Update(&Motor_1, rx_data2);
		qd4310_active_can = 2U;
		qd4310_feedback_count++;
		qd4310_feedback_count_motor1++;
		qd4310_feedback_tick_motor1 = HAL_GetTick();
	}
}
uint8_t rx_data3[8] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{
	if(fdcanx_receive(&hfdcan3, &rec_id3, rx_data3) == 8U &&
	   Motor_0.hcan == &hfdcan3 &&
	   rec_id3 == (uint16_t)(0x500U + Motor_0.id))
	{
		QD4310_Update(&Motor_0, rx_data3);
		qd4310_active_can = 3U;
		qd4310_feedback_count++;
	}
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
	if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
	if(hfdcan == &hfdcan3)
	{
		fdcan3_rx_callback();
	}
}











