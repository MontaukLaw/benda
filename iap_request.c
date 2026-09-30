#include "iap_request.h"

#include <stdint.h>

#include "cy_pdl.h"
/* 使用DFU库切换启动目标，防止软件复位后再次直接跳回App1。 */
#include "cy_dfu.h"

/* ram_common is excluded from application and Bootloader startup clearing. */
#define IAP_REQUEST_ADDRESS   (0x20000004UL)
#define IAP_REQUEST_MAGIC     (0x49504150UL)

typedef struct
{
	uint32_t magic;
	uint32_t transport;
} iap_request_t;

static volatile iap_request_t *const iap_request =
	(volatile iap_request_t *)IAP_REQUEST_ADDRESS;

/* 保存传输方式，并将DFU保留的应用编号设为App0后执行软件复位。 */
void IapRequest_EnterBootloader(iap_transport_t transport)
{
	iap_request->transport = (uint32_t)transport;
	iap_request->magic = IAP_REQUEST_MAGIC;
	__DSB();
	Cy_DFU_ExecuteApp(0u);
}

iap_transport_t IapRequest_Peek(void)
{
	iap_transport_t transport = (iap_transport_t)iap_request->transport;

	if ((iap_request->magic != IAP_REQUEST_MAGIC)
		|| ((transport != IAP_TRANSPORT_UART)
			&& (transport != IAP_TRANSPORT_I2C)))
	{
		return IAP_TRANSPORT_NONE;
	}

	return transport;
}

iap_transport_t IapRequest_Consume(void)
{
	iap_transport_t transport = IapRequest_Peek();

	iap_request->magic = 0u;
	iap_request->transport = 0u;
	__DSB();
	return transport;
}
