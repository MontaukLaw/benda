#ifndef IAP_REQUEST_H
#define IAP_REQUEST_H

#include <stdbool.h>

typedef enum
{
	IAP_TRANSPORT_NONE = 0u,
	IAP_TRANSPORT_UART = 1u,
	IAP_TRANSPORT_I2C = 2u,
} iap_transport_t;

void IapRequest_EnterBootloader(iap_transport_t transport);
iap_transport_t IapRequest_Peek(void);
iap_transport_t IapRequest_Consume(void);

#endif
