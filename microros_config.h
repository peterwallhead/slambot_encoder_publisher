#ifndef MICROROS_CONFIG_H
#define MICROROS_CONFIG_H

// Increase the XRCE history buffer to support larger custom messages
#define UCLIENT_PROFILE_CUSTOM_TRANSPORT_CONFIG
#define UXR_CONFIG_CUSTOM_TRANSPORT_MTU             128
#define UXR_CONFIG_MAX_HISTORY_PAYLOAD_SIZE         128

#endif  // MICROROS_CONFIG_H
