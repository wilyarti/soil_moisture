//
// Created by wilyarti on 3/6/20.
//
#include <lmic.h>
#include <hal/hal.h>
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { INSERT KEY HERE };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { INSERT KEY HERE };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x000000 ; // <-- Change this address for every node!
