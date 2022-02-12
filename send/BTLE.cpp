/*
 * Copyright (C) 2013 Florian Echtler <floe@butterbrot.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation.
*/

#include "BTLE.h"


const uint8_t channel[3]   = {37,38,39};  // logical BTLE channel number (37-39)
const uint8_t frequency[3] = { 2,26,80};  // physical frequency (2400+x MHz)


// This is a rather convoluted hack to extract the month number from the build date in
// the __DATE__ macro using a small hash function + lookup table. Since all inputs are
// const, this can be fully resolved by the compiler and saves over 200 bytes of code.
#define month(m) month_lookup[ (( ((( (m[0] % 24) * 13) + m[1]) % 24) * 13) + m[2]) % 24 ]
const uint8_t month_lookup[24] = { 0,6,0,4,0,1,0,17,0,8,0,0,3,0,0,0,18,2,16,5,9,0,1,7 };


// change buffer contents to "wire bit order"
void BTLE::swapbuf( uint8_t len ) {

	uint8_t* buf = (uint8_t*)&buffer;

	while (len--) {

		uint8_t a = *buf;
		uint8_t v = 0;

		if (a & 0x80) v |= 0x01;
		if (a & 0x40) v |= 0x02;
		if (a & 0x20) v |= 0x04;
		if (a & 0x10) v |= 0x08;
		if (a & 0x08) v |= 0x10;
		if (a & 0x04) v |= 0x20;
		if (a & 0x02) v |= 0x40;
		if (a & 0x01) v |= 0x80;

		*(buf++) = v;
	}
}


// constructor
BTLE::BTLE( RF24* _radio ):
	radio(_radio),
	current(0)
{ }

// Simple converter from arduino float to a nRF_Float.
// Supports values from -167772 to +167772, with two decimal places.
nRF_Float 
BTLE::to_nRF_Float(float t) {
  int32_t ret;
  int32_t exponent = -2;
  ret = ((exponent & 0xff) << 24) | (((int32_t)(t * 100)) & 0xffffff);
  return ret;
}

// set BTLE-compatible radio parameters
void BTLE::begin( const char* _name ) {

	name = _name;
	radio->begin();

	// set standard parameters
	radio->setAutoAck(false);
	radio->setDataRate(RF24_1MBPS);
	radio->disableCRC();
	radio->setChannel( frequency[current] );
	radio->setRetries(0,0);
	radio->setPALevel(RF24_PA_MAX);

	// set advertisement address: 0x8E89BED6 (bit-reversed -> 0x6B7D9171)
	radio->setAddressWidth(4);
	radio->openReadingPipe(0,0x6B7D9171);
	radio->openWritingPipe(  0x6B7D9171);

	radio->powerUp();
}

// set the current channel (from 37 to 39)
void BTLE::setChannel( uint8_t num ) {
	current = min(2,max(0,num-37));
	radio->setChannel( frequency[current] );
}

// hop to the next channel
void BTLE::hopChannel() {
	current++;
	if (current >= sizeof(channel)) current = 0;
	radio->setChannel( frequency[current] );
}


// Broadcast an advertisement packet with optional payload
// Data type will be 0xFF (Manufacturer Specific Data)
bool BTLE::advertise( void* buf, uint8_t buflen ) {
	return advertise(0xFF, buf, buflen);
}

bool BTLE::addChunk(uint8_t chunk_type, uint8_t buflen, const void* buf) {
	if (buffer.pl_size + buflen + 2 > 21 + 6) // (buflen+2) is how much this chunk will take, 21 is payload size without crc and 6 is MAC size
		return false;
	btle_pdu_chunk* chunk = (btle_pdu_chunk*) (buffer.payload+buffer.pl_size-6);
	chunk->type = chunk_type;
	for (uint8_t i = 0; i < buflen; i++)
		chunk->data[i] = ((uint8_t*)buf)[i];
	chunk->size = buflen + 1;
	buffer.pl_size += buflen + 2;
	return true;
}

// Broadcast an advertisement packet with a specific data type
// Standardized data types can be seen here: 
// https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile
bool BTLE::advertise( uint8_t data_type, void* buf, uint8_t buflen ) {
	preparePacket();
	
	// add custom data, if applicable
	if (buflen > 0) {
		bool success = addChunk(data_type, buflen, buf);
		if (!success) {
			return false;
		}
	}
	
	transmitPacket();
	return true;
}

void BTLE::preparePacket() {
	// insert pseudo-random MAC address
	/*buffer.mac[0] = ((__TIME__[6]-0x30) << 4) | (__TIME__[7]-0x30);
	buffer.mac[1] = ((__TIME__[3]-0x30) << 4) | (__TIME__[4]-0x30);
	buffer.mac[2] = ((__TIME__[0]-0x30) << 4) | (__TIME__[1]-0x30);
	buffer.mac[3] = ((__DATE__[4]-0x30) << 4) | (__DATE__[5]-0x30);
	buffer.mac[4] = month(__DATE__);
	buffer.mac[5] = ((__DATE__[9]-0x30) << 4) | (__DATE__[10]-0x30) | 0xC0; // static random address should have two topmost bits set
*/


  buffer.mac[0] = 0x13;
  buffer.mac[1] = 0x24;
  buffer.mac[2] = 0x35;
  buffer.mac[3] = 0x46;
  buffer.mac[4] = 0x57;
  buffer.mac[5] = 0x68; // static random address should have two topmost bits set

  
	buffer.pdu_type = 0x42;    // PDU type: ADV_NONCONN_IND, TX address is random
	buffer.pl_size = 6; //including MAC
	
	// add device descriptor chunk
	uint8_t flags = 0x05;
	addChunk(0x01, 1, &flags);
	
	// add "complete name" chunk
	if (strlen(name) > 0) {
		addChunk(0x09, strlen(name), name);
	}
}

void BTLE::transmitPacket() {
	uint8_t pls = buffer.pl_size - 6;
	// calculate CRC over header+MAC+payload, append after payload
	uint8_t* outbuf = (uint8_t*)&buffer;
	crc( pls+8, outbuf+pls+8);
	
	// whiten header+MAC+payload+CRC, swap bit order
	whiten( pls+11 );
	swapbuf( pls+11 );
	
	// flush buffers and send
	radio->stopListening();
	radio->write( outbuf, pls+11 );
}

// listen for advertisement packets
bool BTLE::listen(int timeout) {

	radio->startListening();
	delay(timeout);

	if (!radio->available())
		return false;

	uint8_t total_size = 0;
	uint8_t* inbuf = (uint8_t*)&buffer;

	while (radio->available()) {

		// fetch the payload, and check if there are more left
		radio->read( inbuf, sizeof(buffer) );

		// decode: swap bit order, un-whiten
		swapbuf( sizeof(buffer) );
		whiten( sizeof(buffer) );
		
		// size is w/o header+CRC -> add 2 bytes header
		total_size = inbuf[1]+2;
		uint8_t in_crc[3];

		// calculate & compare CRC
		crc( total_size, in_crc );
		for (uint8_t i = 0; i < 3; i++)
			if (inbuf[total_size+i] != in_crc[i])
				return false;
	}

	return true;
}


// see BT Core Spec 4.0, Section 6.B.3.2
void BTLE::whiten( uint8_t len ) {

	uint8_t* buf = (uint8_t*)&buffer;

	// initialize LFSR with current channel, set bit 6
	uint8_t lfsr = channel[current] | 0x40;

	while (len--) {
		uint8_t res = 0;
		// LFSR in "wire bit order"
		for (uint8_t i = 1; i; i <<= 1) {
			if (lfsr & 0x01) {
				lfsr ^= 0x88;
				res |= i;
			}
			lfsr >>= 1;
		}
		*(buf++) ^= res;
	}
}

// see BT Core Spec 4.0, Section 6.B.3.1.1
void BTLE::crc( uint8_t len, uint8_t* dst ) {

	uint8_t* buf = (uint8_t*)&buffer;

	// initialize 24-bit shift register in "wire bit order"
	// dst[0] = bits 23-16, dst[1] = bits 15-8, dst[2] = bits 7-0
	dst[0] = 0xAA;
	dst[1] = 0xAA;
	dst[2] = 0xAA;

	while (len--) {

		uint8_t d = *(buf++);

		for (uint8_t i = 1; i; i <<= 1, d >>= 1) {

			// save bit 23 (highest-value), left-shift the entire register by one
			uint8_t t = dst[0] & 0x01;         dst[0] >>= 1;
			if (dst[1] & 0x01) dst[0] |= 0x80; dst[1] >>= 1;
			if (dst[2] & 0x01) dst[1] |= 0x80; dst[2] >>= 1;

			// if the bit just shifted out (former bit 23) and the incoming data
			// bit are not equal (i.e. bit_out ^ bit_in == 1) => toggle tap bits
			if (t != (d & 1)) {
				// toggle register tap bits (=XOR with 1) according to CRC polynom
				dst[2] ^= 0xDA; // 0b11011010 inv. = 0b01011011 ^= x^6+x^4+x^3+x+1
				dst[1] ^= 0x60; // 0b01100000 inv. = 0b00000110 ^= x^10+x^9
			}
		}
	}
}
