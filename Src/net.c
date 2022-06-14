/**
  ******************************************************************************
  * @file    net.c
  * @brief   This file provides code for the configuration
  * 	     of the net instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "net.h"
#include "enc28j60.h"
#include "string.h"
#include "stdio.h"
#include "modbus.h"
/* private macro -------------------------------------------------------------*/
#define TCP_DATA_START ((uint16_t)TCP_SRC_PORT_H_P+(buf[TCP_HEADER_LEN_P]>>4)*4)
/* private defines -----------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables----------------------------------------------------------*/
// make a return ip header from a received ip packet
static uint16_t ip_identifier = 1;
static uint8_t macaddr[6];
static uint8_t ipaddr[4];
//static uint16_t info_data_len=0;
static uint8_t seqnum=0xa; // my initial tcp sequence number
// Web server port, used when implementing webserver
static uint8_t wwwport_l=80; // server port
static uint8_t wwwport_h=0;  // Note: never use same as TCPCLIENT_SRC_PORT_H

uint8_t packet_buf[512];
uint16_t packet_len;
uint8_t data_buf[512];
uint16_t data_len;
/* private function prototypes -----------------------------------------------*/

// The Ip checksum is calculated over the ip header only starting
// with the header length field and a total length of 20 bytes
// unitl ip.dst
// You must set the IP checksum field to zero before you start
// the calculation.
// len for ip is 20.
//
// For UDP/TCP we do not make up the required pseudo header. Instead we
// use the ip.src and ip.dst fields of the real packet:
// The udp checksum calculation starts with the ip.src field
// Ip.src=4bytes,Ip.dst=4 bytes,Udp header=8bytes + data length=16+len
// In other words the len here is 8 + length over which you actually
// want to calculate the checksum.
// You must set the checksum field to zero before you start
// the calculation.
// The same algorithm is also used for udp and tcp checksums.
// len for udp is: 8 + 8 + data length
// len for tcp is: 4+4 + 20 + option len + data length
//
// For more information on how this algorithm works see:
// http://www.netfor2.com/checksum.html
// http://www.msc.uky.edu/ken/cs471/notes/chap3.htm
// The RFC has also a C code example: http://www.faqs.org/rfcs/rfc1071.html
uint16_t checksum(uint8_t *buf, uint16_t len,uint8_t type)
{
	// type 0=ip , icmp
	//      1=udp
	//      2=tcp
	uint32_t sum = 0;

	//if(type==0){
	//        // do not add anything, standard IP checksum as described above
	//        // Usable for ICMP and IP header
	//}
	if (type == 1) {
		sum += IP_PROTO_UDP_V; // protocol udp
		// the length here is the length of udp (data+header len)
		// =length given to this function - (IP.scr+IP.dst length)
		sum += len - 8; // = real udp len
	}
	if (type == 2) {
		sum += IP_PROTO_TCP_V;
		// the length here is the length of tcp (data+header len)
		// =length given to this function - (IP.scr+IP.dst length)
		sum += len - 8; // = real tcp len
	}
	// build the sum of 16bit words
	while (len > 1) {
		sum += 0xFFFF & (((uint32_t) *buf << 8) | *(buf + 1));
		buf += 2;
		len -= 2;
	}
	// if there is a byte left then add it (padded with zero)
	if (len) {
		sum += ((uint32_t) (0xFF & *buf)) << 8;
	}
	// now calculate the sum over the bytes in the sum
	// until the result is only 16bit long
	while (sum >> 16) {
		sum = (sum & 0xFFFF) + (sum >> 16);
	}
	// build 1's complement:
	return ((uint16_t) sum ^ 0xFFFF);
}

// This initializes the web server
// you must call this function once before you use any of the other functions:
void net_init(uint8_t *mymac,uint8_t *myip,uint16_t port)
{
	uint8_t i = 0;
	wwwport_h = (port >> 8) & 0xff;
	wwwport_l = (port & 0xff);
	while (i < 4) {
		ipaddr[i] = myip[i];
		i++;
	}
	i = 0;
	while (i < 6) {
		macaddr[i] = mymac[i];
		i++;
	}
}

uint8_t eth_type_is_arp_and_my_ip(uint8_t *buf, uint16_t len)
{
	uint8_t i = 0;
	//
	if (len < 41) {
		return (0);
	}
	if (buf[ETH_TYPE_H_P] != ETHTYPE_ARP_H_V
			|| buf[ETH_TYPE_L_P] != ETHTYPE_ARP_L_V) {
		return (0);
	}
	while (i < 4) {
		if (buf[ETH_ARP_DST_IP_P + i] != ipaddr[i]) {
			return (0);
		}
		i++;
	}
	return (1);
}

uint8_t eth_type_is_ip_and_my_ip(uint8_t *buf, uint16_t len)
{
	uint8_t i = 0;
	//eth+ip+udp header is 42
	if (len < 42) {
		return (0);
	}
	if (buf[ETH_TYPE_H_P] != ETHTYPE_IP_H_V
			|| buf[ETH_TYPE_L_P] != ETHTYPE_IP_L_V) {
		return (0);
	}
	if (buf[IP_HEADER_LEN_VER_P] != 0x45) {
		// must be IP V4 and 20 byte header
		return (0);
	}
	while (i < 4) {
		if (buf[IP_DST_P + i] != ipaddr[i]) {
			return (0);
		}
		i++;
	}
	return (1);
}

// make a return eth header from a received eth packet
void make_eth(uint8_t *buf)
{
	uint8_t i = 0;
	//
	//copy the destination mac from the source and fill my mac into src
	while (i < 6) {
		buf[ETH_DST_MAC + i] = buf[ETH_SRC_MAC + i];
		buf[ETH_SRC_MAC + i] = macaddr[i];
		i++;
	}
}

// make a new eth header for IP packet
void make_eth_ip_new(uint8_t *buf, uint8_t* dst_mac)
{
	uint8_t i = 0;
	//
	//copy the destination mac from the source and fill my mac into src
	while (i < 6) {
		buf[ETH_DST_MAC + i] = dst_mac[i];
		buf[ETH_SRC_MAC + i] = macaddr[i];
		i++;
	}

	buf[ ETH_TYPE_H_P] = ETHTYPE_IP_H_V;
	buf[ ETH_TYPE_L_P ] = ETHTYPE_IP_L_V;
}

void fill_ip_hdr_checksum(uint8_t *buf)
{
	uint16_t ck;
	// clear the 2 byte checksum
	buf[IP_CHECKSUM_P] = 0;
	buf[IP_CHECKSUM_P + 1] = 0;
	buf[IP_FLAGS_P] = 0x40; // don't fragment
	buf[IP_FLAGS_P + 1] = 0;  // fragement offset
	buf[IP_TTL_P] = 64; // ttl
	// calculate the checksum:
	ck = checksum(&buf[IP_P], IP_HEADER_LEN, 0);
	buf[IP_CHECKSUM_P] = ck >> 8;
	buf[IP_CHECKSUM_P+1]=ck & 0xff;
}

// is it an arp reply (no len check here, you must first call eth_type_is_arp_and_my_ip)
uint8_t eth_type_is_arp_reply(uint8_t *buf){
	return (buf[ETH_ARP_OPCODE_L_P]==ETH_ARP_OPCODE_REPLY_L_V);
}

// is it an arp request (no len check here, you must first call eth_type_is_arp_and_my_ip)
uint8_t eth_type_is_arp_req(uint8_t *buf){
	return (buf[ETH_ARP_OPCODE_L_P]==ETH_ARP_OPCODE_REQ_L_V);
}

// make a new ip header for tcp packet

// make a return ip header from a received ip packet
void make_ip_tcp_new(uint8_t *buf, uint16_t len,uint8_t *dst_ip)
{
    uint8_t i=0;

	// set ipv4 and header length
	buf[ IP_P ] = IP_V4_V | IP_HEADER_LENGTH_V;

	// set TOS to default 0x00
	buf[ IP_TOS_P ] = 0x00;

	// set total length
	buf[ IP_TOTLEN_H_P ] = (len >>8)& 0xff;
	buf[ IP_TOTLEN_L_P ] = len & 0xff;

	// set packet identification
	buf[ IP_ID_H_P ] = (ip_identifier >>8) & 0xff;
	buf[ IP_ID_L_P ] = ip_identifier & 0xff;
	ip_identifier++;

	// set fragment flags
	buf[ IP_FLAGS_H_P ] = 0x00;
	buf[ IP_FLAGS_L_P ] = 0x00;

	// set Time To Live
	buf[ IP_TTL_P ] = 128;

	// set ip packettype to tcp/udp/icmp...
	buf[ IP_PROTO_P ] = IP_PROTO_TCP_V;

	// set source and destination ip address
        while(i<4){
                buf[IP_DST_P+i]=dst_ip[i];
                buf[IP_SRC_P+i]=ipaddr[i];
                i++;
        }
        fill_ip_hdr_checksum(buf);
}


void make_ip(uint8_t *buf)
{
	uint8_t i = 0;
	while (i < 4) {
		buf[IP_DST_P + i] = buf[IP_SRC_P + i];
		buf[IP_SRC_P + i] = ipaddr[i];
		i++;
	}
	fill_ip_hdr_checksum(buf);
}

// swap seq and ack number and count ack number up
void step_seq(uint8_t *buf,uint16_t rel_ack_num,uint8_t cp_seq)
{
	uint8_t i;
	uint8_t tseq;
	i = 4;
	// sequence numbers:
	// add the rel ack num to SEQACK
	while (i > 0) {
		rel_ack_num = buf[TCP_SEQ_H_P + i - 1] + rel_ack_num;
		tseq = buf[TCP_SEQACK_H_P + i - 1];
		buf[TCP_SEQACK_H_P + i - 1] = 0xff & rel_ack_num;
		if (cp_seq) {
			// copy the acknum sent to us into the sequence number
			buf[TCP_SEQ_H_P + i - 1] = tseq;
		} else {
			buf[TCP_SEQ_H_P + i - 1] = 0; // some preset vallue
		}
		rel_ack_num = rel_ack_num >> 8;
		i--;
	}
}

// make a return tcp header from a received tcp packet
// rel_ack_num is how much we must step the seq number received from the
// other side. We do not send more than 765 bytes of text (=data) in the tcp packet.
// No mss is included here.
//
// After calling this function you can fill in the first data byte at TCP_OPTIONS_P+4
// If cp_seq=0 then an initial sequence number is used (should be use in synack)
// otherwise it is copied from the packet we received
void make_tcphead(uint8_t *buf,uint16_t rel_ack_num,uint8_t cp_seq)
{
	uint8_t i;
	// copy ports:
	i = buf[TCP_DST_PORT_H_P];
	buf[TCP_DST_PORT_H_P] = buf[TCP_SRC_PORT_H_P];
	buf[TCP_SRC_PORT_H_P] = i;
	//
	i = buf[TCP_DST_PORT_L_P];
	buf[TCP_DST_PORT_L_P] = buf[TCP_SRC_PORT_L_P];
	buf[TCP_SRC_PORT_L_P] = i;
	step_seq(buf, rel_ack_num, cp_seq);
	// zero the checksum
	buf[TCP_CHECKSUM_H_P] = 0;
	buf[TCP_CHECKSUM_L_P] = 0;
	// no options:
	// 20 bytes:
	// The tcp header length is only a 4 bit field (the upper 4 bits).
	// It is calculated in units of 4 bytes.
	// E.g 20 bytes: 20/4=6 => 0x50=header len field
	buf[TCP_HEADER_LEN_P]=0x50;
}

void make_arp_answer_from_request(uint8_t *buf)
{
	uint8_t i = 0;
	//
	make_eth(buf);
	buf[ETH_ARP_OPCODE_H_P] = ETH_ARP_OPCODE_REPLY_H_V;
	buf[ETH_ARP_OPCODE_L_P] = ETH_ARP_OPCODE_REPLY_L_V;
	// fill the mac addresses:
	while (i < 6) {
		buf[ETH_ARP_DST_MAC_P + i] = buf[ETH_ARP_SRC_MAC_P + i];
		buf[ETH_ARP_SRC_MAC_P + i] = macaddr[i];
		i++;
	}
	i = 0;
	while (i < 4) {
		buf[ETH_ARP_DST_IP_P + i] = buf[ETH_ARP_SRC_IP_P + i];
		buf[ETH_ARP_SRC_IP_P + i] = ipaddr[i];
		i++;
	}
	// eth+arp is 42 bytes:
	enc28j60PacketSend(42,buf);
}

void make_echo_reply_from_request(uint8_t *buf,uint16_t len)
{
	make_eth(buf);
	make_ip(buf);
	buf[ICMP_TYPE_P] = ICMP_TYPE_ECHOREPLY_V;
	// we changed only the icmp.type field from request(=8) to reply(=0).
	// we can therefore easily correct the checksum:
	if (buf[ICMP_CHECKSUM_P] > (0xff - 0x08)) {
		buf[ICMP_CHECKSUM_P + 1]++;
	}
	buf[ICMP_CHECKSUM_P] += 0x08;
	//
	enc28j60PacketSend(len,buf);
}

// this is for the server not the client:
void make_tcp_synack_from_syn(uint8_t *buf)
{
	uint16_t ck;
	make_eth(buf);
	// total length field in the IP header must be set:
	// 20 bytes IP + 24 bytes (20tcp+4tcp options)
	buf[IP_TOTLEN_H_P] = 0;
	buf[IP_TOTLEN_L_P] = IP_HEADER_LEN + TCP_HEADER_LEN_PLAIN + 4;
	make_ip(buf);
	buf[TCP_FLAGS_P] = TCP_FLAGS_SYNACK_V;
	make_tcphead(buf, 1, 0);
	// put an inital seq number
	buf[TCP_SEQ_H_P + 0] = 0;
	buf[TCP_SEQ_H_P + 1] = 0;
	// we step only the second byte, this allows us to send packts
	// with 255 bytes, 512  or 765 (step by 3) without generating
	// overlapping numbers.
	buf[TCP_SEQ_H_P + 2] = seqnum;
	buf[TCP_SEQ_H_P + 3] = 0;
	// step the inititial seq num by something we will not use
	// during this tcp session:
	seqnum += 3;
	// add an mss options field with MSS to 1280:
	// 1280 in hex is 0x500
	buf[TCP_OPTIONS_P] = 2;
	buf[TCP_OPTIONS_P + 1] = 4;
	buf[TCP_OPTIONS_P + 2] = 0x05;
	buf[TCP_OPTIONS_P + 3] = 0x0;
	// The tcp header length is only a 4 bit field (the upper 4 bits).
	// It is calculated in units of 4 bytes.
	// E.g 24 bytes: 24/4=6 => 0x60=header len field
	buf[TCP_HEADER_LEN_P] = 0x60;
	// here we must just be sure that the web browser contacting us
	// will send only one get packet
	buf[TCP_WIN_SIZE] = 0x5; // 1400=0x578
	buf[TCP_WIN_SIZE + 1] = 0x78;
	// calculate the checksum, len=8 (start from ip.src) + TCP_HEADER_LEN_PLAIN + 4 (one option: mss)
	ck = checksum(&buf[IP_SRC_P], 8 + TCP_HEADER_LEN_PLAIN + 4, 2);
	buf[TCP_CHECKSUM_H_P] = ck >> 8;
	buf[TCP_CHECKSUM_L_P] = ck & 0xff;
	// add 4 for option mss:
	enc28j60PacketSend(IP_HEADER_LEN+TCP_HEADER_LEN_PLAIN+4+ETH_HEADER_LEN,buf);
}

// do some basic length calculations and store the result in static variables
uint16_t get_tcp_data_len(uint8_t *buf)
{
	int16_t i;
	i = (((int16_t) buf[IP_TOTLEN_H_P]) << 8) | (buf[IP_TOTLEN_L_P] & 0xff);
	i -= IP_HEADER_LEN;
	i -= (buf[TCP_HEADER_LEN_P] >> 4) * 4; // generate len in bytes;
	if (i <= 0) {
		i = 0;
	}
	return((uint16_t)i);
}

// fill a binary string of len data into the tcp packet
uint16_t fill_tcp_buffer_len(uint8_t *buf,uint16_t pos, uint8_t *data, uint16_t len)
{
	// fill in tcp data at position pos
	//
	// with no options the data starts after the checksum + 2 more bytes (urgent ptr)
	while (len) {
		buf[TCP_CHECKSUM_L_P + 3 + pos] = *data;
		pos++;
		data++;
		len--;
	}
	return (pos);
}

// fill a binary string of len data into the tcp packet
uint16_t fill_tcp_data_len(uint8_t *buf,uint16_t pos, const char *s, uint16_t len)
{
	// fill in tcp data at position pos
	//
	// with no options the data starts after the checksum + 2 more bytes (urgent ptr)
	while (len) {
		buf[TCP_CHECKSUM_L_P + 3 + pos] = *s;
		pos++;
		s++;
		len--;
	}
	return (pos);
}

// fill in tcp data at position pos. pos=0 means start of
// tcp data. Returns the position at which the string after
// this string could be filled.
uint16_t fill_tcp_data(uint8_t *buf,uint16_t pos, const char *s)
{
	return (fill_tcp_data_len(buf,pos,(char*)s,strlen(s)));
}

// Make just an ack packet with no tcp data inside
// This will modify the eth/ip/tcp header
void make_tcp_ack_from_any(uint8_t *buf,int16_t datlentoack,uint8_t addflags)
{
	uint16_t j;
	make_eth(buf);
	// fill the header:
	buf[TCP_FLAGS_P] = TCP_FLAGS_ACK_V | addflags;
	if (addflags == TCP_FLAGS_RST_V) {
		make_tcphead(buf, datlentoack, 1);
	} else {
		if (datlentoack == 0) {
			// if there is no data then we must still acknoledge one packet
			datlentoack = 1;
		}
		make_tcphead(buf, datlentoack, 1); // no options
	}
	// total length field in the IP header must be set:
	// 20 bytes IP + 20 bytes tcp (when no options)
	j = IP_HEADER_LEN + TCP_HEADER_LEN_PLAIN;
	buf[IP_TOTLEN_H_P] = j >> 8;
	buf[IP_TOTLEN_L_P] = j & 0xff;
	make_ip(buf);
	// use a low window size otherwise we have to have
	// timers and can not just react on every packet.
	buf[TCP_WIN_SIZE] = 0x4; // 1024=0x400
	buf[TCP_WIN_SIZE + 1] = 0x0;
	// calculate the checksum, len=8 (start from ip.src) + TCP_HEADER_LEN_PLAIN + data len
	j = checksum(&buf[IP_SRC_P], 8 + TCP_HEADER_LEN_PLAIN, 2);
	buf[TCP_CHECKSUM_H_P] = j >> 8;
	buf[TCP_CHECKSUM_L_P] = j & 0xff;
	enc28j60PacketSend(IP_HEADER_LEN+TCP_HEADER_LEN_PLAIN+ETH_HEADER_LEN,buf);
}

void make_tcp_ack_from_fin(uint8_t *buf)
{
	uint16_t ck;
		make_eth(buf);
		// total length field in the IP header must be set:
		// 20 bytes IP + 24 bytes (20tcp+4tcp options)
		buf[IP_TOTLEN_H_P] = 0;
		buf[IP_TOTLEN_L_P] = IP_HEADER_LEN + TCP_HEADER_LEN_PLAIN + 4;
		make_ip(buf);
		buf[TCP_FLAGS_P] = TCP_FLAGS_ACK_V ;//| TCP_FLAGS_FIN_V;
		make_tcphead(buf, 1, 0);
		// put an inital seq number
		buf[TCP_SEQ_H_P + 0] = 0;
		buf[TCP_SEQ_H_P + 1] = 0;
		// we step only the second byte, this allows us to send packts
		// with 255 bytes, 512  or 765 (step by 3) without generating
		// overlapping numbers.
		buf[TCP_SEQ_H_P + 2] = seqnum;
		buf[TCP_SEQ_H_P + 3] = 0;
		// step the inititial seq num by something we will not use
		// during this tcp session:
		seqnum += 3;
		// add an mss options field with MSS to 1280:
		// 1280 in hex is 0x500
		buf[TCP_OPTIONS_P] = 2;
		buf[TCP_OPTIONS_P + 1] = 4;
		buf[TCP_OPTIONS_P + 2] = 0x05;
		buf[TCP_OPTIONS_P + 3] = 0x0;
		// The tcp header length is only a 4 bit field (the upper 4 bits).
		// It is calculated in units of 4 bytes.
		// E.g 24 bytes: 24/4=6 => 0x60=header len field
		buf[TCP_HEADER_LEN_P] = 0x60;
		// here we must just be sure that the web browser contacting us
		// will send only one get packet
		buf[TCP_WIN_SIZE] = 0x5; // 1400=0x578
		buf[TCP_WIN_SIZE + 1] = 0x78;
		// calculate the checksum, len=8 (start from ip.src) + TCP_HEADER_LEN_PLAIN + 4 (one option: mss)
		ck = checksum(&buf[IP_SRC_P], 8 + TCP_HEADER_LEN_PLAIN + 4, 2);
		buf[TCP_CHECKSUM_H_P] = ck >> 8;
		buf[TCP_CHECKSUM_L_P] = ck & 0xff;
		// add 4 for option mss:
		enc28j60PacketSend(IP_HEADER_LEN+TCP_HEADER_LEN_PLAIN+4+ETH_HEADER_LEN,buf);
}


// dlen is the amount of tcp data (http data) we send in this packet
// You can use this function only immediately after make_tcp_ack_from_any
// This is because this function will NOT modify the eth/ip/tcp header except for
// length and checksum
// You must set TCP_FLAGS before calling this
void make_tcp_ack_with_data_noflags(uint8_t *buf,uint16_t dlen)
{
	uint16_t j;
	// total length field in the IP header must be set:
	// 20 bytes IP + 20 bytes tcp (when no options) + len of data
	j = IP_HEADER_LEN + TCP_HEADER_LEN_PLAIN + dlen;
	buf[IP_TOTLEN_H_P] = j >> 8;
	buf[IP_TOTLEN_L_P] = j & 0xff;
	fill_ip_hdr_checksum(buf);
	// zero the checksum
	buf[TCP_CHECKSUM_H_P] = 0;
	buf[TCP_CHECKSUM_L_P] = 0;
	// calculate the checksum, len=8 (start from ip.src) + TCP_HEADER_LEN_PLAIN + data len
	j = checksum(&buf[IP_SRC_P], 8 + TCP_HEADER_LEN_PLAIN + dlen, 2);
	buf[TCP_CHECKSUM_H_P] = j >> 8;
	buf[TCP_CHECKSUM_L_P] = j & 0xff;
	enc28j60PacketSend(IP_HEADER_LEN+TCP_HEADER_LEN_PLAIN+dlen+ETH_HEADER_LEN,buf);
}


// you must have called init_len_info at some time before calling this function
// dlen is the amount of tcp data (http data) we send in this packet
// You can use this function only immediately after make_tcp_ack_from_any
// This is because this function will NOT modify the eth/ip/tcp header except for
// length and checksum
// Used?
void make_tcp_ack_with_data(uint8_t *buf,uint16_t dlen)
{
	uint16_t j;
	// fill the header:
	// This code requires that we send only one data packet
	// because we keep no state information. We must therefore set
	// the fin here:
	buf[TCP_FLAGS_P] = TCP_FLAGS_ACK_V | TCP_FLAGS_PUSH_V | TCP_FLAGS_FIN_V;

	// total length field in the IP header must be set:
	// 20 bytes IP + 20 bytes tcp (when no options) + len of data
	j = IP_HEADER_LEN + TCP_HEADER_LEN_PLAIN + dlen;
	buf[IP_TOTLEN_H_P] = j >> 8;
	buf[IP_TOTLEN_L_P] = j & 0xff;
	fill_ip_hdr_checksum(buf);
	// zero the checksum
	buf[TCP_CHECKSUM_H_P] = 0;
	buf[TCP_CHECKSUM_L_P] = 0;
	// calculate the checksum, len=8 (start from ip.src) + TCP_HEADER_LEN_PLAIN + data len
	j = checksum(&buf[IP_SRC_P], 8 + TCP_HEADER_LEN_PLAIN + dlen, 2);
	buf[TCP_CHECKSUM_H_P] = j >> 8;
	buf[TCP_CHECKSUM_L_P] = j & 0xff;
	enc28j60PacketSend(IP_HEADER_LEN+TCP_HEADER_LEN_PLAIN+dlen+ETH_HEADER_LEN,buf);
}
// Send the data
void send_tcp_data(uint8_t *buf,uint16_t dlen)
{
	uint16_t j;
	// fill the header:
	// This code requires that we send only one data packet
	// because we keep no state information. We must therefore set
	// the fin here:
	buf[TCP_FLAGS_P] = TCP_FLAGS_PUSH_V;

	// total length field in the IP header must be set:
	// 20 bytes IP + 20 bytes tcp (when no options) + len of data
	j = IP_HEADER_LEN + TCP_HEADER_LEN_PLAIN + dlen;
	buf[IP_TOTLEN_H_P] = j >> 8;
	buf[IP_TOTLEN_L_P] = j & 0xff;
	fill_ip_hdr_checksum(buf);
	// zero the checksum
	buf[TCP_CHECKSUM_H_P] = 0;
	buf[TCP_CHECKSUM_L_P] = 0;
	// calculate the checksum, len=8 (start from ip.src) + TCP_HEADER_LEN_PLAIN + data len
	j = checksum(&buf[IP_SRC_P], 8 + TCP_HEADER_LEN_PLAIN + dlen, 2);
	buf[TCP_CHECKSUM_H_P] = j >> 8;
	buf[TCP_CHECKSUM_L_P] = j & 0xff;
	enc28j60PacketSend(IP_HEADER_LEN+TCP_HEADER_LEN_PLAIN+dlen+ETH_HEADER_LEN,buf);
}

void tcp_send_data(uint8_t *data, uint8_t len)
{
	fill_tcp_buffer_len(packet_buf, 0, data, len);
	packet_buf[TCP_FLAGS_P] = TCP_FLAGS_PSHACK_V;
	make_tcp_ack_with_data_noflags(packet_buf, len);
}

void tcp_analyzer(uint8_t *buf, uint16_t len)
{
	////Goi tin co buf[TCP_FLAGS_P] == SYN (data from step 1 in 3 way-handshakes)
	if (buf[TCP_FLAGS_P] & TCP_FLAGS_SYN_V)  // TCP_FLAGS_SYN_V = 0x02, buf[TCP_FLAGS_P] = buf[47] = {0x10 : ACK; 0x02: SYN; 0x18: PSH,ACK; 0x19: FIN,PSH,ACK;}
	{
		make_tcp_synack_from_syn(buf);  //Tao ACK SYN send to PC  (step 2 in 3 way-handshakes)

//			while(len < BUFFER_SIZE)
//			{
//				bufRemeber[len] = buf[len]; len++;
//			}
		return;
	}
	else if (buf[TCP_FLAGS_P] & TCP_FLAGS_PUSH_V)
	{
//		uint8_t i;
		data_len = get_tcp_data_len(buf);
		memcpy(&data_buf[0], &buf[TCP_DATA_P], data_len);
//		data_buf[data_len] = 0;
//		printf("Nhan 1 goi tcp, data: [");
//		for(i = 0; i < data_len; i ++)
//		{
//			printf(" %.2X", data_buf[i]);
//		}
//		printf(" ]\n");
		modbus_tcp_parse_frame(data_buf, data_len);
		//init_len_info(buf); // init some data structures // we can possibly have no data, just ack: dat_p=get_tcp_data_pointer(); if (dat_p==0) { if (buf[TCP_FLAGS_P] & TCP_FLAGS_FIN_V) // buf[TCP_FLAGS_P] == TCP_FLAGS_FIN_V = 0x01 --> Ko gui tiep du lieu
		//{
		if(buf[TCP_FLAGS_P] & TCP_FLAGS_ACK_V)
		{
			// finack, answer with ack
			make_tcp_ack_from_any(buf, data_len, 0);
		}
		//}
		// just an ack with no data, wait for next packet
		return;
	}
	else if(buf[TCP_FLAGS_P] & TCP_FLAGS_FIN_V)
	{
//		buf[TCP_FLAGS_P] = TCP_FLAGS_ACK_V;
//		make_tcp_ack_with_data_noflags(buf, 0);
		if(buf[TCP_FLAGS_P] & TCP_FLAGS_ACK_V)
		{
			make_tcp_ack_from_any(buf, 0, 0);
//			make_tcp_ack_from_fin(buf);
		}

	}
}

void net_analyzer(void)
{
	//-----------------------------Ethernet Analyzer------------------------------------------
		// get the next new packet:
	packet_len = enc28j60PacketReceive(512, packet_buf);
	/*plen will be unequal to zero if there is a valid packet (without crc error) */
	if(packet_len == 0)
	{
		return;
	}

	// arp is broadcast if unknown but a host may also
	// verify the mac address by sending it to
	// a unicast address.
	if (eth_type_is_arp_and_my_ip(packet_buf, packet_len))
	{
		 make_arp_answer_from_request(packet_buf);
		 return;
	}

	//check if ip packets are for us:
	//Kiem tra goi tin la IP &  IP DST chinh la IP cua STM32
	if (eth_type_is_ip_and_my_ip(packet_buf, packet_len) == 0)
	{
		 return;
	}

	//Kiem tra goi tin la ICMP ... la goi tin Ping tu thiet bi khac den... -> phan hoi Pong data toi thiet bi
	if (packet_buf[IP_PROTO_P] == IP_PROTO_ICMP_V && packet_buf[ICMP_TYPE_P] == ICMP_TYPE_ECHOREQUEST_V)
	{
		 // a ping packet, let's send pong
		 make_echo_reply_from_request(packet_buf, packet_len);
		 return;
	}

	//Goi tin la goi TCP/IP & DST Port la Port cua ST
	if (packet_buf[IP_PROTO_P]==IP_PROTO_TCP_V && packet_buf[TCP_DST_PORT_H_P]==wwwport_h && packet_buf[TCP_DST_PORT_L_P]== wwwport_l)
	{
		tcp_analyzer(packet_buf, packet_len);
	}
}
/*****************************End Of File**************************************/
