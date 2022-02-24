#include "common.hh"
#include <hackrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "gauss_cos_sin_table.hh"
#if defined(__GNUC__)
#include <unistd.h>
#include <sys/time.h>
#endif

#include <signal.h>


#define SAMPLE_PER_SYMBOL 4

#define AMPLITUDE (127.0)
#define MOD_IDX (0.5)
#define LEN_GAUSS_FILTER (4) // pre 2, post 2
#define MAX_NUM_INFO_BYTE (43)
#define MAX_NUM_PHY_BYTE (47)
#define MAX_NUM_PHY_SAMPLE ((MAX_NUM_PHY_BYTE*8*SAMPLE_PER_SYMBOL)+(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL))

#if SAMPLE_PER_SYMBOL==4
float gauss_coef[LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL] = {7.561773e-09, 1.197935e-06, 8.050684e-05, 2.326833e-03, 2.959908e-02, 1.727474e-01, 4.999195e-01, 8.249246e-01, 9.408018e-01, 8.249246e-01, 4.999195e-01, 1.727474e-01, 2.959908e-02, 2.326833e-03, 8.050684e-05, 1.197935e-06};
#endif

uint64_t freq_hz;

volatile bool do_exit = false;

volatile int stop_tx = 1;
volatile int tx_len, tx_buffer_length, tx_valid_length;
volatile int tx_count = 0;

#define NUM_PRE_SEND_DATA (256)

#define HACKRF_ONBOARD_BUF_SIZE (32768) // in usb_bulk_buffer.h
#define HACKRF_USB_BUF_SIZE (4096) // in hackrf.c lib_device->buffer_size
char tx_zeros[HACKRF_USB_BUF_SIZE-NUM_PRE_SEND_DATA] = {0};
volatile char *tx_buf;
static hackrf_device* device = NULL;

int tx_callback(hackrf_transfer* transfer) { 
  //memcpy(transfer->buffer+NUM_PRE_SEND_DATA, (char *)(tx_buf), tx_len);
  for (int i = 0; i < 2; i++) //115
  {
    memcpy(transfer->buffer+NUM_PRE_SEND_DATA + i * tx_len, (char *)(tx_buf), tx_len);
  }
  /*int size_left = (transfer->valid_length - tx_len - NUM_PRE_SEND_DATA);
  printf("size left %d", size_left);
  printf("tx_len %d", tx_len);*/
  return(0);
}


void sigint_callback_handler(int signum)
{
	fprintf(stdout, "Caught signal %d\n", signum);
	do_exit = true;
}

inline void set_freq_by_channel_number(int channel_number) {

  if ( channel_number == 37 ) {
    freq_hz = 2402000000ull;
  } else if (channel_number == 38) {
    freq_hz = 2426000000ull;
  } else if (channel_number == 39) {
    freq_hz = 2480000000ull;
  } else if (channel_number >=0 && channel_number <= 10 ) {
    freq_hz = 2404000000ull + channel_number*2000000ull;
  } else if (channel_number >=11 && channel_number <= 36 ) {
    freq_hz = 2428000000ull + (channel_number-11)*2000000ull;
  }
}

int init_board() {
	int result = hackrf_init();
	if( result != HACKRF_SUCCESS ) {
		printf("open_board: hackrf_init() failed: (%d)\n", result);
		return(-1);
	}


    signal(SIGINT, &sigint_callback_handler);
    signal(SIGILL, &sigint_callback_handler);
    signal(SIGFPE, &sigint_callback_handler);
    signal(SIGSEGV, &sigint_callback_handler);
    signal(SIGTERM, &sigint_callback_handler);
    signal(SIGABRT, &sigint_callback_handler);

  return(0);
}

inline int open_board() {
  int result;

	result = hackrf_open(&device);
	if( result != HACKRF_SUCCESS ) {
		printf("open_board: hackrf_open() failed:(%d)\n", result);
		return(-1);
	}

  result = hackrf_set_freq(device, freq_hz);
  if( result != HACKRF_SUCCESS ) {
    printf("open_board: hackrf_set_freq() failed: (%d)\n", result);
    return(-1);
  }

  result = hackrf_set_sample_rate(device, SAMPLE_PER_SYMBOL*1000000ul);
  if( result != HACKRF_SUCCESS ) {
    printf("open_board: hackrf_set_sample_rate() failed:(%d)\n", result);
    return(-1);
  }

  /* range 0-47 step 1db */
  result = hackrf_set_txvga_gain(device, 47);
  if( result != HACKRF_SUCCESS ) {
    printf("open_board: hackrf_set_txvga_gain() failed: (%d)\n", result);
    return(-1);
  }

  return(0);
}

void exit_board() {
	if(device != NULL)
	{
		hackrf_exit();
		printf("hackrf_exit() done\n");
	}
}

inline int close_board() {
  int result;

    if(device != NULL)
    {
        result = hackrf_stop_tx(device);
        if( result != HACKRF_SUCCESS ) {
        printf("close_board: hackrf_stop_tx() failed: (%d)\n", result);
        return(-1);
        }

        result = hackrf_close(device);
        if( result != HACKRF_SUCCESS )
        {
            printf("close_board: hackrf_close() failed: (%d)\n", result);
            return(-1);
        }

        return(0);
        } else {
            return(-1);
        }
}

typedef enum
{
    INVALID_TYPE,
    IBEACON,
    ADV_IND
} PKT_TYPE;


#define MAX_NUM_CHAR_CMD (256)
char tmp_str[MAX_NUM_CHAR_CMD];
char tmp_str1[MAX_NUM_CHAR_CMD];
float tmp_phy_bit_over_sampling[MAX_NUM_PHY_SAMPLE + 2*LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL];
float tmp_phy_bit_over_sampling1[MAX_NUM_PHY_SAMPLE];
typedef struct
{
    int channel_number;
    PKT_TYPE pkt_type;

    char cmd_str[MAX_NUM_CHAR_CMD]; // hex string format command input

    int num_info_bit;
    char info_bit[MAX_NUM_PHY_BYTE*8]; // without CRC and whitening

    int num_info_byte;
    uint8_t info_byte[MAX_NUM_PHY_BYTE];

    int num_phy_bit;
    char phy_bit[MAX_NUM_PHY_BYTE*8]; // all bits which will be fed to GFSK modulator

    int num_phy_byte;
    uint8_t phy_byte[MAX_NUM_PHY_BYTE];

    int num_phy_sample;
    char phy_sample[2*MAX_NUM_PHY_SAMPLE]; // GFSK output to D/A (hackrf board)
    int8_t phy_sample1[2*MAX_NUM_PHY_SAMPLE]; // GFSK output to D/A (hackrf board)

    int space; // how many millisecond null signal shouwl be padded after this packet
} PKT_INFO;

#define MAX_NUM_PACKET (1024)
PKT_INFO packets[MAX_NUM_PACKET];

char* get_next_field(char *str_input, char *p_out, char *seperator, int size_of_p_out) {
  char *tmp_p = strstr(str_input, seperator);

  if (tmp_p == str_input){
    printf("Duplicated seperator %s!\n", seperator);
    return(NULL);
  } else if (tmp_p == NULL) {
    if (strlen(str_input) > (size_of_p_out-1) ) {
      printf("Number of input exceed output buffer!\n");
      return(NULL);
    } else {
      strcpy(p_out, str_input);
      return(str_input);
    }
  }

  if ( (tmp_p-str_input)>(size_of_p_out-1) ) {
    printf("Number of input exceed output buffer!\n");
    return(NULL);
  }

  char *p;
  for (p=str_input; p<tmp_p; p++) {
    p_out[p-str_input] = (*p);
  }
  p_out[p-str_input] = 0;

  return(tmp_p+1);
}

char* toupper_str(char *input_str, char *output_str) {
  int len_str = strlen(input_str);
  int i;

  for (i=0; i<=len_str; i++) {
    output_str[i] = toupper( input_str[i] );
  }

  return(output_str);
}

void octet_hex_to_bit(char *hex, char *bit) {
  char tmp_hex[3];

  tmp_hex[0] = hex[0];
  tmp_hex[1] = hex[1];
  tmp_hex[2] = 0;

  int n = strtol(tmp_hex, NULL, 16);

  bit[0] = 0x01&(n>>0);
  bit[1] = 0x01&(n>>1);
  bit[2] = 0x01&(n>>2);
  bit[3] = 0x01&(n>>3);
  bit[4] = 0x01&(n>>4);
  bit[5] = 0x01&(n>>5);
  bit[6] = 0x01&(n>>6);
  bit[7] = 0x01&(n>>7);
}

int bit_to_int(char *bit) {
  int n = 0;
  int i;
  for(i=0; i<8; i++) {
    n = ( (n<<1) | bit[7-i] );
  }
  return(n);
}

void int_to_bit(int n, char *bit) {
  bit[0] = 0x01&(n>>0);
  bit[1] = 0x01&(n>>1);
  bit[2] = 0x01&(n>>2);
  bit[3] = 0x01&(n>>3);
  bit[4] = 0x01&(n>>4);
  bit[5] = 0x01&(n>>5);
  bit[6] = 0x01&(n>>6);
  bit[7] = 0x01&(n>>7);
}

int convert_hex_to_bit(char *hex, char *bit){
  int num_hex_orig = strlen(hex);
  //while(hex[num_hex-1]<=32 || hex[num_hex-1]>=127) {
  //  num_hex--;
  //}
  int i, num_hex;
  num_hex = num_hex_orig;
  for(i=0; i<num_hex_orig; i++) {
    if ( !( (hex[i]>=48 && hex[i]<=57) || (hex[i]>=65 && hex[i]<=70) || (hex[i]>=97 && hex[i]<=102) ) ) //not a hex
      num_hex--;
  }

  if (num_hex%2 != 0) {
    printf("convert_hex_to_bit: Half octet is encountered! num_hex %d\n", num_hex);
    printf("%s\n", hex);
    return(-1);
  }

  int num_bit = num_hex*4;

  int j;
  for (i=0; i<num_hex; i=i+2) {
    j = i*4;
    octet_hex_to_bit(hex+i, bit+j);
  }

  return(num_bit);
}

int gen_sample_from_phy_bit(char *bit, char *sample, int num_bit) {
  int num_sample = (num_bit*SAMPLE_PER_SYMBOL)+(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL);

  int8_t *tmp_phy_bit_over_sampling_int8 = (int8_t *)tmp_phy_bit_over_sampling;
  //int16_t *tmp_phy_bit_over_sampling1_int16 = (int16_t *)tmp_phy_bit_over_sampling1;

  int i, j;

  for (i=0; i<(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL-1); i++) {
    tmp_phy_bit_over_sampling_int8[i] = 0;
  }
  for (i=(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL-1+num_bit*SAMPLE_PER_SYMBOL); i<(2*LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL-2+num_bit*SAMPLE_PER_SYMBOL); i++) {
    tmp_phy_bit_over_sampling_int8[i] = 0;
  }
  for (i=0; i<(num_bit*SAMPLE_PER_SYMBOL); i++) {
    if (i%SAMPLE_PER_SYMBOL == 0) {
      tmp_phy_bit_over_sampling_int8[i+(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL-1)] = ( bit[i/SAMPLE_PER_SYMBOL] ) * 2 - 1;
    } else {
      tmp_phy_bit_over_sampling_int8[i+(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL-1)] = 0;
    }
  }

  int16_t tmp = 0;
  sample[0] = cos_table_int8[tmp];
  sample[1] = sin_table_int8[tmp];

  int len_conv_result = num_sample - 1;
  for (i=0; i<len_conv_result; i++) {
    int16_t acc = 0;
    for (j=3; j<(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL-4); j++) {
      acc = acc + gauss_coef_int8[(LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL)-j-1]*tmp_phy_bit_over_sampling_int8[i+j];
    }

    tmp = (tmp + acc)&1023;
    sample[(i+1)*2 + 0] = cos_table_int8[tmp];
    sample[(i+1)*2 + 1] = sin_table_int8[tmp];
  }

  return(num_sample);
}

char* get_next_field_value(char *current_p, int *value_return, int *return_flag) {
// return_flag: -1 failed; 0 success; 1 success and this is the last field
  char *next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if (next_p == NULL) {
    (*return_flag) = -1;
    return(next_p);
  }

  (*value_return) = atol(tmp_str);

  if (next_p == current_p) {
    (*return_flag) = 1;
    return(next_p);
  }

  (*return_flag) = 0;
  return(next_p);
}

char* get_next_field_name(char *current_p, char *name, int *return_flag) {
// return_flag: -1 failed; 0 success; 1 success and this is the last field
  char *next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if (next_p == NULL) {
    (*return_flag) = -1;
    return(next_p);
  }
  if (strcmp(toupper_str(tmp_str, tmp_str), name) != 0) {
    (*return_flag) = -1;
    return(next_p);
  }

  if (next_p == current_p) {
    (*return_flag) = 1;
    return(next_p);
  }

  (*return_flag) = 0;
  return(next_p);
}

char* get_next_field_char(char *current_p, char *bit_return, int *num_bit_return, int stream_flip, int octet_limit, int *return_flag) {
// return_flag: -1 failed; 0 success; 1 success and this is the last field
// stream_flip: 0: normal order; 1: flip octets order in sequence
  int i;
  char *next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if (next_p == NULL) {
    (*return_flag) = -1;
    return(next_p);
  }
  int num_hex = strlen(tmp_str);
  while(tmp_str[num_hex-1]<=32 || tmp_str[num_hex-1]>=127) {
    num_hex--;
  }

  if ( num_hex>octet_limit ) {
    printf("Too many octets(char)! Maximum allowed is %d\n", octet_limit);
    (*return_flag) = -1;
    return(next_p);
  }
  if (num_hex <= 1) { // NULL data
    (*return_flag) = 0;
    (*num_bit_return) = 0;
    return(next_p);
  }

  if (stream_flip == 1) {
    for (i=0; i<num_hex; i++) {
      int_to_bit(tmp_str[num_hex-i-1], bit_return + 8*i);
    }
  } else {
    for (i=0; i<num_hex; i++) {
      int_to_bit(tmp_str[i], bit_return + 8*i);
    }
  }

  (*num_bit_return) = 8*num_hex;

  if (next_p == current_p) {
    (*return_flag) = 1;
    return(next_p);
  }

  (*return_flag) = 0;
  return(next_p);
}

char* get_next_field_bit_part_flip(char *current_p, char *bit_return, int *num_bit_return, int stream_flip, int octet_limit, int *return_flag) {
// return_flag: -1 failed; 0 success; 1 success and this is the last field
// stream_flip: 0: normal order; 1: flip octets order in sequence
  int i;
  char *next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if (next_p == NULL) {
    (*return_flag) = -1;
    return(next_p);
  }
  int num_hex = strlen(tmp_str);
   while(tmp_str[num_hex-1]<=32 || tmp_str[num_hex-1]>=127) {
     num_hex--;
   }

   if (num_hex%2 != 0) {
     printf("get_next_field_bit: Half octet is encountered! num_hex %d\n", num_hex);
     printf("%s\n", tmp_str);
     (*return_flag) = -1;
     return(next_p);
   }

  if ( num_hex>(octet_limit*2) ) {
    printf("Too many octets! Maximum allowed is %d\n", octet_limit);
    (*return_flag) = -1;
    return(next_p);
  }
  if (num_hex <= 1) { // NULL data
    (*return_flag) = 0;
    (*num_bit_return) = 0;
    return(next_p);
  }

  int num_bit_tmp;

  num_hex = 2*stream_flip;
  strcpy(tmp_str1, tmp_str);
  for (i=0; i<num_hex; i=i+2) {
    tmp_str[num_hex-i-2] = tmp_str1[i];
    tmp_str[num_hex-i-1] = tmp_str1[i+1];
  }

  num_bit_tmp = convert_hex_to_bit(tmp_str, bit_return);
  if ( num_bit_tmp == -1 ) {
    (*return_flag) = -1;
    return(next_p);
  }
  (*num_bit_return) = num_bit_tmp;

  if (next_p == current_p) {
    (*return_flag) = 1;
    return(next_p);
  }

  (*return_flag) = 0;
  return(next_p);
}

char* get_next_field_bit(char *current_p, char *bit_return, int *num_bit_return, int stream_flip, int octet_limit, int *return_flag) {
// return_flag: -1 failed; 0 success; 1 success and this is the last field
// stream_flip: 0: normal order; 1: flip octets order in sequence
  int i;
  char *next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if (next_p == NULL) {
    (*return_flag) = -1;
    return(next_p);
  }
  int num_hex = strlen(tmp_str);
   while(tmp_str[num_hex-1]<=32 || tmp_str[num_hex-1]>=127) {
     num_hex--;
   }

   if (num_hex%2 != 0) {
     printf("get_next_field_bit: Half octet is encountered! num_hex %d\n", num_hex);
     printf("%s\n", tmp_str);
     (*return_flag) = -1;
     return(next_p);
   }

  if ( num_hex>(octet_limit*2) ) {
    printf("Too many octets! Maximum allowed is %d\n", octet_limit);
    (*return_flag) = -1;
    return(next_p);
  }
  if (num_hex <= 1) { // NULL data
    (*return_flag) = 0;
    (*num_bit_return) = 0;
    return(next_p);
  }

  int num_bit_tmp;
  if (stream_flip == 1) {
     strcpy(tmp_str1, tmp_str);
    for (i=0; i<num_hex; i=i+2) {
      tmp_str[num_hex-i-2] = tmp_str1[i];
      tmp_str[num_hex-i-1] = tmp_str1[i+1];
    }
  }
  num_bit_tmp = convert_hex_to_bit(tmp_str, bit_return);
  if ( num_bit_tmp == -1 ) {
    (*return_flag) = -1;
    return(next_p);
  }
  (*num_bit_return) = num_bit_tmp;

  if (next_p == current_p) {
    (*return_flag) = 1;
    return(next_p);
  }

  (*return_flag) = 0;
  return(next_p);
}

char *get_next_field_name_value(char *input_p, char *name, int *val, int *ret_last){
// ret_last: -1 failed; 0 success; 1 success and this is the last field
  int ret;
  char *current_p = input_p;

  char *next_p = get_next_field_name(current_p, name, &ret);
  if (ret != 0) { // failed or the last
    (*ret_last) = -1;
    return(NULL);
  }

  current_p = next_p;
  next_p = get_next_field_value(current_p, val, &ret);
  (*ret_last) = ret;
  if (ret == -1) { // failed
    return(NULL);
  }

  return(next_p);
}

#define DEFAULT_SPACE_MS (200)

/**
 * Static table used for the table_driven implementation.
 *****************************************************************************/
static const uint_fast32_t crc_table[256] = {
    0x000000, 0x01b4c0, 0x036980, 0x02dd40, 0x06d300, 0x0767c0, 0x05ba80, 0x040e40,
    0x0da600, 0x0c12c0, 0x0ecf80, 0x0f7b40, 0x0b7500, 0x0ac1c0, 0x081c80, 0x09a840,
    0x1b4c00, 0x1af8c0, 0x182580, 0x199140, 0x1d9f00, 0x1c2bc0, 0x1ef680, 0x1f4240,
    0x16ea00, 0x175ec0, 0x158380, 0x143740, 0x103900, 0x118dc0, 0x135080, 0x12e440,
    0x369800, 0x372cc0, 0x35f180, 0x344540, 0x304b00, 0x31ffc0, 0x332280, 0x329640,
    0x3b3e00, 0x3a8ac0, 0x385780, 0x39e340, 0x3ded00, 0x3c59c0, 0x3e8480, 0x3f3040,
    0x2dd400, 0x2c60c0, 0x2ebd80, 0x2f0940, 0x2b0700, 0x2ab3c0, 0x286e80, 0x29da40,
    0x207200, 0x21c6c0, 0x231b80, 0x22af40, 0x26a100, 0x2715c0, 0x25c880, 0x247c40,
    0x6d3000, 0x6c84c0, 0x6e5980, 0x6fed40, 0x6be300, 0x6a57c0, 0x688a80, 0x693e40,
    0x609600, 0x6122c0, 0x63ff80, 0x624b40, 0x664500, 0x67f1c0, 0x652c80, 0x649840,
    0x767c00, 0x77c8c0, 0x751580, 0x74a140, 0x70af00, 0x711bc0, 0x73c680, 0x727240,
    0x7bda00, 0x7a6ec0, 0x78b380, 0x790740, 0x7d0900, 0x7cbdc0, 0x7e6080, 0x7fd440,
    0x5ba800, 0x5a1cc0, 0x58c180, 0x597540, 0x5d7b00, 0x5ccfc0, 0x5e1280, 0x5fa640,
    0x560e00, 0x57bac0, 0x556780, 0x54d340, 0x50dd00, 0x5169c0, 0x53b480, 0x520040,
    0x40e400, 0x4150c0, 0x438d80, 0x423940, 0x463700, 0x4783c0, 0x455e80, 0x44ea40,
    0x4d4200, 0x4cf6c0, 0x4e2b80, 0x4f9f40, 0x4b9100, 0x4a25c0, 0x48f880, 0x494c40,
    0xda6000, 0xdbd4c0, 0xd90980, 0xd8bd40, 0xdcb300, 0xdd07c0, 0xdfda80, 0xde6e40,
    0xd7c600, 0xd672c0, 0xd4af80, 0xd51b40, 0xd11500, 0xd0a1c0, 0xd27c80, 0xd3c840,
    0xc12c00, 0xc098c0, 0xc24580, 0xc3f140, 0xc7ff00, 0xc64bc0, 0xc49680, 0xc52240,
    0xcc8a00, 0xcd3ec0, 0xcfe380, 0xce5740, 0xca5900, 0xcbedc0, 0xc93080, 0xc88440,
    0xecf800, 0xed4cc0, 0xef9180, 0xee2540, 0xea2b00, 0xeb9fc0, 0xe94280, 0xe8f640,
    0xe15e00, 0xe0eac0, 0xe23780, 0xe38340, 0xe78d00, 0xe639c0, 0xe4e480, 0xe55040,
    0xf7b400, 0xf600c0, 0xf4dd80, 0xf56940, 0xf16700, 0xf0d3c0, 0xf20e80, 0xf3ba40,
    0xfa1200, 0xfba6c0, 0xf97b80, 0xf8cf40, 0xfcc100, 0xfd75c0, 0xffa880, 0xfe1c40,
    0xb75000, 0xb6e4c0, 0xb43980, 0xb58d40, 0xb18300, 0xb037c0, 0xb2ea80, 0xb35e40,
    0xbaf600, 0xbb42c0, 0xb99f80, 0xb82b40, 0xbc2500, 0xbd91c0, 0xbf4c80, 0xbef840,
    0xac1c00, 0xada8c0, 0xaf7580, 0xaec140, 0xaacf00, 0xab7bc0, 0xa9a680, 0xa81240,
    0xa1ba00, 0xa00ec0, 0xa2d380, 0xa36740, 0xa76900, 0xa6ddc0, 0xa40080, 0xa5b440,
    0x81c800, 0x807cc0, 0x82a180, 0x831540, 0x871b00, 0x86afc0, 0x847280, 0x85c640,
    0x8c6e00, 0x8ddac0, 0x8f0780, 0x8eb340, 0x8abd00, 0x8b09c0, 0x89d480, 0x886040,
    0x9a8400, 0x9b30c0, 0x99ed80, 0x985940, 0x9c5700, 0x9de3c0, 0x9f3e80, 0x9e8a40,
    0x972200, 0x9696c0, 0x944b80, 0x95ff40, 0x91f100, 0x9045c0, 0x929880, 0x932c40
};

/**
 * Update the crc value with new data.
 *
 * \param crc      The current crc value.
 * \param data     Pointer to a buffer of \a data_len bytes.
 * \param data_len Number of bytes in the \a data buffer.
 * \return         The updated crc value.
 *****************************************************************************/
uint_fast32_t crc_update(uint_fast32_t crc, const void *data, size_t data_len)
{
    const unsigned char *d = (const unsigned char *)data;
    unsigned int tbl_idx;

    while (data_len--) {
            tbl_idx = (crc ^ *d) & 0xff;
            crc = (crc_table[tbl_idx] ^ (crc >> 8)) & 0xffffff;

        d++;
    }
    return crc & 0xffffff;
}

void crc24(char *bit_in, int num_bit, char *init_hex, char *crc_result) {
  char bit_store[24], bit_store_update[24];
  int i;
  convert_hex_to_bit(init_hex, bit_store);

  for (i=0; i<num_bit; i++) {
    char new_bit = (bit_store[23]+bit_in[i])%2;
    bit_store_update[0] = new_bit;
    bit_store_update[1] = (bit_store[0]+new_bit)%2;
    bit_store_update[2] = bit_store[1];
    bit_store_update[3] = (bit_store[2]+new_bit)%2;
    bit_store_update[4] = (bit_store[3]+new_bit)%2;
    bit_store_update[5] = bit_store[4];
    bit_store_update[6] = (bit_store[5]+new_bit)%2;

    bit_store_update[7] = bit_store[6];
    bit_store_update[8] = bit_store[7];

    bit_store_update[9] = (bit_store[8]+new_bit)%2;
    bit_store_update[10] = (bit_store[9]+new_bit)%2;

    memcpy(bit_store_update+11, bit_store+10, 13);

    memcpy(bit_store, bit_store_update, 24);
  }

  for (i=0; i<24; i++) {
    crc_result[i] = bit_store[23-i];
  }
}

void scramble(char *bit_in, int num_bit, int channel_number, char *bit_out) {
  char bit_store[7], bit_store_update[7];
  int i;

  bit_store[0] = 1;
  bit_store[1] = 0x01&(channel_number>>5);
  bit_store[2] = 0x01&(channel_number>>4);
  bit_store[3] = 0x01&(channel_number>>3);
  bit_store[4] = 0x01&(channel_number>>2);
  bit_store[5] = 0x01&(channel_number>>1);
  bit_store[6] = 0x01&(channel_number>>0);

  for (i=0; i<num_bit; i++) {
    bit_out[i] = ( bit_store[6] + bit_in[i] )%2;

    bit_store_update[0] = bit_store[6];

    bit_store_update[1] = bit_store[0];
    bit_store_update[2] = bit_store[1];
    bit_store_update[3] = bit_store[2];

    bit_store_update[4] = (bit_store[3]+bit_store[6])%2;

    bit_store_update[5] = bit_store[4];
    bit_store_update[6] = bit_store[5];

    memcpy(bit_store, bit_store_update, 7);
  }
}

void fill_adv_pdu_header(PKT_TYPE pkt_type, int txadd, int rxadd, int payload_len, char *bit_out) {
  if (pkt_type == ADV_IND || pkt_type == IBEACON) {
    bit_out[3] = 0; bit_out[2] = 0; bit_out[1] = 0; bit_out[0] = 0;
  } else {
    bit_out[3] = 1; bit_out[2] = 1; bit_out[1] = 1; bit_out[0] = 1;
    printf("Warning! Reserved TYPE!\n");
  }

  bit_out[4] = 0;
  bit_out[5] = 0;

  bit_out[6] = txadd;
  bit_out[7] = rxadd;

  bit_out[8] = 0x01&(payload_len>>0);
  bit_out[9] = 0x01&(payload_len>>1);
  bit_out[10] = 0x01&(payload_len>>2);
  bit_out[11] = 0x01&(payload_len>>3);
  bit_out[12] = 0x01&(payload_len>>4);
  bit_out[13] = 0x01&(payload_len>>5);

  bit_out[14] = 0;
  bit_out[15] = 0;
}

char* get_next_field_hex(char *current_p, char *hex_return, int stream_flip, int octet_limit, int *return_flag) {
// return_flag: -1 failed; 0 success; 1 success and this is the last field
// stream_flip: 0: normal order; 1: flip octets order in sequence
  int i;
  char *next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if (next_p == NULL) {
    (*return_flag) = -1;
    return(next_p);
  }
  int num_hex = strlen(tmp_str);
  while(tmp_str[num_hex-1]<=32 || tmp_str[num_hex-1]>=127) {
    num_hex--;
  }

  if (num_hex%2 != 0) {
    printf("get_next_field_hex: Half octet is encountered! num_hex %d\n", num_hex);
    printf("%s\n", tmp_str);
    (*return_flag) = -1;
    return(next_p);
  }

  if ( num_hex>(octet_limit*2) ) {
    printf("Too many octets! Maximum allowed is %d\n", octet_limit);
    (*return_flag) = -1;
    return(next_p);
  }

  if (stream_flip == 1) {
    strcpy(tmp_str1, tmp_str);
    for (i=0; i<num_hex; i=i+2) {
      tmp_str[num_hex-i-2] = tmp_str1[i];
      tmp_str[num_hex-i-1] = tmp_str1[i+1];
    }
  }

  strcpy(hex_return, tmp_str);
  hex_return[num_hex] = 0;

  if (next_p == current_p) {
    (*return_flag) = 1;
    return(next_p);
  }

  (*return_flag) = 0;
  return(next_p);
}

char *get_next_field_name_bit(char *input_p, char *name, char *out_bit, int *num_bit, int flip_flag, int octet_limit, int *ret_last){
// ret_last: -1 failed; 0 success; 1 success and this is the last field
  int ret;
  char *current_p = input_p;

  char *next_p = get_next_field_name(current_p, name, &ret);
  if (ret != 0) { // failed or the last
    (*ret_last) = -1;
    return(NULL);
  }

  current_p = next_p;
  next_p = get_next_field_bit(current_p, out_bit, num_bit, flip_flag, octet_limit, &ret);
  (*ret_last) = ret;
  if (ret == -1) { // failed
    return(NULL);
  }

  return(next_p);
}

void disp_bit(char *bit, int num_bit)
{
  int i, bit_val;
  for(i=0; i<num_bit; i++) {
    bit_val = bit[i];
    if (i%8 == 0 && i != 0) {
      printf(" ");
    } else if (i%4 == 0 && i != 0) {
      printf("-");
    }
    printf("%d", bit_val);
  }
  printf("\n");
}

void disp_bit_in_hex(char *bit, int num_bit)
{
  int i, a;
  for(i=0; i<num_bit; i=i+8) {
    a = bit[i] + bit[i+1]*2 + bit[i+2]*4 + bit[i+3]*8 + bit[i+4]*16 + bit[i+5]*32 + bit[i+6]*64 + bit[i+7]*128;
    printf("%02x", a);
  }
  printf("\n");
}

void crc24_and_scramble_to_gen_phy_bit(char *crc_init_hex, PKT_INFO *pkt) {
  crc24(pkt->info_bit+5*8, pkt->num_info_bit-5*8, crc_init_hex, pkt->info_bit+pkt->num_info_bit);

  //printf("after crc24\n");
  //disp_bit_in_hex(pkt->info_bit, pkt->num_info_bit + 3*8);

  scramble(pkt->info_bit+5*8, pkt->num_info_bit-5*8+24, pkt->channel_number, pkt->phy_bit+5*8);
  memcpy(pkt->phy_bit, pkt->info_bit, 5*8);
  pkt->num_phy_bit = pkt->num_info_bit + 24;

  //printf("after scramble %d %d\n", pkt->num_phy_bit , pkt->num_phy_byte);
  //disp_bit_in_hex(pkt->phy_bit, pkt->num_phy_bit);
}

int calculate_sample_for_ADV_IND(char *pkt_str, PKT_INFO *pkt) {
// example
// ./btle_tx 37-ADV_IND-TxAdd-1-RxAdd-0-AdvA-010203040506-AdvData-00112233445566778899AABBCCDDEEFF
  char *current_p;
  int ret, num_bit_tmp;

  pkt->num_info_bit = 0;

// gen preamble and access address
  pkt->num_info_bit = pkt->num_info_bit + convert_hex_to_bit("AA", pkt->info_bit);
  pkt->num_info_bit = pkt->num_info_bit + convert_hex_to_bit("D6BE898E", pkt->info_bit + pkt->num_info_bit);

// get txadd and rxadd
  current_p = pkt_str;
  int txadd, rxadd;
  current_p = get_next_field_name_value(current_p, "TXADD", &txadd, &ret);
  if (ret != 0) { // failed or the last
    return(-1);
  }

  current_p = get_next_field_name_value(current_p, "RXADD", &rxadd, &ret);
  if (ret != 0) { // failed or the last
    return(-1);
  }
  pkt->num_info_bit = pkt->num_info_bit + 16; // 16 is header length

// get AdvA and AdvData
  current_p = get_next_field_name_bit(current_p, "ADVA", pkt->info_bit+pkt->num_info_bit, &num_bit_tmp, 1, 6, &ret);
  if (ret != 0) { // failed or the last
    return(-1);
  }
  pkt->num_info_bit = pkt->num_info_bit + num_bit_tmp;

  current_p = get_next_field_name_bit(current_p, "ADVDATA", pkt->info_bit+pkt->num_info_bit, &num_bit_tmp, 0, 31, &ret);
  if (ret == -1) { // failed
    return(-1);
  }
  pkt->num_info_bit = pkt->num_info_bit + num_bit_tmp;

  int payload_len = (pkt->num_info_bit/8) - 7;
  /*printf("payload_len %d\n", payload_len);
  printf("num_info_bit %d\n", pkt->num_info_bit);*/

  fill_adv_pdu_header(pkt->pkt_type, txadd, rxadd, payload_len, pkt->info_bit+5*8);

  crc24_and_scramble_to_gen_phy_bit("555555", pkt);
  //printf("num_phy_bit %d\n", pkt->num_phy_bit);

  pkt->num_phy_sample = gen_sample_from_phy_bit(pkt->phy_bit, pkt->phy_sample, pkt->num_phy_bit);
  //printf("num_phy_sample %d\n", pkt->num_phy_sample);


  //pkt->space = DEFAULT_SPACE_MS;

  return(0);
  

}

int calculate_sample_from_pkt_type(char *type_str, char *pkt_str, PKT_INFO *pkt) {
  if ( strcmp( toupper_str(type_str, tmp_str), "ADV_IND" ) == 0 ) {
    pkt->pkt_type = ADV_IND;
    //printf("pkt_type ADV_IND\n");
    if ( calculate_sample_for_ADV_IND(pkt_str, pkt) == -1 ) {
      return(-1);
    }
  } else {
    pkt->pkt_type = INVALID_TYPE;
    printf("pkt_type INVALID_TYPE\n");
    return(-1);
  }

  return(0);
}

int calculate_pkt_info( PKT_INFO *pkt ){
  char *cmd_str = pkt->cmd_str;
  char *next_p;
  int ret;

  // get channel number
  int channel_number;
  next_p = get_next_field_value(cmd_str, &channel_number, &ret);
  if (ret != 0) {
    printf("Getting channel number failed! It should be 0~39.\n");
    return(-1);
  }

  if (channel_number < 0 || channel_number > 39){
    printf("Invalid channel number is found. It should be 0~39.\n");
    return(-1);
  }

  if (channel_number == 0) {
    if (tmp_str[0] != '0' ||  tmp_str[1] != 0  ) {
      printf("Invalid channel number is found. It should be 0~39.\n");
      return(-1);
    }
  }

  pkt->channel_number = channel_number;
  //printf("channel_number %d\n", channel_number);

  // get pkt_type
  char *current_p = next_p;
  next_p = get_next_field(current_p, tmp_str, "-", MAX_NUM_CHAR_CMD);
  if ( next_p == NULL  || next_p==current_p ) {
    printf("Getting packet type failed!\n");
    return(-1);
  }

  if ( calculate_sample_from_pkt_type(tmp_str, next_p, pkt) == -1 ){
    if ( pkt->pkt_type == INVALID_TYPE ) {
      printf("Invalid packet type!\n");
    } else {
      printf("Invalid packet content for specific packet type!\n");
    }
    return(-1);
  }

  return(0);
}

void save_phy_sample(char *IQ_sample, int num_IQ_sample, char *filename)
{
  int i;

  FILE *fp = fopen(filename, "w");
  if (fp == NULL) {
    printf("save_phy_sample: fopen failed!\n");
    return;
  }

  for(i=0; i<num_IQ_sample; i++) {
    if (i%24 == 0) {
      fprintf(fp, "\n");
    }
    fprintf(fp, "%d, ", IQ_sample[i]);
  }
  fprintf(fp, "\n");

  fclose(fp);
}

int parse_input(int num_input, char* data){
    strcpy(packets[0].cmd_str, data);

    if (calculate_pkt_info( &(packets[0]) ) == -1){
      printf("failed!\n");
      return(-2);
    }
   /* printf("INFO bit:"); disp_bit_in_hex(packets[0].info_bit, packets[0].num_info_bit);
    printf(" PHY bit:"); disp_bit_in_hex(packets[0].phy_bit, packets[0].num_phy_bit);
    printf("PHY SMPL: PHY_bit_for_matlab.txt IQ_sample_for_matlab.txt IQ_sample.txt IQ_sample_byte.txt\n");
    save_phy_sample(packets[0].phy_sample, 2*packets[0].num_phy_sample, "IQ_sample.txt");
    save_phy_sample((char*)(packets[0].phy_sample1), 2*packets[0].num_phy_sample, "IQ_sample_byte.txt");*/

  return(1);
}

void int_to_char(int value, char fraction_data[])
{
    if (value > 15)
    {
      sprintf(fraction_data, "%x", value);
    }
    else
    {
      sprintf(fraction_data, "0%x", value);
    }
    return;
}


int main(int argc, char** argv) {

  /*Joystick js;
  Joystick * joystickPTR;
  js = Joystick();
  joystickPTR = &js;
  printf("output: %d\n", joystickPTR->isFound());*/

  char buffer_joystick[8];

  int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);


  char data_roll[2];
  char data_pitch[2];
  char data_throttle[2];
  char data_yaw[2];

  //int value_roll, value_pitch, value_throttle, value_yaw;
  int value_roll = 127;
  int value_pitch = 127;
  int value_throttle = 0;
  int value_yaw = 127;

  if ( init_board() == -1 )
      return(-1);
 
  int result;


  set_freq_by_channel_number(38);
  // open the board-----------------------------------------
  if (open_board() == -1) {
    printf("tx_one_buf: open_board() failed\n");
    return(-1);
  }

  result = hackrf_start_tx(device, tx_callback, NULL);
  if( result != HACKRF_SUCCESS ) {
    printf("hackrf_start_tx() failed: (%d)\n", result);
    return(-1);
  }

  while( (hackrf_is_streaming(device) == HACKRF_TRUE) &&
      (do_exit == false) )
  {

    int bytes = read(fd, buffer_joystick, sizeof(buffer_joystick));

    if(buffer_joystick[6] & 0x80)
    {
      printf("initial\n");
    }
    else if(buffer_joystick[6] & 0x01)
    {
      printf("button number: %u\n", buffer_joystick[7] );
    }
    else if(buffer_joystick[6] & 0x02)
    {
      printf("axis number: %u\n", buffer_joystick[7]);
      int value = (int)(buffer_joystick[5]) + 128;
      printf("value: %d\n",  value);
      if (buffer_joystick[7] == 0)
          value_roll = value;
      if (buffer_joystick[7] == 1)
          value_pitch = value;
      if (buffer_joystick[7] == 3)
          value_throttle = 255 - value;
      if (buffer_joystick[7] == 2)
          value_yaw = value;
    }

    printf("receiver: %d %d %d %d\n", value_roll, value_pitch, value_throttle, value_yaw);
    char data[83];
    //char data[83] = "38-ADV_IND-TxAdd-1-RxAdd-0-AdvA-685746352413-AdvData-0201060608534841524604ffffffff";

    //value_roll = 127;
    int_to_char(value_roll, data_roll);

    //value_pitch = 127;
    int_to_char(value_pitch, data_pitch);

    //value_throttle = 0;
    int_to_char(value_throttle, data_throttle);

    //value_yaw = 127;
    int_to_char(value_yaw, data_yaw);


    sprintf(data, "38-ADV_IND-TxAdd-1-RxAdd-0-AdvA-685746352413-AdvData-0201060608534841524604%s%s%s%s", data_roll,data_pitch,data_throttle,data_yaw);


    parse_input(2, data);

    tx_buf = packets[0].phy_sample;
    tx_len = 2*packets[0].num_phy_sample;

  }

  if (do_exit)
  {
    printf("\nExiting...\n");
    return(-1);
  }

  result = hackrf_stop_tx(device);
  if( result != HACKRF_SUCCESS ) {
    printf("hackrf_stop_tx() failed: (%d)\n", result);
    return(-1);
  }


  // close the board---------------------------------------
  if (close_board() == -1) {
    printf("close_board() failed\n");
    return(-1);
  }

  do_exit = false;

  return(0);
}
