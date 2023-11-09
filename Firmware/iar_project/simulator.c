#include "simulator.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "config.h"

#define SIM_OFFSET_WORDS	50

#define MAX_NOISE_VAL 50

//1 bit is one PRN chip
uint8_t prn_array[PRN_LENGTH];

//2KiB for 1 ms, filled at startup and do not change
uint16_t spi_prn_array[PRN_SPI_WORDS_CNT];
uint16_t spi_prn_array_inv[PRN_SPI_WORDS_CNT];

/// This bufffer is send to SPI using DMA
/// Contain data for two PRN periods - this is needed to swap data in realtime
uint16_t spi_tx_array[PRN_SPI_WORDS_CNT];



/// Used for noise generation, 0 is no noise, max is (MAX_NOISE_VAL-1)
/// See the code - some values are hardcoded!
uint8_t tmp_noise = 15;

//*************************************************************

void gencode_L1CA(int prn);
void sim_fill_target_buffer(uint16_t* buff_p);


//*************************************************************


/// Simlate noise
void sim_add_noise(uint16_t* buff_p, uint8_t noise_level)
{
  if (noise_level >= MAX_NOISE_VAL)
    noise_level = MAX_NOISE_VAL - 1;
  
  uint8_t noise_cnt = 0;
  for (uint16_t word_cnt = 0; word_cnt < PRN_SPI_WORDS_CNT; word_cnt++)//1000
  {
    if (noise_cnt < noise_level)
    {
      //buff_p[word_cnt] = 0x3333;//4MHz signal
      buff_p[word_cnt] = (uint16_t)(rand() & 0xFFFF);
    }
   
    noise_cnt++;
    if (noise_cnt >= MAX_NOISE_VAL)
      noise_cnt = 0;
  }
}

void gps_generate_prn_data(uint16_t* data, uint8_t prn)
{
	uint32_t total_bit_cnt = 0;
	uint16_t word_cnt = 0;

	gencode_L1CA(prn);

	// Fill two constant buffers
	float period_prn = (float)BITS_IN_PRN / (float)PRN_LENGTH;
	total_bit_cnt = 0;
	for (word_cnt = 0; word_cnt < PRN_SPI_WORDS_CNT; word_cnt++)
	{
		uint16_t value = 0;
		for (uint8_t bit_cnt = 0; bit_cnt < 16; bit_cnt++)
		{
			uint16_t prn_bit_cnt = (uint16_t)((float)total_bit_cnt / period_prn);
			uint8_t prn_bit = prn_array[prn_bit_cnt];

			if (prn_bit == 1) //not inverted
				value |= 1 << bit_cnt;

			total_bit_cnt++;
		}
		data[word_cnt] = value;
	}
}

uint16_t* sim_generate_data(void)
{
  //SPI is LSB first
  uint32_t total_bit_cnt = 0;
  uint16_t word_cnt = 0;
  
  /// Period in SPI bits
  double period = (double)SPI_BAUDRATE_HZ / ((double)IF_FREQ_HZ + 2000);
  double threshold = period / 2;
  if  (period < 0.0f)
    return spi_tx_array;
  
  for (word_cnt = 0; word_cnt < PRN_SPI_WORDS_CNT; word_cnt++)//1000
  {
    uint16_t word_value = 0;
    for (uint8_t bit_cnt = 0; bit_cnt < 16; bit_cnt++)
    {      
      word_value = word_value >> 1;
      uint32_t div1 = (uint32_t)((double)total_bit_cnt / period);
      double zone = (double)total_bit_cnt - ((double)div1 * period); //=bit_cnt % period
      if (zone < threshold)
        word_value |= 0x8000;
      total_bit_cnt++;
    }
    spi_prn_array[word_cnt] = word_value;
	//spi_prn_array[word_cnt] = 0xffff;
  }
  
  gencode_L1CA(SIM_PRN_CODE);
  

  // Fill two constant buffers
  float period_prn = (float)BITS_IN_PRN / (float)PRN_LENGTH;
  total_bit_cnt = 0;
  for (word_cnt = 0; word_cnt < PRN_SPI_WORDS_CNT; word_cnt++)
  {
    uint16_t invert_value = 0;
    for (uint8_t bit_cnt = 0; bit_cnt < 16; bit_cnt++)
    {
      uint16_t prn_bit_cnt = (uint16_t)((float)total_bit_cnt / period_prn);
      uint8_t prn_bit = prn_array[prn_bit_cnt];
      
      if (prn_bit == 0)
        invert_value |= 1 << bit_cnt;
      
      total_bit_cnt++;
    }

	uint16_t pos = word_cnt + SIM_OFFSET_WORDS;
	if (pos >= PRN_SPI_WORDS_CNT)
		pos = pos - PRN_SPI_WORDS_CNT;

    spi_prn_array[pos] ^= invert_value;
    spi_prn_array_inv[pos] = ~spi_prn_array[pos];//inverted data
  }
  
  memcpy(&spi_tx_array[0], spi_prn_array, sizeof(spi_prn_array));
  return spi_tx_array;
}


//Code is taken from https://github.com/taroz/GNSS-SDRLIB
/* C/A code (IS-GPS-200) -----------------------------------------------------*/
void gencode_L1CA(int prn)
{
  const static int16_t delay[]={ /* G2 delay (chips) */
    5,   6,   7,   8,  17,  18, 139, 140, 141, 251,   /*   1- 10 */
    252, 254, 255, 256, 257, 258, 469, 470, 471, 472,   /*  11- 20 */
    473, 474, 509, 512, 513, 514, 515, 516, 859, 860,   /*  21- 30 */
    861, 862, 863, 950, 947, 948, 950,  67, 103,  91,   /*  31- 40 */
    19, 679, 225, 625, 946, 638, 161,1001, 554, 280,   /*  41- 50 */
    710, 709, 775, 864, 558, 220, 397,  55, 898, 759,   /*  51- 60 */
    367, 299,1018, 729, 695, 780, 801, 788, 732,  34,   /*  61- 70 */
    320, 327, 389, 407, 525, 405, 221, 761, 260, 326,   /*  71- 80 */
    955, 653, 699, 422, 188, 438, 959, 539, 879, 677,   /*  81- 90 */
    586, 153, 792, 814, 446, 264,1015, 278, 536, 819,   /*  91-100 */
    156, 957, 159, 712, 885, 461, 248, 713, 126, 807,   /* 101-110 */
    279, 122, 197, 693, 632, 771, 467, 647, 203, 145,   /* 111-120 */
    175,  52,  21, 237, 235, 886, 657, 634, 762, 355,   /* 121-130 */
    1012, 176, 603, 130, 359, 595,  68, 386, 797, 456,   /* 131-140 */
    499, 883, 307, 127, 211, 121, 118, 163, 628, 853,   /* 141-150 */
    484, 289, 811, 202,1021, 463, 568, 904, 670, 230,   /* 151-160 */
    911, 684, 309, 644, 932,  12, 314, 891, 212, 185,   /* 161-170 */
    675, 503, 150, 395, 345, 846, 798, 992, 357, 995,   /* 171-180 */
    877, 112, 144, 476, 193, 109, 445, 291,  87, 399,   /* 181-190 */
    292, 901, 339, 208, 711, 189, 263, 537, 663, 942,   /* 191-200 */
    173, 900,  30, 500, 935, 556, 373,  85, 652, 310    /* 201-210 */
  };
  int8_t G1[PRN_LENGTH],G2[PRN_LENGTH],R1[10],R2[10],C1,C2;
  int i,j;
  
  if (prn < 1)
    return;
  
  for (i=0;i<10;i++) 
    R1[i]=R2[i]=-1;
  
  for (i=0; i<PRN_LENGTH; i++) 
  {
    G1[i]=R1[9];
    G2[i]=R2[9];
    C1=R1[2]*R1[9];
    C2=R2[1]*R2[2]*R2[5]*R2[7]*R2[8]*R2[9];
    for (j=9; j>0; j--) 
    {
      R1[j]=R1[j-1];
      R2[j]=R2[j-1];
    }
    R1[0]=C1;
    R2[0]=C2;
  }
  for (i=0, j=PRN_LENGTH-delay[prn-1]; i<PRN_LENGTH; i++,j++) 
  {
    short tmp_val = -G1[i]*G2[j % PRN_LENGTH];
    prn_array[i]= (tmp_val < 0) ? 0 : 1;
  }
  
  return;
}