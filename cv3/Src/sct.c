/*
 * sct.c
 *
 *  Created on: 7. 10. 2021
 *      Author: student
 */

#include <stdint.h>
#include "stm32f0xx.h"
#include "sct.h"

/*  Inicializace HW (povoleni hodin pro port B a nastaveni vystupu  */
void sct_init(void){

	/*
	 * 	SDI	->	PB4
	 * 	CLK	->	PB3
	 * 	LA	->	PB5
	 * 	OE	->	PB10
	 */

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER10_0;
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
}

/*  Fce plnici shift registr 32 bit slovem  */
void sct_led(uint32_t value){
	for( uint8_t i = 0; i < 32; i++ ){
		sct_sdi(value & 1);
		sct_clk(1);
		sct_clk(0);
		value >>= 1;
	}
	sct_nla(1);
	sct_nla(0);
}

/*  Fce pro preklad cislovek na segmenty  */
void sct_value(uint16_t value){
	uint32_t reg = 0;

	/*  Preklad cislovek na jednotlive segmenty  */
	static const uint32_t reg_values[3][10] = {
			{
					//PCDE--------GFAB @ DIS1
					0b0111000000000111 << 16,
					0b0100000000000001 << 16,
					0b0011000000001011 << 16,
					0b0110000000001011 << 16,
					0b0100000000001101 << 16,
					0b0110000000001110 << 16,
					0b0111000000001110 << 16,
					0b0100000000000011 << 16,
					0b0111000000001111 << 16,
					0b0110000000001111 << 16,
			},
			{
					//----PCDEGFAB---- @ DIS2
					0b0000011101110000 << 0,
					0b0000010000010000 << 0,
					0b0000001110110000 << 0,
					0b0000011010110000 << 0,
					0b0000010011010000 << 0,
					0b0000011011100000 << 0,
					0b0000011111100000 << 0,
					0b0000010000110000 << 0,
					0b0000011111110000 << 0,
					0b0000011011110000 << 0,
			},
			{
					//PCDE--------GFAB @ DIS3
					0b0111000000000111 << 0,
					0b0100000000000001 << 0,
					0b0011000000001011 << 0,
					0b0110000000001011 << 0,
					0b0100000000001101 << 0,
					0b0110000000001110 << 0,
					0b0111000000001110 << 0,
					0b0100000000000011 << 0,
					0b0111000000001111 << 0,
					0b0110000000001111 << 0,
			},
	};

	reg |= reg_values[0][value / 100 % 10];  // zjisteni stovek v cisle
	reg |= reg_values[1][value / 10 % 10];   // zjisteni desitek v cisle
	reg |= reg_values[2][value % 10];        // zjisteni jednotek

	sct_led(reg);  // volani fce pro zobrazeni hodnoty

}
