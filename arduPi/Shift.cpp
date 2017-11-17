uint8_t shiftIn(uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order){
	uint8_t value = 0 ;
	int8_t  i ;

	if (order == BCM2835_SPI_BIT_ORDER_MSBFIRST )
		for (i = 7 ; i >= 0 ; --i){
			digitalWrite (cPin, HIGH);
			value |= digitalRead (dPin) << i;
			digitalWrite (cPin, LOW);
		}
	else
		for (i = 0 ; i < 8 ; ++i){
		  digitalWrite (cPin, HIGH);
		  value |= digitalRead (dPin) << i;
		  digitalWrite (cPin, LOW);
		}

	return value;
}

void shiftOut(uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order, uint8_t val){
	int8_t i;

	if (order == BCM2835_SPI_BIT_ORDER_MSBFIRST )
		for (i = 7 ; i >= 0 ; --i){	
			digitalWrite (dPin, val & (1 << i)) ;
			digitalWrite (cPin, HIGH) ;
			digitalWrite (cPin, LOW) ;
		}
	else
		for (i = 0 ; i < 8 ; ++i){
			digitalWrite (dPin, val & (1 << i)) ;
			digitalWrite (cPin, HIGH) ;
			digitalWrite (cPin, LOW) ;
		}
}
