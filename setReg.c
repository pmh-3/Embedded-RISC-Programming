//Hw #1
//setting register bits 

struct port_registers {	
	uint8_t IN;   // offset 0
	uint8_t null1;
	uint8_t OUT;   // offset 2
	uint8_t null2;
	uint8_t DIR;   // offset 4
	uint8_t null3;
	uint8_t REN;   // offset 6
	uint8_t null4;
	uint8_t DS;   // offset 8
	uint8_t null5;
	uint8_t SEL0;   // offset 10
	uint8_t null6;
	uint8_t SEL1;   // offset 12
					};
	
	
// Set pointers

struct port-registers *P1, *P2, *P3, *P4, *P5;
uint8_t data;
p1 = 0x400;
data = p1 -> IN;



void SetReg(void){

	//P4.2 to be an output with extra strength 
	P4->SEL0 &= ~0x02;
	P4->SEl1 &= ~0x02;
	P4->DIR |= 0x02;
	P4->DS |= 0x02; //Increased Drive strength 
	P4->OUT &=~0x02;
	
	//P2.0 to be an input without resistor enabled
	P2->SEL0 &= ~0X00;
	P2->SEL1 &= ~0X00;
	P2->DIR &= ~0X00;
	P2->REN |= 0X00;
	
	//P6.4 to be an input with pullup resistor
	P6->SEL0 &= ~0X03;
	P6->SEL1 &= ~0X03;
	P6->DIR &= ~0X03;  
	P6->REN |= 0X03;  
	P6->OUT |= 0X03; //pullup enabled resistor to Vcc

};