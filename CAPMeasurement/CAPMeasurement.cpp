/*
 * CAPMeasurement.cpp
 *
 * Created: 10-01-2016 11:46:04
 *  Author: neutron

we can implement autoranging by detecting timer has overflown by checking Tov flag

we need to implement a uart_printFloat(float num,...)
{
use varlist and specify precision
default is 2
multiply number with 10 raised to power of precision and print them as separate numbers with a dot in between use printnum()
}
 */ 


#include <avr/io.h>
#include "../../uartlib.h"
#include <util/delay.h>
#include <avr/interrupt.h>

void adc_init();
unsigned int adc_read(unsigned char channel);
void configure_AnalogComparator();
void configure_Timer_and_ICP();
void startTimer();
void stopTimer();
void resetTimer();
uint16_t readICPdata();
uint16_t readVoltage(uint8_t channel);
void calculateCapacitance();

#define VOLTAGE_DIVIDER 1 //direct connection 
#define RC_RESISTOR_1 100000UL //100k
#define RC_RESISTOR_2 1000UL //1k
#define RC_RESISTOR_1_PIN	4 //PORTD pin 4
#define RC_RESISTOR_2_PIN	3 //PORTD pin 3
#define DIVIDE_BY_1		0x01
#define DIVIDE_BY_8		0x02
#define DIVIDE_BY_64	0x03
#define DIVIDE_BY_256	0x04
#define DIVIDE_BY_1024	0x05

#define DISCHARGE_RESISTOR_PIN		2
#define DISCHARGE_RESISTOR_VALUE	330

#define TIMER_MIN_SPEED 5
#define TIMER_MAX_SPEED 1

struct rcResistor{
	uint8_t pin;
	uint32_t value;
	};
	
const rcResistor rcResistor_10P_TO_60U = {RC_RESISTOR_1_PIN, RC_RESISTOR_1}; //theoretical range is 10p to 60u but we will calculate only upto 1u
const rcResistor rcResistor_1U_TO_6M = {RC_RESISTOR_2_PIN, RC_RESISTOR_2};	//from 1u we will use this one.


float time_period=2;


int main(void)
{
	uart_init(9600);
	u_printPMEM("Board Initialized\r\n");
	adc_init();
	
	DDRB |= (1<<5);
	
	//initially set all RC_RESISTORS as INPUT LOW, so that both resistors are not active same time ever.
	DDRD &= ~((1<<RC_RESISTOR_1_PIN) | (1<<RC_RESISTOR_2_PIN));
	PORTD &= ~((1<<RC_RESISTOR_1_PIN) | (1<<RC_RESISTOR_2_PIN));
    
	while(1)
    {
		/*
		uart_printnum((adc_read(2)*1100UL*11UL)/1024UL);
		
		u_print("mV\r\n");
		_delay_ms(500);
		*/
		/*
		uart_printFloat(34.45896,3);
		u_print("\r\n");
		*/
		calculateCapacitance();
		/*
		//testing analog comparator
		if(ACSR & (1<<ACO))
			PORTB |= (1<<5);
		else PORTB &= ~(1<<5);
		*/
			
        //TODO:: Please write your application code 
    }
}

void adc_init()
{
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //clk divided by 128 for highest accuracy.
	ADMUX = (1<<REFS0)|(1<<REFS1) ; //Internal Vbg 1.1V as Reference
}

unsigned int adc_read(unsigned char channel)
{
	ADMUX |= (0b00001111 & channel); //there are some other uses of channel values beyond 0-7, read datasheet of 328p for more clarification.
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADIF)) == 0);
	ADCSRA |= (1<<ADIF);
	return ADC;
}

void configure_AnalogComparator()
{
	ADCSRA &= ~(1<<ADEN);	//disable ADC since we will be using one of ADC channels as VIN- for Comparator.
	ADMUX = 0;	//selecting channel i.e. pin A0 as VIN- substitute the principle is same as selecting ADC channel for ADC conversion
				//however since we don't need REFS I am removing the overhead required in retaining those settings.
	ADCSRB |= (1<<ACME);
	ACSR = (1<<ACIC);	//enable Comparator, don't select Vbg as VIN+, no interrupts, Connect output to ICP1.
}

void configure_Timer_and_ICP()
{
	PRR &= ~(1<<PRTIM1); //Enable Timer1 (in power reduction register PRR)
	TIFR1 |= (1<<ICF1); //clear the Input Capture Interrupt flag, which was set due to change the ICP source from pin to ACO.
	TCCR1A = 0;
	TCCR1B = 0; //clear all settings 
	TCCR1B &= ~(1<<ICES1); //Capture on Falling edge.
	TIMSK1 = (1<<ICIE1);
}

void startTimer(uint8_t val)
{
	TCCR1B &= ~(0x07);
	switch(val)
	{
		case 1:	TCCR1B |= (DIVIDE_BY_1);
		time_period = 0.0625;
		break;
		case 2:	TCCR1B |= (DIVIDE_BY_8);
		time_period = 0.5;
		break;
		case 3:	TCCR1B |= (DIVIDE_BY_64);
		time_period = 4UL;
		break;
		case 4:	TCCR1B |= (DIVIDE_BY_256);
		time_period = 16UL;
		break;
		case 0:
		case 5:	TCCR1B |= (DIVIDE_BY_1024);
		time_period = 64UL;
		break;
		default:break;
	}
	
	//TCCR1B  |= (DIVIDE_BY_256);
}

void stopTimer()
{
	TCCR1B &= ~0x07;
}

void resetTimer()
{
	TCNT1 = 0;	
}

uint16_t readICPdata()
{
	return ICR1;
}

void disCharge(uint8_t pin, uint16_t mV_to_wait_for)
{
	adc_init();
	DDRD |= (1<<pin);
	PORTD &= ~(1<<pin); //warning don't use values below 220Ohm for pull down can damage GPIO.
	while(!(readVoltage(0) <= mV_to_wait_for));
	_delay_ms(50);		//actuall this 50ms plays an important role in that even if I give to wait until 1000mV for mV_to_wait_for
						//still with in this 50ms of wait period voltage nearly reaches zero and the above statement becomes meaning less
						//for atleast 4.7uF.
						
						//only for around 100uf and higher the above wait statement would make some sense.
						//anyway we will keep it.
						
	DDRD &= ~(1<<pin);		//make the discharge resistor pin HiZ again.
}


void resetRCIO()
{
	DDRD &= ~((1<<rcResistor_10P_TO_60U.pin) | (1<<rcResistor_1U_TO_6M.pin));	//initially reset state of Both Control IO's to HIZ
	PORTD &= ~((1<<rcResistor_10P_TO_60U.pin) | (1<<rcResistor_1U_TO_6M.pin));
}

void chargeUp(rcResistor *resistor)
{
	DDRD |= (1<<resistor->pin);
	PORTD |= (1<<resistor->pin);
}

uint8_t prepare(uint8_t timerSpeed, rcResistor *resistor)
{
	stopTimer();
	
	
//	DDRD |= (1<<resistor->pin);		//set the GPIO pin as OUPUT
//	PORTD &= ~(1<<resistor->pin);
	
	resetRCIO();
	disCharge(DISCHARGE_RESISTOR_PIN,50);	//wait until voltage on CAP less than 50mV(decrease value for accuracy), channel 0 is also the VIN- of ACO
									//if we decrease the value we get more accuracy at the cost of increased time from one read to next.
	
									//another important thing is that when everytime capacitancemeas() starts it starts again with first resistor
									//to discharge , since last 1k was used to charge now the capacitancemeas() doesn't remeber, it will try to
									//discharge first using 100k, which is waste of time until, so I am thinking to not use either 1k or 100k for 
									//discharge instead we will have another IO pin with 220Ohm only for discharge and it will be used for discharging
									//irrespective of charging resistor. That way I will be able to increase the accuracy by lowering the 50mV to 10mV.
									//But the problem that I have to face is the parasitics keep increasing with increasing number of Leads on the cap
									//connection terminals. When even 1k and 100k both are connected 33pf becomes 47pf, If I remove the 1k from board
									//then it is 35pf as it used to show before. Maybe we will be able to solve it while designing the board or we will
									//have to have two different connection points for different range, In test equipment I believe they might be using
									//high quality reed relay like the one from pickering and good board design to minimize parasitics.
									//or maybe we can use some good fet for charging which might have ultra low leakage, but before that
									//see if we can minimize it just be placing resistors properly in layout.
									
									//otherwise only way would be to use two different terminals for measurement one for pf scale and
									//another for uf scale.
									
									//I have another idea for autoranging which is we go from top to bottom instead of bottom-up,
									//i.e. we first go with faster speed and try to find out the value in nf if it is likely to be 0
									//then we keep switching frequency higher until we find that nf value reached maximum or we start
									//getting pf values in which case we have to some assumptions and do the rest of calculation or
									//if we have a precalibrated set of values to compare with we can switch frequency higher
									//and similarly we will keep going down.
									
									//or we can just simply keep an option in the software to switch which ever we want and 
									//at a command we will be able to switch, any how if we are building a multimeter we will definitely
									//have switches or some interface to select what to measure so we can keep some interfacing option
									//to toggle range.
	configure_AnalogComparator();
	configure_Timer_and_ICP();
	resetTimer();
	
	//PORTD |= (1<<resistor->pin);
	chargeUp(resistor);

	startTimer(timerSpeed);
	while(!(TIFR1 & (1<<ICF1)))	//wait until ICF1 flag is set.
	{
		if(TIFR1 & (1<<TOV1))	//if timer has overflown before an Input capture has been registered
		{						//increase time period and restart function.
			//timerSpeed = (++timerSpeed)/TIMER_MIN_SPEED; //this statement doesn't work, C compiler unable to understand, so I wrote it below like this
			TIFR1 |= (1<<TOV1); //clear timer overflow flag first
			timerSpeed = (timerSpeed+1)%TIMER_MIN_SPEED;
			
			u_print("Resistor = ");
			uart_printnum(resistor->pin);
			u_print(" timer_speed = ");
			uart_printnum(timerSpeed);
			u_print("\r\n");
			
			return timerSpeed;
		}
	}
	//now if you want to print timer speed data you have to put the print statement here because this function will always returns 
	//a value of TIMER_MIN_SPEED+1 when it was successful, if not successful then it will return timer value, but in that case
	//the do{}while() loop will not let the control go any further.
	
	stopTimer();
	
	return TIMER_MIN_SPEED+1; //return a value which outside of the range of value that can be returned above, so we can find out return was from
								//Timer overflow or natural.
}

void calculateCapacitance()
{
	uint8_t timerSpeed = TIMER_MAX_SPEED;
	
	rcResistor resistor = rcResistor_10P_TO_60U;
	
	do
	{
		
		timerSpeed = prepare(timerSpeed, &resistor);
		
		if(timerSpeed==0 && resistor.pin == rcResistor_10P_TO_60U.pin)
		{
		//	DDRD &= ~(1<<rcResistor_10P_TO_60U.pin);		//NOW we know that for range of 10p to 60u timer overflowed 
		//	PORTD &= ~(1<<rcResistor_10P_TO_60U.pin);		//so we are now going to HiZ the Control IO  for 10p to 60U and
		//taken care in prepare() now
			resistor = rcResistor_1U_TO_6M;
			timerSpeed = TIMER_MAX_SPEED;				//turn the Control IO on for 1U to 6M
		}												//even at this stage if timer overflows we can do anything it will stuck here
														//hoping we are not going to connect it to a battery or anything else beyond
														//6MilliFarad.
	} while (timerSpeed != TIMER_MIN_SPEED+1);
	
	
	
	//we know at 0.7RC voltage will reach 0.5Vs
	//so 0.7RC = ourICP data
	//C = timing/0.7R
	uint16_t timing = readICPdata();
	
	//1000000000 * 0.5us = 1000 * 0.5s, i multiplied 1000000000 to get cap value in nanofarad.
	//timing * 0.5us is our actual time, since ICP data only contains number of clocks with a period of 0.5us went in.
	//100Ul * 5UL/0.7
	//10UL*5UL/7UL
	
	uint32_t capacitance = (1000000UL * time_period * timing)/(0.7 * resistor.value); 
	u_print("timing=");
	uart_printnum(timing);
	u_print("  ");
	u_print("capacitance=");
	uart_printnum(capacitance);
	u_print("pF or ");
	uart_printnum(capacitance/1000UL);
	u_print("nF or ");
	uart_printFloat((float)capacitance/1000000UL,3);
	u_print("uF\r\n");
//	u_print("timer_speed = ");
//	uart_printnum(timerSpeed);
//	u_print("\r\n");
	
}

uint16_t readVoltage(uint8_t channel)
{
	return ((adc_read(channel)*1100UL*VOLTAGE_DIVIDER)/1024UL);
}

ISR(TIMER1_CAPT_vect)
{
	
}

/*
timer_speed = 4timing=42879  capacitance=9800914pF or 9800nF or 9.800uF
timer_speed = 4timing=42873  capacitance=9799543pF or 9799nF or 9.799uFtimer_speed = 4timing=21296  capacitance=19470630pF or 19470nF or 19.470uF
timer_speed = 0timing=21294  capacitance=19468800pF or 19468nF or 19.468uF
timer_speed = 0timing=21291  capacitance=19466056pF or 19466nF or 19.466uF
timer_speed = 0

with this data i found that it cannot calculate beyond 60uf using a 100k resistor
we will need to need to decrease resistor value for that.

Anyway 100k is pretty good for pf scale since i was able to quite accurately measure 22pf and 33pf
and i found for the first time that actually manufacturing process variation by identify same capacitors
from a pack lets say for 270pf out of 5 three actually where close to 270pf one was 280pf and one was 247pf

i found similar variation in many cap values, first i thought it was my circuit parasitic capacitance problem
so i placed them in various ways and inserted in different holes of breadboard always same result, then
i thought maybe my program gives wrong result, but i understood that it is manufacturing process variation or ageing 
effect when i found that from one pack like 270pf i got many values as well as the right ones. I also found that
22pf was showing 21pf and 33pf was showing 31pf so there shouldn't be much error for higher values.


Ok so i thought if i can try this trick which is connect a 1k from another gpio and keep it hiZ state when a speed 0/5 timer
overflows then we make 100k gpio to hiz state and start using 1k and also change the value in the equation accordingly, now
this should theoretically gives us a range of 6000UF.

Next i would like to do some small changes in program and get this same setup to measure inductance
the concept is same here the voltage rises there the voltage will drop and some change in calculation formula.

The calculation formula stays same there once the supply is connected initially the voltage across the inductor will be same as supply
then with in (0.7L/R) of time constant voltage will drop to half of the supply voltage so what change in software we have to do is that
we have to change the edge detection for ICR. Thats all and some changes in the function which initially setup the edge for ICR.
*/

/*
Default Pins for VIN+ and VIN- are PD6 and PD7 respectively.

We have to connect the reference to VIN+ of Analog Comparator.

VCC
+
R1
++++++AIN0
R2
+
GND

R1=R2 = 10k or 1k so that we can make sure the reference voltage is exactly what I wanted
like if I am using 0.7RC then reference should be 0.5Vs and if I am using 5RC then 0.99Vs.

For 0.99Vs the resistors to be used should be R1=1k and R2=100k.

Circuit for CAP will be

GPIO +++++ Rmeas+++++++ VIN-/ADC_PIN
				+
				+
			CAPACITOR(dut)
				+
				+
				+
				+
				GND
				
ADC will be using Internal 1.1V reference. So I am now counting on the accuracy of the Vbg 1.1v
and also the accuracy of the resistors. Because either I can try to measure the accuracy of supply
voltage or if I am sure that Supply is accurate I can measure the reference of VIN- of comparator
is accurately divided by the resistors. I can't measure the accuracy of both of them. Because in
an equation we can find two unknowns, only one is possible to find, if we know other one.

So I thought for now lets get away with the accuracy measurements and focus on concept.

And the operation sequence will be first we will use our Rmeas and short the capacitor to ground
by pulling the GPIO LOW and at that time we will start measuring voltage across CAP using ADC pin, 
and once it reaches quite close to 0(zero) we will start the measurement function.

Measurement function will set first the ADC pin as analog comparator's VIN- and make the Analog 
comparator ready for measurement, set all the interrupts properly and make the timer ready for
running.

Then we will set the GPIO to high and next instruction we will start the timer.

We wait for the Comparator output falling edge(output going low), since initially the VIN+ will be connected
to Vref (which is 0.5Vs) and which will initially higher than voltage across CAP at time T=0.

Once the Voltage across CAP reaches(or goes beyond Vref) we will have a Comparator Interrupt which will
trigger the INPUT CAPTURE and we will save the value from Timer_1 ICP1 register and take the Timer Time
period and start the calculation and print the CAPACITANCE measured value on UART.

For more accurate relation between Vs and Vref we can make the Higher end of the Voltage divider connect
to one of GPIO pins so that the difference between Vs and Voh is doesn't affect Vref. But some pins at
some instant may have a different Voh. So I thought of another thing but I am not sure it will work
that is we can connect the High end of Vref Voltage divider to the GPIO controlling the CAP but, there
is one problem with this because when the measurement is started the Comparator wants that the VIN+ already
connected to Vref, but at time T=0, now Vref will be zero, so we may need to initialize Comparator after
a few cycles but that will also loose accuracy, So we will compromise anyway at some point with accuracy.

*/