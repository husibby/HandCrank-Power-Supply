#include <TinyLiquidCrystal_I2C.h>
#include <avr/sleep.h>

// MACROS:
#define CHANGE_PIN PB3
#define COUNT_PIN PB1
#define AREAD_PIN PB4

// GLOBALS:
double vcc, pout,vout, iout;
int muxAddress;

TinyLiquidCrystal_I2C lcd(0x27, 16, 2);

// Inactivity protocol:
void deactivateReadings()
{
	lcd.clear();
	lcd.noBacklight();  // disable backlight to lower current consumption

  GIMSK |= (1 << PCIE);  // enable pin change interrupts
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // enter power down mode
	sleep_enable();
  sleep_cpu();  // enter sleep
  
  // ISR will wake up MCU from pin change PB3

  sleep_disable();
	GIMSK &= ~(1 << PCIE);  // disable pin change interrupts

	lcd.backlight();
	displayConfigSet();
	updateVCC(vcc);
}


ISR (PCINT0_vect)
{
  // wake up from Power Down mode by triggering this interrupt
}

// Supply voltage calculation:
double getVcc()
{
	uint16_t vbgAD = 0;
	ADMUX = 0x0C;  // Select bandgap voltage (1.1 V) to read in reference to Vcc (right-aligned)
	for (int i=0; i<5; i++)  // take 5 conversions and record the last for stabilization
	{
		ADCSRA |= (1 << ADSC);  // begin single conversion
		while (ADCSRA & (1 << ADSC)) {
		}  // wait for conversion to complete
		vbgAD = ADCL | (ADCH << 8);  // read right aligned 10-bit adc value
	}

	return (1.1/(vbgAD))*1024.0;  // calculate vcc and return
}

// External voltage data retrieval:
double getData()
{
	uint16_t sumVData = 0;
	ADMUX = 0x02;  // set read channel to A2 (right-aligned)
	for (int i=0; i<5; i++)
	{
		ADCSRA |= (1 << ADSC);  // begin single conversion
		while (ADCSRA & (1 << ADSC)) {
		}  // wait for conversion to complete
		uint16_t vData = ADCL | (ADCH << 8);  // read right aligned 10-bit value
		sumVData += vData;  // accumulate all conversion values
	}

	return (vcc/1024.0)*(sumVData/5.0);  // convert to voltage reading using vcc and avg'd data
}

// Text formatting:
void displayConfigSet()
{
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.setCursor(0, 1);
  lcd.print("I:");
  lcd.setCursor(8, 0);
  lcd.print("P:");
	lcd.setCursor(8, 1);
	lcd.print("VCC:");
}

// LCD data display:
void displayData()
{
	// Display vcc calculation, vout calculation, and iout calculation for debugging
	lcd.setCursor(0,0);
	lcd.print(vcc, 3);

	lcd.setCursor(0,1);
	lcd.print(vout, 3);

	lcd.setCursor(10, 1);
	lcd.print(iout, 3);

	delay(250);
	lcd.clear();
	
}

// Cycle through external mux by clocking an addressing counter:
uint8_t incMux()
{
  bitWrite(PORTB, COUNT_PIN, 1);
	delay(1);  // wait 1 ms
	bitWrite(PORTB, COUNT_PIN, 0);  // increment MUX
  return (muxAddress+1)%4;  // keep track of mux address (0-3)
}

void updateVoltage(double v)
{
  lcd.setCursor(2,0);  // clear previous value **NOTE: THIS WILL LOOK JITTERY
  lcd.print("      ");

  lcd.setCursor(2,0);
  lcd.print(v, 2);
}

void updateCurrent(double i)
{
  lcd.setCursor(2, 1);  // clear previous value
  lcd.print("      ");
  lcd.setCursor(2, 1);
  if (i >= 1)  // if greater than 1 amp is detected, display overload
  {
    lcd.print("OL!");
    return;
  }
  lcd.print(i, 3);
}

void updatePower(double p)
{
  lcd.setCursor(10, 0);
  lcd.print("   ");
  lcd.setCursor(10, 0);
  lcd.print(p, 2);
}

void updateVCC(double vc)
{
  lcd.setCursor(12, 1);
  lcd.print("   ");
  lcd.setCursor(12, 1);
  lcd.print(vc, 2);
}

void setup()
{
	sei();  // Ensure interrupts will be listened to (might not matter since i2c will likely activate it on its own)

	// Disable Analog Comparator for reduced power consumption in active mode
	ACSR &= ~(1 << ACD);

	// ADC Setup
	ADCSRA = 0x87;  // enable control register and set sampling freq. to 62.5 kHz
	DIDR0 = 0x10;  // disable digital input buffer for ADC pin

  // Pin Change interrupt setup for PB3
	PCMSK |= (1 << PCINT3);

	pinMode(CHANGE_PIN, INPUT);  // pin change interrupt pin
	pinMode(AREAD_PIN, INPUT);  // adc read pin
	pinMode(COUNT_PIN, OUTPUT);  // increment mux select bits with a pulse

  bitWrite(PORTB, COUNT_PIN, 0);  // ensure pin tied low by default

	lcd.init();
 	lcd.setBacklight(100);
	lcd.clear();
	displayConfigSet();  // display default text labels on screen
}


void loop()
{
	static uint8_t vccUpdateCount = 0;  // start a counter that will check the battery's vcc every 255 loop iterations

	if (vccUpdateCount == 0)
	{
		vcc = getVcc();
	}
	
	// Check status of comparator pin
	if (bitRead(PINB, CHANGE_PIN) == 0)  // CHANGE_PIN is 0, indicating output voltage is low
  {
		deactivateReadings();  // this will essentially pause all operations by entering power-down mode until a voltage output is read again
  }

	// Voltage:
	vout = getData();
	vout *= 11;

  muxAddress = incMux();  // increment MUX

	// Current A:
	iout = getData();
	iout /= 25.0;   // divide gain

	muxAddress = incMux();

	// Current B:
	if (iout > 0.1)  // if original reading was capped out (above 100 mA)
	{
	iout = getData();  // retry with the higher range amplifier
	iout /= 3.36;  // divide gain
	}

	muxAddress = incMux();

	// insert RPM readings here

	muxAddress = incMux();

	pout = vout*iout;

	updateVoltage(vout);
  updateCurrent(iout);
  updatePower(pout);
	delay(250);

	vccUpdateCount++;
}