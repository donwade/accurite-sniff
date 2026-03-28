#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <avr/eeprom.h>

#include <M5Unified.h>
#include <M5GFX.h>
#include "DisplayPage.hpp"

#define LINE Serial.printf("%s:%d %s\n", __FILE__,__LINE__,__FUNCTION__)

#define FG_DONE 	  "\033[0m" 
#define FG_RED        "\033[0;31m" 
#define FG_GREEN      "\033[0;32m" 
#define FG_YELLOW     "\033[0;33m" 
#define FG_BLUE       "\033[0;34m" 
#define FG_MAGENTA    "\033[0;35m" 
#define FG_CYAN       "\033[0;36m" 
#define FG_WHITE      "\033[0;37m" 

//------------------------------------------------------------------
void RadioSetup();
// include the library
#include <RadioLib.h>

uint16_t soundBeep = 0;

// SX1276 has the following connections:
#define NSS     27
#define DIO0    -1
#define REESET  -1
#define DIO1    -1

SX1276 radio = new Module(NSS,      /*NSS*/
                          DIO0,     //2     /*DIO0*/
                          REESET,   /*RESET*/
                          DIO1      /*DIO1*/
                          );

const int DI02 = 25;
//------------------------------------------------------------------
#define BUCKET_SIZE 20
uint16_t bucketHi[BUCKET_SIZE];
uint16_t bucketLo[BUCKET_SIZE];
bool bStopRecording = false;

#define LIM_LO 100
#define LIM_HI 900


#define EEMEM

// === Receiver Code Constants and Variables ===

#define MAXBITS      65

// Wind directions lookup
const float winddirections[] = { 315.0, 247.5, 292.5, 270.0,
                                 337.5, 225.0, 0.0,	  202.5,
                                 67.5,	135.0, 90.0,  112.5,
                                 45.0,	157.5, 22.5,  180.0 };

// Message types
#define MT_WS_WD_RF  49
#define MT_WS_T_RH   56

// Variables for decoding
volatile unsigned int pulsecnt = 0;
volatile unsigned long lastTime = 0;
volatile unsigned int syncpulses = 0;
volatile byte state = 0;
volatile byte buf[8] = { 0 };
volatile bool bucketFull = false;

#define RESET 0
#define INSYNC 1
#define SYNCDONE 2

// EEPROM persistence (unused in this example, can keep if you want)
unsigned int raincounter = 0;
unsigned int EEMEM raincounter_persist;
#define MARKER 0x5AA5
unsigned int EEMEM eeprom_marker = MARKER;

// === Display ===

// === Variables to hold latest decoded data ===
float latestWindspeed = -1;         // km/h
float latestWindDirection = -1;     // degrees
float latestTemperature = -1000;    // sentinel invalid temp (C)

// Timing for LCD updates
unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 1000; // ms
// === Helper Functions ===
String degreesToCompass(float degrees)
{
    // 8 point compass
    const char *directions[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    int index = (int)((degrees + 22.5) / 45) % 8;

    return String(directions[index]);
}


bool acurite_crc(volatile byte row[], int cols)
{
    cols -= 1; // last byte is CRC
    int sum = 0;

    for (int i = 0; i < cols; i++)
        sum += row[i];

    return sum != 0 && sum % 256 == row[cols];
}


float getTempF(byte hibyte, byte lobyte)
{
    int highbits = (hibyte & 0x0F) << 7;
    int lowbits = lobyte & 0x7F;
    int rawtemp = highbits | lowbits;
    float temp = (rawtemp - 400) / 10.0;

    return temp;
}


float getWindSpeed(byte hibyte, byte lobyte)
{
    int highbits = (hibyte & 0x7F) << 3;
    int lowbits = (lobyte & 0x7F) >> 4;
    float speed = highbits | lowbits;

    if (speed > 0)
        speed = speed * 0.23 + 0.28;

    float kph = speed * 60 * 60 / 1000;
    return kph;
}


float getWindDirection(byte b)
{
    int direction = b & 0x0F;

    return winddirections[direction];
}


float kphToKnots(float kph)
{
    return kph * 0.539957;
}


// === ISR prototype ===
void My_ISR();

volatile uint32_t isrCtr = 0;

// === Setup ===
void setup()
{
    Serial.begin(115200);

    M5.begin();

	auto cfg = M5.config();
	
	// Set the items you want to configure. Omit the following two lines if you use the default settings.
	cfg.serial_baudrate = 115200;
	cfg.output_power = true;
	
	M5.begin(cfg);

    
    M5.Lcd.init();
    M5.Lcd.clear();
    M5.Lcd.setCursor(3, 0);

    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextWrap(false);

    lprintf("WeatherSys");
    M5.Lcd.setCursor(5, 1);
    lprintf("Starting");
    delay(1500);
    M5.Lcd.clear();

    pinMode(DI02, INPUT);
    attachInterrupt(digitalPinToInterrupt(DI02), My_ISR, CHANGE);

    RadioSetup();
    findFloor();

    uint8_t pin;
    uint8_t cnt = 100;
    uint32_t now = millis();
    Serial.println("waiting for DIO2 to move");

    while (cnt)
    {
        pin = digitalRead(DI02);
        while (pin == digitalRead(DI02));
        cnt--;
    }
    Serial.printf("pass: DI02 %d samples in %d ms\n", isrCtr, millis() - now);  

}


#define HISTLEN 1024
uint16_t rssiIndex = 0;
int8_t rssiHistory[HISTLEN];

float rssiHi = -999;
float rssiLo = 0;


// === Main Loop ===
void loop()
{
	yield();
#if 0
	// NO NO NO 
    float rssiNow = radio.getRSSI();
	
    if (rssiHi < rssiNow)
        rssiHi = rssiNow;

    if (rssiLo > rssiNow)
        rssiLo = rssiNow;

    rssiHistory[rssiIndex++] = rssiNow;

    if (rssiIndex == HISTLEN)
    {
        rssiIndex = 0;
        Serial.printf("rssi Hi = %6.1f Lo = %6.1f Delta = %5.1f\n", rssiHi, rssiLo, rssiHi - rssiLo);

		rssiHi = -999;
		rssiLo = 0;
    }
#endif

	static uint32_t ok;
	if (100000 < ok++) 
	{
		ok = 0;
		Serial.printf("%d vs %d \n", pulsecnt, isrCtr);
		report("STATS");
	}

	if (soundBeep)
	{
		soundBeep = 0;
		M5.Speaker.tone(soundBeep, 100);
	}
	
    if (bucketFull)
    {
        //noInterrupts();
        bool valid = acurite_crc(buf, sizeof(buf));
        //interrupts();

        if (valid)
        {

        	soundBeep = 500;
            int msgtype = (buf[2] & 0x3F);

            // Decode wind speed
            float ws = getWindSpeed(buf[3], buf[4]);

            if (ws > 0)
                latestWindspeed = ws;

            if (msgtype == MT_WS_WD_RF)
            {
                float wd = getWindDirection(buf[4]);
                latestWindDirection = wd;
            }
            else if (msgtype == MT_WS_T_RH)
            {
                float tf = getTempF(buf[4], buf[5]);
                float tc = (tf - 32) / 1.8; // convert F to C
                latestTemperature = tc;
            }

            bucketFull = false;
        }
    }

    unsigned long now = millis();

    if (now - lastLcdUpdate > lcdUpdateInterval)
    {
        lastLcdUpdate = now;

        Home();

        if (latestWindspeed >= 0)
        {
            lprintf("%d km/h",latestWindspeed);
            //float knots = kphToKnots(latestWindspeed);
        }
        else
        {
            lprintf("--.-km/h");
        }

        // Line 2: wind direction degrees + cardinal + temperature if available

        if (latestWindDirection >= 0)
        {
            lprintf("%d %c %s", (int)latestWindDirection, 223, degreesToCompass(latestWindDirection));
        }
        else
        {
            lprintf("No Wind Dir");
        }

        if (latestTemperature > -100 && latestTemperature < 100)
        {
            lprintf("Temp:%4d %c C", latestTemperature, 223);
        }
        else
        {
            lprintf("Temp: ??? %c C", 223);
        }
    }
}

//==============================================================
void report(char *msg)
{
	Serial.printf(FG_YELLOW "\n%s -----\n", msg);
	for(int i = 1; i < BUCKET_SIZE-1; i++)
	{
		//reverse map.
		// show values of buckets.
		uint16_t undo = map(i, 1, BUCKET_SIZE-2, LIM_LO, LIM_HI);
		Serial.printf(" %03d  ", undo);
	}
	
	Serial.println(FG_RED);

	for(int i = 1; i < BUCKET_SIZE-1; i++)
	{
		//reverse map.
		// show values.
		Serial.printf("%05d ", bucketHi[i]);
		if (bucketHi[i] > 99990) bStopRecording = true;
	}

	Serial.println(FG_GREEN);

	for(int i = 1; i < BUCKET_SIZE-1; i++)
	{
		//reverse map.
		// show values.
		Serial.printf("%05d ", bucketLo[i]);
		if (bucketLo[i] > 99990) bStopRecording = true;
	}
	Serial.println(FG_DONE);
	Serial.println();
}


// === ISR ===
void My_ISR()
{
    unsigned long now = micros();

	bool pinState = digitalRead(DI02);
    unsigned long duration = now - lastTime;
	lastTime = now;
    
    isrCtr++;

	//--------------
	if (!bStopRecording)
	{
		if (duration >= LIM_LO && duration <= LIM_HI)
		{
			uint16_t idx;
			idx = map(duration, LIM_LO, LIM_HI, 1, BUCKET_SIZE-2);
			if (!pinState)
			{
				bucketHi[idx]++;
			}
			else
			{
				bucketLo[idx]++;
			}
		}
		else
		{
			// out of bounds.
			if (!pinState)
			{
				if (duration > LIM_HI) bucketHi[BUCKET_SIZE-1]++;
				if (duration < LIM_LO) bucketHi[0]++;
			}
			else
			{
				if (duration > LIM_HI) bucketLo[BUCKET_SIZE-1]++;
				if (duration < LIM_LO) bucketLo[0]++;
			}
		}
	}
	//--------------


    if (pinState == HIGH)  // just went hi, so time represents lo time.
    {
        if (now - lastTime > 10000)
        {
            state = RESET;
            syncpulses = 0;
            pulsecnt = 0;
        }

        return;
    }
	

	

    if (state == RESET || state == INSYNC)
    {
        if (duration > 575 && duration < 675)
        {
            state = INSYNC;
            syncpulses++;

            if (syncpulses > 3)
            {
                state = SYNCDONE;
                syncpulses = 0;
                pulsecnt = 0;
            }

            return;
        }
        else
        {
            syncpulses = 0;
            pulsecnt = 0;
            state = RESET;
            return;
        }
    }
    else
    {
        if (pulsecnt > MAXBITS)
        {
            state = RESET;
            pulsecnt = 0;
            
            bucketFull = true;
            return;
        }

        byte bytepos = pulsecnt / 8;
        byte bitpos = 7 - (pulsecnt % 8);

        if (duration > 375 && duration < 450)
        {
            bitSet(buf[bytepos], bitpos);
            pulsecnt++;
        }
        else if (duration > 175 && duration < 250)
        {
            bitClear(buf[bytepos], bitpos);
            pulsecnt++;
        }
    }
}


//----------------------------------------------------
#define OOK_FIXED_THRESHOLD 15
#define RADIOLIB_STATE(STATEVAR, FUNCTION)                              \
        {                                                                     \
            if ((STATEVAR) == RADIOLIB_ERR_NONE) {                              \
                Serial.printf(" " FUNCTION " - success!\n");     \
            } else {                                                            \
                Serial.printf(" " FUNCTION " failed, code: %d\n", \
                              STATEVAR);                                            \
                while (true)                                                      \
                ;                                                               \
            }                                                                   \
        }

//----------------------------------------------------
void findFloor(void)
{
	uint32_t intCtrMax = 0;
	uint8_t  squelchMax = 0;

 	uint8_t  squelchFirst = 0;

 	uint8_t  squelchLast = 0;

	
	state = radio.setFrequency(438.0);
    RADIOLIB_STATE(state, "setFrequency");

	state = radio.setOokThresholdType(RADIOLIB_SX127X_OOK_THRESH_FIXED);
	RADIOLIB_STATE(state, "setOokThresholdType");

	for (uint8_t squelch = 30; squelch < 0xff; squelch++)
	{
		state = radio.setOokFixedOrFloorThreshold(squelch); 
		
		Serial.printf("squelch %d " , squelch);

		uint32_t now = millis();
		isrCtr = 0;
		
		while (millis() < now + 1000)
		{
		}

		if(isrCtr && squelchFirst == 0) squelchFirst = squelch;
		if(isrCtr) 
		{
			squelchLast = squelch;
			M5.Speaker.tone(1000, 100);
		}
		
		if (isrCtr > intCtrMax) 
		{
			intCtrMax = isrCtr;
			squelchMax = squelch;
		}

		
		Serial.printf("%6d %d < %d < %d\n", squelch, squelchFirst, squelchMax, squelchLast);
		//Serial.printf("isrCtr=%d squelchMax=%d\n", isrCtr, squelchMax);

		if (isrCtr == 0 && intCtrMax != 0) break;  //done
				
	}

	squelchLast++;
	Serial.printf("\n %d < %d < %d\n", squelchFirst, squelchMax, squelchLast);
	
	// back to real channel.
	state = radio.setFrequency(434.);
    RADIOLIB_STATE(state, "setFrequency");

	
    
	state = radio.setOokFixedOrFloorThreshold(squelchLast); 
    RADIOLIB_STATE(state, "setOokFixedOrFloorThreshold");
}
//-------------------------------------------------------------

uint8_t OokFixedThreshold = OOK_FIXED_THRESHOLD;
void RadioSetup()
{
	// power off for reset
	M5.Power.setExtPower(false); // TIP
	delay(1000);
	M5.Power.setExtPower(true);  // TIP
	

    // initialize SX1278 with FSK modem at 9600 bps
    Serial.print(F("[SX1278] Initializing ... "));

    /*!
     *    \brief FSK modem initialization method. Must be called at least once from Arduino sketch to initialize the module.
     *    \param freq Carrier frequency in MHz. Allowed values range from 137.0 MHz to 1020.0 MHz.
     *    \param br Bit rate of the FSK transmission in kbps (kilobits per second). Allowed values range from 1.2 to 300.0 kbps.
     *    \param freqDev Frequency deviation of the FSK transmission in kHz. Allowed values range from 0.6 to 200.0 kHz.
     *    Note that the allowed range changes based on bit rate setting, so that the condition FreqDev + BitRate/2 <= 250 kHz is always met.
     *    \param rxBw Receiver bandwidth in kHz. Allowed values are 2.6, 3.1, 3.9, 5.2, 6.3, 7.8, 10.4, 12.5, 15.6, 20.8, 25, 31.3, 41.7, 50, 62.5, 83.3, 100, 125, 166.7, 200 and 250 kHz.
     *    \param power Transmission output power in dBm. Allowed values range from 2 to 17 dBm.
     *    \param preambleLength Length of FSK preamble in bits.
     *    \param enableOOK Use OOK modulation instead of FSK.
     *    \returns \ref status_codes
     */

    int state = radio.beginFSK(434.0,           // freq
                               4.8,             // bitrate
                               20.0,            // fsk dev
                               250.0,           // rxbw khz
                               2.0,             // txpower
                               8,               // fsk preamble bits.
                               true             // enable ook
                               );

    RADIOLIB_STATE(state, "beginFSK");

    delay(3000);

    state = radio.setDataShapingOOK(2);     // Default 0 ( 0, 1, 2 )
    RADIOLIB_STATE(state, "setDataShapingOOK");

    state = radio.setOokThresholdType(
        RADIOLIB_SX127X_OOK_THRESH_PEAK);     // Peak is default
    RADIOLIB_STATE(state, "OOK Thresh PEAK");

    state = radio.setOokPeakThresholdDecrement(
        RADIOLIB_SX127X_OOK_PEAK_THRESH_DEC_1_1_CHIP);     // default
    RADIOLIB_STATE(state, "OOK PEAK Thresh Decrement");

    state = radio.setOokPeakThresholdStep(
        RADIOLIB_SX127X_OOK_PEAK_THRESH_STEP_0_5_DB);     // default
    RADIOLIB_STATE(state, "Ook Peak Threshold Step");

    state = radio.setOokFixedOrFloorThreshold(
        OokFixedThreshold);     // Default 0x0C RADIOLIB_SX127X_OOK_FIXED_THRESHOLD
    RADIOLIB_STATE(state, "OokFixedThreshold");

    state = radio.setBitRate(4.8);
    RADIOLIB_STATE(state, "setBitRate");

	state = radio.setGain(0); //0=autogain 1=max 5=low
    RADIOLIB_STATE(state, "setGain(max)");
	
    // set function that will be called each time a bit is received
    radio.setDirectAction(My_ISR);

    // start direct mode reception
    radio.receiveDirect();

}
