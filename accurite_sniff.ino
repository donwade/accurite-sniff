#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <avr/eeprom.h>

//#include <M5Unified.h>
#include <_m5Core2-only.h>
#include <FastLED.h>
#include <M5GFX.h>
#include "DisplayPage.hpp"

#include "_viewController.h"

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
void RadioSetupRx();
// include the library
#include <RadioLib.h>

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

const int DI02 = 25; 	// don't enable LEDBAR, it uses 25 too!
//------------------------------------------------------------------
#define BUCKET_SIZE 16
uint16_t bucketHi[BUCKET_SIZE];
uint16_t bucketLo[BUCKET_SIZE];
bool bStopRecording = false;

#define STAT_LIMIT_LO 100
#define STAT_LIMIT_HI 900

#ifdef LEGACY
    // original time defs for a logic 1 or 0        
    #define LEFT_LO		175
    #define RIGHT_LO	250
    #define LEFT_HI		375
    #define RIGHT_HI	450
	#define TSYNC_LO	575
	#define TSYNC_HI	675
#else
    // original time defs for a logic 1 or 0		
    #define LEFT_LO 	224
    #define RIGHT_LO	284
    #define LEFT_HI 	347
    #define RIGHT_HI	469
    #define TSYNC_LO 	531
    #define TSYNC_HI 	592
	
#endif

uint8_t calculatedFloor = 2;

#define iRUNNING_FREQ (433948200 - 2100)     // GOLD

//#define iRUNNING_FREQ (433943700 - 2100)     // CONSTANT tx
//#define iRUNNING_FREQ (433928000 - 2100)
//#define iRUNNING_FREQ (433910000 - 2300 + 1400)
#define RUNNING_FREQ ((float)(iRUNNING_FREQ)/1000000.)

// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz
#define RUNNING_BW  7.8  //15.6 //250. //15.6 //31.25 //250. //10.4

#define CAL_FREQUENCY (RUNNING_FREQ + .008000)
float currentFreq = RUNNING_FREQ;




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
volatile unsigned int dataCtr = 0;
volatile unsigned long lastTime = 0;
volatile unsigned int syncCtr = 0;
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

 #define RADIOLIB_STATE(STATEVAR, FUNCTION)                              \
        {                                                                     \
            if ((STATEVAR) == RADIOLIB_ERR_NONE) {                              \
                Serial.printf(" " FUNCTION " - success!\n");     \
            } else {                                                            \
                Serial.printf(" " FUNCTION " failed, code: %d\n", \
                              STATEVAR);                                            \
                delay(3000); \
                assert(STATEVAR == 0); \
            }                                                                   \
        }

String degreesToCompass(float degrees)
{
    // 8 point compass
    const char *directions[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    int index = (int)((degrees + 22.5) / 45) % 8;

    return String(directions[index]);
}


bool acurite_crc(volatile byte pkt[], int cols)
{
	int i;
    cols -= 1; // last byte is CRC
    int sum = 0;

    for (i = 0; i < cols; i++)
    {	
    	sum += pkt[i];
    	Serial.printf(" pkt[%2d] = 0x%02X  run_sum = 0x%02X \n", i, pkt[i], sum);
    }

	Serial.printf(" pkt.crc=0x%02X  end_sum = 0x%02X \n", pkt[i], sum % 256);

    return sum != 0 && sum % 256 == pkt[cols];
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

//---------------------------------------------------------------------
void beacon(void)
{
	// the following settings can also
	// be modified at run-time
	currentFreq = RUNNING_FREQ;
	
	state = radio.setFrequency(currentFreq);	 // freq for wx?
    RADIOLIB_STATE(state, "setDataShapingOOK");
    
	state = radio.setBitRate(.5);
    RADIOLIB_STATE(state, "setBitRate");

	state = radio.setFrequencyDeviation(.21);
	RADIOLIB_STATE(state, "setFrequencyDeviation");

	state = radio.setRxBandwidth(8.0);
    RADIOLIB_STATE(state, "setRxBandwidth");

	state = radio.setOutputPower(2.0);	  //lowest pwr.
    RADIOLIB_STATE(state, "setOutputPower");

	state = radio.setCurrentLimit(100);
    RADIOLIB_STATE(state, "setCurrentLimit");

/*
	state = radio.setDataShaping(RADIOLIB_SHAPING_0_5);
	uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
						  0x89, 0xAB, 0xCD, 0xEF};

	state = radio.setSyncWord(syncWord, 8);
	if (state != RADIOLIB_ERR_NONE) {
	  Serial.print(F("Unable to set configuration, code "));
	  Serial.println(state);
	  while (true) { delay(10); }
	}
*/

	// FSK modulation can be changed to OOK
	// NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
	//		 Also, data shaping changes from Gaussian filter to
	//		 simple filter with cutoff frequency. Make sure to call
	//		 setDataShapingOOK() to set the correct shaping!

	state = radio.setOOK(true);
    RADIOLIB_STATE(state, "setOOK");
    
	state = radio.setDataShapingOOK(2);  // no shaping 
    RADIOLIB_STATE(state, "setDataShapingOOK");

	// ------------------------------------------------

	/*
	  byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
						0x89, 0xAB, 0xCD, 0xEF};
	  int state = radio.transmit(byteArr, 8);
	*/

	const char *test = "01234567890123456789012345678901234567890123456789";  

	for (int i = 0; i < 3; i++)
	{
		// transmit OOK packet
		//state = radio.transmit("Hello World!");
		state = radio.transmit( test, 0);
		if (state == RADIOLIB_ERR_NONE) 
		{
		  Serial.printf("[SX1278] Packet %d transmitted successfully!\n", i);
		} else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
		  Serial.println(F("[SX1278] Packet too long!"));
		} else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
		  Serial.println(F("[SX1278] Timed out while transmitting!"));
		} else {
		  Serial.println(F("[SX1278] Failed to transmit packet, code "));
		  Serial.println(state);
		}
		delay(200);
	}
}
//---------------------------------------------------------------------

// === Setup ===
void setup()
{
    Serial.begin(115200);

	_setup_M5();
    // _setup_lightbar(); cannot use as lb uses GPIO25 as well :(

/*	
    M5.begin();
	auto cfg = M5.config();
	
	// Set the items you want to configure. Omit the following two lines if you use the default settings.
	cfg.serial_baudrate = 115200;
	cfg.output_power = true;
	
	M5.begin(cfg);
*/
    
    M5.Lcd.init();
    M5.Lcd.clear();
    M5.Lcd.setCursor(3, 0);

    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextWrap(false);

    lprintf("WeatherSys");
    M5.Lcd.setCursor(5, 1);
    lprintf("Starting");

    
	M5.Speaker.setVolume(25);
    M5.Lcd.clear();


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

	// power off for reset
	M5.Power.setExtPower(false); // TIP
	delay(1000);
	M5.Power.setExtPower(true);  // TIP
	delay(1000);

    int state = radio.beginFSK(434.0,           // freq
                               .5,             // bitrate
                               0.0,            // fsk dev
                               RUNNING_BW,     // rxbw khz
                               2.0,             // txpower
                               8,               // fsk preamble bits.
                               true             // enable ook
                               );

    RADIOLIB_STATE(state, "beginFSK");


	///RadioSetupTx();
	//beacon();      //do not transmit into the rtl+antenna

	
    RadioSetupRx();
    findFloor();

	memset(bucketHi, 0, sizeof(bucketHi));
	memset(bucketLo, 0, sizeof(bucketLo));
	
    Serial.println("**** end of setup ****\n");
    
}

//---------------------------------------------------------------

void loop()
{
	yield();
	_loop_M5();
	
	static uint32_t lastPC;
	if (millis() >= lastPC) 
	{
		//Serial.printf("%d vs %d \n", dataCtr, lastPC);
		report("STATS");
		lastPC = millis() + 5000;
	}
		
    if (bucketFull)
    {
		detachInterrupt(digitalPinToInterrupt(DI02));
        bool valid = acurite_crc(buf, sizeof(buf));


		M5.Speaker.tone(900, 100, 10);
		
        if (valid)
        {
			M5.Speaker.tone(1800, 100, 30);
			
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
            else 
            {
            	Serial.printf("unknown msgtype = 0x%X\n", msgtype);
            }

        }
        
        dataCtr = 0;
		bucketFull = false;
		state == RESET;
		
		attachInterrupt(digitalPinToInterrupt(DI02), My_ISR, CHANGE);
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
	int k;
	Serial.printf(FG_YELLOW "\n%s ----- %d \n", msg, isrCtr);
	Serial.printf("  %4d  ", 0);
	for(int i = 0; i < BUCKET_SIZE; i++)
	{
		//reverse map to show bucket windows.
 		uint16_t undo = map(i, 1, BUCKET_SIZE-2, STAT_LIMIT_LO, STAT_LIMIT_HI);
		Serial.printf(" %4d  ", undo+1);
	}
	Serial.println();
	for(int i = 0; i < BUCKET_SIZE; i++)
	{
		//reverse map to show bucket windows.
 		uint16_t undo = map(i, 1, BUCKET_SIZE-2, STAT_LIMIT_LO, STAT_LIMIT_HI);
		Serial.printf(" -%4d ", undo);
	}
	
	Serial.println(FG_RED);

	uint32_t avg1 = 0;
	uint32_t avg2 = 0;
	uint32_t avg3 = 0;

	uint64_t std1 = 0;
	uint64_t std2 = 0;
	uint64_t std3 = 0;
		
	k = 0;

	// end buckets don't get averaged

	// avg ---- bucketHi ----------------
	for(int i = 1; i < BUCKET_SIZE-2; i++)
	{
		k++;
 		avg1 += bucketHi[i];
		if (bucketHi[i] > 64000) bStopRecording = true;
	}
	avg1 /= k;

	// stdev
	for(int i = 1; i < BUCKET_SIZE-2; i++)
	{
 		std1 += (avg1 - bucketHi[i]) * (avg1 - bucketHi[i]); 
	}
	
	std1 = sqrt(std1/k);
	std1 /= 2;  // make window 
	//Serial.printf("avg = %d +/- std = %d\n", avg1, std1/2);
	
	for(int i = 0; i < BUCKET_SIZE; i++)
	{
		if ( bucketHi[i] > (avg1 - std1) && bucketHi[i] < (avg1 + std1) )
		{
	 		Serial.printf(" %05d ", bucketHi[i]);
	 	}
		else
		{
	 		Serial.print("       ");
 		}
		if (bucketHi[i] > 64000) bStopRecording = true;
	}

	
	Serial.println(FG_GREEN);
	k = 0;
	// end buckets don't get averaged

	// avg ---- bucketLo ----------------
	for(int i = 1; i < BUCKET_SIZE-2; i++)
	{
		k++;
 		avg2 += bucketLo[i];
		if (bucketLo[i] > 64000) bStopRecording = true;
	}
	avg2 /= k;

	// stdev
	for(int i = 1; i < BUCKET_SIZE-2; i++)
	{
 		std2 += (avg2 - bucketLo[i]) * (avg2 - bucketLo[i]); 
	}

	std2 = sqrt(std2/k);
	std2 /= 2;  // make window 
	//Serial.printf("avg = %d +/- std = %d\n", avg2, std2/2);
	
	
	for(int i = 0; i < BUCKET_SIZE; i++)
	{
		if ( bucketLo[i] > (avg2 - std2) && bucketLo[i] < (avg2 + std2) )
		{
	 		Serial.printf(" %05d ", bucketLo[i]);
	 	}
		else
		{
	 		Serial.print("       ");
 		}
		if (bucketLo[i] > 64000) bStopRecording = true;
	}

	Serial.println(FG_DONE);
	Serial.println();
}


// === ISR ===
void My_ISR()
{
	uint16_t idx;
    unsigned long now = micros();

	bool pinState = digitalRead(DI02);
	
    unsigned long duration = now - lastTime;
    
    isrCtr++;

/*
	testing
	idx = map(500, STAT_LIMIT_LO, STAT_LIMIT_HI, 1, BUCKET_SIZE-2);
	bucketHi[idx] += 20;

	idx = map(300, STAT_LIMIT_LO, STAT_LIMIT_HI, 1, BUCKET_SIZE-2);
	bucketLo[idx] += 5;
*/	
	
	//--------------
	if (!bStopRecording)
	{
		if (duration >= STAT_LIMIT_LO && duration <= STAT_LIMIT_HI)
		{
			idx = map(duration, STAT_LIMIT_LO, STAT_LIMIT_HI, 1, BUCKET_SIZE-2);
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
				if (duration > STAT_LIMIT_HI) bucketHi[BUCKET_SIZE-1]++;
				if (duration < STAT_LIMIT_LO) bucketHi[0]++;
			}
			else
			{
				if (duration > STAT_LIMIT_HI) bucketLo[BUCKET_SIZE-1]++;
				if (duration < STAT_LIMIT_LO) bucketLo[0]++;
			}
		}
	}
	//--------------
	

    //if (pinState == HIGH)  // just went hi, so time represents lo time.
    {
        if (now - lastTime > 10000)
        {
            state = RESET;
            syncCtr = 0;
            dataCtr = 0;
			lastTime = now;
			return;
        }
    }

	lastTime = now;

	if (bucketFull) return;	// wait for loop to handle it.
	
    if (state == RESET || state == INSYNC)
    {
		
        if (duration > TSYNC_LO && duration < TSYNC_HI)
        {
            state = INSYNC;
            syncCtr++;

            if (syncCtr > 3)
            {
                state = SYNCDONE;
                syncCtr = 0;
                dataCtr = 0;
            }

            return;
        }
        else
        {
            syncCtr = 0;
            dataCtr = 0;
            state = RESET;
            return;
        }
    }
    else
    {
        if (dataCtr > MAXBITS)
        {
            state = RESET;
            dataCtr = 0;
            
            bucketFull = true;
            return;
        }

        byte bytepos = dataCtr / 8;
        byte bitpos = 7 - (dataCtr % 8);

        if (duration > LEFT_HI && duration < RIGHT_HI)
        {
            bitSet(buf[bytepos], bitpos);
            dataCtr++;
        }
        else if (duration > LEFT_LO && duration < RIGHT_LO)
        {
            bitClear(buf[bytepos], bitpos);
            dataCtr++;
        }
    }
}



//----------------------------------------------------
/*
	bw	setting
	250	42
	31	23
	15	17
	7	12
*/
void findFloor(void)
{
	uint32_t intCtrMax = 0;
	uint8_t  squelchMiddle = 0;

 	uint8_t  squelchFirst = 0;

 	uint8_t  squelchLast = 0;

	currentFreq = CAL_FREQUENCY;
	
	state = radio.setFrequency(currentFreq);
    RADIOLIB_STATE(state, "setFrequency");

	state = radio.setOokThresholdType(RADIOLIB_SX127X_OOK_THRESH_FIXED);
	RADIOLIB_STATE(state, "setOokThresholdType");

	uint16_t squelch;
	for (squelch = 1; squelch < 0xff; squelch++)
	{
		state = radio.setOokFixedOrFloorThreshold(squelch); 
		
		Serial.printf("F=%f mHz BW=%.1f squelch %d " , currentFreq, RUNNING_BW * 1000., squelch);

		uint32_t now = millis();
		isrCtr = 0;
		
		while (millis() < now + 2000)
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
			squelchMiddle = squelch;
		}

		
		Serial.printf("%6d %d < %d < %d cnt=%d\n", squelch, squelchFirst, squelchMiddle, squelchLast, isrCtr);
		//Serial.printf("isrCtr=%d squelchMiddle=%d\n", isrCtr, squelchMiddle);

		if (isrCtr == 0 && intCtrMax != 0) break;  //done
				
	}

	assert(squelch != 0xFF);  // couldn't find value. bail
	
	squelchLast++;

	// Middle is one with most noise, first and last have least.
	Serial.printf("\n %d < %d < %d\n", squelchFirst, squelchMiddle, squelchLast );
	
	// back to real channel.
	currentFreq = RUNNING_FREQ;

	state = radio.setFrequency(currentFreq);
    RADIOLIB_STATE(state, "setFrequency");

	calculatedFloor = squelchLast;
	calculatedFloor += 12; // extra X half dbs
	
	Serial.printf(FG_RED"\ntaking %d as squelch setting \n"FG_DONE, calculatedFloor);

	state = radio.setOokFixedOrFloorThreshold(calculatedFloor); 
    RADIOLIB_STATE(state, "setOokFixedOrFloorThreshold");

	state = radio.setOokThresholdType(RADIOLIB_SX127X_OOK_THRESH_FIXED);
	RADIOLIB_STATE(state, "setOokThresholdType");

	// floor is set for PEAK to gently fall onto 
	// set mode to go from fixed (find floor) to PEAK mode
	// state = radio.setOokThresholdType(RADIOLIB_SX127X_OOK_THRESH_PEAK);
	// RADIOLIB_STATE(state, "setOokThresholdType");

    
}
//-------------------------------------------------------------


void RadioSetupRx()
{
	pinMode(DI02, INPUT);

    state = radio.setOOK(true);
    RADIOLIB_STATE(state, "setOOK");

    state = radio.setDataShapingOOK(2);     // Default 0 ( 0, 1, 2 )
    RADIOLIB_STATE(state, "setDataShapingOOK");

    state = radio.setOokThresholdType(
        RADIOLIB_SX127X_OOK_THRESH_FIXED);
    RADIOLIB_STATE(state, "OOK Thresh FIXED");

    state = radio.setOokFixedOrFloorThreshold(
        calculatedFloor);
    RADIOLIB_STATE(state, "calculatedFloor");
 
	state = radio.disableBitSync();
	RADIOLIB_STATE(state, "disableBitSync");

#if 0
    state = radio.setOokPeakThresholdDecrement(
        RADIOLIB_SX127X_OOK_PEAK_THRESH_DEC_1_1_CHIP);     // default
    RADIOLIB_STATE(state, "OOK PEAK Thresh Decrement");

    state = radio.setOokPeakThresholdStep(
        RADIOLIB_SX127X_OOK_PEAK_THRESH_STEP_0_5_DB);     // default
    RADIOLIB_STATE(state, "Ook Peak Threshold Step");
#endif

    //state = radio.setBitRate(100);
    //RADIOLIB_STATE(state, "setBitRate");

	state = radio.setGain(1); //0=autogain 1=max 6=low
    RADIOLIB_STATE(state, "setGain(max)");
	
    // set function that will be called each time a bit is received
    //radio.setDirectAction(My_ISR);

	state = radio.setRxBandwidth(RUNNING_BW);
    RADIOLIB_STATE(state, "running bw" );
	
   
    // start direct mode reception
    state = radio.receiveDirect();
    RADIOLIB_STATE(state, "receiveDirect");

	attachInterrupt(digitalPinToInterrupt(DI02), My_ISR, CHANGE);


}

void RadioSetupTx()
{
	// power off for reset
	M5.Power.setExtPower(false); // TIP
	delay(1000);
	M5.Power.setExtPower(true);  // TIP
	delay(1000);
	

    // initialize SX1278 with FSK modem at 9600 bps
    Serial.print(F("[SX1278] Initializing for TX ... "));

	//radio.setTxPower(2);

	Serial.println("tx ON");

	//4.2.4. Operating Modes in FSK/OOK Mode
    state = radio.setMode(3);
    RADIOLIB_STATE(state, "setMode");

    // start direct mode reception
	state =radio.transmitDirect(iRUNNING_FREQ);
    RADIOLIB_STATE(state, "transmitDirect");
	
	//detachInterrupt(digitalPinToInterrupt(DI02));

	delay(1000);
	Serial.println("tx OFF");
	
	pinMode(DI02, OUTPUT);
	digitalWrite(DI02, 0);

}

