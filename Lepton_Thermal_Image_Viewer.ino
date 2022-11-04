//#define DEBUG

// Lepton's i2c
#define ADDRESS	(0x2A)

// Lepton commands
#define AGC (0x01)
#define SYS (0x02)
#define VID (0x03)
#define OEM (0x08)

#define GET (0x00)
#define SET (0x01)
#define RUN (0x02)

#define COMMANDID_REG (0x04)
#define DATALEN_REG (0x06)
#define DATA0 (0x08)

#define VOSPI_FRAME_SIZE (164)
#define IMAGE_SIZE (800)


#include <SPI.h>
//#include <WIRE.h>
#include <i2c_t3.h>
#include <ILI9341_t3.h>
//#include "font_Arial.h"
#include "colormap.h"

#define SPICLOCK 30000000	//TFT(ILI9341)

SPISettings settingsA(20000000, MSBFIRST, SPI_MODE0);	//LEPTON


// LEPTON and TFT(ILI9341) pins
#define Lepton_CS	15

#define TFT_CS		10
#define TFT_DC		9

// Use hardware SPI (#13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

//Array to store one Lepton frame
byte leptonFrame[VOSPI_FRAME_SIZE];
static uint16_t lepton_image[60][80]; //Array to store 80 x 60 RAW14 lepton_image
long do_ffc = 0;
int first_ffc_done = 0;
//unsigned int min = 65536;
//unsigned int max = 0;
float diff;
uint16_t men = 65535;
uint16_t mex = 0;

void lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command) {
  byte error;
  Wire.beginTransmission(ADDRESS);

  // Command Register is a 16-bit register located at Register Address 0x0004
  Wire.write(0x00);
  Wire.write(0x04);

  if (moduleID == 0x08) {	//OEM module ID
    Wire.write(0x48);
  } else {
    Wire.write(moduleID & 0x0f);
  }
  Wire.write( ((commandID << 2 ) & 0xfc) | (command & 0x3));

  error = Wire.endTransmission();		// stop transmitting
  if (error != 0) {
#ifdef DEBUG
    Serial.print("error=");
    Serial.println(error);
#endif
  }
}


void lepton_agc_enable() {
  byte error;
  Wire.beginTransmission(ADDRESS); // transmit to device #4
  //16bit data_reg address
  Wire.write(0x00);
  Wire.write(DATA0);
  //16bit command equivalent to SDK LEP_GetAgcEnableState()
  Wire.write(0x00);
  Wire.write(0x01);
  error = Wire.endTransmission();
  if (error != 0) {
#ifdef DEBUG
    Serial.print("error=");
    Serial.println(error);
#endif
  }
  Wire.beginTransmission(ADDRESS);
  //16bit data_len address
  Wire.write(0x00);
  Wire.write(DATALEN_REG);
  //16bit value for number of bytes in data_regs (not number of regs)
  Wire.write(0x00);
  Wire.write(0x02);
  error = Wire.endTransmission();
  if (error != 0) {
#ifdef DEBUG
    Serial.print("error=");
    Serial.println(error);
#endif
  }
  Wire.beginTransmission(ADDRESS);
  //16bit command_reg address
  Wire.write(0x00);
  Wire.write(COMMANDID_REG);
  //16bit module id of AGC (0x0100) binary AND with SET (0x01) and then split into 2 bytes (0x0101)
  Wire.write(0x01);
  Wire.write(0x01);
  error = Wire.endTransmission();
  if (error != 0) {
#ifdef DEBUG
    Serial.print("error=");
    Serial.println(error);
#endif
  }
}


void lepton_set_reg(unsigned int reg) {
  byte error;
  Wire.beginTransmission(ADDRESS); // transmit to device #4
  Wire.write(reg >> 8 & 0xff);
  Wire.write(reg & 0xff);						// sends one byte

  error = Wire.endTransmission();		// stop transmitting
  if (error != 0) {
#ifdef DEBUG
    Serial.print("error=");
    Serial.println(error);
#endif
  }
}


//Status reg 15:8 Error Code	7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy
int lepton_read_reg(unsigned int reg) {
  int reading = 0;
  lepton_set_reg(reg);

  //	Serial.println("req");
  Wire.requestFrom(ADDRESS, 2);

  reading = Wire.read();	// receive high byte (overwrites previous reading)
  //Serial.println(reading);
  reading = reading << 8;		// shift high byte to be high 8 bits

  reading |= Wire.read(); // receive low byte as lower 8 bits
#ifdef DEBUG
  Serial.print("reg:");
  Serial.print(reg);
  Serial.print("==0x");
  Serial.print(reading, HEX);
  Serial.print(" binary:");
  Serial.println(reading, BIN);
#endif
  return reading;
}


int lepton_read_data() {
  int i;
  int data;
  int payload_length;

  while (lepton_read_reg(0x2) & 0x01)	{
#ifdef DEBUG
    //		Serial.println("busy");
#endif
  }

  payload_length = lepton_read_reg(0x6);
#ifdef DEBUG
  Serial.print("payload_length=");
  Serial.println(payload_length);
#endif

  Wire.requestFrom(ADDRESS, payload_length);
  //lepton_set_reg(0x08);
  for (i = 0; i < (payload_length / 2); i++)	{
    data = Wire.read() << 8;
    data |= Wire.read();
#ifdef DEBUG
    Serial.println(data, HEX);
#endif
  }
  return data;
}


void setup() {
  pinMode(Lepton_CS, OUTPUT);
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(Lepton_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);
  SPI.begin();
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  tft.begin();
  Serial.begin(115200);
  //	while(!Serial){};

  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  //	tft.setFont(Arial_14);

#ifdef DEBUG
  Serial.println("setup complete");
#endif

#ifdef DEBUG
  Serial.println("beginTransmission");
#endif

  //lepton_set_reg(0);

  //lepton_read_reg(0x0);

#ifdef DEBUG
  Serial.println("test");
#endif
  lepton_read_reg(0x2);
#ifdef DEBUG
  Serial.println("test2");
  Serial.println("test");
#endif

#ifdef DEBUG
  Serial.println("SYS Camera Customer Serial Number");
#endif
  lepton_command(SYS, 0x28 >> 2 , GET);
  lepton_read_data();

#ifdef DEBUG
  Serial.println("SYS Flir Serial Number");
#endif
  lepton_command(SYS, 0x08 >> 2 , GET);
  lepton_read_data();

#ifdef DEBUG
  Serial.println("SYS Camera Uptime");
#endif
  lepton_command(SYS, 0x0C >> 2 , GET);
  lepton_read_data();

#ifdef DEBUG
  Serial.println("SYS Fpa Temperature Kelvin");
#endif
  lepton_command(SYS, 0x14 >> 2 , GET);
  lepton_read_data();

#ifdef DEBUG
  Serial.println("SYS Aux Temperature Kelvin");
#endif
  lepton_command(SYS, 0x10 >> 2 , GET);
  lepton_read_data();

#ifdef DEBUG
  Serial.println("OEM Chip Mask Revision");
#endif
  lepton_command(OEM, 0x14 >> 2 , GET);
  lepton_read_data();

  //Serial.println("OEM Part Number");
  //lepton_command(OEM, 0x1C >> 2 , GET);
  //lepton_read_data();

#ifdef DEBUG
  Serial.println("OEM Camera Software Revision");
#endif
  lepton_command(OEM, 0x20 >> 2 , GET);
  lepton_read_data();

#ifdef DEBUG
  Serial.println("AGC Enable");
#endif
  //	lepton_command(AGC, 0x01	, SET);
  //	lepton_agc_enable();
  //	lepton_read_data();

#ifdef DEBUG
  Serial.println("AGC READ");
#endif
  lepton_command(AGC, 0x00	, GET);
  lepton_read_data();
}


/* Reads one line (164 Bytes) from the lepton over SPI */
boolean leptonReadFrame(uint8_t line) {
  bool success = true;
  //Receive one frame over SPI
  SPI.transfer(leptonFrame, 164);
  //Check for success
  if ((leptonFrame[0] & 0x0F) == 0x0F) {
    success = false;
  }
  else if (leptonFrame[1] != line) {
    success = false;
  }
  return success;
}

/* Get one image from the Lepton module */
void getTemperatures() {
  byte leptonError = 0;
  byte line;
  uint16_t result;

  //Begin SPI Transmission
  beginLeptonSPI();
  do {
    leptonError = 0;
    for (line = 0; line < 60; line++) {
      //Reset if the expected line does not match the answer
      if (!leptonReadFrame(line)) {
        //Reset line to -1, will be zero in the next cycle
        line = -1;
        //Raise Error count
        leptonError++;
        // Serial.println(leptonError);

        //Small delay
        delay(1);
        //If the Error count is too high, reset the device
        if (leptonError > 100) {
          //Re-Sync the Lepton
          endLeptonSPI();
          delay(186);
          beginLeptonSPI();
          break;
        }
      }
      //If line matches answer, save the packet
      else {
        //Go through the video pixels for one video line
        for (int column = 0; column < 80; column++) {
          result = (uint16_t)(leptonFrame[2 * column + 4] << 8
                              | leptonFrame[2 * column + 5]);
          if (result < 65536)
          { men = result;
          }
          if (result > 0)
          {
            mex = result;
          }
          //Save to array
          lepton_image[line][column] = result;
        }
      }
    }
    diff = mex - men;
    diff = diff / 256.0f;
    if (diff < 0.56f) diff = 0.56f; // 0.66

  } while ((leptonError > 100) || (line != 60));
  //End Lepton SPI
  endLeptonSPI();
}


/* Start Lepton SPI Transmission */
void beginLeptonSPI() {
  //Begin SPI Transaction on alternative Clock
  SPI.beginTransaction(settingsA);
  //Start transfer	- CS LOW
  digitalWriteFast(Lepton_CS, LOW);
}


/* End Lepton SPI Transmission */
void endLeptonSPI() {
  //End transfer - CS HIGH
  digitalWriteFast(Lepton_CS, HIGH);
  //End SPI Transaction
  SPI.endTransaction();
}


/* Print out the 80 x 60 raw values array for every complete image */
void printValues() {
  for (int i = 0; i < 60; i++) {
    for (int j = 0; j < 80; j++) {
      Serial.print(lepton_image[i][j]);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
}

//
///* Main loop, get temperatures and print them */
void loop() {
  getTemperatures();
  delay(2000);
  Serial.println("Start!");
  printValues();
  Serial.println("End!");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  delay(2000);

  do_ffc++;
  if (((do_ffc > 150) && (first_ffc_done == 0)) || (do_ffc % 600 == 0)) {
    //		lepton_command(SYS, 0x40 >> 2 , GET);
    //		lepton_read_data();

    //This command executes the camera's Flat-Field Correction (FFC) normalization.
    lepton_command(SYS, 0x42 >> 2 , RUN);
    Serial.println("RUN FFC");
    lepton_read_data();
    first_ffc_done = 1;
  }
  uint16_t pixrow[240];
  int16_t x = 0;
  int16_t y = 0;
  unsigned int col;

  //	Thermo viewing of lepton
  for (y = 0; y < 60; y++) {
    for (x = 0; x < 80; x++) {
      col = lepton_image[y][x];
      if ((col != 0x0000) && (col != 0x3fff)) {
        col = col - men;
        col = col / diff;
        if (col < 0) col = 0;
        if (col > 255) col = 255;
        pixrow[ 3 * x ] = colormap[col];		//x3 enlargement
        pixrow[ 3 * x + 1 ] = colormap[col];	//x3 enlargement
        pixrow[ 3 * x + 2 ] = colormap[col];	//x3 enlargement
      }
    }
    tft.writeRect(0, 3 * y, 240, 1, pixrow);
    tft.writeRect(0, 3 * y + 1, 240, 1, pixrow);
    tft.writeRect(0, 3 * y + 2, 240, 1, pixrow);
  }
  }
  ////
  //////	Thermo scaler
  ////	float temp = min;
  ////	float stepp = (max-min) /200.0f;
  ////	uint16_t scaler[200];
  ////		for(x=0;x<200;x++){
  ////			col = temp;
  ////			col = col - min;
  ////			col = col / diff;
  ////			if(col < 0) col = 0;
  ////			if(col > 255) col = 255;
  ////			scaler[x] = colormap[col];
  ////			temp += stepp;
  ////		}
  ////	for(y=0;y<20;y++){
  ////		tft.writeRect(0,210+y, 200,1, scaler);
  ////	}
  ////
  ////	char temptext[10];
  //////	Serial.println("the Lepton housing's Temperature");
  ////	lepton_command(SYS, 0x10 >> 2 , GET); // 0x14 = chip	 0x10 = aux
  ////	float fpatemp = (float)lepton_read_data() / 100.0f;
  ////	float fpatemp_f = fpatemp * 1.8f - 459.67f;
  ////
  ////	float value;
  //////	display min temperature
  ////	value = ((0.05872 * (float)min - 472.22999f + fpatemp_f));
  ////	value = (value - 32.0f) / 1.8f;
  ////	sprintf(temptext, " %-4d", (int)(value + 0.5f));
  ////	tft.fillRect(0,190,80,20,ILI9341_BLACK);
  ////	tft.setCursor(0,190);
  ////	tft.print(temptext);
  ////
  //////	Display max temperature
  ////	value = ((0.05872 * (float)max - 472.22999f + fpatemp_f));
  ////	value = (value - 32.0f) / 1.8f;
  ////	sprintf(temptext, " %4d", (int)(value + 0.5f));
  ////	tft.fillRect(180,190,80,20,ILI9341_BLACK);
  ////	tft.setCursor(180, 190);
  ////	tft.print(temptext);
  ////
  ////	min = 65536;
  ////	max = 0;
  ////
  ////	delay(100);
  ////}
