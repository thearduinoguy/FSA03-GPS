// This code gives an example of configuring an FSA03 connected to a software serial port on an Arduino
 
#include <SoftwareSerial.h>

SoftwareSerial nss(2, 3); // RX, TX
 
// GPS Tx output is connected to Arduino input on pin 2
// GPS Rx input is connected to Arduino output on pin 3
 
byte navmode = 99;
 
void setup() {
 
	// Start up serial ports
	nss.begin(38400);
	Serial.begin(115200); // used for debug ouput
 
	delay(2000); // Give the GPS time to come boot
 
	// Lower the baud rate to 9600 from 38.4k
	Serial.print("Setting uBlox port mode: ");
	uint8_t setPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x95};
	sendUBX(setPort, sizeof(setPort)/sizeof(uint8_t));
 
	// Switch baud rates on the software serial
	Serial.println("Switching to 9600b GPS serial");
	nss.begin(9600);
	delay(1000);
 
	// Set the navigation mode (Airborne, 1G)
	Serial.print("Setting uBlox nav mode: ");
	uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
	sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
	getUBX_ACK(setNav);
 
 
	// Switch off GLL
	Serial.print("Switching off NMEA GLL: ");
	uint8_t setGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
	sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
	getUBX_ACK(setGLL);
 
	// Switch off GSA
	Serial.print("Switching off NMEA GSA: ");
	uint8_t setGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
	sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
	getUBX_ACK(setGSA);
 
	// Switch off GSV
	Serial.print("Switching off NMEA GSV: ");
	uint8_t setGSV[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
	sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
	getUBX_ACK(setGSV);
 
	// Switch off RMC
	Serial.print("Switching off NMEA RMC: ");
	uint8_t setRMC[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40 };
	sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
	getUBX_ACK(setRMC);
 
}
 
// Dump bytes to debug as they appear
void loop() {
	if (nss.available()) {
		Serial.write(nss.read());
	}
}
 
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
	for(int i=0; i<len; i++) {
		nss.write(MSG[i]);
		Serial.print(MSG[i], HEX);
	}
	Serial.println();
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
	unsigned long startTime = millis();
	Serial.print(" * Reading ACK response: ");
 
	// Construct the expected ACK packet    
	ackPacket[0] = 0xB5;	// header
	ackPacket[1] = 0x62;	// header
	ackPacket[2] = 0x05;	// class
	ackPacket[3] = 0x01;	// id
	ackPacket[4] = 0x02;	// length
	ackPacket[5] = 0x00;
	ackPacket[6] = MSG[2];	// ACK class
	ackPacket[7] = MSG[3];	// ACK id
	ackPacket[8] = 0;		// CK_A
	ackPacket[9] = 0;		// CK_B
 
	// Calculate the checksums
	for (uint8_t i=2; i<8; i++) {
		ackPacket[8] = ackPacket[8] + ackPacket[i];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}
 
	while (1) {
 
		// Test for success
		if (ackByteID > 9) {
				// All packets in order!
				Serial.println(" (SUCCESS!)");
				return true;
		}
 
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
			Serial.println(" (FAILED!)");
			return false;
		}
 
		// Make sure data is available to read
		if (nss.available()) {
			b = nss.read();
 
			// Check that bytes arrive in sequence as per expected ACK packet
			if (b == ackPacket[ackByteID]) { 
				ackByteID++;
				Serial.print(b, HEX);
			} else {
				ackByteID = 0;	// Reset and look again, invalid order
			}
 
		}
	}
}
 
//Function to poll the NAV5 status of a Ublox GPS module (5/6)
//Sends a UBX command (requires the function sendUBX()) and waits 3 seconds
// for a reply from the module. The then isolates the byte which contains 
// the information regarding the NAV5 mode,
// 0 = Pedestrian mode (default, will not work above 12km)
// 6 = Airborne 1G (works up to 50km altitude)
//Adapted by jcoxon from getUBX_ACK() from the example code on UKHAS wiki
// http://wiki.ukhas.org.uk/guides:falcom_fsa03
boolean checkNAV(){
  uint8_t b, bytePos = 0;
  uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 }; //Poll NAV5 status
 
  nss.flush();
  unsigned long startTime = millis();
  sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
 
  while (1) {
    // Make sure data is available to read
    if (nss.available()) {
      b = nss.read();
 
      if(bytePos == 8){
        navmode = b;
        return true;
      }
 
      bytePos++;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      navmode = 0;
      return false;
    }
  }
}
