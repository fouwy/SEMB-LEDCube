#include <Wire.h>

#define RECON    1
#define ALL_HIGH 2

int mode = 0;
int mode_read = 0;

//var para butao
int switch_old = 255;

// Slave addr -> 010.0A2A1A0
int PortExpLEFT = 0x24; //100
int PortExpCENTER = 0x26; //110
int PortExpRIGHT = 0x27; //111
int IOCON   = 0x0A;
int IODIR   = 0x00;
int IPOL    = 0x01;
int GPINTEN = 0x02;
int GPPU    = 0x06;
int GPIO    = 0x09;
int OLAT    = 0x0A;

int SR_DIN = 2; //cabo verde
int SR_CLK = 3; //cabo azul

int PortExpLEFT_A = 1;
int PortExpLEFT_B = 1;
int PortExpCENTER_A = 1;
int PortExpCENTER_B = 1;
int PortExpRIGHT_A = 1;
int PortExpRIGHT_B = 1;

int LED1 = 1;
int LED2 = 1;


void setup() {
  Wire.begin(); // Initiate the Wire library
  Serial.begin(9600);
  I2C_write(PortExpLEFT, IOCON, 0b10100000);
  //BANK   = 1 : sequential register addresses
  //MIRROR = 0 : use configureInterrupt 
  //SEQOP  = 1 : sequential operation disabled, address pointer does not increment
  //DISSLW = 0 : slew rate enabled
  //HAEN   = 0 : hardware address pin is always enabled on 23017
  //ODR    = 0 : open drain output
  //INTPOL = 0 : interrupt active low
  
  IOCON   = 0x05; //update IOCON addr because of BANK update
  
  //Initialization of LEDs
  I2C_write(PortExpLEFT, OLAT, 0b11111100);        //Port A
  I2C_write(PortExpLEFT, OLAT+0x10, 0b00111111);   //Port B
  I2C_write(PortExpCENTER, OLAT, 0b11111100);      //Port A
  I2C_write(PortExpCENTER, OLAT+0x10, 0b00111111); //Port B
  I2C_write(PortExpRIGHT, OLAT, 0b11111100);       //Port A
  I2C_write(PortExpRIGHT, OLAT+0x10, 0b00111111);  //Port B
  
  //I/O configuration -> device 0x24 (LEFT)  
  I2C_write(PortExpLEFT, IODIR, 0b00000011);       //Port A
  I2C_write(PortExpLEFT, IODIR+0x10, 0b11000000);  //Port B
  I2C_write(PortExpCENTER, IODIR, 0b00000011);     //Port A
  I2C_write(PortExpCENTER, IODIR+0x10, 0b11000000);//Port B
  I2C_write(PortExpRIGHT, IODIR, 0b00000011);      //Port A
  I2C_write(PortExpRIGHT, IODIR+0x10, 0b11000000); //Port B
 
  //Pullup configuration -> device 0x24 (LEFT)
  I2C_write(PortExpLEFT, GPPU, 0b00000011);        //Port A
  I2C_write(PortExpLEFT, GPPU+0x10, 0b11000000);   //Port B
  I2C_write(PortExpCENTER, GPPU, 0b00000011);      //Port A
  I2C_write(PortExpCENTER, GPPU+0x10, 0b11000000); //Port B
  I2C_write(PortExpRIGHT, GPPU, 0b00000011);       //Port A
  I2C_write(PortExpRIGHT, GPPU+0x10, 0b11000000);  //Port B

  pinMode(SR_DIN, OUTPUT);
  pinMode(SR_CLK, OUTPUT);

  digitalWrite(SR_DIN, LOW);
  digitalWrite(SR_CLK, LOW);
  
  for(int i = 0; i < 16; i++){  
    CLK_cycle(SR_CLK, 1);
  }

  

}


void loop() {
  // put your main code here, to run repeatedly:
  
  int curr_state;
  int yellow1_on = 0;
  I2C_read(PortExpLEFT, GPIO+0x10, 1);
  
  if(Wire.available() <= 1){   
    PortExpLEFT_A = Wire.read();
    if (PortExpLEFT_A != switch_old) {
    Serial.print("switch_curr: ");
    Serial.println(PortExpLEFT_A, BIN);
    }
    yellow1_on = 0;
    if (switch_old == 192 && PortExpLEFT_A == 64) {
      Serial.print("Yellow 2 > on");
      yellow1_on = 1;
    }
    if (switch_old == 255 && PortExpLEFT_A == 127) {
      Serial.print("Yellow 2 > on");
      yellow1_on = 1;
    }

    if (PortExpLEFT_A == 128) {
      I2C_write(PortExpLEFT, OLAT+0x10, 0b11111111);
      digitalWrite(SR_DIN, LOW);
      CLK_cycle(SR_CLK, 16);  
    }
    if (PortExpLEFT_A != switch_old) {
    Serial.print("switch_old: ");
    Serial.println(switch_old, BIN);
    }
    switch_old = PortExpLEFT_A;
  }

  if(yellow1_on == 1){
    Serial.println("hey");
    I2C_write(PortExpLEFT  , OLAT+0x10, 0x00000000);
    digitalWrite(SR_DIN, HIGH);
    //digitalWrite(SR_CLK, LOW);

    CLK_cycle(SR_CLK, 1);
    delay(100);

    digitalWrite(SR_DIN, LOW);

  }


    delay(100);
}

void I2C_write(int device, int reg, int val){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void I2C_read(int device, int reg, int n){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(device, n);
}

void CLK_cycle(int clk, int num){
  for(int i = 0; i < num; i++){
      digitalWrite(clk, HIGH);
      digitalWrite(clk, LOW);
  }
}
