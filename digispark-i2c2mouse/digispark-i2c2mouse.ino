// board manager url: http://digistump.com/package_digistump_index.json

#include <DigiMouse.h>
#include "TinyWireS.h"

#define I2C_SLAVE_ADDRESS 0x28 // the 7-bit address
#define LED_PIN 1 // DigiSpark LED pin

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

volatile uint8_t i2c_regs[] =
{
    0x00, //DUMMY
    0x11, //REG_MOUSE_CTRL 
    0x22, //DUMMY 
    0x33, //DUMMY 
};

#define RCV_DATA_SIZE 16
uint8_t rcvData[RCV_DATA_SIZE];
uint8_t rcvDataCnt;
uint8_t rcvDataWp;
uint8_t rcvDataRp;

#define REG_MOUSE_CTRL 0x01

#define MOUSE_SPEED (10)
signed char x = 0;
signed char y = 0;

#define CROSS_DOWN  0x10
#define CROSS_UP    0x20
#define CROSS_RIGHT 0x40
#define CROSS_LEFT  0x80
#define BUTTON_1    0x02
#define BUTTON_2    0x01

void convertRcvData(uint8_t data) {

  if((data & 0xF0) > 0) { //contain crosskey info
    switch (data & 0xF0) {
      case CROSS_DOWN:
        y = MOUSE_SPEED;
        break;
      case CROSS_UP:
        y = -MOUSE_SPEED;
        break;
      case CROSS_RIGHT:
        x = MOUSE_SPEED;
        break;
      case CROSS_LEFT:
        x = -MOUSE_SPEED;
        break;
      default:
        break;
    }
  }
  else { //release crosskey
    x = 0;
    y = 0;
  }

  if((data & 0x0F) > 0) { //contain button info
    switch (data & 0x0F) {
      case BUTTON_1:
        DigiMouse.setButtons(1<<0); //left click
        break;
      case BUTTON_2:
        DigiMouse.setButtons(1<<1); //right click
        break;
      default:
        break;
    }
  }
  else { //release all button
    DigiMouse.setButtons(0); //unclick all
    DigiMouse.delay(10);
  }
  
}

void mouseSetup()
{
    DigiMouse.begin();
}

void mouseTask()
{
    DigiMouse.moveX(x); // keep moving
    DigiMouse.delay(10);
    DigiMouse.moveY(y);
    DigiMouse.delay(10);  
}

volatile byte reg_position;
const byte reg_size = sizeof(i2c_regs);

void requestEvent()
{  
    reg_position %= reg_size;
    TinyWireS.send(i2c_regs[reg_position]);
    reg_position++;
}

void receiveEvent(uint8_t howMany)
{
    // Sanity check
    if ((howMany < 1) || (howMany > TWI_RX_BUFFER_SIZE)) return;
    
    reg_position = TinyWireS.receive();
    howMany--;

    while(howMany--)
    {
        reg_position %= reg_size;
        i2c_regs[reg_position] = TinyWireS.receive();
        if( (reg_position == REG_MOUSE_CTRL) && ((i2c_regs[reg_position] & 0x0C) > 0) ) { // check valid ctrl data or not
          if(rcvDataCnt < RCV_DATA_SIZE ) {
            rcvDataCnt++;
            i2c_regs[reg_position] -= 0x0C; // clear validation bit
            rcvData[rcvDataWp] = i2c_regs[reg_position];
            rcvDataWp++;
            rcvDataWp %= RCV_DATA_SIZE;
          }
        }
    }
}

void setup(){
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN, LOW);  // off LED

    rcvDataCnt = 0;
    rcvDataWp = 0;
    rcvDataRp = 0;

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);

    mouseSetup();
    digitalWrite(LED_PIN, HIGH);  // on LED
}

void loop()
{
    TinyWireS_stop_check();
    if(rcvDataCnt > 0) {
      rcvDataCnt--;
      convertRcvData(rcvData[rcvDataRp]);
      rcvDataRp++;
      rcvDataRp %= RCV_DATA_SIZE;
    }

    mouseTask();
}
