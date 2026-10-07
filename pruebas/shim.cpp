#include "Arduino.h"
#include "Servo.h"
uint32_t g_micros=0; int g_adc=512; int g_txFree=63; int g_pulse=-1;
MockSerial Serial;
#include "../firmware/MPA_Firmware/MPA_Firmware.ino"
extern "C" {
void fw_setup(){ setup(); }
void fw_loop(){ loop(); }
void set_micros(uint32_t t){ g_micros=t; }
void set_adc(int a){ g_adc=a; }
void set_txfree(int n){ g_txFree=n; }
int  get_pulse(){ return g_pulse; }
void push_rx(const uint8_t* b,int n){ for(int i=0;i<n;i++) Serial.rx.push_back(b[i]); }
int  pop_tx(uint8_t* out,int max){ int n=0; while(!Serial.tx.empty() && n<max){ out[n++]=Serial.tx.front(); Serial.tx.pop_front(); } return n; }
int  get_estado(){ return (int)estado; }
}
