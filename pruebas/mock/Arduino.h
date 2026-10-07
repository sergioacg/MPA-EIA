// Mock minimo de Arduino para compilar el firmware en el PC y probarlo.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <deque>
#define A0 14
extern uint32_t g_micros;
extern int g_adc;
extern int g_txFree;
inline uint32_t micros(){ return g_micros; }
inline uint32_t millis(){ return g_micros/1000; }
inline int analogRead(uint8_t){ return g_adc; }
struct MockSerial {
  std::deque<uint8_t> rx, tx;
  void begin(uint32_t){}
  int available(){ return (int)rx.size(); }
  int read(){ if(rx.empty()) return -1; uint8_t b=rx.front(); rx.pop_front(); return b; }
  int availableForWrite(){ return g_txFree; }
  size_t write(const uint8_t* b, size_t n){ for(size_t i=0;i<n;i++) tx.push_back(b[i]); return n; }
};
extern MockSerial Serial;
