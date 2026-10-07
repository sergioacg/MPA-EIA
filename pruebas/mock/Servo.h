#pragma once
#include <stdint.h>
extern int g_pulse;
struct Servo { void attach(int,int,int){} void writeMicroseconds(int p){ g_pulse=p; } };
