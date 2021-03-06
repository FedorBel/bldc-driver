
// note: a "4 pole motor" has 2 pole pairs
#define polepairs 2

// 1200 is 100% duty cycle
#define alignmentdc 300

// 1200 is 100% duty cycle
#define rampupdc 440 // 300

// unit is 50uS PWM periods
// set long enough for rotor to come to rest
#define alignmenttime 10000

// 0-255 => 0 to 255/256 of a step time
// may need longer if high inductance or high current
// requires longer tome for current to fall to zero
// should not be set greater than 128
#define demagallowance 64

// unit is 50uS PWM periods
// commutation state machine will advance as if a true
// zero crossing has been detected if this time period has
// elapsed while waiting for ZC.  This helps to re-start
// a motor that has fallen to very low speed
#define maxstep 20000

// RPM at end of ramp
#define holdrpm 500 //300

// unit is 50uS PWM periods, time at hold RPM
#define holdtime 3000 //2000

// ramp up rate in rpm per second
// reduce if stator is moving too fast to hold rotor
#define startuprpmpersecond 300

// determined completely by other settings
#define minstep (200000 / polepairs / holdrpm)

// determined completely by other settings
#define rampuprate (startuprpmpersecond * polepairs)

#define currentlimitenable
#define overcurrentenable

#define ontimesampleenable
