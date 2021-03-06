

#define b0 1
#define b1 2
#define b2 4
#define b3 8
#define b4 16
#define b5 32
#define b6 64
#define b7 128
#define b8 256
#define b9 512
#define b10 1024
#define b11 2048
#define b12 4096
#define b13 8192
#define b14 16384
#define b15 32768
#define b16 (1 << 16)
#define b17 (1 << 17)
#define b18 (1 << 18)
#define b19 (1 << 19)
#define b20 (1 << 20)
#define b21 (1 << 21)
#define b22 (1 << 22)
#define b23 (1 << 23)
#define b24 (1 << 24)
#define b25 (1 << 25)
#define b26 (1 << 26)
#define b27 (1 << 27)
#define b28 (1 << 28)
#define b29 (1 << 29)
#define b30 (1 << 30)
#define b31 (unsigned int)(1 << 31)

//#define setmark GPIOA->ODR |=  b3
//#define clearmark GPIOA->ODR &=  ~b3
//#define togglemark GPIOA->ODR ^= b3

#define setmark GPIOB->ODR |= b11
#define clearmark GPIOB->ODR &= ~b11
#define togglemark GPIOB->ODR ^= b11

#define ledon GPIOC->ODR &= ~b13;
#define ledoff GPIOC->ODR |= b13;

#define phaseaenable GPIOB->ODR |= b13
#define phasebenable GPIOB->ODR |= b14
#define phasecenable GPIOB->ODR |= b15
#define phaseadisable GPIOB->ODR &= ~b13
#define phasebdisable GPIOB->ODR &= ~b14
#define phasecdisable GPIOB->ODR &= ~b15
#define phaseahigh GPIOA->ODR |= b8
#define phasebhigh GPIOA->ODR |= b9
#define phasechigh GPIOA->ODR |= b10
#define phasealow GPIOA->ODR &= ~b8
#define phaseblow GPIOA->ODR &= ~b9
#define phaseclow GPIOA->ODR &= ~b10
