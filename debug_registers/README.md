## About

Small program to debug sx127x registers. It will convert them from binary form into parsed tree with human-readable names.

## Build

```bash
mkdir build
cd build
cmake ..
make
```

## Run

First, obtain sx127x registers. It can be done using ```sx127x_dump_registers``` function.

```c
uint8_t registers[0x80];
sx127x_dump_registers(registers, device);
for (int i = 0; i < sizeof(registers); i++) {
  if (i != 0) {
    printf(",");
  }
  printf("0x%02x", registers[i]);
}
printf("\n");
```

Then convert to the human-readable form:

```
./debug_registers 0,9,26,11,0,82,108,128,0,79,9,43,32,8,2,10,255,0,21,11,40,12,18,71,50,62,0,0,0,0,0,64,0,0,0,0,5,0,3,147,85,85,85,85,85,85,85,85,144,64,64,0,0,15,0,0,0,245,32,130,244,2,128,64,0,0,18,36,45,0,3,0,4,35,0,9,5,132,50,43,20,0,0,14,0,0,0,15,224,0,12,243,8,0,92,120,0,25,12,75,204,15,1,32,4,71,175,63,221,0,26,11,208,1,17,0,0,0,0,0,0,0,0,0,0,0,0,0
```

The result will be:

```
0x01 RegOpMode: 
	LongRangeMode=FSK
	AccessSharedReg=Access LoRa registers
	LowFrequencyModeOn=Low Frequency Mode
	Mode=STDBY
0x02: RegBitrateMsb:
	BitRate=3283.735249
0x04: RegFdevMsb
	Fdev=7934.570312
0x06: RegFr:
	Frf=32625000
0x09: RegPaConfig:
	PaSelect=RFO pin
	MaxPower=0x2
	OutputPower=0x8
0x0a: RegPaRamp:
	ModulationShaping=No shaping
	PaRamp=0x9
0x0b: RegOcp:
	OcpOn=OCP disabled
	OcpTrim=0x3
...
```
