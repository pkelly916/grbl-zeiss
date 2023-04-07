## GRBL Zeiss
 
***

This project was created to allow [pyuscope](https://github.com/Labsmore/pyuscope) to be used with a Zeiss Axiotron. The stepper motor planner and controller logic has been replaced with a CAN message handler that simply passes off the speed and coordinate information to the Zeiss stage. 

*** 

### Hardware Requirements

***

- Zeiss Axiotron with CAN stage controller  
- Arduino Uno 
- MCP2515 CAN shield 

*** 

### Disclaimer 

***

This version is **NOT** recommended for general G-code CNC operation as it has been tuned directly for controlling a Zeiss Axiotron. The CAN commands may apply to other Zeiss scope stages but this has not been tested. 

***

### GRBL Fork

*** 

Based on [GRBL v1.1](https://github.com/grbl/grbl). 

