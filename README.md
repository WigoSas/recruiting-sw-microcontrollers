## Project for E-Agle recruiting

Programs used: 
- PuTTY and Cutecom for sending commands
- BetterSerialPlotter to plot data. Data comes in the format `"<analog> <digital>\n"`

### What works now?
- CLI works, by inserting `raw`, `moving average` or `random noise` the program will accept the program and starts to send the data. Every other string will not work and the program will listen for a new command.

- Data messaging, it is possible to receive data about the hall sensor

### To do

- Implementing the pause, warning and error states and their transitions
- Implementing the time peripheral and its usage