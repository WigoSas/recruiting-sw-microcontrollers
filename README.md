## Project for E-Agle recruiting

Programs used: 
- PuTTY and Cutecom for sending commands
- BetterSerialPlotter to plot data. Data comes in the format `"<analog> <digital>\n"`

### What works now?
- CLI works, by inserting `raw`, `moving average` or `random noise` the program will accept the command and pressing the button will make it send the data. Every other string will not work and the program will listen for a new command. When pressing button again in listening state, the filter will be reset, if a new command is not sent as input you will get an error

- Data messaging, it is possible to receive data about the hall sensor filtered by the selected filter mode

- Led works except as expected, as well as the 5 sec rule

### To do
- Implementing audio