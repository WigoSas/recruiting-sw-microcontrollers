## Project for E-Agle recruiting

Programs used: 
- PuTTY and Cutecom for sending commands
- BetterSerialPlotter to plot data. Data comes in the format `"<analog> <digital>\n"`

### What works now?
- CLI works, by inserting `raw`, `moving average` or `random noise` the program will accept the command and pressing the button will make it send the data. Every other string will not work and the program will listen for a new command. When pressing button again in listening state, the filter will be reset, if a new command is not sent as input you will get an error

- Data messaging, it is possible to receive data about the hall sensor filtered by the selected filter mode

- Led works except as expected, as well as the 5 sec rule

- An additional executable compiled from audio_modulation uses the miniaudio library. A sine wave is produced from the analog values, ranging an octave. Noise's amplitude comes instead from the digital value. To correctly open the program specify the serial port used. E.g. `./audio.out ttyACM0`, and to correctly close it, use Ctrl-C.

The default behaviour of the audio modulator (if there aren't any bugs) is the following:

- In state LISTENING it will change sound according to the values received.

- In states WAIT REQUEST and PAUSE it will continue to use the last sound configuration (due to the fact that the read is blocking and the failure to read counter doesn't get updated)

- In states WARNING and ERROR after 10 unparsable messages sound will stop

- In state INIT it's undefined (the MCU will not be stuck it that state anyway)

#### ADDITION TO project
- The program will work with either a hall sensor or a potentiometer, and the default setting is hall. You can switch from one mode to the other by typing `hall` or `potentiometer` in the CLI. The default is hall. The difference between the two modes is that, since potentiometers don't have a digital output, the MC will simulate it using the analog value with a threshold of 2047.

### to do
- more thorough testing