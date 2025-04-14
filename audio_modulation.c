#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <signal.h>
#include <stdbool.h>


#define DEVICE_FORMAT       ma_format_f32
#define DEVICE_CHANNELS     2
#define DEVICE_SAMPLE_RATE  48000

int gl_analog = 0;
int gl_digital = 0;

bool exitLoop = false;

void sighandler(int sig){
    exitLoop = true;
}

double serialToSine(int analog){
    double ratio = (double) analog / 4095;
    return ratio * 220 + 220; //from A to A, an octave
}

double serialToNoise(int digital){
    double ratio = (double) digital / 100;
    return ratio * 0.1; //from 0.1 to 0
}

// returns true when it can be parsed as received values, otherwise it's a string
bool parseString(char* buf, int dim){
    int analog = 0, digital = 0;
    if(buf[0]=='\t') return false;
    bool isFirst = true;
    dim--;

    for(int i=0; i<dim; i++){

        if(buf[i] >= '0' && buf[i] <= '9'){
            int* turn = isFirst ? &analog : &digital;
            *turn *= 10;
            *turn += buf[i]-'0';
        } 
        else if(buf[i]=='\t' && isFirst) isFirst = false;
        else {
            return false;
        }
    }
    gl_analog = analog;
    gl_digital = digital;
    return true;
}

void sine_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount)
{
    ma_waveform* pWave;

    pWave = (ma_waveform*)pDevice->pUserData;
    
    ma_waveform_set_frequency(pWave,serialToSine(gl_analog));
    ma_data_source_read_pcm_frames(pWave,pOutput,frameCount,NULL);

}

void noise_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount)
{
    ma_noise* pWave;

    pWave = (ma_noise*)pDevice->pUserData;

    ma_noise_set_amplitude(pWave,serialToNoise(gl_digital));
    ma_data_source_read_pcm_frames(pWave,pOutput,frameCount,NULL);

}

int main(int argc, char** argv){

    if(argc < 2){
        printf("Too few arguments... add serial port name without address\n");
        return -1;
    }

    char buf[100];

    strncpy(buf,"/dev/",100);

    strncat(buf,argv[1],90);

    int serial_port = open(buf, O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;


    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 0;    // Wait for up to 1 decisecond, returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;     // It doesn't work until it is set to non canonical mode
    cfsetispeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    signal(SIGINT,sighandler);
    for(int i=0;i<100;i++) buf[i] = 0;

    ///////////////////////////////////////////////////////
    //configuration of the sine wave, which depends on the analog values

    ma_waveform sineWave;
    ma_device_config sineDeviceConfig;
    ma_device sineDevice;
    ma_waveform_config sineWaveConfig;

    sineDeviceConfig = ma_device_config_init(ma_device_type_playback);
    sineDeviceConfig.playback.format   = DEVICE_FORMAT;
    sineDeviceConfig.playback.channels = DEVICE_CHANNELS;
    sineDeviceConfig.sampleRate        = DEVICE_SAMPLE_RATE;
    sineDeviceConfig.dataCallback      = sine_callback;
    sineDeviceConfig.pUserData         = &sineWave;

    if (ma_device_init(NULL, &sineDeviceConfig, &sineDevice) != MA_SUCCESS) {
        printf("Failed to open playback device for sine.\n");
        close(serial_port);
        return -4;
    }

    sineWaveConfig = ma_waveform_config_init(sineDevice.playback.format, sineDevice.playback.channels, sineDevice.sampleRate, ma_waveform_type_sine, 0.2, 220);
    if(ma_waveform_init(&sineWaveConfig, &sineWave) != MA_SUCCESS){
        printf("Failed to initialise sine.\n");
        close(serial_port);
        ma_device_uninit(&sineDevice);
        return -1; 
    }

    //////////////////////////////////////////////////////////////////////////
    //configuration for the noise, which depends on digital values

    ma_noise noiseWave;
    ma_device_config noiseDeviceConfig;
    ma_device noiseDevice;
    ma_noise_config noiseWaveConfig;

    noiseDeviceConfig = ma_device_config_init(ma_device_type_playback);
    noiseDeviceConfig.playback.format   = DEVICE_FORMAT;
    noiseDeviceConfig.playback.channels = DEVICE_CHANNELS;
    noiseDeviceConfig.sampleRate        = DEVICE_SAMPLE_RATE;
    noiseDeviceConfig.dataCallback      = noise_callback;
    noiseDeviceConfig.pUserData         = &noiseWave;

    if (ma_device_init(NULL, &noiseDeviceConfig, &noiseDevice) != MA_SUCCESS) {
        printf("Failed to open playback device for sine.\n");
        close(serial_port);
        ma_device_uninit(&sineDevice);
        ma_waveform_uninit(&sineWave);
        return -4;
    }

    noiseWaveConfig = ma_noise_config_init(DEVICE_FORMAT,DEVICE_CHANNELS,ma_noise_type_white,0,0.1);
    
    if (ma_noise_init(&noiseWaveConfig, NULL, &noiseWave) != MA_SUCCESS) {
        printf("Failed to initialise noise.\n");
        close(serial_port);
        ma_device_uninit(&sineDevice);
        ma_device_uninit(&noiseDevice);
        ma_waveform_uninit(&sineWave);
        return -1;
    }

    /////////////////////////////////////////////////////////////////////////

    if (ma_device_start(&sineDevice) != MA_SUCCESS) {
        printf("Failed to start playback device.\n");
        close(serial_port);
        ma_device_uninit(&sineDevice);
        ma_device_uninit(&noiseDevice);
        ma_waveform_uninit(&sineWave);
        ma_noise_uninit(&noiseWave,NULL);
        return -5;
    }

    if (ma_device_start(&noiseDevice) != MA_SUCCESS) {
        printf("Failed to start playback device.\n");
        close(serial_port);
        ma_device_uninit(&sineDevice);
        ma_device_uninit(&noiseDevice);
        ma_waveform_uninit(&sineWave);
        ma_noise_uninit(&noiseWave,NULL);
        return -5;
    }

    int failedRead = 0;
    printf("Now entering loop, to exit press Ctrl-C\n");

    while(!exitLoop){
        
        int n = read(serial_port, &buf, sizeof(buf));
        if(n>1){
            buf[n-1] = '\0';
        }
        bool isParsed = parseString(buf,n);
        for(int i=0;i<n;i++) buf[i] = 0;

        int previousFR = failedRead;

        if(n==0 || !isParsed) {failedRead++;}
        else if(n==-1){
            printf("Something went wrong reading from serial");
            break;
        } else {failedRead = 0;}

        //arbitrary number of failures
        if(failedRead==10){
            ma_waveform_set_amplitude(&sineWave,0); // so we make sine silent
            //ma_noise_set_amplitude(&noiseWave,0);
            gl_digital = 0; // so we make noise silent
        }
        else if(failedRead == 0 && previousFR > 9) ma_waveform_set_amplitude(&sineWave,0.2);
        else if(failedRead > 10) failedRead--;

    }
    printf("Exiting the loop...\n");

    
    close(serial_port);
    ma_device_uninit(&sineDevice);
    ma_device_uninit(&noiseDevice);
    ma_waveform_uninit(&sineWave);
    ma_noise_uninit(&noiseWave,NULL);

    return 0;
}