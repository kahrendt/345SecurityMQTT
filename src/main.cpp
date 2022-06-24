#include "digitalDecoder.h"
#include "analogDecoder.h"
#include "mqtt.h"
#include "mqtt_config.h"

#include <rtl-sdr.h>

#include <iostream>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <sys/time.h>
#include <cstdlib>
#include <string>
#include <yaml-cpp/yaml.h>

// TODO: MQTT Will doesn't seem to be working with HA as expected

float magLut[0x10000];

// void alarmHandler(int signal)
// {
//     dDecoder.setRxGood(false);
// }

void usage(const char *argv0)
{
    std::cout << "Usage: " << std::endl
        << argv0 << " [-d <device-id>] [-f <frequency in Hz]" << std::endl;
}

int main(int argc, char ** argv)
{
    YAML::Node config = YAML::LoadFile("/config/settings.yaml");

    const char *mqttHost = "127.0.0.1";
    if (config["mqtt_host"]) {
        mqttHost = const_cast<char*>(config["mqtt_host"].as<std::string>().c_str());
    }

    int mqttPort = 1883;
    if (config["mqtt_port"]) {
        mqttPort = std::stoi(config["mqtt_port"].as<std::string>());
    }

    const char *mqttUsername = "";
    if (config["mqtt_username"]) {
        mqttUsername = const_cast<char*>(config["mqtt_username"].as<std::string>().c_str());
    }

    const char *mqttPassword = "";
    if (config["mqtt_password"]) {
        mqttPassword = const_cast<char*>(config["mqtt_password"].as<std::string>().c_str());
    }

    bool automaticGain = true;
    if (config["automatic_gain"]) {
        automaticGain = config["automatic_gain"].as<bool>();
    }

    int gainValue = 490;
    if (!automaticGain && config["gain_value"]) {
        gainValue = std::stoi(config["gain_value"].as<std::string>());
    }
    else if (!automaticGain) {
        std::cout << "Automatic gain and specific gain not set in YAML, using default specific gain of 490" << std::endl;
    }

    int devId = 0;
    if (config["device_id"]) {
        devId = std::stoi(config["device_id"].as<std::string>());
    }

    int freq = 345000000;
    if (config["frequency"]) {
        freq = std::stoi(config["frequency"].as<std::string>());
    }

    
    Mqtt mqtt = Mqtt("sensors345", mqttHost, mqttPort, mqttUsername, mqttPassword, "security/sensors345/rx_status", "FAILED");
    DigitalDecoder dDecoder = DigitalDecoder(mqtt);
    AnalogDecoder aDecoder;
    
    //int devId = 0;
    //int freq = 345000000;
    signed char c;
    while ((c = getopt(argc, argv, "hd:f:")) != -1)
    {
        switch(c)
        {
            case 'h':
            {
                usage(argv[0]);
                exit(0);
            }
            case 'd':
            {
                devId = atoi(optarg);
                break;
            }
            case 'f':
            {
                freq = atoi(optarg);
                break;
            }
            default: // including '?' unknown character
            {
                std::cerr << "Unknown flag '" << c << std::endl;
                usage(argv[0]);
                exit(1);
            }
        }
    }
    
    //
    // Open the device
    //
    if(rtlsdr_get_device_count() < 1)
    {
        std::cout << "Could not find any devices" << std::endl;
        return -1;
    }
        
    rtlsdr_dev_t *dev = nullptr;
        
    if(rtlsdr_open(&dev, devId) < 0)
    {
        std::cout << "Failed to open device" << std::endl;
        return -1;
    }
    
    //
    // Set the frequency
    //
    if(rtlsdr_set_center_freq(dev, freq) < 0)
    {
        std::cout << "Failed to set frequency" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully set the frequency to " << rtlsdr_get_center_freq(dev) << std::endl;
    
    //
    // Set the gain
    //
    if (automaticGain) {
        if(rtlsdr_set_tuner_gain_mode(dev, 0) < 0)
        {
            std::cout << "Failed to set gain mode" << std::endl;
            return -1;
        }

        std::cout << "Successfully set gain to automatic" << std::endl;
    }
    else {
        if(rtlsdr_set_tuner_gain_mode(dev, 1) < 0)
        {
            std::cout << "Failed to set gain mode" << std::endl;
            return -1;
        }
        
        if(rtlsdr_set_tuner_gain(dev, gainValue) < 0)
        {
            std::cout << "Failed to set gain" << std::endl;
            return -1;
        }

        std::cout << "Successfully set gain to " << rtlsdr_get_tuner_gain(dev) << std::endl;
    }
    

    
    //
    // Set the sample rate
    //
    if(rtlsdr_set_sample_rate(dev, 1000000) < 0)
    {
        std::cout << "Failed to set sample rate" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully set the sample rate to " << rtlsdr_get_sample_rate(dev) << std::endl;
    
    //
    // Prepare for streaming
    //
    rtlsdr_reset_buffer(dev);
    
    for(uint32_t ii = 0; ii < 0x10000; ++ii)
    {
        uint8_t real_i = ii & 0xFF;
        uint8_t imag_i = ii >> 8;
        
        float real = (((float)real_i) - 127.4) * (1.0f/128.0f);
        float imag = (((float)imag_i) - 127.4) * (1.0f/128.0f);
        
        float mag = std::sqrt(real*real + imag*imag);
        magLut[ii] = mag;
    }
    
    //
    // Common Receive
    //
    
    aDecoder.setCallback([&](char data){dDecoder.handleData(data);});
    
    //
    // Async Receive
    //
    
    typedef void(*rtlsdr_read_async_cb_t)(unsigned char *buf, uint32_t len, void *ctx);
    
    auto cb = [](unsigned char *buf, uint32_t len, void *ctx)
    {
        AnalogDecoder *adec = (AnalogDecoder *)ctx;
        
        int n_samples = len/2;
        for(int i = 0; i < n_samples; ++i)
        {
            float mag = magLut[*((uint16_t*)(buf + i*2))];
            adec->handleMagnitude(mag);
        }
    };

    // Setup watchdog to check for a common-mode failure (e.g. antenna disconnection)
    //std::signal(SIGALRM, alarmHandler);
  
    // Initialize RX state to good
    dDecoder.setRxGood(true);
    const int err = rtlsdr_read_async(dev, cb, &aDecoder, 0, 0);
    std::cout << "Read Async returned " << err << std::endl;
   
/*    
    //
    // Synchronous Receive
    //
    static const size_t BUF_SIZE = 1024*256;
    uint8_t buffer[BUF_SIZE];
    
    while(true)
    {
        int n_read = 0;
        if(rtlsdr_read_sync(dev, buffer, BUF_SIZE, &n_read) < 0)
        {
            std::cout << "Failed to read from device" << std::endl;
            return -1;
        }
        
        int n_samples = n_read/2;
        for(int i = 0; i < n_samples; ++i)
        {
            float mag = magLut[*((uint16_t*)(buffer + i*2))];
            aDecoder.handleMagnitude(mag);
        }
    }
*/    
    //
    // Shut down
    //
    rtlsdr_close(dev);
    return 0;
}




