#include "digitalDecoder.h"
#include "analogDecoder.h"
#include "mqtt.h"
#include "mqtt_config.h"

#include <rtl-sdr.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <sys/time.h>
#include <cstdlib>
#include <string>


// TODO: MQTT Will doesn't seem to be working with HA as expected

float magLut[0x10000];

// void alarmHandler(int signal)
// {
//     dDecoder.setRxGood(false);
// }

void usage(const char *argv0)
{
    std::cout << "Usage: " << std::endl
        << argv0 << " [-c <path of configuration file>]" << std::endl;
}

int main(int argc, char ** argv)
{
    const char *configFile = "settings.yaml";

    signed char c;
    while ((c = getopt(argc, argv, "hc:")) != -1)
    {
        switch(c)
        {
            case 'h':
            {
                usage(argv[0]);
                exit(0);
            }
            case 'c':
            {
                configFile = optarg;
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
    
    YAML::Node config = YAML::Load("");

    std::ifstream configurationFile;
    configurationFile.open(configFile);
    if (!configurationFile) {
        std::cout << "Config file not given/found, using default settings" << std::endl;
    }
    else {
        config = YAML::LoadFile(configFile);
    }

    const char *mqttHost = "127.0.0.1";
    int mqttPort = 1883;
    const char *mqttUsername = "";
    const char *mqttPassword = "";

    if (config["mqtt"]) {
        YAML::Node mqttSettings = config["mqtt"];
        if (mqttSettings["mqtt_host"]) {
            mqttHost = const_cast<char*>(config["mqtt"]["mqtt_host"].as<std::string>().c_str());
        }

        if (mqttSettings["mqtt_port"]) {
            mqttPort = std::stoi(config["mqtt"]["mqtt_port"].as<std::string>());
        }


        if (mqttSettings["mqtt_username"]) {
            mqttUsername = const_cast<char*>(config["mqtt"]["mqtt_username"].as<std::string>().c_str());
        }


        if (mqttSettings["mqtt_password"]) {
            mqttPassword = const_cast<char*>(config["mqtt"]["mqtt_password"].as<std::string>().c_str());
        }
    }

    bool automaticGain = true;
    int gainValue = 490;

    int devId = 0;
    int freq = 345000000;
    int sampleRate = 1000000;

    if (config["sdr"]) {
        YAML::Node sdrSettings = config["sdr"];
        if (sdrSettings["automatic_gain"]) {
            automaticGain = config["sdr"]["automatic_gain"].as<bool>();
        }

        if (!automaticGain && sdrSettings["gain_value"]) {
            gainValue = std::stoi(sdrSettings["gain_value"].as<std::string>());
        }
        else if (!automaticGain) {
            std::cout << "Automatic gain and specific gain not set in YAML, using default specific gain of 490" << std::endl;
        }

        if (sdrSettings["device_id"]) {
            devId = std::stoi(config["sdr"]["device_id"].as<std::string>());
        }

        if (sdrSettings["frequency"]) {
            freq = std::stoi(sdrSettings["frequency"].as<std::string>());
        }

        if (sdrSettings["sample_rate"]) {
            sampleRate = std::stoi(sdrSettings["sample_rate"].as<std::string>());
        }
    }

    
    Mqtt mqtt = Mqtt("sensors345", mqttHost, mqttPort, mqttUsername, mqttPassword, "security/sensors345/rx_status", "FAILED");
    DigitalDecoder dDecoder = DigitalDecoder(mqtt);
    AnalogDecoder aDecoder;
    

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
    if(rtlsdr_set_sample_rate(dev, sampleRate) < 0)
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