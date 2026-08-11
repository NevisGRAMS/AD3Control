//
// Created by Jon Sensenig on 2/23/26.
//
#include "WF_SDK/WF_SDK.h"
#include "digilent/waveforms/dwf.h"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <unistd.h> // for sleep function

using namespace wf;


/* ----------------------------------------------------- */
namespace pps_ctrl {

    bool or_func(int iAddress);
    void configure_rom(int chan, bool (*func)(int));

    // Custom waveform, each element is a tick of the 10MHz clock
    // so represents 100ns.
    size_t NUM_10MHZ_SAMPLES = 10000; // each sample equivalent to 1ms
    std::vector<double> pulse(NUM_10MHZ_SAMPLES, 0);
    // AD3 device
    Device::Data *device_data;


    void GetError() {
        char err_msg[512];  // variable for the error message
        FDwfGetLastErrorMsg(err_msg);
        auto error_string = std::string(err_msg);
        std::cerr << "Error: " << error_string << std::endl;
    }


    void configure_rom(int chan, bool (*func)(int)) {
        unsigned int customSize = 0;
        int is_error = -1;
        is_error = FDwfDigitalOutDataInfo(device_data->handle, chan, &customSize);
        if (is_error == 0) GetError();

        std::cout << "Custom size: " << customSize
                  << " Address space: " << (int)log2(customSize) << std::endl;

        int bufferSizeBytes = customSize / 8;
        std::vector<uint8_t> rgbSamples(bufferSizeBytes, 0);

        for (unsigned int iAddress = 0; iAddress < customSize; ++iAddress)
        {
            if (func(iAddress))
            {
                rgbSamples[iAddress / 8] |= (1 << (iAddress % 8));
            }
        }

        for (int i = 0; i < 3; ++i)
        {
            is_error *= FDwfDigitalOutEnableSet(device_data->handle, 0, 0);
        }
        if (is_error == 0) GetError();

        is_error = FDwfDigitalOutEnableSet(device_data->handle, chan, 1); // enable output
        if (is_error == 0) GetError();
        is_error = FDwfDigitalOutTypeSet(device_data->handle, chan, DwfDigitalOutTypeROM);
        if (is_error == 0) GetError();
        is_error = FDwfDigitalOutDividerSet(device_data->handle, chan, 1); // set minimum delay
        if (is_error == 0) GetError();
        is_error = FDwfDigitalOutDataSet(device_data->handle, chan, rgbSamples.data(), customSize);
        if (is_error == 0) GetError();
    }

    bool or_func(int iAddress) {
        int fDio2 = (iAddress >> 2) & 1;
        int fDio1 = (iAddress >> 1) & 1;
        return (fDio2 == 1 || fDio1 == 1);
    }

    void InitDevice() {
        // Set up power supply
        int is_error = -1;
        is_error = FDwfAnalogIOReset(device_data->handle);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogIOChannelNodeSet(device_data->handle, 0, 0, 1); // turn on ps
        if (is_error == 0) GetError();
        is_error = FDwfAnalogIOChannelNodeSet(device_data->handle, 0, 1, 3.3); // set ps voltage to 3.3V
        if (is_error == 0) GetError();
        is_error = FDwfAnalogIOEnableSet(device_data->handle, true); // master enable
        if (is_error == 0) GetError();
        sleep(1); // wait a second for the power supply voltage to come up (possibly unnecessary)
    }

    void StopPps() {
        int is_error = -1;
        is_error = FDwfAnalogOutNodeEnableSet(device_data->handle, 0, AnalogOutNodeCarrier, false);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutConfigure(device_data->handle, 0, false);
        if (is_error == 0) GetError();
    }

    void StopPulseTrain() {
        int is_error = -1;
        is_error = FDwfAnalogOutNodeEnableSet(device_data->handle, 1, AnalogOutNodeCarrier, false);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutConfigure(device_data->handle, 1, false);
        if (is_error == 0) GetError();
    }

    void StartPps() {
        int is_error = -1;
        is_error = FDwfAnalogOutNodeEnableSet(device_data->handle, 0, AnalogOutNodeCarrier, true);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeFunctionSet(device_data->handle, 0, AnalogOutNodeCarrier, funcCustom);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeFrequencySet(device_data->handle, 0, AnalogOutNodeCarrier, 1000.0); // [Hz]
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeAmplitudeSet(device_data->handle, 0, AnalogOutNodeCarrier, 3.3); // 3.3V pulse amplitude
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeOffsetSet(device_data->handle, 0, AnalogOutNodeCarrier, 0.0);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodePhaseSet(device_data->handle, 0, AnalogOutNodeCarrier, 0.0);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeDataSet(device_data->handle, 0, AnalogOutNodeCarrier, pulse.data(), pulse.size()); // pulse, 100ns width
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutModeSet(device_data->handle, 0, DwfAnalogOutModeVoltage);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutIdleSet(device_data->handle, 0, DwfAnalogOutIdleOffset);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutRunSet(device_data->handle, 0, 0.001); // run for 1ms, one period
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutWaitSet(device_data->handle, 0, 0.0); // no delay after receiving PPS
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutRepeatSet(device_data->handle, 0, 0);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutTriggerSourceSet(device_data->handle, 0, trigsrcExternal1); // triggered on GPS PPS
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutTriggerSlopeSet(device_data->handle, 0, DwfTriggerSlopeRise);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutRepeatTriggerSet(device_data->handle, 0, true); // repeat for every received GPS PPS
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutConfigure(device_data->handle, 0, true);
        if (is_error == 0) GetError();

        // FIXME Is this the right place to call it??
        // OR
        // Need this to combine the PPS and pulse train onto same output pin
        configure_rom(15, or_func);
        is_error = FDwfDigitalOutConfigure(device_data->handle, 1);
        if (is_error == 0) GetError();

    }

    void RunPulseTrain() {
        int is_error = -1;
        is_error = FDwfAnalogOutNodeEnableSet(device_data->handle, 1, AnalogOutNodeCarrier, true);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeFunctionSet(device_data->handle, 1, AnalogOutNodeCarrier, funcCustom);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeFrequencySet(device_data->handle, 1, AnalogOutNodeCarrier, 1000.0); // [Hz]
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeAmplitudeSet(device_data->handle, 1, AnalogOutNodeCarrier, 3.3); // 3.3V amplitude pulse
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeOffsetSet(device_data->handle, 1, AnalogOutNodeCarrier, 0.0);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodePhaseSet(device_data->handle, 1, AnalogOutNodeCarrier, 0.0);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutNodeDataSet(device_data->handle, 1, AnalogOutNodeCarrier, pulse.data(), pulse.size());
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutModeSet(device_data->handle, 1, DwfAnalogOutModeVoltage);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutIdleSet(device_data->handle, 1, DwfAnalogOutIdleOffset);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutRunSet(device_data->handle, 1, 0.1); // run for 100ms = 100 pulses
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutWaitSet(device_data->handle, 1, 0.01); // wait 10ms after trigger to send pulse
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutRepeatSet(device_data->handle, 1, 1); // only send the pulse once
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutTriggerSourceSet(device_data->handle, 1, trigsrcExternal1); // trigger pulse train on PPS
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutTriggerSlopeSet(device_data->handle, 1, DwfTriggerSlopeRise);
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutRepeatTriggerSet(device_data->handle, 1, false); // only send the pulse train once
        if (is_error == 0) GetError();
        is_error = FDwfAnalogOutConfigure(device_data->handle, 1, true); // configure & arm
        if (is_error == 0) GetError();

    }

}


int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cerr << "Wrong number of args! Need 1 but got " << argc-1 << std::endl;
        std::cerr << "1 = Init AD3" << std::endl;
        std::cerr << "2 = Start PPS" << std::endl;
        std::cerr << "3 = Run Pulse Train" << std::endl;
        std::cerr << "4 = Stop PPS" << std::endl;
        return 1;
    }
    int operation = std::stoi(argv[1]);

    // Set one sample high, this is 1 pulse 100ns wide
    pps_ctrl::pulse.at(9) = 1;

    // To keep the AD3 running even after the program ends we need to set the
    // DwfParamOnClose parameter to 0
    FDwfParamSet(DwfParamOnClose, 0); // 0 = run, 1 = stop, 2 = shutdown
    // The AD3 device, open it
    pps_ctrl::device_data = device.open("Analog Discovery 3");
    if (pps_ctrl::device_data == nullptr) {
        std::cerr << "Failed to open device.." << std::endl;
        return 3;
    }

    switch (operation) {
        case 1: { // Power on AD3
            pps_ctrl::InitDevice();
            std::cout << "Powered on AD3.." << std::endl;
            break;
        }
        case 2: { // Configure/Start PPS
            pps_ctrl::StartPps();
            std::cout << "Started PPS.." << std::endl;
            break;
       } 
       case 3: { // Configure/Run Pulse Train
            pps_ctrl::RunPulseTrain();
            std::cout << "Sent Pulse Train.." << std::endl;
            break;
       } 
       case 4: { // Stop PPS
            pps_ctrl::StopPps();                      
            pps_ctrl::StopPulseTrain();                      
            std::cout << "Stopped PPS.." << std::endl;
            break;
       }
       default: {
            std::cerr << "Wrong argument! Need one of the following:" << std::endl;
            std::cerr << "1 = Init AD3" << std::endl;
            std::cerr << "2 = Start PPS" << std::endl;
            std::cerr << "3 = Run Pulse Train" << std::endl;
            std::cerr << "4 = Stop PPS" << std::endl;
            break;
       }

   }

   sleep(1);

   // Close the device to finish
   device.close(pps_ctrl::device_data);

 }
