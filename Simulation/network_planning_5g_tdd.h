#ifndef NETWORK_PLANNING_5G_TDD__H
#define NETWORK_PLANNING_5G_TDD__H

#include "../Interface/OutdoorPlugIn.h"
#include "../Net/Net.h"
#include "../Interface/Init.h"

// Callback functions
int _STD_CALL CallbackMessage(const char *Text);
int _STD_CALL CallbackProgress(int value, const char* text);
int _STD_CALL CallbackError(const char *Message, int Mode);

// Helper functions
void write_ascii(
    const WinProp_Result *Resultmatrix,
    const char* Filename
);

void AntennaPropertiesSet(
    WinProp_Antenna* Antenna,
    double CoordinateX,
    double CoordinateY,
    double Height,
    double Frequency,
    char* Name,
    double Power,
    int PowerMode,
    int Model,
    double Azimuth,
    double Downtilt,
    int Id
);

void CarrierPropertiesSet(
    WinProp_Antenna* Antenna,
    const WinProp_Carrier* Carrier
);

#endif // NETWORK_PLANNING_5G_TDD__H