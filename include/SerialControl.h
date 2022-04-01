#pragma once

#include "./serialport.h"
#include "./wt61pc.h"
#include "./Debug.h"

class SerialControl
{
public:
    SerialControl();
    void ReceiveControl();
    void SendControl();
};
