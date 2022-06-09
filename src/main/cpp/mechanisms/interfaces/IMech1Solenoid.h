
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

//========================================================================================================
/// @interface IMech1IndSolenoid
/// @brief     This is the interface for mechanisms that have one independently controlled solenoid.
//========================================================================================================

// C++ Includes

// FRC includes

// Team 302 includes
#include <mechanisms/controllers/ControlModes.h>
#include <mechanisms/interfaces/IMech.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/controllers/ControlData.h>
// Third Party Includes


///	 @interface IMech1Solenoid : IMech
///  @brief	    Interface for subsystems
class IMech1Solenoid : public IMech
{
	public:
        /// @brief      Activate/deactivate pneumatic solenoid
        /// @param [in] bool - true == extend, false == retract
        /// @return     void 
        virtual void ActivateSolenoid
        (
            bool     activate
        ) = 0;

        /// @brief      Check if the pneumatic solenoid is activated
        /// @return     bool - true == extended, false == retract
        virtual bool IsSolenoidActivated() const = 0;
        
        IMech1Solenoid() = default;
	    virtual ~IMech1Solenoid() = default;
};



