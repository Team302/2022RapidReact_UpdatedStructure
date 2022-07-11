
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <basemechanisms/interfaces/IState.h>
#include <basemechanisms/MechSolenoidState.h>
#include <basemechanisms/interfaces/IMech1Solenoid.h>
#include <utils/Logger.h>

#include <TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class MechSolenoidState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
MechSolenoidState::MechSolenoidState
(
    IMech1Solenoid*                 mechanism,
    MechanismTargetData::SOLENOID   solState
) : IState(),
    m_mechanism( mechanism ),
    m_solenoidState( solState )
{
    if ( mechanism == nullptr )
    {
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("MechSolenoidState"), string("MechSolenoidState"), string("no mechanism"));
    }    
}

void MechSolenoidState::Init()
{
}


void MechSolenoidState::Run()           
{
    if ( m_mechanism != nullptr )
    {
        switch ( m_solenoidState )
        {
            case MechanismTargetData::SOLENOID::REVERSE:
                m_mechanism->ActivateSolenoid( false );
                break;
            
            case MechanismTargetData::SOLENOID::ON:
                m_mechanism->ActivateSolenoid( true );
                break;

            default:
                break;
        }   
    }
}

bool MechSolenoidState::AtTarget() const
{
    return true;
}

