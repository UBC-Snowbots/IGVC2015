////////////////////////////////////////////////////////////////////////////////////
///
///  \file ReportRangeSensorConfiguration.h
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Daniel Barber
///  Created: 8 March 2010
///  Copyright (c) 2010
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: dbarber@ist.ucf.edu
///  <br>Web:  http://active.ist.ucf.edu
///
///  Redistribution and use in source and binary forms, with or without
///  modification, are permitted provided that the following conditions are met:
///      * Redistributions of source code must retain the above copyright
///        notice, this list of conditions and the following disclaimer.
///      * Redistributions in binary form must reproduce the above copyright
///        notice, this list of conditions and the following disclaimer in the
///        documentation and/or other materials provided with the distribution.
///      * Neither the name of the ACTIVE LAB, IST, UCF, nor the
///        names of its contributors may be used to endorse or promote products
///        derived from this software without specific prior written permission.
/// 
///  THIS SOFTWARE IS PROVIDED BY THE ACTIVE LAB''AS IS'' AND ANY
///  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
///  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
///  DISCLAIMED. IN NO EVENT SHALL UCF BE LIABLE FOR ANY
///  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
///  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
///  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
///  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
///  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
///  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef __JAUS_EXTRAS_RANGE_SENSOR_REPORT_RANGE_SENSOR_CONFIGURATION__H
#define __JAUS_EXTRAS_RANGE_SENSOR_REPORT_RANGE_SENSOR_CONFIGURATION__H

#ifndef JPP_IGNORE_DEPRECATED
#pragma message("JAUS++ EXTRAS DEPRECATED - Use New Range Sensor in Environment Library")
#endif

#include "jaus/extras/ExtrasCodes.h"
#include "jaus/core/Message.h"
#include "jaus/extras/rangesensor/RangeSensorConfig.h"

namespace JAUS
{
    const UShort REPORT_RANGE_SENSOR_CONFIGURATION_EXTRAS                =   0xD90A; ///<  Message code.
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class ReportRangeSensorConfiguration
    ///   \brief This message is used to report the configuration information for
    ///          range sensors located on a component.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_EXTRAS_DLL ReportRangeSensorConfiguration : public Message
    {
    public:
        ReportRangeSensorConfiguration(const Address& dest = Address(), 
                                       const Address& src = Address()) : Message(REPORT_RANGE_SENSOR_CONFIGURATION_EXTRAS, dest, src)
        {
        }
        ReportRangeSensorConfiguration(const ReportRangeSensorConfiguration& message) : Message(REPORT_RANGE_SENSOR_CONFIGURATION_EXTRAS)
        {
            *this = message;
        }
        ~ReportRangeSensorConfiguration() {}  
        RangeSensorConfig::List* GetConfiguration() { return &mConfiguration; }
        const RangeSensorConfig::List* GetConfiguration() const { return &mConfiguration; }
        virtual bool IsCommand() const { return false; }
        virtual int WriteMessageBody(Packet& packet) const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual Message* Clone() const { return new ReportRangeSensorConfiguration(*this); }
        virtual UInt GetPresenceVector() const { return 0; }
        virtual UInt GetPresenceVectorSize() const { return 0; }
        virtual UInt GetPresenceVectorMask() const { return 0; }
        virtual UShort GetMessageCodeOfResponse() const { return 0; }
        virtual std::string GetMessageName() const { return "Report Range Sensor Configuration"; }
        virtual void ClearMessageBody() {}
        virtual bool IsLargeDataSet(const unsigned int maxPayloadSize) const;
        ReportRangeSensorConfiguration& operator=(const ReportRangeSensorConfiguration& message)
        {
            CopyHeaderData(&message);
            mConfiguration = message.mConfiguration;
            return *this;
        }
    protected:
        RangeSensorConfig::List mConfiguration; ///<  Sensor configuration info.
    };
}

#endif
/*  End of File */