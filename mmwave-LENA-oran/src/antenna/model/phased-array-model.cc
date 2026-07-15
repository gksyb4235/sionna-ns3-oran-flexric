/*
 *   Copyright (c) 2020 University of Padova, Dep. of Information Engineering, SIGNET lab.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "phased-array-model.h"

#include "isotropic-antenna-model.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/pointer.h>
#include <ns3/uinteger.h>

namespace ns3
{

uint32_t PhasedArrayModel::m_idCounter = 0;

NS_LOG_COMPONENT_DEFINE("PhasedArrayModel");

NS_OBJECT_ENSURE_REGISTERED(PhasedArrayModel);

PhasedArrayModel::PhasedArrayModel()
    : m_isBfVectorValid{false}
{
    m_id = m_idCounter++;
}

PhasedArrayModel::~PhasedArrayModel()
{
}

TypeId
PhasedArrayModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::PhasedArrayModel")
            .SetParent<Object>()
            .SetGroupName("Antenna")
            .AddAttribute("AntennaElement",
                          "A pointer to the antenna element used by the phased array",
                          PointerValue(CreateObject<IsotropicAntennaModel>()),
                          MakePointerAccessor(&PhasedArrayModel::m_antennaElement),
                          MakePointerChecker<AntennaModel>())
            .AddAttribute("RetAngle",
                          "Electrical downtilt angle in degrees",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PhasedArrayModel::SetRetAngle, 
                                             &PhasedArrayModel::GetRetAngle),
                          MakeDoubleChecker<double>(-30.0, 30.0));
    return tid;
}

void
PhasedArrayModel::SetBeamformingVector(const ComplexVector& beamformingVector)
{
    //NS_LOG_UNCOND("[BF] SetBeamformingVector called, id=" << m_id
    //    << " norm=" << norm(beamformingVector)); // 추가
    NS_LOG_FUNCTION(this << beamformingVector);
    NS_ASSERT_MSG(beamformingVector.GetSize() == GetNumElems(),
                  beamformingVector.GetSize() << " != " << GetNumElems());
    m_beamformingVector = beamformingVector;
    m_isBfVectorValid = true;
}

PhasedArrayModel::ComplexVector
PhasedArrayModel::GetBeamformingVector() const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isBfVectorValid,
                  "The beamforming vector should be Set before it's Get, and should refer to the "
                  "current array configuration");
    return m_beamformingVector;
}

const PhasedArrayModel::ComplexVector&
PhasedArrayModel::GetBeamformingVectorRef() const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isBfVectorValid,
                  "The beamforming vector should be Set before it's Get, and should refer to the "
                  "current array configuration");
    return m_beamformingVector;
}

PhasedArrayModel::ComplexVector
PhasedArrayModel::GetBeamformingVector(Angles a) const
{
    NS_LOG_FUNCTION(this << a);

    ComplexVector beamformingVector = GetSteeringVector(a);
    // The normalization takes into account the total number of ports as only a
    // portion (K,L) of beam weights associated with a specific port are non-zero.
    // See 3GPP Section 5.2.2 36.897. This normalization corresponds to
    // a sub-array partition model (which is different from the full-connection
    // model). Note that the total number of ports used to perform normalization
    // is the ratio between the total number of antenna elements and the
    // number of antenna elements per port.
    double normRes = norm(beamformingVector) / sqrt(GetNumPorts());

    for (size_t i = 0; i < GetNumElems(); i++)
    {
        beamformingVector[i] = std::conj(beamformingVector[i]) / normRes;
    }

    return beamformingVector;
}

PhasedArrayModel::ComplexVector
PhasedArrayModel::GetSteeringVector(Angles a) const
{

    // 1) 기존 각도 획득
    double phi   = a.GetAzimuth();       // radians
    double theta = a.GetInclination();   // radians

    double thetaOrig = theta;  // 로그용 저장

    // 2) RET 적용 (degrees → radians)
    double retRad = m_retAngleDeg * M_PI / 180.0;
    theta = theta - retRad;    // 전기적 다운틸트는 θ -= tilt 가 맞음

    // // 로그 출력
    // NS_LOG_INFO("[PhasedArrayModel::GetSteeringVector] CALLED!"
    //     << " RET=" << m_retAngleDeg
    //     << " phi=" << phi
    //     << " theta_orig=" << thetaOrig
    //     << " theta_after=" << theta);

    // 3) 범위 제한 (안 하면 SINR error 발생 가능)
    if (theta < 0)   theta = 0;
    if (theta > M_PI) theta = M_PI;

    ComplexVector steeringVector(GetNumElems());

    for (size_t i = 0; i < GetNumElems(); i++)
    {
        Vector loc = GetElementLocation(i);

        double phase =
            -2 * M_PI * (
                sin(theta) * cos(phi) * loc.x +
                sin(theta) * sin(phi) * loc.y +
                cos(theta) * loc.z
            );

        steeringVector[i] = std::polar<double>(1.0, phase);
    }

    return steeringVector;
}


void
PhasedArrayModel::SetAntennaElement(Ptr<AntennaModel> antennaElement)
{
    NS_LOG_FUNCTION(this);
    m_antennaElement = antennaElement;
}

Ptr<const AntennaModel>
PhasedArrayModel::GetAntennaElement() const
{
    NS_LOG_FUNCTION(this);
    return m_antennaElement;
}

uint32_t
PhasedArrayModel::GetId() const
{
    return m_id;
}

void
PhasedArrayModel::SetRetAngle(double angleDeg)
{
    NS_LOG_FUNCTION(this << angleDeg);
    m_retAngleDeg = angleDeg;
}

double
PhasedArrayModel::GetRetAngle() const
{
    NS_LOG_FUNCTION(this);
    return m_retAngleDeg;
}

} /* namespace ns3 */
