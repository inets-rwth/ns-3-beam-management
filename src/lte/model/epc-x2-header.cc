/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

#include "ns3/log.h"
#include "ns3/epc-x2-header.h"

#include <cmath>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("EpcX2Header");

NS_OBJECT_ENSURE_REGISTERED (EpcX2Header);

EpcX2Header::EpcX2Header ()
  : m_messageType (0xfa),
    m_procedureCode (0xfa),
    m_lengthOfIes (0xfa),
    m_numberOfIes (0xfa)
{
}

EpcX2Header::~EpcX2Header ()
{
  m_messageType = 0xfb;
  m_procedureCode = 0xfb;
  m_lengthOfIes = 0xfb;
  m_numberOfIes = 0xfb;
}

TypeId
EpcX2Header::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2Header")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2Header> ()
  ;
  return tid;
}

TypeId
EpcX2Header::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2Header::GetSerializedSize (void) const
{
  return 7;
}

void
EpcX2Header::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteU8 (m_messageType);
  i.WriteU8 (m_procedureCode);

  i.WriteU8 (0x00); // criticality = REJECT
  i.WriteU8 (m_lengthOfIes + 3);
  i.WriteHtonU16 (0);
  i.WriteU8 (m_numberOfIes);
}

uint32_t
EpcX2Header::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_messageType = i.ReadU8 ();
  m_procedureCode = i.ReadU8 ();

  i.ReadU8 ();
  m_lengthOfIes = i.ReadU8 () - 3;
  i.ReadNtohU16 ();
  m_numberOfIes = i.ReadU8 ();
  
  return GetSerializedSize ();
}

void
EpcX2Header::Print (std::ostream &os) const
{
  os << "MessageType=" << (uint32_t) m_messageType;
  os << " ProcedureCode=" << (uint32_t) m_procedureCode;
  os << " LengthOfIEs=" << (uint32_t) m_lengthOfIes;
  os << " NumberOfIEs=" << (uint32_t) m_numberOfIes;
}

uint8_t
EpcX2Header::GetMessageType () const
{
  return m_messageType;
}

void
EpcX2Header::SetMessageType (uint8_t messageType)
{
  m_messageType = messageType;
}

uint8_t
EpcX2Header::GetProcedureCode () const
{
  return m_procedureCode;
}

void
EpcX2Header::SetProcedureCode (uint8_t procedureCode)
{
  m_procedureCode = procedureCode;
}


void
EpcX2Header::SetLengthOfIes (uint32_t lengthOfIes)
{
  m_lengthOfIes = lengthOfIes;
}

void
EpcX2Header::SetNumberOfIes (uint32_t numberOfIes)
{
  m_numberOfIes = numberOfIes;
}

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2HandoverRequestHeader);

EpcX2HandoverRequestHeader::EpcX2HandoverRequestHeader ()
  : m_numberOfIes (1 + 1 + 1 + 1),
    m_headerLength (6 + 5 + 12 + (3 + 4 + 8 + 8 + 4)),
    m_oldEnbUeX2apId (0xfffa),
    m_cause (0xfffa),
    m_targetCellId (0xfffa),
    m_mmeUeS1apId (0xfffffffa)
{
  m_erabsToBeSetupList.clear ();
}

EpcX2HandoverRequestHeader::~EpcX2HandoverRequestHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_oldEnbUeX2apId = 0xfffb;
  m_cause = 0xfffb;
  m_targetCellId = 0xfffb;
  m_mmeUeS1apId = 0xfffffffb;
  m_erabsToBeSetupList.clear ();
}

TypeId
EpcX2HandoverRequestHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2HandoverRequestHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2HandoverRequestHeader> ()
  ;
  return tid;
}

TypeId
EpcX2HandoverRequestHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2HandoverRequestHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2HandoverRequestHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (10);              // id = OLD_ENB_UE_X2AP_ID
  i.WriteU8 (0);                    // criticality = REJECT
  i.WriteU8 (2);                    // length of OLD_ENB_UE_X2AP_ID
  i.WriteHtonU16 (m_oldEnbUeX2apId);

  i.WriteHtonU16 (5);               // id = CAUSE
  i.WriteU8 (1 << 6);               // criticality = IGNORE
  i.WriteU8 (1);                    // length of CAUSE
  i.WriteU8 (m_cause);

  i.WriteHtonU16 (11);              // id = TARGET_CELLID
  i.WriteU8 (0);                    // criticality = REJECT
  i.WriteU8 (8);                    // length of TARGET_CELLID
  i.WriteHtonU32 (0x123456);        // fake PLMN
  i.WriteHtonU32 (m_targetCellId << 4);

  i.WriteHtonU16 (14);              // id = UE_CONTEXT_INFORMATION
  i.WriteU8 (0);                    // criticality = REJECT

  i.WriteHtonU32 (m_mmeUeS1apId);
  i.WriteHtonU64 (m_ueAggregateMaxBitRateDownlink);
  i.WriteHtonU64 (m_ueAggregateMaxBitRateUplink);

  std::vector <EpcX2Sap::ErabToBeSetupItem>::size_type sz = m_erabsToBeSetupList.size (); 
  i.WriteHtonU32 (sz);              // number of bearers
  for (int j = 0; j < (int) sz; j++)
    {
      i.WriteHtonU16 (m_erabsToBeSetupList [j].erabId);
      i.WriteHtonU16 (m_erabsToBeSetupList [j].erabLevelQosParameters.qci);
      i.WriteHtonU64 (m_erabsToBeSetupList [j].erabLevelQosParameters.gbrQosInfo.gbrDl);
      i.WriteHtonU64 (m_erabsToBeSetupList [j].erabLevelQosParameters.gbrQosInfo.gbrUl);
      i.WriteHtonU64 (m_erabsToBeSetupList [j].erabLevelQosParameters.gbrQosInfo.mbrDl);
      i.WriteHtonU64 (m_erabsToBeSetupList [j].erabLevelQosParameters.gbrQosInfo.mbrUl);
      i.WriteU8 (m_erabsToBeSetupList [j].erabLevelQosParameters.arp.priorityLevel);
      i.WriteU8 (m_erabsToBeSetupList [j].erabLevelQosParameters.arp.preemptionCapability);
      i.WriteU8 (m_erabsToBeSetupList [j].erabLevelQosParameters.arp.preemptionVulnerability);
      i.WriteU8 (m_erabsToBeSetupList [j].dlForwarding);
      i.WriteHtonU32 (m_erabsToBeSetupList [j].transportLayerAddress.Get ());
      i.WriteHtonU32 (m_erabsToBeSetupList [j].gtpTeid);
    }

}

uint32_t
EpcX2HandoverRequestHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;
  m_numberOfIes = 0;

  i.ReadNtohU16 ();
  i.ReadU8 ();
  i.ReadU8 ();
  m_oldEnbUeX2apId = i.ReadNtohU16 ();
  m_headerLength += 6;
  m_numberOfIes++;

  i.ReadNtohU16 ();
  i.ReadU8 ();
  i.ReadU8 ();
  m_cause = i.ReadU8 ();
  m_headerLength += 5;
  m_numberOfIes++;
  
  i.ReadNtohU16 ();
  i.ReadU8 ();
  i.ReadU8 ();
  i.ReadNtohU32 ();
  m_targetCellId = i.ReadNtohU32 () >> 4;
  m_headerLength += 12;
  m_numberOfIes++;

  i.ReadNtohU16 ();
  i.ReadU8 ();
  m_mmeUeS1apId = i.ReadNtohU32 ();
  m_ueAggregateMaxBitRateDownlink = i.ReadNtohU64 ();
  m_ueAggregateMaxBitRateUplink   = i.ReadNtohU64 ();
  int sz = i.ReadNtohU32 ();
  m_headerLength += 27;
  m_numberOfIes++;

  for (int j = 0; j < sz; j++)
    {
      EpcX2Sap::ErabToBeSetupItem erabItem;

      erabItem.erabId = i.ReadNtohU16 ();
 
      erabItem.erabLevelQosParameters = EpsBearer ((EpsBearer::Qci) i.ReadNtohU16 ());
      erabItem.erabLevelQosParameters.gbrQosInfo.gbrDl = i.ReadNtohU64 ();
      erabItem.erabLevelQosParameters.gbrQosInfo.gbrUl = i.ReadNtohU64 ();
      erabItem.erabLevelQosParameters.gbrQosInfo.mbrDl = i.ReadNtohU64 ();
      erabItem.erabLevelQosParameters.gbrQosInfo.mbrUl = i.ReadNtohU64 ();
      erabItem.erabLevelQosParameters.arp.priorityLevel = i.ReadU8 ();
      erabItem.erabLevelQosParameters.arp.preemptionCapability = i.ReadU8 ();
      erabItem.erabLevelQosParameters.arp.preemptionVulnerability = i.ReadU8 ();

      erabItem.dlForwarding = i.ReadU8 ();
      erabItem.transportLayerAddress = Ipv4Address (i.ReadNtohU32 ());
      erabItem.gtpTeid = i.ReadNtohU32 ();

      m_erabsToBeSetupList.push_back (erabItem);
      m_headerLength += 48;
    }

  return GetSerializedSize ();
}

void
EpcX2HandoverRequestHeader::Print (std::ostream &os) const
{
  os << "OldEnbUeX2apId = " << m_oldEnbUeX2apId;
  os << " Cause = " << m_cause;
  os << " TargetCellId = " << m_targetCellId;
  os << " MmeUeS1apId = " << m_mmeUeS1apId;
  os << " UeAggrMaxBitRateDownlink = " << m_ueAggregateMaxBitRateDownlink;
  os << " UeAggrMaxBitRateUplink = " << m_ueAggregateMaxBitRateUplink;
  os << " NumOfBearers = " << m_erabsToBeSetupList.size ();

  std::vector <EpcX2Sap::ErabToBeSetupItem>::size_type sz = m_erabsToBeSetupList.size ();
  if (sz > 0)
    {
      os << " [";
    }
  for (int j = 0; j < (int) sz; j++)
    {
      os << m_erabsToBeSetupList[j].erabId;
      if (j < (int) sz - 1)
        {
          os << ", ";
        }
      else
        {
          os << "]";
        }
    }
}

uint16_t
EpcX2HandoverRequestHeader::GetOldEnbUeX2apId () const
{
  return m_oldEnbUeX2apId;
}

void
EpcX2HandoverRequestHeader::SetOldEnbUeX2apId (uint16_t x2apId)
{
  m_oldEnbUeX2apId = x2apId;
}

uint16_t
EpcX2HandoverRequestHeader::GetCause () const
{
  return m_cause;
}

void
EpcX2HandoverRequestHeader::SetCause (uint16_t cause)
{
  m_cause = cause;
}

uint16_t
EpcX2HandoverRequestHeader::GetTargetCellId () const
{
  return m_targetCellId;
}

void
EpcX2HandoverRequestHeader::SetTargetCellId (uint16_t targetCellId)
{
  m_targetCellId = targetCellId;
}

uint32_t
EpcX2HandoverRequestHeader::GetMmeUeS1apId () const
{
  return m_mmeUeS1apId;
}

void
EpcX2HandoverRequestHeader::SetMmeUeS1apId (uint32_t mmeUeS1apId)
{
  m_mmeUeS1apId = mmeUeS1apId;
}

std::vector <EpcX2Sap::ErabToBeSetupItem>
EpcX2HandoverRequestHeader::GetBearers () const
{
  return m_erabsToBeSetupList;
}

void
EpcX2HandoverRequestHeader::SetBearers (std::vector <EpcX2Sap::ErabToBeSetupItem> bearers)
{
  m_headerLength += 48 * bearers.size ();
  m_erabsToBeSetupList = bearers;
}

uint64_t
EpcX2HandoverRequestHeader::GetUeAggregateMaxBitRateDownlink () const
{
  return m_ueAggregateMaxBitRateDownlink;
}

void
EpcX2HandoverRequestHeader::SetUeAggregateMaxBitRateDownlink (uint64_t bitRate)
{
  m_ueAggregateMaxBitRateDownlink = bitRate;
}

uint64_t
EpcX2HandoverRequestHeader::GetUeAggregateMaxBitRateUplink () const
{
  return m_ueAggregateMaxBitRateUplink;
}

void
EpcX2HandoverRequestHeader::SetUeAggregateMaxBitRateUplink (uint64_t bitRate)
{
  m_ueAggregateMaxBitRateUplink = bitRate;
}

uint32_t
EpcX2HandoverRequestHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2HandoverRequestHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2HandoverRequestAckHeader);

EpcX2HandoverRequestAckHeader::EpcX2HandoverRequestAckHeader ()
  : m_numberOfIes (1 + 1 + 1 + 1),
    m_headerLength (2 + 2 + 4 + 4),
    m_oldEnbUeX2apId (0xfffa),
    m_newEnbUeX2apId (0xfffa)
{
}

EpcX2HandoverRequestAckHeader::~EpcX2HandoverRequestAckHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_oldEnbUeX2apId = 0xfffb;
  m_newEnbUeX2apId = 0xfffb;
  m_erabsAdmittedList.clear ();
  m_erabsNotAdmittedList.clear ();
}

TypeId
EpcX2HandoverRequestAckHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2HandoverRequestAckHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2HandoverRequestAckHeader> ()
  ;
  return tid;
}

TypeId
EpcX2HandoverRequestAckHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2HandoverRequestAckHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2HandoverRequestAckHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_oldEnbUeX2apId);
  i.WriteHtonU16 (m_newEnbUeX2apId);

  std::vector <EpcX2Sap::ErabAdmittedItem>::size_type sz = m_erabsAdmittedList.size (); 
  i.WriteHtonU32 (sz);
  for (int j = 0; j < (int) sz; j++)
    {
      i.WriteHtonU16 (m_erabsAdmittedList [j].erabId);
      i.WriteHtonU32 (m_erabsAdmittedList [j].ulGtpTeid);
      i.WriteHtonU32 (m_erabsAdmittedList [j].dlGtpTeid);
    }

  std::vector <EpcX2Sap::ErabNotAdmittedItem>::size_type sz2 = m_erabsNotAdmittedList.size (); 
  i.WriteHtonU32 (sz2);
  for (int j = 0; j < (int) sz2; j++)
    {
      i.WriteHtonU16 (m_erabsNotAdmittedList [j].erabId);
      i.WriteHtonU16 (m_erabsNotAdmittedList [j].cause);
    }
}

uint32_t
EpcX2HandoverRequestAckHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;
  m_numberOfIes = 0;

  m_oldEnbUeX2apId = i.ReadNtohU16 ();
  m_newEnbUeX2apId = i.ReadNtohU16 ();
  m_headerLength += 4;
  m_numberOfIes += 2;

  int sz = i.ReadNtohU32 ();
  m_headerLength += 4;
  m_numberOfIes++;

  for (int j = 0; j < sz; j++)
    {
      EpcX2Sap::ErabAdmittedItem erabItem;

      erabItem.erabId = i.ReadNtohU16 ();
      erabItem.ulGtpTeid = i.ReadNtohU32 ();
      erabItem.dlGtpTeid = i.ReadNtohU32 ();

      m_erabsAdmittedList.push_back (erabItem);
      m_headerLength += 10;
    }

  sz = i.ReadNtohU32 ();
  m_headerLength += 4;
  m_numberOfIes++;

  for (int j = 0; j < sz; j++)
    {
      EpcX2Sap::ErabNotAdmittedItem erabItem;

      erabItem.erabId = i.ReadNtohU16 ();
      erabItem.cause  = i.ReadNtohU16 ();

      m_erabsNotAdmittedList.push_back (erabItem);
      m_headerLength += 4;
    }

  return GetSerializedSize ();
}

void
EpcX2HandoverRequestAckHeader::Print (std::ostream &os) const
{
  os << "OldEnbUeX2apId=" << m_oldEnbUeX2apId;
  os << " NewEnbUeX2apId=" << m_newEnbUeX2apId;

  os << " AdmittedBearers=" << m_erabsAdmittedList.size ();
  std::vector <EpcX2Sap::ErabAdmittedItem>::size_type sz = m_erabsAdmittedList.size ();
  if (sz > 0)
    {
      os << " [";
    }
  for (int j = 0; j < (int) sz; j++)
    {
      os << m_erabsAdmittedList[j].erabId;
      if (j < (int) sz - 1)
        {
          os << ", ";
        }
      else
        {
          os << "]";
        }
    }
  
  os << " NotAdmittedBearers=" << m_erabsNotAdmittedList.size ();
  std::vector <EpcX2Sap::ErabNotAdmittedItem>::size_type sz2 = m_erabsNotAdmittedList.size ();
  if (sz2 > 0)
    {
      os << " [";
    }
  for (int j = 0; j < (int) sz2; j++)
    {
      os << m_erabsNotAdmittedList[j].erabId;
      if (j < (int) sz2 - 1)
        {
          os << ", ";
        }
      else
        {
          os << "]";
        }
    }

}

uint16_t
EpcX2HandoverRequestAckHeader::GetOldEnbUeX2apId () const
{
  return m_oldEnbUeX2apId;
}

void
EpcX2HandoverRequestAckHeader::SetOldEnbUeX2apId (uint16_t x2apId)
{
  m_oldEnbUeX2apId = x2apId;
}

uint16_t
EpcX2HandoverRequestAckHeader::GetNewEnbUeX2apId () const
{
  return m_newEnbUeX2apId;
}

void
EpcX2HandoverRequestAckHeader::SetNewEnbUeX2apId (uint16_t x2apId)
{
  m_newEnbUeX2apId = x2apId;
}

std::vector <EpcX2Sap::ErabAdmittedItem> 
EpcX2HandoverRequestAckHeader::GetAdmittedBearers () const
{
  return m_erabsAdmittedList;
}

void
EpcX2HandoverRequestAckHeader::SetAdmittedBearers (std::vector <EpcX2Sap::ErabAdmittedItem> bearers)
{
  m_headerLength += 10 * bearers.size ();
  m_erabsAdmittedList = bearers;
}

std::vector <EpcX2Sap::ErabNotAdmittedItem>
EpcX2HandoverRequestAckHeader::GetNotAdmittedBearers () const
{
  return m_erabsNotAdmittedList;
}

void
EpcX2HandoverRequestAckHeader::SetNotAdmittedBearers (std::vector <EpcX2Sap::ErabNotAdmittedItem> bearers)
{
  m_headerLength += 4 * bearers.size ();
  m_erabsNotAdmittedList = bearers;
}

uint32_t
EpcX2HandoverRequestAckHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2HandoverRequestAckHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2HandoverPreparationFailureHeader);

EpcX2HandoverPreparationFailureHeader::EpcX2HandoverPreparationFailureHeader ()
  : m_numberOfIes (1 + 1 + 1),
    m_headerLength (2 + 2 + 2),
    m_oldEnbUeX2apId (0xfffa),
    m_cause (0xfffa),
    m_criticalityDiagnostics (0xfffa)
{
}

EpcX2HandoverPreparationFailureHeader::~EpcX2HandoverPreparationFailureHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_oldEnbUeX2apId = 0xfffb;
  m_cause = 0xfffb;
  m_criticalityDiagnostics = 0xfffb;
}

TypeId
EpcX2HandoverPreparationFailureHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2HandoverPreparationFailureHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2HandoverPreparationFailureHeader> ()
  ;
  return tid;
}

TypeId
EpcX2HandoverPreparationFailureHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2HandoverPreparationFailureHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2HandoverPreparationFailureHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_oldEnbUeX2apId);
  i.WriteHtonU16 (m_cause);
  i.WriteHtonU16 (m_criticalityDiagnostics);
}

uint32_t
EpcX2HandoverPreparationFailureHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_oldEnbUeX2apId = i.ReadNtohU16 ();
  m_cause = i.ReadNtohU16 ();
  m_criticalityDiagnostics = i.ReadNtohU16 ();

  m_headerLength = 6;
  m_numberOfIes = 3;

  return GetSerializedSize ();
}

void
EpcX2HandoverPreparationFailureHeader::Print (std::ostream &os) const
{
  os << "OldEnbUeX2apId = " << m_oldEnbUeX2apId;
  os << " Cause = " << m_cause;
  os << " CriticalityDiagnostics = " << m_criticalityDiagnostics;
}

uint16_t
EpcX2HandoverPreparationFailureHeader::GetOldEnbUeX2apId () const
{
  return m_oldEnbUeX2apId;
}

void
EpcX2HandoverPreparationFailureHeader::SetOldEnbUeX2apId (uint16_t x2apId)
{
  m_oldEnbUeX2apId = x2apId;
}

uint16_t
EpcX2HandoverPreparationFailureHeader::GetCause () const
{
  return m_cause;
}

void
EpcX2HandoverPreparationFailureHeader::SetCause (uint16_t cause)
{
  m_cause = cause;
}

uint16_t
EpcX2HandoverPreparationFailureHeader::GetCriticalityDiagnostics () const
{
  return m_criticalityDiagnostics;
}

void
EpcX2HandoverPreparationFailureHeader::SetCriticalityDiagnostics (uint16_t criticalityDiagnostics)
{
  m_criticalityDiagnostics = criticalityDiagnostics;
}

uint32_t
EpcX2HandoverPreparationFailureHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2HandoverPreparationFailureHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2SnStatusTransferHeader);

EpcX2SnStatusTransferHeader::EpcX2SnStatusTransferHeader ()
  : m_numberOfIes (3),
    m_headerLength (6),
    m_oldEnbUeX2apId (0xfffa),
    m_newEnbUeX2apId (0xfffa)
{
  m_erabsSubjectToStatusTransferList.clear (); 
}

EpcX2SnStatusTransferHeader::~EpcX2SnStatusTransferHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_oldEnbUeX2apId = 0xfffb;
  m_newEnbUeX2apId = 0xfffb;
  m_erabsSubjectToStatusTransferList.clear (); 
}

TypeId
EpcX2SnStatusTransferHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2SnStatusTransferHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2SnStatusTransferHeader> ()
  ;
  return tid;
}

TypeId
EpcX2SnStatusTransferHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2SnStatusTransferHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2SnStatusTransferHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_oldEnbUeX2apId);
  i.WriteHtonU16 (m_newEnbUeX2apId);

  std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem>::size_type sz = m_erabsSubjectToStatusTransferList.size ();
  i.WriteHtonU16 (sz);              // number of ErabsSubjectToStatusTransferItems

  for (int j = 0; j < (int) sz; j++)
    {
      EpcX2Sap::ErabsSubjectToStatusTransferItem item = m_erabsSubjectToStatusTransferList [j];

      i.WriteHtonU16 (item.erabId);

      uint16_t bitsetSize = EpcX2Sap::m_maxPdcpSn / 64;
      for (int k = 0; k < bitsetSize; k++)
        {
          uint64_t statusValue = 0;
          for (int m = 0; m < 64; m++)
            {
              statusValue |= item.receiveStatusOfUlPdcpSdus[64 * k + m] << m;
            }
          i.WriteHtonU64 (statusValue);
        }

      i.WriteHtonU16 (item.ulPdcpSn);
      i.WriteHtonU32 (item.ulHfn);
      i.WriteHtonU16 (item.dlPdcpSn);
      i.WriteHtonU32 (item.dlHfn);
    }
}

uint32_t
EpcX2SnStatusTransferHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_oldEnbUeX2apId = i.ReadNtohU16 ();
  m_newEnbUeX2apId = i.ReadNtohU16 ();
  int sz = i.ReadNtohU16 ();

  m_numberOfIes = 3;
  m_headerLength = 6 + sz * (14 + (EpcX2Sap::m_maxPdcpSn / 64));

  for (int j = 0; j < sz; j++)
    {
      EpcX2Sap::ErabsSubjectToStatusTransferItem ErabItem;
      ErabItem.erabId = i.ReadNtohU16 ();

      uint16_t bitsetSize = EpcX2Sap::m_maxPdcpSn / 64;
      for (int k = 0; k < bitsetSize; k++)
        {
          uint64_t statusValue = i.ReadNtohU64 ();
          for (int m = 0; m < 64; m++)
            {
              ErabItem.receiveStatusOfUlPdcpSdus[64 * k + m] = (statusValue >> m) & 1;
            }
        }

      ErabItem.ulPdcpSn = i.ReadNtohU16 ();
      ErabItem.ulHfn    = i.ReadNtohU32 ();
      ErabItem.dlPdcpSn = i.ReadNtohU16 ();
      ErabItem.dlHfn    = i.ReadNtohU32 ();

      m_erabsSubjectToStatusTransferList.push_back (ErabItem);
    }

  return GetSerializedSize ();
}

void
EpcX2SnStatusTransferHeader::Print (std::ostream &os) const
{
  os << "OldEnbUeX2apId = " << m_oldEnbUeX2apId;
  os << " NewEnbUeX2apId = " << m_newEnbUeX2apId;
  os << " ErabsSubjectToStatusTransferList size = " << m_erabsSubjectToStatusTransferList.size ();

  std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem>::size_type sz = m_erabsSubjectToStatusTransferList.size ();
  if (sz > 0)
    {
      os << " [";
    }
  for (int j = 0; j < (int) sz; j++)
    {
      os << m_erabsSubjectToStatusTransferList[j].erabId;
      if (j < (int) sz - 1)
        {
          os << ", ";
        }
      else
        {
          os << "]";
        }
    }
}

uint16_t
EpcX2SnStatusTransferHeader::GetOldEnbUeX2apId () const
{
  return m_oldEnbUeX2apId;
}

void
EpcX2SnStatusTransferHeader::SetOldEnbUeX2apId (uint16_t x2apId)
{
  m_oldEnbUeX2apId = x2apId;
}

uint16_t
EpcX2SnStatusTransferHeader::GetNewEnbUeX2apId () const
{
  return m_newEnbUeX2apId;
}

void
EpcX2SnStatusTransferHeader::SetNewEnbUeX2apId (uint16_t x2apId)
{
  m_newEnbUeX2apId = x2apId;
}

std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem>
EpcX2SnStatusTransferHeader::GetErabsSubjectToStatusTransferList () const
{
  return m_erabsSubjectToStatusTransferList;
}

void
EpcX2SnStatusTransferHeader::SetErabsSubjectToStatusTransferList (std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> erabs)
{
  m_headerLength += erabs.size () * (14 + (EpcX2Sap::m_maxPdcpSn / 8));
  m_erabsSubjectToStatusTransferList = erabs;
}

uint32_t
EpcX2SnStatusTransferHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2SnStatusTransferHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2UeContextReleaseHeader);

EpcX2UeContextReleaseHeader::EpcX2UeContextReleaseHeader ()
  : m_numberOfIes (1 + 1),
    m_headerLength (2 + 2),
    m_oldEnbUeX2apId (0xfffa),
    m_newEnbUeX2apId (0xfffa)
{
}

EpcX2UeContextReleaseHeader::~EpcX2UeContextReleaseHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_oldEnbUeX2apId = 0xfffb;
  m_newEnbUeX2apId = 0xfffb;
}

TypeId
EpcX2UeContextReleaseHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2UeContextReleaseHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2UeContextReleaseHeader> ()
  ;
  return tid;
}

TypeId
EpcX2UeContextReleaseHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2UeContextReleaseHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2UeContextReleaseHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_oldEnbUeX2apId);
  i.WriteHtonU16 (m_newEnbUeX2apId);
}

uint32_t
EpcX2UeContextReleaseHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_oldEnbUeX2apId = i.ReadNtohU16 ();
  m_newEnbUeX2apId = i.ReadNtohU16 ();
  m_numberOfIes = 2;
  m_headerLength = 4;

  return GetSerializedSize ();
}

void
EpcX2UeContextReleaseHeader::Print (std::ostream &os) const
{
  os << "OldEnbUeX2apId=" << m_oldEnbUeX2apId;
  os << " NewEnbUeX2apId=" << m_newEnbUeX2apId;
}

uint16_t
EpcX2UeContextReleaseHeader::GetOldEnbUeX2apId () const
{
  return m_oldEnbUeX2apId;
}

void
EpcX2UeContextReleaseHeader::SetOldEnbUeX2apId (uint16_t x2apId)
{
  m_oldEnbUeX2apId = x2apId;
}

uint16_t
EpcX2UeContextReleaseHeader::GetNewEnbUeX2apId () const
{
  return m_newEnbUeX2apId;
}

void
EpcX2UeContextReleaseHeader::SetNewEnbUeX2apId (uint16_t x2apId)
{
  m_newEnbUeX2apId = x2apId;
}

uint32_t
EpcX2UeContextReleaseHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2UeContextReleaseHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

/////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2LoadInformationHeader);

EpcX2LoadInformationHeader::EpcX2LoadInformationHeader ()
  : m_numberOfIes (1),
    m_headerLength (6)
{
  m_cellInformationList.clear ();
}

EpcX2LoadInformationHeader::~EpcX2LoadInformationHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_cellInformationList.clear ();
}

TypeId
EpcX2LoadInformationHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2LoadInformationHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2LoadInformationHeader> ()
  ;
  return tid;
}

TypeId
EpcX2LoadInformationHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2LoadInformationHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2LoadInformationHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (6);               // id = CELL_INFORMATION
  i.WriteU8 (1 << 6);               // criticality = IGNORE
  i.WriteU8 (4);                    // length of CELL_INFORMATION_ID

  std::vector <EpcX2Sap::CellInformationItem>::size_type sz = m_cellInformationList.size ();
  i.WriteHtonU16 (sz);              // number of cellInformationItems

  for (int j = 0; j < (int) sz; j++)
    {
      i.WriteHtonU16 (m_cellInformationList [j].sourceCellId);

      std::vector <EpcX2Sap::UlInterferenceOverloadIndicationItem>::size_type sz2;
      sz2 = m_cellInformationList [j].ulInterferenceOverloadIndicationList.size ();
      i.WriteHtonU16 (sz2);         // number of UlInterferenceOverloadIndicationItem

      for (int k = 0; k < (int) sz2; k++)
        {
          i.WriteU8 (m_cellInformationList [j].ulInterferenceOverloadIndicationList [k]);
        }

      std::vector <EpcX2Sap::UlHighInterferenceInformationItem>::size_type sz3;
      sz3 = m_cellInformationList [j].ulHighInterferenceInformationList.size ();
      i.WriteHtonU16 (sz3);         // number of UlHighInterferenceInformationItem

      for (int k = 0; k < (int) sz3; k++)
        {
          i.WriteHtonU16 (m_cellInformationList [j].ulHighInterferenceInformationList [k].targetCellId);

          std::vector <bool>::size_type sz4;
          sz4 = m_cellInformationList [j].ulHighInterferenceInformationList [k].ulHighInterferenceIndicationList.size ();
          i.WriteHtonU16 (sz4);

          for (int m = 0; m < (int) sz4; m++)
            {
              i.WriteU8 (m_cellInformationList [j].ulHighInterferenceInformationList [k].ulHighInterferenceIndicationList [m]);
            }
        }

      std::vector <bool>::size_type sz5;
      sz5 = m_cellInformationList [j].relativeNarrowbandTxBand.rntpPerPrbList.size ();
      i.WriteHtonU16 (sz5);

      for (int k = 0; k < (int) sz5; k++)
        {
          i.WriteU8 (m_cellInformationList [j].relativeNarrowbandTxBand.rntpPerPrbList [k]);
        }

      i.WriteHtonU16 (m_cellInformationList [j].relativeNarrowbandTxBand.rntpThreshold);
      i.WriteHtonU16 (m_cellInformationList [j].relativeNarrowbandTxBand.antennaPorts);
      i.WriteHtonU16 (m_cellInformationList [j].relativeNarrowbandTxBand.pB);
      i.WriteHtonU16 (m_cellInformationList [j].relativeNarrowbandTxBand.pdcchInterferenceImpact);
    }
}

uint32_t
EpcX2LoadInformationHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;
  m_numberOfIes = 0;

  i.ReadNtohU16 ();
  i.ReadU8 ();
  i.ReadU8 ();
  int sz = i.ReadNtohU16 ();
  m_headerLength += 6;
  m_numberOfIes++;

  for (int j = 0; j < sz; j++)
    {
      EpcX2Sap::CellInformationItem cellInfoItem;
      cellInfoItem.sourceCellId = i.ReadNtohU16 ();
      m_headerLength += 2;

      int sz2 = i.ReadNtohU16 ();
      m_headerLength += 2;
      for (int k = 0; k < sz2; k++)
        {
          EpcX2Sap::UlInterferenceOverloadIndicationItem item = (EpcX2Sap::UlInterferenceOverloadIndicationItem) i.ReadU8 ();
          cellInfoItem.ulInterferenceOverloadIndicationList.push_back (item);
        }
      m_headerLength += sz2;

      int sz3 = i.ReadNtohU16 ();
      m_headerLength += 2;
      for (int k = 0; k < sz3; k++)
        {
          EpcX2Sap::UlHighInterferenceInformationItem item;
          item.targetCellId = i.ReadNtohU16 ();
          m_headerLength += 2;

          int sz4 = i.ReadNtohU16 ();
          m_headerLength += 2;
          for (int m = 0; m < sz4; m++)
            {
              item.ulHighInterferenceIndicationList.push_back (i.ReadU8 ());
            }
          m_headerLength += sz4;

          cellInfoItem.ulHighInterferenceInformationList.push_back (item);
        }

      int sz5 = i.ReadNtohU16 ();
      m_headerLength += 2;
      for (int k = 0; k < sz5; k++)
        {
          cellInfoItem.relativeNarrowbandTxBand.rntpPerPrbList.push_back (i.ReadU8 ());
        }
      m_headerLength += sz5;

      cellInfoItem.relativeNarrowbandTxBand.rntpThreshold = i.ReadNtohU16 ();
      cellInfoItem.relativeNarrowbandTxBand.antennaPorts = i.ReadNtohU16 ();
      cellInfoItem.relativeNarrowbandTxBand.pB = i.ReadNtohU16 ();
      cellInfoItem.relativeNarrowbandTxBand.pdcchInterferenceImpact = i.ReadNtohU16 ();
      m_headerLength += 8;

      m_cellInformationList.push_back (cellInfoItem);
    }

  return GetSerializedSize ();
}

void
EpcX2LoadInformationHeader::Print (std::ostream &os) const
{
  os << "NumOfCellInformationItems=" << m_cellInformationList.size ();
}

std::vector <EpcX2Sap::CellInformationItem>
EpcX2LoadInformationHeader::GetCellInformationList () const
{
  return m_cellInformationList;
}

void
EpcX2LoadInformationHeader::SetCellInformationList (std::vector <EpcX2Sap::CellInformationItem> cellInformationList)
{
  m_cellInformationList = cellInformationList;
  m_headerLength += 2;

  std::vector <EpcX2Sap::CellInformationItem>::size_type sz = m_cellInformationList.size ();
  for (int j = 0; j < (int) sz; j++)
    {
      m_headerLength += 2;

      std::vector <EpcX2Sap::UlInterferenceOverloadIndicationItem>::size_type sz2;
      sz2 = m_cellInformationList [j].ulInterferenceOverloadIndicationList.size ();
      m_headerLength += 2 + sz2;

      std::vector <EpcX2Sap::UlHighInterferenceInformationItem>::size_type sz3;
      sz3 = m_cellInformationList [j].ulHighInterferenceInformationList.size ();
      m_headerLength += 2;

      for (int k = 0; k < (int) sz3; k++)
        {
          std::vector <bool>::size_type sz4;
          sz4 = m_cellInformationList [j].ulHighInterferenceInformationList [k].ulHighInterferenceIndicationList.size ();
          m_headerLength += 2 + 2 + sz4;
        }

      std::vector <bool>::size_type sz5;
      sz5 = m_cellInformationList [j].relativeNarrowbandTxBand.rntpPerPrbList.size ();
      m_headerLength += 2 + sz5 + 8;
    }
}

uint32_t
EpcX2LoadInformationHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2LoadInformationHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2ResourceStatusUpdateHeader);

EpcX2ResourceStatusUpdateHeader::EpcX2ResourceStatusUpdateHeader ()
  : m_numberOfIes (3),
    m_headerLength (6),
    m_enb1MeasurementId (0xfffa),
    m_enb2MeasurementId (0xfffa)
{
  m_cellMeasurementResultList.clear ();
}

EpcX2ResourceStatusUpdateHeader::~EpcX2ResourceStatusUpdateHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_enb1MeasurementId = 0xfffb;
  m_enb2MeasurementId = 0xfffb;
  m_cellMeasurementResultList.clear ();
}

TypeId
EpcX2ResourceStatusUpdateHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2ResourceStatusUpdateHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2ResourceStatusUpdateHeader> ()
  ;
  return tid;
}

TypeId
EpcX2ResourceStatusUpdateHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2ResourceStatusUpdateHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2ResourceStatusUpdateHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_enb1MeasurementId);
  i.WriteHtonU16 (m_enb2MeasurementId);

  std::vector <EpcX2Sap::CellMeasurementResultItem>::size_type sz = m_cellMeasurementResultList.size ();
  i.WriteHtonU16 (sz);              // number of CellMeasurementResultItem

  for (int j = 0; j < (int) sz; j++)
    {
      EpcX2Sap::CellMeasurementResultItem item = m_cellMeasurementResultList [j];

      i.WriteHtonU16 (item.sourceCellId);
      i.WriteU8 (item.dlHardwareLoadIndicator);
      i.WriteU8 (item.ulHardwareLoadIndicator);
      i.WriteU8 (item.dlS1TnlLoadIndicator);
      i.WriteU8 (item.ulS1TnlLoadIndicator);

      i.WriteHtonU16 (item.dlGbrPrbUsage);
      i.WriteHtonU16 (item.ulGbrPrbUsage);
      i.WriteHtonU16 (item.dlNonGbrPrbUsage);
      i.WriteHtonU16 (item.ulNonGbrPrbUsage);
      i.WriteHtonU16 (item.dlTotalPrbUsage);
      i.WriteHtonU16 (item.ulTotalPrbUsage);

      i.WriteHtonU16 (item.dlCompositeAvailableCapacity.cellCapacityClassValue);
      i.WriteHtonU16 (item.dlCompositeAvailableCapacity.capacityValue);
      i.WriteHtonU16 (item.ulCompositeAvailableCapacity.cellCapacityClassValue);
      i.WriteHtonU16 (item.ulCompositeAvailableCapacity.capacityValue);
    }
}

uint32_t
EpcX2ResourceStatusUpdateHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_enb1MeasurementId = i.ReadNtohU16 ();
  m_enb2MeasurementId = i.ReadNtohU16 ();

  int sz = i.ReadNtohU16 ();
  for (int j = 0; j < sz; j++)
    {
      EpcX2Sap::CellMeasurementResultItem item;

      item.sourceCellId = i.ReadNtohU16 ();
      item.dlHardwareLoadIndicator = (EpcX2Sap::LoadIndicator) i.ReadU8 ();
      item.ulHardwareLoadIndicator = (EpcX2Sap::LoadIndicator) i.ReadU8 ();
      item.dlS1TnlLoadIndicator = (EpcX2Sap::LoadIndicator) i.ReadU8 ();
      item.ulS1TnlLoadIndicator = (EpcX2Sap::LoadIndicator) i.ReadU8 ();

      item.dlGbrPrbUsage = i.ReadNtohU16 ();
      item.ulGbrPrbUsage = i.ReadNtohU16 ();
      item.dlNonGbrPrbUsage = i.ReadNtohU16 ();
      item.ulNonGbrPrbUsage = i.ReadNtohU16 ();
      item.dlTotalPrbUsage = i.ReadNtohU16 ();
      item.ulTotalPrbUsage = i.ReadNtohU16 ();

      item.dlCompositeAvailableCapacity.cellCapacityClassValue = i.ReadNtohU16 ();
      item.dlCompositeAvailableCapacity.capacityValue = i.ReadNtohU16 ();
      item.ulCompositeAvailableCapacity.cellCapacityClassValue = i.ReadNtohU16 ();
      item.ulCompositeAvailableCapacity.capacityValue = i.ReadNtohU16 ();

      m_cellMeasurementResultList.push_back (item);
    }

  m_headerLength = 6 + sz * 26;
  m_numberOfIes = 3;

  return GetSerializedSize ();
}

void
EpcX2ResourceStatusUpdateHeader::Print (std::ostream &os) const
{
  os << "Enb1MeasurementId = " << m_enb1MeasurementId
     << " Enb2MeasurementId = " << m_enb2MeasurementId
     << " NumOfCellMeasurementResultItems = " << m_cellMeasurementResultList.size ();
}

uint16_t
EpcX2ResourceStatusUpdateHeader::GetEnb1MeasurementId () const
{
  return m_enb1MeasurementId;
}

void
EpcX2ResourceStatusUpdateHeader::SetEnb1MeasurementId (uint16_t enb1MeasurementId)
{
  m_enb1MeasurementId = enb1MeasurementId;
}

uint16_t
EpcX2ResourceStatusUpdateHeader::GetEnb2MeasurementId () const
{
  return m_enb2MeasurementId;
}

void
EpcX2ResourceStatusUpdateHeader::SetEnb2MeasurementId (uint16_t enb2MeasurementId)
{
  m_enb2MeasurementId = enb2MeasurementId;
}

std::vector <EpcX2Sap::CellMeasurementResultItem>
EpcX2ResourceStatusUpdateHeader::GetCellMeasurementResultList () const
{
  return m_cellMeasurementResultList;
}

void
EpcX2ResourceStatusUpdateHeader::SetCellMeasurementResultList (std::vector <EpcX2Sap::CellMeasurementResultItem> cellMeasurementResultList)
{
  m_cellMeasurementResultList = cellMeasurementResultList;

  std::vector <EpcX2Sap::CellMeasurementResultItem>::size_type sz = m_cellMeasurementResultList.size ();
  m_headerLength += sz * 26;
}

uint32_t
EpcX2ResourceStatusUpdateHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2ResourceStatusUpdateHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

//****************************************************************************************************
NS_OBJECT_ENSURE_REGISTERED (EpcX2UeImsiSinrUpdateHeader);

EpcX2UeImsiSinrUpdateHeader::EpcX2UeImsiSinrUpdateHeader ()
  : m_numberOfIes (1 + 1),
    m_headerLength (2 + 2)
{
  m_map.clear ();
}

EpcX2UeImsiSinrUpdateHeader::~EpcX2UeImsiSinrUpdateHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_map.clear ();
}

TypeId
EpcX2UeImsiSinrUpdateHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2UeImsiSinrUpdateHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2UeImsiSinrUpdateHeader> ()
  ;
  return tid;
}

TypeId
EpcX2UeImsiSinrUpdateHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2UeImsiSinrUpdateHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2UeImsiSinrUpdateHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_sourceCellId);

  std::map <uint64_t, double>::size_type sz = m_map.size ();
  i.WriteHtonU16 (sz);              // number of elements in the map

  for (std::map<uint64_t, double>::const_iterator iter = m_map.begin(); iter != m_map.end(); ++iter)
    {
      i.WriteHtonU64 (iter->first); // imsi
      i.WriteHtonU64 (pack754(iter->second)); // sinr
    }
}

uint32_t
EpcX2UeImsiSinrUpdateHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;

  m_sourceCellId = i.ReadNtohU16();
  m_headerLength += 2;
  m_numberOfIes = 1;

  int sz = i.ReadNtohU16 ();
  for (int j = 0; j < sz; j++)
    {
      uint64_t imsi = i.ReadNtohU64();
      double sinr = unpack754(i.ReadNtohU64());
      m_map[imsi] = sinr;
    }

  m_headerLength += 2 + sz * 16;
  m_numberOfIes += 1 + sz;

  return GetSerializedSize ();
}

void
EpcX2UeImsiSinrUpdateHeader::Print (std::ostream &os) const
{
  os << "SourceCellId " << m_sourceCellId;
  for(std::map<uint64_t, double>::const_iterator iter = m_map.begin(); iter != m_map.end(); ++iter)
  {
    os << " Imsi " << iter->first << " sinr " << 10*std::log10(iter->second);
  }
}

uint16_t 
EpcX2UeImsiSinrUpdateHeader::GetSourceCellId () const
{
  return m_sourceCellId;
}

void
EpcX2UeImsiSinrUpdateHeader::SetSourceCellId(uint16_t cellId)
{
  m_sourceCellId = cellId;
}

std::map <uint64_t, double>
EpcX2UeImsiSinrUpdateHeader::GetUeImsiSinrMap () const
{
  return m_map;
}

void
EpcX2UeImsiSinrUpdateHeader::SetUeImsiSinrMap (std::map <uint64_t, double> map)
{
  m_map = map;

  std::map <uint64_t, double>::size_type sz = m_map.size ();
  m_headerLength += sz * 16;
  m_numberOfIes += sz;
}

uint32_t
EpcX2UeImsiSinrUpdateHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2UeImsiSinrUpdateHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

uint64_t 
EpcX2UeImsiSinrUpdateHeader::pack754(long double f)
{
  uint16_t bits = 64;
  uint16_t expbits = 11;
  long double fnorm;
  int shift;
  long long sign, exp, significand;
  unsigned significandbits = bits - expbits - 1; // -1 for sign bit

  if (f == 0.0) return 0; // get this special case out of the way

  // check sign and begin normalization
  if (f < 0) { sign = 1; fnorm = -f; }
  else { sign = 0; fnorm = f; }

  // get the normalized form of f and track the exponent
  shift = 0;
  while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
  while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
  fnorm = fnorm - 1.0;

  // calculate the binary form (non-float) of the significand data
  significand = fnorm * ((1LL<<significandbits) + 0.5f);

  // get the biased exponent
  exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

  // return the final answer
  return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

long double 
EpcX2UeImsiSinrUpdateHeader::unpack754(uint64_t i)
{
  uint16_t bits = 64;
  uint16_t expbits = 11;
  long double result;
  long long shift;
  unsigned bias;
  unsigned significandbits = bits - expbits - 1; // -1 for sign bit

  if (i == 0) return 0.0;

  // pull the significand
  result = (i&((1LL<<significandbits)-1)); // mask
  result /= (1LL<<significandbits); // convert back to float
  result += 1.0f; // add the one back on

  // deal with the exponent
  bias = (1<<(expbits-1)) - 1;
  shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
  while(shift > 0) { result *= 2.0; shift--; }
  while(shift < 0) { result /= 2.0; shift++; }

  // sign it
  result *= (i>>(bits-1))&1? -1.0: 1.0;

  return result;
}

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2CoordinatorHandoverHeader);

EpcX2CoordinatorHandoverHeader::EpcX2CoordinatorHandoverHeader ()
  : m_numberOfIes (1 + 1 + 1),
    m_headerLength (2 + 2 + 8),
    m_targetCellId (0xfffa),
    m_oldCellId (0xfffa),
    m_imsi (0xfffffffffffffffa)
{
}

EpcX2CoordinatorHandoverHeader::~EpcX2CoordinatorHandoverHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
  m_targetCellId = 0xfffb;
  m_oldCellId = 0xfffb;
  m_imsi = 0xfffffffffffffffb;
}

TypeId
EpcX2CoordinatorHandoverHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2CoordinatorHandoverHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2CoordinatorHandoverHeader> ()
  ;
  return tid;
}

TypeId
EpcX2CoordinatorHandoverHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2CoordinatorHandoverHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2CoordinatorHandoverHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_targetCellId); 
  i.WriteHtonU16 (m_oldCellId); 
  i.WriteHtonU64 (m_imsi); 
}

uint32_t
EpcX2CoordinatorHandoverHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;
  m_numberOfIes = 0;
  
  m_targetCellId = i.ReadNtohU16 ();
  m_headerLength += 2;
  m_numberOfIes++;

  m_oldCellId = i.ReadNtohU16 ();
  m_headerLength += 2;
  m_numberOfIes++;

  m_imsi = i.ReadNtohU64 ();
  m_headerLength += 8;
  m_numberOfIes++;

  return GetSerializedSize ();
}

void
EpcX2CoordinatorHandoverHeader::Print (std::ostream &os) const
{
  os << " TargetCellId = " << m_targetCellId;
  os << " oldCellId = " << m_oldCellId;
  os << " imsi = " << m_imsi;
}

uint16_t
EpcX2CoordinatorHandoverHeader::GetTargetCellId () const
{
  return m_targetCellId;
}

void
EpcX2CoordinatorHandoverHeader::SetTargetCellId (uint16_t targetCellId)
{
  m_targetCellId = targetCellId;
}

uint64_t
EpcX2CoordinatorHandoverHeader::GetImsi () const
{
  return m_imsi;
}

void
EpcX2CoordinatorHandoverHeader::SetImsi (uint64_t imsi)
{
  m_imsi = imsi;
}

uint16_t
EpcX2CoordinatorHandoverHeader::GetOldCellId () const
{
  return m_oldCellId;
}

void
EpcX2CoordinatorHandoverHeader::SetOldCellId (uint16_t oldCellId)
{
  m_oldCellId = oldCellId;
}


uint32_t
EpcX2CoordinatorHandoverHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2CoordinatorHandoverHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

//****************************************************************************************************
NS_OBJECT_ENSURE_REGISTERED (EpcX2InformLteCoordinatorHeader);

EpcX2InformLteCoordinatorHeader::EpcX2InformLteCoordinatorHeader ()
  : m_numberOfIes (1 + 1 + 1),
    m_headerLength (2 + 2 + 8)
{
}

EpcX2InformLteCoordinatorHeader::~EpcX2InformLteCoordinatorHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
}

TypeId
EpcX2InformLteCoordinatorHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2InformLteCoordinatorHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor<EpcX2InformLteCoordinatorHeader> ()
  ;
  return tid;
}

TypeId
EpcX2InformLteCoordinatorHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2InformLteCoordinatorHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void
EpcX2InformLteCoordinatorHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU16 (m_sourceCellId);
  i.WriteHtonU64 (m_ueImsi);
  i.WriteHtonU16 (m_ueRnti);
}

uint32_t
EpcX2InformLteCoordinatorHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;

  m_sourceCellId = i.ReadNtohU16();
  m_ueImsi = i.ReadNtohU64();
  m_ueRnti = i.ReadNtohU16();
  
  m_headerLength = 12;
  m_numberOfIes = 3;

  return GetSerializedSize ();
}

void
EpcX2InformLteCoordinatorHeader::Print (std::ostream &os) const
{
  os << "SourceCellId " << m_sourceCellId << " IMSI " << m_ueImsi << " RNTI " << m_ueRnti;
}

uint16_t 
EpcX2InformLteCoordinatorHeader::GetSourceCellId () const
{
  return m_sourceCellId;
}

void
EpcX2InformLteCoordinatorHeader::SetSourceCellId(uint16_t cellId)
{
  m_sourceCellId = cellId;
}

uint64_t
EpcX2InformLteCoordinatorHeader::GetUeImsi () const
{
  return m_ueImsi;
}

void
EpcX2InformLteCoordinatorHeader::SetUeImsi (uint64_t ueImsi)
{
  m_ueImsi = ueImsi;
}

uint16_t
EpcX2InformLteCoordinatorHeader::GetUeRnti () const
{
  return m_ueRnti;
}

void
EpcX2InformLteCoordinatorHeader::SetUeRnti (uint16_t ueRnti)
{
  m_ueRnti = ueRnti;
}

uint32_t
EpcX2InformLteCoordinatorHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2InformLteCoordinatorHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

uint64_t 
EpcX2InformLteCoordinatorHeader::pack754(long double f)
{
  uint16_t bits = 64;
  uint16_t expbits = 11;
  long double fnorm;
  int shift;
  long long sign, exp, significand;
  unsigned significandbits = bits - expbits - 1; // -1 for sign bit

  if (f == 0.0) return 0; // get this special case out of the way

  // check sign and begin normalization
  if (f < 0) { sign = 1; fnorm = -f; }
  else { sign = 0; fnorm = f; }

  // get the normalized form of f and track the exponent
  shift = 0;
  while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
  while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
  fnorm = fnorm - 1.0;

  // calculate the binary form (non-float) of the significand data
  significand = fnorm * ((1LL<<significandbits) + 0.5f);

  // get the biased exponent
  exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

  // return the final answer
  return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

long double 
EpcX2InformLteCoordinatorHeader::unpack754(uint64_t i)
{
  uint16_t bits = 64;
  uint16_t expbits = 11;
  long double result;
  long long shift;
  unsigned bias;
  unsigned significandbits = bits - expbits - 1; // -1 for sign bit

  if (i == 0) return 0.0;

  // pull the significand
  result = (i&((1LL<<significandbits)-1)); // mask
  result /= (1LL<<significandbits); // convert back to float
  result += 1.0f; // add the one back on

  // deal with the exponent
  bias = (1<<(expbits-1)) - 1;
  shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
  while(shift > 0) { result *= 2.0; shift--; }
  while(shift < 0) { result /= 2.0; shift++; }

  // sign it
  result *= (i>>(bits-1))&1? -1.0: 1.0;

  return result;
}
//****************************************************************************************************
NS_OBJECT_ENSURE_REGISTERED (EpcX2OptimalGnbBeamReportHeader);

EpcX2OptimalGnbBeamReportHeader::EpcX2OptimalGnbBeamReportHeader ()
  : m_numberOfIes (1 + 1 + 1 + 1 + 1 +1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1),
    m_headerLength (8 + 2 + 2 + 1 + 2 + 1 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 2)
{
}

EpcX2OptimalGnbBeamReportHeader::~EpcX2OptimalGnbBeamReportHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
}

TypeId
EpcX2OptimalGnbBeamReportHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2OptimalGnbBeamReportHeader")
    .SetParent<Header> ()
    .SetGroupName("Lte")
    .AddConstructor <EpcX2OptimalGnbBeamReportHeader> ()
  ;
  return tid;
}

TypeId
EpcX2OptimalGnbBeamReportHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2OptimalGnbBeamReportHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void 
EpcX2OptimalGnbBeamReportHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU64 (m_ueImsi);
  i.WriteHtonU16 (m_startingFrame);
  i.WriteU8 (m_startingSubframe);
  i.WriteHtonU16 (m_startingSlot);
  i.WriteU8 (m_numerology);
  i.WriteHtonU16 (m_optimalBeamIndex);
  i.WriteHtonU16 (m_secondBeamIndex);
  i.WriteHtonU16 (m_thirdBeamIndex);
  i.WriteHtonU16 (m_fourthBeamIndex);
  i.WriteHtonU16 (m_fifthBeamIndex);
  i.WriteHtonU16 (m_sixthBeamIndex);
  i.WriteHtonU16 (m_seventhBeamIndex);
  i.WriteHtonU16 (m_eigthBeamIndex);
  i.WriteHtonU16 (m_isMaxSNRCell);
}

uint32_t
EpcX2OptimalGnbBeamReportHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;

  m_ueImsi = i.ReadNtohU64 ();
  m_startingFrame = i.ReadNtohU16 ();
  m_startingSubframe = i.ReadU8 ();
  m_startingSlot = i.ReadNtohU16 ();
  m_numerology = i.ReadU8 ();
  m_optimalBeamIndex = i.ReadNtohU16 ();
  m_secondBeamIndex = i.ReadNtohU16 ();
  m_thirdBeamIndex = i.ReadNtohU16 ();
  m_fourthBeamIndex = i.ReadNtohU16 ();
  m_fifthBeamIndex = i.ReadNtohU16 ();
  m_sixthBeamIndex = i.ReadNtohU16 ();
  m_seventhBeamIndex = i.ReadNtohU16 ();
  m_eigthBeamIndex = i.ReadNtohU16 ();
  m_isMaxSNRCell = i.ReadNtohU16 ();

  m_headerLength = 32;
  m_numberOfIes = 14;

  return GetSerializedSize ();
}

void 
EpcX2OptimalGnbBeamReportHeader::Print (std::ostream &os) const
{
  os << "UE IMSI " << m_ueImsi << " Starting Frame Number " << m_startingFrame << " Starting SubFrame Number "
     << m_startingSubframe << " Starting Slot Number " << m_startingSlot << " Numerology " << m_numerology 
     << " Optimal Beam Index " << m_numerology;
}

uint64_t 
EpcX2OptimalGnbBeamReportHeader::GetUeImsi () const
{
  return m_ueImsi;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetUeImsi (uint64_t ueImsi)
{
  m_ueImsi = ueImsi;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetStartingFrame () const
{
  return m_startingFrame;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetStartingFrame (uint16_t startingFrame) 
{
  m_startingFrame = startingFrame;
}

uint8_t 
EpcX2OptimalGnbBeamReportHeader::GetStartingSubframe () const
{
  return m_startingSubframe;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetStartingSubframe (uint8_t startingSubframe)
{
  m_startingSubframe = startingSubframe;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetStartingSlot () const
{
  return m_startingSlot;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetStartingSlot (uint16_t startingSlot)
{
  m_startingSlot = startingSlot;
}

uint8_t
EpcX2OptimalGnbBeamReportHeader::GetNumerology () const
{
  return m_numerology;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetNumerology (uint8_t numerology)
{
  m_numerology = numerology;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetOptimalBeamIndex () const
{
  return m_optimalBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetOptimalBeamIndex (uint16_t optimalBeamIndex)
{
  m_optimalBeamIndex = optimalBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetSecondBeamIndex () const
{
  return m_secondBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetSecondBeamIndex (uint16_t secondBeamIndex)
{
  m_secondBeamIndex = secondBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetThirdBeamIndex () const
{
  return m_thirdBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetThirdBeamIndex (uint16_t thirdBeamIndex)
{
  m_thirdBeamIndex = thirdBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetFourthBeamIndex () const
{
  return m_fourthBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetFourthBeamIndex (uint16_t fourthBeamIndex)
{
  m_fourthBeamIndex = fourthBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetFifthBeamIndex () const
{
  return m_fifthBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetFifthBeamIndex (uint16_t fifthBeamIndex)
{
  m_fifthBeamIndex = fifthBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetSixthBeamIndex () const
{
  return m_sixthBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetSixthBeamIndex (uint16_t sixthBeamIndex)
{
  m_sixthBeamIndex = sixthBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetSeventhBeamIndex () const
{
  return m_seventhBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetSeventhBeamIndex (uint16_t seventBeamIndex)
{
  m_seventhBeamIndex = seventBeamIndex;
}

uint16_t
EpcX2OptimalGnbBeamReportHeader::GetEightBeamIndex () const
{
  return m_eigthBeamIndex;
}

void 
EpcX2OptimalGnbBeamReportHeader::SetEigthBeamIndex (uint16_t eigthBeamINdex)
{
  m_eigthBeamIndex = eigthBeamINdex;
}

bool 
EpcX2OptimalGnbBeamReportHeader::GetMaxSNRCell () const
{
  if (m_isMaxSNRCell == 1)
  {
    return true;
  }
  else if (m_isMaxSNRCell == 0)
  {
    return false;
  }
  else
  {
    NS_ABORT_MSG ("Undefined max SNR Cell");
  }
}

void 
EpcX2OptimalGnbBeamReportHeader::SetMaxSNRCell (bool isMaxSNRCell)
{
  if (isMaxSNRCell)
  {
    m_isMaxSNRCell = 1;
  }
  else
  {
    m_isMaxSNRCell = 0;
  }
}

uint32_t
EpcX2OptimalGnbBeamReportHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t
EpcX2OptimalGnbBeamReportHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

//****************************************************************************************************
NS_OBJECT_ENSURE_REGISTERED (EpcX2DeRegisterUeContextHeader);

EpcX2DeRegisterUeContextHeader::EpcX2DeRegisterUeContextHeader ()
  : m_numberOfIes (1 + 1 + 1),
    m_headerLength (8 + 1 + 1)
  {
  }

EpcX2DeRegisterUeContextHeader::~EpcX2DeRegisterUeContextHeader ()
{
  m_numberOfIes = 0;
  m_headerLength = 0;
}

TypeId
EpcX2DeRegisterUeContextHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2DeRegisterUeContextHeader")
    .SetParent<Header> ()
    .SetGroupName ("Lte")
    .AddConstructor <EpcX2DeRegisterUeContextHeader> ()
  ;
  return tid;
}

TypeId
EpcX2DeRegisterUeContextHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2DeRegisterUeContextHeader::GetSerializedSize (void) const
{
  return m_headerLength;
}

void 
EpcX2DeRegisterUeContextHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU64 (m_ueImsi);
  i.WriteU8 (m_cellId);
  i.WriteU8 (m_sourceOfCommand);
}

uint32_t
EpcX2DeRegisterUeContextHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLength = 0;

  m_ueImsi = i.ReadNtohU64 ();
  m_cellId = i.ReadU8 ();
  m_sourceOfCommand = i.ReadU8 ();

  m_headerLength = 10;
  m_numberOfIes = 3;

  return GetSerializedSize ();
}

void 
EpcX2DeRegisterUeContextHeader::Print (std::ostream &os) const
{
  os << "UE IMSI " << m_ueImsi << " Cell Id " << m_cellId;
}

uint64_t
EpcX2DeRegisterUeContextHeader::GetUeImsi () const
{
  return m_ueImsi;
}

void 
EpcX2DeRegisterUeContextHeader::SetUeImsi (uint64_t ueImsi)
{
  m_ueImsi = ueImsi;
}

uint8_t 
EpcX2DeRegisterUeContextHeader::GetCellId () const
{
  return m_cellId;
}

void 
EpcX2DeRegisterUeContextHeader::SetCellId (uint8_t cellId)
{
  m_cellId = cellId;
}

uint8_t
EpcX2DeRegisterUeContextHeader::GetSourceOfCommand () const
{
  return m_sourceOfCommand;
}

void 
EpcX2DeRegisterUeContextHeader::SetSourceOfCommand (uint8_t sourceOfCommand)
{
  m_sourceOfCommand = sourceOfCommand;
}

uint32_t
EpcX2DeRegisterUeContextHeader::GetLengthOfIes () const
{
  return m_headerLength;
}

uint32_t 
EpcX2DeRegisterUeContextHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

//****************************************************************************************************
NS_OBJECT_ENSURE_REGISTERED (EpcX2UeCSIRSSinrUpdateHeader);

EpcX2UeCSIRSSinrUpdateHeader::EpcX2UeCSIRSSinrUpdateHeader ()
  : m_numberOfIes (1 + 1 + 1),
    m_headerLenght (2 + 8 + 2)
  {
    m_map.clear ();
  }

  EpcX2UeCSIRSSinrUpdateHeader::~EpcX2UeCSIRSSinrUpdateHeader ()
  {
    m_numberOfIes = 0;
    m_headerLenght = 0;
    m_map.clear ();
  }

  TypeId
  EpcX2UeCSIRSSinrUpdateHeader::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::EpcX2UeCSIRSSinrUpdateHeader")
      .SetParent<Header> ()
      .SetGroupName ("Lte")
      .AddConstructor<EpcX2UeCSIRSSinrUpdateHeader> ()
    ;
    return tid;
  }

  TypeId
  EpcX2UeCSIRSSinrUpdateHeader::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  EpcX2UeCSIRSSinrUpdateHeader::GetSerializedSize (void) const
  {
    return m_headerLenght;
  }

  void 
  EpcX2UeCSIRSSinrUpdateHeader::Serialize (Buffer::Iterator start) const
  {
    Buffer::Iterator i = start;

    i.WriteHtonU16 (m_sourceCellId);
    i.WriteHtonU64 (m_ueImsi);

    std::vector<double>::size_type sz = m_map.size ();
    i.WriteHtonU16 (sz);          // number of elemnents in the map

    for (auto n = 0; n < m_map.size (); n++)
    {
      i.WriteHtonU64 (pack754(m_map.at(n)));
    }
  }

  uint32_t
  EpcX2UeCSIRSSinrUpdateHeader::Deserialize (Buffer::Iterator start)
  {
    Buffer::Iterator i = start;

    m_headerLenght = 0;

    m_sourceCellId = i.ReadNtohU16 ();
    m_headerLenght += 2;
    m_numberOfIes = 1;

    m_ueImsi = i.ReadNtohU64 ();
    m_headerLenght += 8;
    m_numberOfIes += 1;

    int sz = i.ReadNtohU16 ();
    for (int j = 0; j < sz; j++)
      {
        double sinr = unpack754 (i.ReadNtohU64 ());
        m_map.emplace_back(sinr) ;
      }

    m_headerLenght += 2 + sz * 16;
    m_numberOfIes += 1 + sz;

    return GetSerializedSize ();
  }

  void 
  EpcX2UeCSIRSSinrUpdateHeader::Print (std::ostream &os) const
  {
    os << "SourceCellId " << m_sourceCellId;
    os << "Ue Imsi " << m_ueImsi;
    for (auto n = 0; n < m_map.size (); n++)
    {
      os << "Sinr " << 10*std::log10(m_map.at(n));
    }
  }

  uint16_t
  EpcX2UeCSIRSSinrUpdateHeader::GetSourceCellId () const
  {
    return m_sourceCellId;
  }

  void 
  EpcX2UeCSIRSSinrUpdateHeader::SetSourceCellId (uint16_t cellId)
  {
    m_sourceCellId = cellId;
  }

  std::vector<double>
  EpcX2UeCSIRSSinrUpdateHeader::GetUeCSIRSSinrMap () const
  {
    return m_map;
  }

  void
  EpcX2UeCSIRSSinrUpdateHeader::SetUeCSIRSSinrMap (std::vector <double> map)
  {
    m_map = map;

    std::vector<double>::size_type sz = m_map.size ();
    m_headerLenght += sz * 16;
    m_numberOfIes += 1;
  }

  uint64_t 
  EpcX2UeCSIRSSinrUpdateHeader::GetUeImsi () const
  {
    return m_ueImsi;
  }

  void 
  EpcX2UeCSIRSSinrUpdateHeader::SetUeImsi (uint64_t ueImsi)
  {
    m_ueImsi = ueImsi;
  }

  uint32_t
  EpcX2UeCSIRSSinrUpdateHeader::GetLengthOfIes () const
  {
    return m_headerLenght;
  }

  uint32_t
  EpcX2UeCSIRSSinrUpdateHeader::GetNumberOfIes () const
  {
    return m_numberOfIes;
  }

  uint64_t 
  EpcX2UeCSIRSSinrUpdateHeader::pack754(long double f)
  {
    uint16_t bits = 64;
    uint16_t expbits = 11;
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
  }

  long double 
  EpcX2UeCSIRSSinrUpdateHeader::unpack754(uint64_t i)
  {
    uint16_t bits = 64;
    uint16_t expbits = 11;
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;

    return result;
  }

/////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (EpcX2UeSSBRSBeamUpdateHeader);

EpcX2UeSSBRSBeamUpdateHeader::EpcX2UeSSBRSBeamUpdateHeader ()
  : m_numberOfIes (1 + 1 + 1 + 1 + 1 + 1 + 1),
    m_headerLenght (8 + 2 + 8 + 1 + 2 + 2 + 2)
{
  m_ssbRSBeamIdVector.clear ();
}

EpcX2UeSSBRSBeamUpdateHeader::~EpcX2UeSSBRSBeamUpdateHeader ()
{
  m_numberOfIes = 0;
  m_headerLenght = 0;
  m_ssbRSBeamIdVector.clear ();
}

TypeId
EpcX2UeSSBRSBeamUpdateHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcX2UeSSBRSBeamUpdateHeader")
    .SetParent<Header> ()
    .SetGroupName ("Lte")
    .AddConstructor<EpcX2UeSSBRSBeamUpdateHeader> ()
  ;
  return tid;
}

TypeId
EpcX2UeSSBRSBeamUpdateHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
EpcX2UeSSBRSBeamUpdateHeader::GetSerializedSize (void) const
{
  return m_headerLenght;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU64 (m_ueImsi);
  i.WriteHtonU16 (m_sourceCellId);
  i.WriteHtonU64 (m_startingFrame);
  i.WriteU8 (m_startingSubframe);
  i.WriteHtonU16 (m_startingSlot);
  i.WriteHtonU16 (m_numerology);

  std::vector <uint16_t>::size_type sz = m_ssbRSBeamIdVector.size ();
  i.WriteHtonU16 (sz);
  for (int j = 0; j < (int) sz; j++)
  {
    i.WriteHtonU16 (m_ssbRSBeamIdVector[j]);
  }
}

uint32_t
EpcX2UeSSBRSBeamUpdateHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_headerLenght = 0;
  m_numberOfIes = 0;

  m_ueImsi = i.ReadNtohU64 ();
  m_headerLenght = +8;
  m_numberOfIes++;

  m_sourceCellId = i.ReadNtohU16 ();
  m_headerLenght = +2;
  m_numberOfIes++;

  m_startingFrame = i.ReadNtohU64 ();
  m_headerLenght = +8;
  m_numberOfIes++;

  m_startingSubframe = i.ReadU8 ();
  m_headerLenght = +1;
  m_numberOfIes++;

  m_startingSlot = i.ReadNtohU16 ();
  m_headerLenght = +2;
  m_numberOfIes++;

  m_numerology = i.ReadNtohU16 ();
  m_headerLenght = +2;
  m_numberOfIes++;

  int sz = i.ReadNtohU16 ();
  m_numberOfIes++;
  m_headerLenght = +2;

  for (int j = 0; j < sz; j++)
  {
    uint16_t optimalBeamIndex = i.ReadNtohU16 ();
    m_ssbRSBeamIdVector.emplace_back (optimalBeamIndex);
    m_numberOfIes++;
    m_headerLenght = +2;
  }

  return GetSerializedSize ();
}

void 
EpcX2UeSSBRSBeamUpdateHeader::Print (std::ostream &os) const
{
  os << "UE IMSI " << m_ueImsi << " Starting Frame Number " << m_startingFrame << " Starting SubFrame Number "
     << m_startingSubframe << " Starting Slot Number " << m_startingSlot << " Numerology " << m_numerology 
     << " Optimal Beam Index " << m_numerology;
}

std::vector<uint16_t>
EpcX2UeSSBRSBeamUpdateHeader::GetUeSSBRSBeamVector () const
{
  return m_ssbRSBeamIdVector;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetUeSSRBRSBeamVector (std::vector<uint16_t> vector)
{
  m_ssbRSBeamIdVector = vector;
  std::vector<uint16_t>::size_type sz = m_ssbRSBeamIdVector.size ();
  m_headerLenght += sz * 2;
  m_numberOfIes += 1;
}

uint16_t
EpcX2UeSSBRSBeamUpdateHeader::GetSourceCellId () const
{
  return m_sourceCellId;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetSourceCellId (uint16_t sourceCellId)
{
  m_sourceCellId = sourceCellId;
}

uint64_t
EpcX2UeSSBRSBeamUpdateHeader::GetUeImsi () const
{
  return m_ueImsi;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetUeImsi (uint64_t ueImsi)
{
  m_ueImsi = ueImsi;
}

uint64_t 
EpcX2UeSSBRSBeamUpdateHeader::GetStartingFrame () const
{
  return m_startingFrame;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetStartingFrame (uint64_t startingFrame)
{
  m_startingFrame = startingFrame;
}

uint8_t
EpcX2UeSSBRSBeamUpdateHeader::GetStartingSubframe () const
{
  return m_startingSubframe;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetStartingSubframe (uint8_t startingSubframe)
{
  m_startingSubframe = startingSubframe;
}

uint16_t
EpcX2UeSSBRSBeamUpdateHeader::GetStartingSlot () const
{
  return m_startingSlot;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetStartingSlot (uint16_t startingSlot)
{
  m_startingSlot = startingSlot;
}

uint16_t 
EpcX2UeSSBRSBeamUpdateHeader::GetNumerology () const
{
  return m_numerology;
}

void 
EpcX2UeSSBRSBeamUpdateHeader::SetNumerology (uint16_t numerology)
{
  m_numerology = numerology;
}

uint32_t
EpcX2UeSSBRSBeamUpdateHeader::GetLengthOfIes () const
{
  return m_headerLenght;
}

uint32_t
EpcX2UeSSBRSBeamUpdateHeader::GetNumberOfIes () const
{
  return m_numberOfIes;
}

} // namespace ns3
