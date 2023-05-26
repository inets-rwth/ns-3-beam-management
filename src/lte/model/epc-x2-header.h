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

#ifndef EPC_X2_HEADER_H
#define EPC_X2_HEADER_H

#include "ns3/epc-x2-sap.h"
#include "ns3/header.h"

#include <vector>


namespace ns3 {


class EpcX2Header : public Header
{
public:
  EpcX2Header ();
  virtual ~EpcX2Header ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get message type function
   * \returns the message type
   */
  uint8_t GetMessageType () const;
  /**
   * Set message type function
   * \param messageType the message type
   */
  void SetMessageType (uint8_t messageType);

  /**
   * Get procedure code function
   * \returns the procedure code
   */
  uint8_t GetProcedureCode () const;
  /**
   * Set procedure code function
   * \param procedureCode the procedure code
   */
  void SetProcedureCode (uint8_t procedureCode);

  /**
   * Set length of IEs function
   * \param lengthOfIes the length of IEs
   */
  void SetLengthOfIes (uint32_t lengthOfIes);
  /**
   * Set number of IEs function
   * \param numberOfIes the number of IEs
   */
  void SetNumberOfIes (uint32_t numberOfIes);


  /// Procedure code enumeration
  enum ProcedureCode_t {
    HandoverPreparation     = 0,
    LoadIndication          = 2,
    SnStatusTransfer        = 4,
    UeContextRelease        = 5,
    ResourceStatusReporting = 10,
    UpdateUeSinr            = 14,
    RequestCoordinatorHandover       = 15,
    InformLteCoordinator    = 16,
    OptimalGnbBeamReport = 17,
    DeRegisterUeContext = 18,
    DeRegisterUeCompletedUpdate = 19,
    CSIRSRRCReport = 20,
    SSBRSReport = 21
  };

  /// Type of message enumeration
  enum TypeOfMessage_t {
    InitiatingMessage       = 0,
    SuccessfulOutcome       = 1,
    UnsuccessfulOutcome     = 2
  };

private:
  uint8_t m_messageType; ///< message type
  uint8_t m_procedureCode; ///< procedure code

  uint32_t m_lengthOfIes; ///< length of IEs
  uint32_t m_numberOfIes; ///< number of IEs
};


/**
 * EpcX2HandoverRequestHeader
 */
class EpcX2HandoverRequestHeader : public Header
{
public:
  EpcX2HandoverRequestHeader ();
  virtual ~EpcX2HandoverRequestHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get old ENB X2 AP ID function
   * \returns the old ENB UE X2 AP ID
   */
  uint16_t GetOldEnbUeX2apId () const;
  /**
   * Set old ENB X2 AP ID function
   * \param x2apId the X2 AP ID
   */
  void SetOldEnbUeX2apId (uint16_t x2apId);

  /**
   * Get cause function
   * \returns the cause
   */
  uint16_t GetCause () const;
  /**
   * Set cause function
   * \param cause
   */
  void SetCause (uint16_t cause);

  /**
   * Get target cell id function
   * \returns the target cell ID
   */
  uint16_t GetTargetCellId () const;
  /**
   * Set target cell id function
   * \param targetCellId the target cell ID
   */
  void SetTargetCellId (uint16_t targetCellId);

  /**
   * Get MME UE S1 AP ID function
   * \returns the MME UE S1 AP ID
   */
  uint32_t GetMmeUeS1apId () const;
  /**
   * Set MME UE S1 AP ID function
   * \param mmeUeS1apId the MME UE S1 AP ID
   */
  void SetMmeUeS1apId (uint32_t mmeUeS1apId);

  /**
   * Get bearers function
   * \returns <EpcX2Sap::ErabToBeSetupItem>
   */
  std::vector <EpcX2Sap::ErabToBeSetupItem> GetBearers () const;
  /**
   * Set bearers function
   * \param bearers std::vector <EpcX2Sap::ErabToBeSetupItem>
   */ 
  void SetBearers (std::vector <EpcX2Sap::ErabToBeSetupItem> bearers);

  /**
   * Get UE Aggregate Max Bit Rate Downlink function
   * \returns the UE aggregate max bit rate downlink
   */
  uint64_t GetUeAggregateMaxBitRateDownlink () const;
  /**
   * Set UE Aggregrate Max Bit Rate Downlink function
   * \param bitRate the bit rate
   */
  void SetUeAggregateMaxBitRateDownlink (uint64_t bitRate);

  /**
   * Get UE Aggregrate Max Bit Rate Uplik function
   * \returns the UE aggregate max bit rate uplink
   */
  uint64_t GetUeAggregateMaxBitRateUplink () const;
  /**
   * Set UE Aggregrate Max Bit Rate Uplik function
   * \param bitRate the bit rate
   */
  void SetUeAggregateMaxBitRateUplink (uint64_t bitRate);

  /**
   * Get length of IEs
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Get number of IEs
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< header length

  uint16_t          m_oldEnbUeX2apId; ///< old ENB UE X1 AP ID
  uint16_t          m_cause; ///< cause
  uint16_t          m_targetCellId; ///< target cell ID
  uint32_t          m_mmeUeS1apId; ///< MME UE S1 AP ID
  uint64_t          m_ueAggregateMaxBitRateDownlink; ///< aggregate max bit rate downlink
  uint64_t          m_ueAggregateMaxBitRateUplink; ///< aggregate max bit rate uplink
  std::vector <EpcX2Sap::ErabToBeSetupItem> m_erabsToBeSetupList; ///< ERAB to be setup list
};

/**
 * EpcX2HandoverRequestAckHeader
 */
class EpcX2HandoverRequestAckHeader : public Header
{
public:
  EpcX2HandoverRequestAckHeader ();
  virtual ~EpcX2HandoverRequestAckHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get old ENB UE X2 AP ID function
   * \returns the old ENB UE X2 AP ID
   */
  uint16_t GetOldEnbUeX2apId () const;
  /**
   * Set old ENB UE X2 AP ID function
   * \param x2apId the old ENB UE X2 AP ID
   */
  void SetOldEnbUeX2apId (uint16_t x2apId);

  /**
   * Get new ENB UE X2 AP ID function
   * \returns the new ENB UE X2 AP ID
   */
  uint16_t GetNewEnbUeX2apId () const;
  /**
   * Set new ENB UE X2 AP ID function
   * \param x2apId the new ENB UE X2 AP ID
   */
  void SetNewEnbUeX2apId (uint16_t x2apId);

  /**
   * Get admittied bearers function
   * \returns <EpcX2Sap::ErabAdmittedItem>
   */
  std::vector <EpcX2Sap::ErabAdmittedItem> GetAdmittedBearers () const;
  /**
   * Set admitted bearers function
   * \param bearers the admitted bearers
   */
  void SetAdmittedBearers (std::vector <EpcX2Sap::ErabAdmittedItem> bearers);

  /**
   * Get not admitted bearers function
   * \returns the not admitted bearers
   */
  std::vector <EpcX2Sap::ErabNotAdmittedItem> GetNotAdmittedBearers () const;
  /**
   * Set not admitted bearers function
   * \param bearers the not admitted bearers
   */
  void SetNotAdmittedBearers (std::vector <EpcX2Sap::ErabNotAdmittedItem> bearers);

  /**
   * Get length of IEs function
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Get number of IEs function
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< header length

  uint16_t          m_oldEnbUeX2apId; ///< old ENB UE X2 AP ID
  uint16_t          m_newEnbUeX2apId; ///< new ENB UE X2 AP ID
  std::vector <EpcX2Sap::ErabAdmittedItem>     m_erabsAdmittedList; ///< ERABs admitted list
  std::vector <EpcX2Sap::ErabNotAdmittedItem>  m_erabsNotAdmittedList; ///< ERABs not admitted list
};


/**
 * EpcX2HandoverPreparationFailureHeader
 */
class EpcX2HandoverPreparationFailureHeader : public Header
{
public:
  EpcX2HandoverPreparationFailureHeader ();
  virtual ~EpcX2HandoverPreparationFailureHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get old ENB UE X2 AP ID function
   * \returns the old ENB UE X2 AP ID
   */
  uint16_t GetOldEnbUeX2apId () const;
  /**
   * Set old ENB UE X2 AP ID function
   * \param x2apId the old ENB UE X2 AP ID
   */
  void SetOldEnbUeX2apId (uint16_t x2apId);

  /**
   * Get cause function
   * \returns the cause
   */
  uint16_t GetCause () const;
  /**
   * Set cause function
   * \param cause
   */
  void SetCause (uint16_t cause);

  /**
   * Get criticality diagnostics function
   * \returns the criticality diagnostics
   */
  uint16_t GetCriticalityDiagnostics () const;
  /**
   * Set criticality diagnostics function
   * \param criticalityDiagnostics the criticality diagnostics
   */
  void SetCriticalityDiagnostics (uint16_t criticalityDiagnostics);

  /**
   * Get length of IEs function
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Get number of IEs function
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< header length

  uint16_t          m_oldEnbUeX2apId; ///< old ENB UE X2 AP ID
  uint16_t          m_cause; ///< cause
  uint16_t          m_criticalityDiagnostics; ///< criticality diagnostics
};


/**
 * EpcX2SnStatusTransferHeader
 */
class EpcX2SnStatusTransferHeader : public Header
{
public:
  EpcX2SnStatusTransferHeader ();
  virtual ~EpcX2SnStatusTransferHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get old ENB UE X2 AP ID function
   * \returns the old ENB UE X2 AP ID
   */
  uint16_t GetOldEnbUeX2apId () const;
  /**
   * Set old ENB UE X2 AP ID function
   * \param x2apId the old ENB UE X2 AP ID
   */
  void SetOldEnbUeX2apId (uint16_t x2apId);

  /**
   * Get new ENB UE X2 AP ID function
   * \returns the new ENB UE X2AP ID
   */
  uint16_t GetNewEnbUeX2apId () const;
  /**
   * Set new ENB UE X2 AP ID function
   * \param x2apId the new ENB UE X2AP ID
   */
  void SetNewEnbUeX2apId (uint16_t x2apId);

  /**
   * Get ERABs subject to status transfer list function
   * \returns std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem>
   */
  std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> GetErabsSubjectToStatusTransferList () const;
  /**
   * Set ERABs subject to status transfer list function
   * \param erabs std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem>
   */
  void SetErabsSubjectToStatusTransferList (std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> erabs);

  /**
   * Get length of IEs function
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Get number of IEs function
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< header length

  uint16_t          m_oldEnbUeX2apId; ///< old ENB UE X2 AP ID 
  uint16_t          m_newEnbUeX2apId; ///< new ENB UE X2 AP ID
  std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> m_erabsSubjectToStatusTransferList; ///< ERABs subject to status transfer list
};

/**
 * EpcX2UeContextReleaseHeader
 */
class EpcX2UeContextReleaseHeader : public Header
{
public:
  EpcX2UeContextReleaseHeader ();
  virtual ~EpcX2UeContextReleaseHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get old ENB UE X2 AP ID function
   * \returns the old ENB UE X2 AP ID
   */
  uint16_t GetOldEnbUeX2apId () const;
  /**
   * Set old ENB UE X2 AP ID function
   * \param x2apId the old ENB UE X2 AP ID
   */
  void SetOldEnbUeX2apId (uint16_t x2apId);

  /**
   * Get new ENB UE X2 AP ID function
   * \returns the new ENB UE X2 AP ID
   */
  uint16_t GetNewEnbUeX2apId () const;
  /**
   * Set new ENB UE X2 AP ID function
   * \param x2apId the new ENB UE X2 AP ID
   */
  void SetNewEnbUeX2apId (uint16_t x2apId);

  /**
   * Get length of IEs function
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Set length of IEs function
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< header length

  uint16_t          m_oldEnbUeX2apId; ///< old ENB UE X2 AP ID
  uint16_t          m_newEnbUeX2apId; ///< new ENB UE X2 AP ID
};


/**
 * EpcX2LoadInformationHeader
 */
class EpcX2LoadInformationHeader : public Header
{
public:
  EpcX2LoadInformationHeader ();
  virtual ~EpcX2LoadInformationHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get cell information list function
   * \returns std::vector <EpcX2Sap::CellInformationItem>
   */
  std::vector <EpcX2Sap::CellInformationItem> GetCellInformationList () const;
  /**
   * Set cell information list function
   * \param cellInformationList std::vector <EpcX2Sap::CellInformationItem> 
   */
  void SetCellInformationList (std::vector <EpcX2Sap::CellInformationItem> cellInformationList);

  /**
   * Get length of IEs function
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Get number of IEs function
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< length of IEs

  std::vector <EpcX2Sap::CellInformationItem> m_cellInformationList; ///< cell information list
};


/**
 * EpcX2ResourceStatusUpdateHeader
 */
class EpcX2ResourceStatusUpdateHeader : public Header
{
public:
  EpcX2ResourceStatusUpdateHeader ();
  virtual ~EpcX2ResourceStatusUpdateHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  /**
   * Get ENB1 measurement ID function
   * \returns the ENB1 measurement ID
   */
  uint16_t GetEnb1MeasurementId () const;
  /**
   * Set ENB1 measurement ID function
   * \param enb1MeasurementId the ENB1 measurement ID
   */
  void SetEnb1MeasurementId (uint16_t enb1MeasurementId);

  /**
   * Get ENB2 measurement ID function
   * \returns the ENB2 measurement ID
   */
  uint16_t GetEnb2MeasurementId () const;
  /**
   * Set ENB2 measurement ID function
   * \param enb2MeasurementId ENB2 measruement ID
   */
  void SetEnb2MeasurementId (uint16_t enb2MeasurementId);

  /**
   * Get cell measurement results list function
   * \returns the cell measurement results list
   */ 
  std::vector <EpcX2Sap::CellMeasurementResultItem> GetCellMeasurementResultList () const;
  /**
   * Set cell measurement results list function
   * \param cellMeasurementResultList the cell measurement results list
   */
  void SetCellMeasurementResultList (std::vector <EpcX2Sap::CellMeasurementResultItem> cellMeasurementResultList);

  /**
   * Get length of IEs function
   * \returns the length of IEs
   */
  uint32_t GetLengthOfIes () const;
  /**
   * Get number of IEs function
   * \returns the number of IEs
   */
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes; ///< number of IEs
  uint32_t          m_headerLength; ///< header length

  uint16_t          m_enb1MeasurementId; ///< ENB1 measurement
  uint16_t          m_enb2MeasurementId; ///< ENB2 measurement
  std::vector <EpcX2Sap::CellMeasurementResultItem> m_cellMeasurementResultList; ///< cell measurement result list
};

class EpcX2UeImsiSinrUpdateHeader : public Header
{
public:
  EpcX2UeImsiSinrUpdateHeader ();
  virtual ~EpcX2UeImsiSinrUpdateHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  std::map <uint64_t, double> GetUeImsiSinrMap () const;
  void SetUeImsiSinrMap (std::map<uint64_t, double> map);

  uint16_t GetSourceCellId () const;
  void SetSourceCellId (uint16_t sourceCellId);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  // from http://beej.us/guide/bgnet/examples/ieee754.c, to convert
  // uint64_t to double and viceversa according to IEEE754 format
  static uint64_t pack754(long double f);
  static long double unpack754(uint64_t i);

  std::map <uint64_t, double> m_map;
  uint16_t m_sourceCellId;
};

class EpcX2CoordinatorHandoverHeader : public Header
{
public:
  EpcX2CoordinatorHandoverHeader ();
  virtual ~EpcX2CoordinatorHandoverHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

  uint16_t GetTargetCellId () const;
  void SetTargetCellId (uint16_t targetCellId);

  uint16_t GetOldCellId () const;
  void SetOldCellId (uint16_t oldCellId);

  uint64_t GetImsi () const;
  void SetImsi (uint64_t imsi);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  uint16_t          m_targetCellId;
  uint16_t          m_oldCellId;
  uint64_t          m_imsi;
};

class EpcX2InformLteCoordinatorHeader : public Header
{
public:
  EpcX2InformLteCoordinatorHeader ();
  virtual ~EpcX2InformLteCoordinatorHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint64_t GetUeImsi () const;
  void SetUeImsi (uint64_t ueImsi);

  uint16_t GetUeRnti () const;
  void SetUeRnti (uint16_t ueRnti);

  uint16_t GetSourceCellId () const;
  void SetSourceCellId (uint16_t sourceCellId);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  // from http://beej.us/guide/bgnet/examples/ieee754.c, to convert
  // uint64_t to double and viceversa according to IEEE754 format
  static uint64_t pack754(long double f);
  static long double unpack754(uint64_t i);

  uint64_t m_ueImsi;
  uint16_t m_ueRnti;
  uint16_t m_sourceCellId;
};

class EpcX2OptimalGnbBeamReportHeader : public Header
{
  public:
    EpcX2OptimalGnbBeamReportHeader ();
    virtual ~EpcX2OptimalGnbBeamReportHeader ();

    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start);
    virtual void Print (std::ostream &os) const;

    uint64_t GetUeImsi () const;
    void SetUeImsi (uint64_t ueImsi);

    uint16_t GetOptimalBeamIndex () const;
    void SetOptimalBeamIndex (uint16_t optimalBeamIndex);

    uint16_t GetSecondBeamIndex () const;
    void SetSecondBeamIndex (uint16_t secondBeamIndex);

    uint16_t GetThirdBeamIndex () const;
    void SetThirdBeamIndex (uint16_t thirdBeamIndex);

    uint16_t GetFourthBeamIndex () const;
    void SetFourthBeamIndex (uint16_t fourthBeamIndex);

    uint16_t GetFifthBeamIndex () const;
    void SetFifthBeamIndex (uint16_t optimalBeamIndex);

    uint16_t GetSixthBeamIndex () const;
    void SetSixthBeamIndex (uint16_t secondBeamIndex);

    uint16_t GetSeventhBeamIndex () const;
    void SetSeventhBeamIndex (uint16_t thirdBeamIndex);

    uint16_t GetEightBeamIndex () const;
    void SetEigthBeamIndex (uint16_t fourthBeamIndex);

    uint16_t GetStartingFrame () const;
    void SetStartingFrame (uint16_t startingFrame);

    uint8_t GetStartingSubframe () const;
    void SetStartingSubframe (uint8_t startingSubframe);

    uint16_t GetStartingSlot () const;
    void SetStartingSlot (uint16_t startingSlot);

    uint8_t GetNumerology () const;
    void SetNumerology (uint8_t numerology);

    bool GetMaxSNRCell () const;
    void SetMaxSNRCell (bool isMaxSNRCell);

    uint32_t GetLengthOfIes () const;
    uint32_t GetNumberOfIes () const;

  private:
    uint32_t          m_numberOfIes; ///< number of IEs
    uint32_t          m_headerLength; ///< header length

    uint64_t          m_ueImsi;
    uint16_t          m_optimalBeamIndex;
    uint16_t          m_secondBeamIndex;
    uint16_t          m_thirdBeamIndex;
    uint16_t          m_fourthBeamIndex;
    uint16_t          m_fifthBeamIndex;
    uint16_t          m_sixthBeamIndex;
    uint16_t          m_seventhBeamIndex;
    uint16_t          m_eigthBeamIndex;
    uint16_t          m_startingFrame;
    uint8_t           m_startingSubframe;
    uint16_t          m_startingSlot;
    uint16_t           m_numerology;
    bool              m_isMaxSNRCell;
};

class EpcX2DeRegisterUeContextHeader : public Header
{
  public:
    EpcX2DeRegisterUeContextHeader ();
    virtual ~EpcX2DeRegisterUeContextHeader ();

    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (Buffer::Iterator start) const;
    virtual uint32_t Deserialize (Buffer::Iterator start);
    virtual void Print (std::ostream &os) const;

    uint64_t GetUeImsi () const;
    void SetUeImsi (uint64_t ueImsi);

    uint8_t GetCellId () const;
    void SetCellId (uint8_t cellId);

    uint8_t GetSourceOfCommand () const;
    void SetSourceOfCommand (uint8_t sourceOfCommand);

    uint32_t GetLengthOfIes () const;
    uint32_t GetNumberOfIes () const;

  private:
    uint32_t          m_numberOfIes; ///< number of IEs
    uint32_t          m_headerLength; ///< header length

    uint64_t          m_ueImsi;
    uint8_t           m_cellId;
    uint8_t           m_sourceOfCommand;
};

class EpcX2UeCSIRSSinrUpdateHeader : public Header
{
public:
  EpcX2UeCSIRSSinrUpdateHeader ();
  virtual ~EpcX2UeCSIRSSinrUpdateHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

  std::vector<double> GetUeCSIRSSinrMap () const;
  void SetUeCSIRSSinrMap (std::vector<double> map);

  uint16_t GetSourceCellId () const;
  void SetSourceCellId (uint16_t sourceCellId);

  uint64_t GetUeImsi () const;
  void SetUeImsi (uint64_t ueImsi);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t m_numberOfIes;
  uint32_t m_headerLenght;

  // from http://beej.us/guide/bgnet/examples/ieee754.c, to convert
  // uint64_t to double and viceversa according to IEEE754 format
  static uint64_t pack754(long double f);
  static long double unpack754(uint64_t i);

  std::vector<double> m_map;
  uint64_t m_ueImsi;
  uint16_t m_sourceCellId; 
};

class EpcX2UeSSBRSBeamUpdateHeader : public Header
{
public: 
  EpcX2UeSSBRSBeamUpdateHeader ();
  virtual ~EpcX2UeSSBRSBeamUpdateHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

  std::vector<uint16_t> GetUeSSBRSBeamVector () const;
  void SetUeSSRBRSBeamVector (std::vector<uint16_t> vector);

  uint16_t GetSourceCellId () const;
  void SetSourceCellId (uint16_t sourceCellId);

  uint64_t GetUeImsi () const;
  void SetUeImsi (uint64_t ueImsi);

  uint64_t GetStartingFrame () const;
  void SetStartingFrame (uint64_t startingFrame);

  uint8_t GetStartingSubframe () const;
  void SetStartingSubframe (uint8_t startingSubframe);

  uint16_t GetStartingSlot () const;
  void SetStartingSlot (uint16_t startingSlot);

  uint16_t GetNumerology () const;
  void SetNumerology (uint16_t numerology);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t m_numberOfIes;
  uint32_t m_headerLenght;

  uint64_t m_ueImsi;
  uint16_t m_sourceCellId;
  uint64_t m_startingFrame;
  uint8_t  m_startingSubframe;
  uint16_t m_startingSlot;
  uint16_t m_numerology;
  std::vector<uint16_t> m_ssbRSBeamIdVector;
  
};

} // namespace ns3

#endif // EPC_X2_HEADER_H
