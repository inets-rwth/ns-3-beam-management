/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
*
*/

#include "ideal-beamforming-algorithm.h"
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/node.h>
#include "nr-spectrum-phy.h"
#include "nr-ue-phy.h"
#include "nr-gnb-phy.h"
#include "nr-gnb-net-device.h"
#include "nr-ue-net-device.h"
#include "beam-manager.h"
#include <ns3/double.h>
#include <ns3/angles.h>
#include <ns3/uinteger.h>
#include "nr-spectrum-value-helper.h"

namespace ns3{

NS_LOG_COMPONENT_DEFINE ("IdealBeamformingAlgorithm");
NS_OBJECT_ENSURE_REGISTERED (CellScanBeamforming);
NS_OBJECT_ENSURE_REGISTERED (DirectPathBeamforming);
NS_OBJECT_ENSURE_REGISTERED (QuasiOmniDirectPathBeamforming);
NS_OBJECT_ENSURE_REGISTERED (OptimalCovMatrixBeamforming);

IdealBeamformingAlgorithm::IdealBeamformingAlgorithm ()
{
}

IdealBeamformingAlgorithm::~IdealBeamformingAlgorithm()
{

}

TypeId
IdealBeamformingAlgorithm::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::IdealBeamformingAlgorithm")
                      .SetParent<Object> ()
  ;

  return tid;
}

void
IdealBeamformingAlgorithm::GetBeamformingVectors(const Ptr<const NrGnbNetDevice>& gnbDev,
                                                 const Ptr<const NrUeNetDevice>& ueDev,
                                                 BeamformingVector* gnbBfv,
                                                 BeamformingVector* ueBfv,
                                                 uint16_t ccId) const
{
  DoGetBeamformingVectors (gnbDev, ueDev, gnbBfv, ueBfv, ccId);
}

TypeId
CellScanBeamforming::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CellScanBeamforming")
                     .SetParent<IdealBeamformingAlgorithm> ()
                     .AddConstructor<CellScanBeamforming> ()
                     .AddAttribute ("BeamSearchAngleStep",
                                    "Angle step when searching for the best beam",
                                    DoubleValue (30),
                                    MakeDoubleAccessor (&CellScanBeamforming::SetBeamSearchAngleStep,
                                                        &CellScanBeamforming::GetBeamSearchAngleStep),
                                    MakeDoubleChecker<double> ());

  return tid;
}

void
CellScanBeamforming::SetBeamSearchAngleStep (double beamSearchAngleStep)
{
  m_beamSearchAngleStep = beamSearchAngleStep;
}

double
CellScanBeamforming::GetBeamSearchAngleStep () const
{
  return m_beamSearchAngleStep;
}

void
CellScanBeamforming::DoGetBeamformingVectors (const Ptr<const NrGnbNetDevice>& gnbDev,
                                              const Ptr<const NrUeNetDevice>& ueDev,
                                              BeamformingVector* gnbBfv,
                                              BeamformingVector* ueBfv,
                                              uint16_t ccId) const
{
  NS_ABORT_MSG_IF (gnbDev == nullptr || ueDev == nullptr, "Something went wrong, gnb or UE device does not exist.");

  NS_ABORT_MSG_IF (gnbDev->GetNode ()->GetObject<MobilityModel> ()->GetDistanceFrom (ueDev->GetNode ()->GetObject<MobilityModel> ()) == 0,
                   "Beamforming method cannot be performed between two devices that are placed in the same position.");
  NS_LOG_UNCOND ("Beamforming performed from UE: " << ueDev->GetImsi() << "to gNB with cellId: " << gnbDev->GetCellId ());
  // TODO check if this is correct: assuming the ccId of gNB PHY and corresponding UE PHY are the equal
  Ptr<NrPhy> txPhyLayer = DynamicCast<NrPhy> (gnbDev->GetPhy (ccId));
  Ptr<const NrGnbPhy> txPhy = gnbDev->GetPhy (ccId);
  Ptr<const NrUePhy> rxPhy = ueDev->GetPhy (ccId);

  Ptr<const NrSpectrumPhy> txSpectrumPhy = txPhy->GetSpectrumPhy ();
  Ptr<const NrSpectrumPhy> rxSpectrumPhy = rxPhy->GetSpectrumPhy ();

  Ptr<SpectrumChannel> txSpectrumChannel = txSpectrumPhy->GetSpectrumChannel (); // SpectrumChannel should be const.. but need to change ns-3-dev
  Ptr<SpectrumChannel> rxSpectrumChannel = rxSpectrumPhy->GetSpectrumChannel ();

  Ptr<const SpectrumPropagationLossModel> txThreeGppSpectrumPropModel = txSpectrumChannel->GetSpectrumPropagationLossModel ();
  Ptr<const SpectrumPropagationLossModel> rxThreeGppSpectrumPropModel = rxSpectrumChannel->GetSpectrumPropagationLossModel ();

  NS_ASSERT_MSG (txThreeGppSpectrumPropModel == rxThreeGppSpectrumPropModel, "Devices should be connected on the same spectrum channel");

  Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity (0.0, txSpectrumPhy->GetRxSpectrumModel ());
  Ptr<SpectrumValue> noisePsd = NrSpectrumValueHelper::CreateNoisePowerSpectralDensity (rxPhy->GetNoiseFigure (), txSpectrumPhy->GetRxSpectrumModel ());

  double max = 0, maxTxTheta = 0, maxRxTheta = 0, snrAvgMax = 0, maxForVector = 0;
   std::vector<double> maxVector, maxTxThetaVector, maxRxThetaVector, maxTxSectorVector, maxRxSectorVector;
  uint16_t maxTxSector = 0, maxRxSector = 0;
  complexVector_t  maxTxW, maxRxW;

  UintegerValue uintValue;
  txPhy->GetAntennaArray ()->GetAttribute ("NumRows", uintValue);
  uint32_t txNumRows = static_cast<uint32_t> (uintValue.Get ());
  rxPhy->GetAntennaArray ()->GetAttribute ("NumRows", uintValue);
  uint32_t rxNumRows = static_cast<uint32_t> (uintValue.Get ());
  double minTxTheta;
  double topTxTheta;
  double minRxTheta;
  double topRxTheta;
  double gNBAzimuthStep, ueAzimuthStep, gNBVerticalStep, ueVerticalStep;

  NrPhy::AntennaConfiguration antennaConfig = txPhyLayer->GetAntennaConfig ();

  switch (antennaConfig)
  {
  case NrPhy::AntennaConfiguration::AntennaConfigDefault:
    gNBAzimuthStep = 180.0 / (txNumRows + 1); //For 8X8 Array at gNB-> equal to 20 degrees
    ueAzimuthStep = 180.0 / (rxNumRows + 1);  //For 4x4 Array at UE-> equal to 36 degrees
    gNBVerticalStep = txPhy->GetGnbVerticalAngleStep ();
    ueVerticalStep = rxPhy->GetUeVerticalAngleStep ();

    minRxTheta = minTxTheta = 60.0;
    topRxTheta = topTxTheta = 120.0;
    break;
  case NrPhy::AntennaConfiguration::AntennaConfigInets:
    gNBAzimuthStep = 9.0;
    ueAzimuthStep = 18.0;
    gNBVerticalStep = ueVerticalStep = 30.0;    

    minRxTheta = 30.0;
    topRxTheta = minTxTheta = 90.0;
    topTxTheta = 150.0;

    txNumRows = 180.0 / gNBAzimuthStep;
    rxNumRows = 180.0 / (ueAzimuthStep);
    break;
  default:
    NS_ABORT_MSG ("Undefined Antenna Configuration for IdealBeamformingAlgorithm");
    break;
  } 

  
  
  std::map<uint16_t, std::vector<std::pair<uint16_t, double>>> txRxSNRVector;

  for (double txTheta = minTxTheta; txTheta <= topTxTheta; txTheta = txTheta + gNBVerticalStep)
    {
      for (uint16_t txSector = 0; txSector <= txNumRows; txSector++)
        {
          NS_ASSERT(txSector < UINT16_MAX);

          txPhy->GetBeamManager()->SetSector (txSector, txTheta);
          complexVector_t txW = txPhy->GetBeamManager ()->GetCurrentBeamformingVector ();

          std::vector <std::pair<uint16_t, double>> rxSNRVector;
          
          for (double rxTheta = minRxTheta; rxTheta <= topRxTheta ; rxTheta = rxTheta + ueVerticalStep)
            {
              for (uint16_t rxSector = 0; rxSector <= rxNumRows; rxSector++)
                {
                  NS_ASSERT(rxSector < UINT16_MAX);
                  rxPhy->GetBeamManager ()->SetSector (rxSector, rxTheta);
                  complexVector_t rxW = rxPhy->GetBeamManager ()->GetCurrentBeamformingVector ();

                  NS_ABORT_MSG_IF (txW.size()==0 || rxW.size()==0, "Beamforming vectors must be initialized in order to calculate the long term matrix.");

                  Ptr<SpectrumValue> rxPsd = txThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity (fakePsd, gnbDev->GetNode ()->GetObject<MobilityModel>(), ueDev->GetNode()->GetObject<MobilityModel>());

                  SpectrumValue bfGain = (*rxPsd) / (*fakePsd);
                  SpectrumValue snr = (*rxPsd) / (*noisePsd);

                  size_t nbands = bfGain.GetSpectrumModel ()->GetNumBands ();
                  double power = Sum (bfGain) / nbands;

                  double snrAvg = Sum (snr) / (snr.GetSpectrumModel ()->GetNumBands ());
                  
                  NS_LOG_LOGIC (" Rx power: "<< power << "txTheta " << txTheta << " rxTheta " << rxTheta << " tx sector " <<
                                (M_PI *  static_cast<double> (txSector) / static_cast<double> (txNumRows) - 0.5 * M_PI) / (M_PI) * 180 << " rx sector " <<
                                (M_PI * static_cast<double> (rxSector) / static_cast<double> (rxNumRows) - 0.5 * M_PI) / (M_PI) * 180);

                  if (txTheta == 105.0 && rxTheta == 75.0)
                  {
                    rxSNRVector.emplace_back (std::pair<uint16_t, double> (rxSector, snrAvg));
                  }
                  if (max < power)
                    {
                      max = power;
                      maxTxSector = txSector;
                      maxRxSector = rxSector;
                      maxTxTheta = txTheta;
                      maxRxTheta = rxTheta;
                      maxTxW = txW;
                      maxRxW = rxW;
                      snrAvgMax = snrAvg;
                    }
                    else if (max == power)
                    { 
                      if (maxForVector < power)
                      {
                        maxForVector = power;
                        maxRxSectorVector.clear ();
                        maxTxSectorVector.clear ();
                        maxRxThetaVector.clear ();
                        maxTxThetaVector.clear ();
                      }

                        maxTxThetaVector.emplace_back (txTheta);
                        maxRxThetaVector.emplace_back (rxTheta);
                        maxTxSectorVector.emplace_back (txSector);
                        maxRxSectorVector.emplace_back (rxSector);
                    }
                }
            }

          if (rxSNRVector.size () != 0)
          {
            txRxSNRVector.insert ({txSector, rxSNRVector});
          }
        }
    }

  /*double gNBHorizontalAngle = 360.0 * (static_cast<double> (maxTxSector) / static_cast<double> (360.0 / 18.0));
  double ueHorizontalAngle = 360.0 * (static_cast<double> (maxRxSector) / static_cast<double> (360.0 / 24.0));

  maxTxSector = gNBHorizontalAngle / 18.0;
  maxRxSector = ueHorizontalAngle / 24.0;
  
  maxTxSector = 20;
  maxRxSector = 5;*/

  if (maxForVector == max && maxTxSectorVector.size () != 0 && maxForVector != 1.0000000000000026e-22 &&
      DynamicCast<ThreeGppSpectrumPropagationLossModel>(txSpectrumChannel->GetSpectrumPropagationLossModel ()) != nullptr)
  {
    maxTxThetaVector.emplace_back (maxTxTheta);
    maxRxThetaVector.emplace_back (maxRxTheta);
    maxTxSectorVector.emplace_back (maxTxSector);
    maxRxSectorVector.emplace_back (maxRxSector);
    double aodAzDiff = 0, aodEleDiff = 0, aoaAziDiff = 0, aoaEleDiff = 0, angularDiff = 0.0, minAngularDiff = 510.0;
    std::pair<std::pair<double, double>, std::pair<double, double>> aodAoaPair = 
                                      DynamicCast<ThreeGppSpectrumPropagationLossModel>(txSpectrumChannel->GetSpectrumPropagationLossModel ())->GetAodAoaFromCM (
                                      gnbDev->GetNode ()->GetObject<MobilityModel>()->GetPosition ());
    for (auto i = 0; i < maxTxSectorVector.size (); i++)
    {
      aodAzDiff = fabs((aodAoaPair.first.first / gNBAzimuthStep) - maxTxSectorVector.at(i));
      aodEleDiff = fabs ((90.0 - aodAoaPair.first.second) - maxTxThetaVector.at (i));
      aoaAziDiff = fabs ((aodAoaPair.second.first / ueAzimuthStep) - maxRxSectorVector.at (i));
      aoaEleDiff = fabs ((90.0 - aodAoaPair.second.second) - maxRxThetaVector.at(i));

      angularDiff = sqrt (pow (aodAzDiff, 2.0) + pow (aodEleDiff, 2.0) + pow (aoaAziDiff, 2.0) + pow (aoaEleDiff, 2.0));
      if (angularDiff < minAngularDiff)
      {
        minAngularDiff = angularDiff;
        maxTxSector = maxTxSectorVector.at (i);
        maxRxSector = maxRxSectorVector.at (i);
        maxTxTheta = maxTxThetaVector.at (i);
        maxRxTheta = maxRxThetaVector.at (i);
      }
    }

    txPhy->GetBeamManager()->SetSector(maxTxSector, maxTxTheta);
    rxPhy->GetBeamManager()->SetSector(maxRxSector, maxRxTheta);

    maxTxW = txPhy->GetBeamManager ()->GetCurrentBeamformingVector ();
    maxRxW = rxPhy->GetBeamManager ()->GetCurrentBeamformingVector ();
  }

  
      
  ueDev->GetPhy (ccId)->SetIdealSNRForGnb (gnbDev->GetCellId (), m_idealBeamformingSNROffset * snrAvgMax,
                          BeamId (maxRxSector, maxRxTheta));
  *gnbBfv = BeamformingVector (std::make_pair(maxTxW, BeamId (maxTxSector, maxTxTheta)));
  *ueBfv = BeamformingVector (std::make_pair (maxRxW, BeamId (maxRxSector, maxRxTheta)));

  NS_LOG_DEBUG ("Beamforming vectors for gNB with node id: "<< gnbDev->GetNode()->GetId () <<
                " and UE with node id: " << ueDev->GetNode()->GetId () <<
                " are txTheta " << maxTxTheta << " rxTheta " << maxRxTheta <<
                " tx sector " << (M_PI * static_cast<double> (maxTxSector) / static_cast<double> (txNumRows) - 0.5 * M_PI) / (M_PI) * 180 <<
                " rx sector " << (M_PI * static_cast<double> (maxRxSector) / static_cast<double> (rxNumRows) - 0.5 * M_PI) / (M_PI) * 180);
}


TypeId
CellScanQuasiOmniBeamforming::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CellScanQuasiOmniBeamforming")
                     .SetParent<IdealBeamformingAlgorithm> ()
                     .AddConstructor<CellScanQuasiOmniBeamforming>()
                     .AddAttribute ("BeamSearchAngleStep",
                                    "Angle step when searching for the best beam",
                                    DoubleValue (30),
                                    MakeDoubleAccessor (&CellScanQuasiOmniBeamforming::SetBeamSearchAngleStep,
                                                        &CellScanQuasiOmniBeamforming::GetBeamSearchAngleStep),
                                    MakeDoubleChecker<double> ());

  return tid;
}

void
CellScanQuasiOmniBeamforming::SetBeamSearchAngleStep (double beamSearchAngleStep)
{
  m_beamSearchAngleStep = beamSearchAngleStep;
}

double
CellScanQuasiOmniBeamforming::GetBeamSearchAngleStep () const
{
  return m_beamSearchAngleStep;
}

void
CellScanQuasiOmniBeamforming::DoGetBeamformingVectors (const Ptr<const NrGnbNetDevice>& gnbDev,
                                                       const Ptr<const NrUeNetDevice>& ueDev,
                                                       BeamformingVector* gnbBfv,
                                                       BeamformingVector* ueBfv,
                                                       uint16_t ccId) const
{
  NS_ABORT_MSG_IF (gnbDev == nullptr || ueDev == nullptr,
                   "Something went wrong, gnb or UE device does not exist.");

  NS_ABORT_MSG_IF (gnbDev->GetNode ()->GetObject<MobilityModel> ()->GetDistanceFrom (ueDev->GetNode ()->GetObject<MobilityModel> ()) == 0,
                   "Beamforming method cannot be performed between two devices that are placed in the same position.");

  // TODO check if this is correct: assuming the ccId of gNB PHY and corresponding UE PHY are the equal
  Ptr<const NrGnbPhy> txPhy = gnbDev->GetPhy (ccId);
  Ptr<const NrUePhy> rxPhy = ueDev->GetPhy (ccId);

  Ptr<const SpectrumPropagationLossModel> txThreeGppSpectrumPropModel = txPhy->GetSpectrumPhy ()->GetSpectrumChannel ()->GetSpectrumPropagationLossModel ();
  Ptr<const SpectrumPropagationLossModel> rxThreeGppSpectrumPropModel = rxPhy->GetSpectrumPhy ()->GetSpectrumChannel ()->GetSpectrumPropagationLossModel ();
  NS_ASSERT_MSG (txThreeGppSpectrumPropModel == rxThreeGppSpectrumPropModel, "Devices should be connected to the same spectrum channel");
  Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity (0.0, txPhy->GetSpectrumPhy ()->GetRxSpectrumModel ());

  double max = 0, maxTxTheta = 0;
  uint16_t maxTxSector = 0;
  complexVector_t maxTxW;

  UintegerValue uintValue;
  txPhy->GetAntennaArray ()->GetAttribute("NumRows", uintValue);
  uint32_t txNumRows = static_cast<uint32_t> (uintValue.Get ());

  rxPhy->GetBeamManager ()->ChangeToOmniTx (); // we have to set it inmediatelly to q-omni so that we can perform calculations when calling spectrum model above

  complexVector_t rxW = rxPhy->GetBeamManager ()->GetCurrentBeamformingVector ();
  *ueBfv = std::make_pair (rxW, OMNI_BEAM_ID);

  for (double txTheta = 60; txTheta < 121; txTheta = txTheta + m_beamSearchAngleStep)
    {
      for (uint16_t txSector = 0; txSector <= txNumRows; txSector++)
        {
          NS_ASSERT(txSector < UINT16_MAX);

          txPhy->GetBeamManager ()->SetSector (txSector, txTheta);
          complexVector_t txW = txPhy->GetBeamManager ()->GetCurrentBeamformingVector ();

          NS_ABORT_MSG_IF (txW.size ()== 0 || rxW.size ()== 0,
                           "Beamforming vectors must be initialized in order to calculate the long term matrix.");
          Ptr<SpectrumValue> rxPsd = txThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity
              (fakePsd, gnbDev->GetNode ()->GetObject<MobilityModel> (), ueDev->GetNode ()->GetObject<MobilityModel> ());

          size_t nbands = rxPsd->GetSpectrumModel ()->GetNumBands ();
          double power = Sum (*rxPsd) / nbands;

          NS_LOG_LOGIC (" Rx power: "<< power << "txTheta " << txTheta  << " tx sector " <<
                        (M_PI *  static_cast<double> (txSector) / static_cast<double>(txNumRows) - 0.5 * M_PI) / (M_PI) * 180);

          if (max < power)
             {
                max = power;
                maxTxSector = txSector;
                maxTxTheta = txTheta;
                maxTxW = txW;
             }

        }
    }

  *gnbBfv = BeamformingVector (std::make_pair(maxTxW, BeamId (maxTxSector, maxTxTheta)));

  NS_LOG_DEBUG ("Beamforming vectors for gNB with node id: "<< gnbDev->GetNode()->GetId () <<
                " and UE with node id: " << ueDev->GetNode()->GetId () <<
                " are txTheta " << maxTxTheta << " tx sector " <<
                (M_PI * static_cast<double> (maxTxSector) / static_cast<double> (txNumRows) - 0.5 * M_PI) / (M_PI) * 180);
}

TypeId
DirectPathBeamforming::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DirectPathBeamforming")
                     .SetParent<IdealBeamformingAlgorithm> ()
                     .AddConstructor<DirectPathBeamforming>()
  ;
  return tid;
}


void
DirectPathBeamforming::DoGetBeamformingVectors (const Ptr<const NrGnbNetDevice> &gnbDev,
                                                const Ptr<const NrUeNetDevice> &ueDev,
                                                BeamformingVector* gnbBfv,
                                                BeamformingVector* ueBfv,
                                                uint16_t ccId) const
{
  NS_LOG_FUNCTION (this);

  Ptr<MobilityModel> gnbMob = gnbDev->GetNode ()->GetObject<MobilityModel> ();
  Ptr<MobilityModel> ueMob = ueDev->GetNode ()->GetObject<MobilityModel> ();
  Ptr<const ThreeGppAntennaArrayModel> gnbAntenna = gnbDev->GetPhy (ccId)->GetAntennaArray ();
  Ptr<const ThreeGppAntennaArrayModel> ueAntenna = ueDev->GetPhy (ccId)->GetAntennaArray ();

  complexVector_t gNbAntennaWeights = CreateDirectPathBfv (gnbMob, ueMob, gnbAntenna);
  // store the antenna weights
  *gnbBfv = BeamformingVector (std::make_pair (gNbAntennaWeights, BeamId::GetEmptyBeamId ()));


  complexVector_t ueAntennaWeights = CreateDirectPathBfv (ueMob, gnbMob, ueAntenna);
  // store the antenna weights
  *ueBfv = BeamformingVector (std::make_pair(ueAntennaWeights, BeamId::GetEmptyBeamId ()));

}

TypeId
QuasiOmniDirectPathBeamforming::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::QuasiOmniDirectPathBeamforming")
                      .SetParent<DirectPathBeamforming> ()
                      .AddConstructor<QuasiOmniDirectPathBeamforming>();
  return tid;
}


void
QuasiOmniDirectPathBeamforming::DoGetBeamformingVectors (const Ptr<const NrGnbNetDevice> &gnbDev,
                                                         const Ptr<const NrUeNetDevice> &ueDev,
                                                         BeamformingVector* gnbBfv,
                                                         BeamformingVector* ueBfv,
                                                         uint16_t ccId) const
{
  NS_LOG_FUNCTION (this);

  Ptr<MobilityModel> gnbMob = gnbDev->GetNode ()->GetObject<MobilityModel> ();
  Ptr<MobilityModel> ueMob = ueDev->GetNode ()->GetObject<MobilityModel> ();
  Ptr<const ThreeGppAntennaArrayModel> gnbAntenna = gnbDev->GetPhy (ccId)->GetAntennaArray ();
  Ptr<const ThreeGppAntennaArrayModel> ueAntenna = ueDev->GetPhy (ccId)->GetAntennaArray ();

  // configure gNb beamforming vector to be quasi omni
  UintegerValue numRows, numColumns;
  gnbAntenna->GetAttribute ("NumRows", numRows);
  gnbAntenna->GetAttribute ("NumColumns", numColumns);
  *gnbBfv = std::make_pair (CreateQuasiOmniBfv (numRows.Get (), numColumns.Get ()), OMNI_BEAM_ID);

  //configure UE beamforming vector to be directed towards gNB
  complexVector_t ueAntennaWeights = CreateDirectPathBfv (ueMob, gnbMob, ueAntenna);
  // store the antenna weights
  *ueBfv = BeamformingVector (std::make_pair (ueAntennaWeights, BeamId::GetEmptyBeamId ()));
}

TypeId
DirectPathQuasiOmniBeamforming::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DirectPathQuasiOmniBeamforming")
                      .SetParent<DirectPathBeamforming> ()
                      .AddConstructor<DirectPathQuasiOmniBeamforming>();
  return tid;
}


void
DirectPathQuasiOmniBeamforming::DoGetBeamformingVectors (const Ptr<const NrGnbNetDevice> &gnbDev,
                                                         const Ptr<const NrUeNetDevice> &ueDev,
                                                         BeamformingVector* gnbBfv,
                                                         BeamformingVector* ueBfv,
                                                         uint16_t ccId) const
{
  NS_LOG_FUNCTION (this);

  Ptr<MobilityModel> gnbMob = gnbDev->GetNode ()->GetObject<MobilityModel> ();
  Ptr<MobilityModel> ueMob = ueDev->GetNode ()->GetObject<MobilityModel> ();
  Ptr<const ThreeGppAntennaArrayModel> gnbAntenna = gnbDev->GetPhy (ccId)->GetAntennaArray ();
  Ptr<const ThreeGppAntennaArrayModel> ueAntenna = ueDev->GetPhy (ccId)->GetAntennaArray ();

  // configure ue beamforming vector to be quasi omni
  UintegerValue numRows, numColumns;
  ueAntenna->GetAttribute ("NumRows", numRows);
  ueAntenna->GetAttribute ("NumColumns", numColumns);
  *ueBfv = std::make_pair (CreateQuasiOmniBfv (numRows.Get (), numColumns.Get ()), OMNI_BEAM_ID);

  //configure gNB beamforming vector to be directed towards UE
  complexVector_t gnbAntennaWeights = CreateDirectPathBfv (gnbMob, ueMob, gnbAntenna);
  // store the antenna weights
  *gnbBfv = BeamformingVector (std::make_pair (gnbAntennaWeights, BeamId::GetEmptyBeamId ()));
}


TypeId
OptimalCovMatrixBeamforming::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::OptimalCovMatrixBeamforming")
    .SetParent<IdealBeamformingAlgorithm> ()
    .AddConstructor<OptimalCovMatrixBeamforming>()
  ;

  return tid;
}

void
OptimalCovMatrixBeamforming::DoGetBeamformingVectors (const Ptr<const NrGnbNetDevice>& gnbDev,
                                                      const Ptr<const NrUeNetDevice>& ueDev,
                                                      BeamformingVector* gnbBfv,
                                                      BeamformingVector* ueBfv,
                                                      uint16_t ccId) const
{
  NS_UNUSED (gnbDev);
  NS_UNUSED (ueDev);
  NS_UNUSED (gnbBfv);
  NS_UNUSED (ueBfv);
}

} // end of ns3 namespace
