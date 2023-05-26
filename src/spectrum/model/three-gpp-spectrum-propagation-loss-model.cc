/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering,
 * New York University
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
 * University of Padova
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
 */

#include "ns3/log.h"
#include "three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/net-device.h"
#include "ns3/three-gpp-antenna-array-model.h"
#include "ns3/node.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/simulator.h"
#include "ns3/pointer.h"
#include <map>
#include "ns3/node.h"
#include <ns3/nr-gnb-phy.h>

// ADDED DURING MERGING
#include <ns3/three-gpp-channel-model.h>
#include <random>

namespace ns3 {

std::random_device rdSM;
std::mt19937 mtSM (rdSM ());

std::uniform_real_distribution<double> m_uniformDistSM (0, 1);
std::uniform_real_distribution<double> m_uniformAziSM (1, 360);
std::uniform_real_distribution<double> m_uniformEleSM (1, 20);
std::uniform_real_distribution<double> m_uniformPowerSM (-80, -120);

NS_LOG_COMPONENT_DEFINE ("ThreeGppSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (ThreeGppSpectrumPropagationLossModel);

ThreeGppSpectrumPropagationLossModel::ThreeGppSpectrumPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

ThreeGppSpectrumPropagationLossModel::~ThreeGppSpectrumPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

void
ThreeGppSpectrumPropagationLossModel::DoDispose ()
{
  m_deviceAntennaMap.clear ();
  m_longTermMap.clear ();
  m_channelModel->Dispose ();
  m_channelModel = nullptr;
}

TypeId
ThreeGppSpectrumPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppSpectrumPropagationLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<ThreeGppSpectrumPropagationLossModel> ()
    .AddAttribute("ChannelModel", 
                  "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                  StringValue("ns3::ThreeGppChannelModel"),
                  MakePointerAccessor (&ThreeGppSpectrumPropagationLossModel::SetChannelModel,
                                       &ThreeGppSpectrumPropagationLossModel::GetChannelModel),
      MakePointerChecker<MatrixBasedChannelModel> ())
    .AddAttribute ("ChannelTypeRaytracing",
                   "If true, then raytracing is used; else 3gpp",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ThreeGppSpectrumPropagationLossModel::channelTypeRaytracingSM),
                   MakeBooleanChecker ())
    .AddAttribute ("RaySourceType",
                   "Source of RayData that will be used for the simulation. According to its value either LoadInventory or LoadEnbTraceData will be run. ",
                   StringValue ("Inventory"),
                   MakeStringAccessor (&ThreeGppSpectrumPropagationLossModel::SMSetRaySourceType),
                   MakeStringChecker ())
    .AddAttribute ("Frequency",
                   "Central operation frequency",
                   StringValue ("60GHz"),
                   MakeStringAccessor (&ThreeGppSpectrumPropagationLossModel::m_frequency),
                   MakeStringChecker ())
    ;
  return tid;
}

void
ThreeGppSpectrumPropagationLossModel::SetChannelModel (Ptr<MatrixBasedChannelModel> channel)
{
  m_channelModel = channel;
}

Ptr<MatrixBasedChannelModel>
ThreeGppSpectrumPropagationLossModel::GetChannelModel () const
{
  return m_channelModel;
}

void
ThreeGppSpectrumPropagationLossModel::AddDevice (Ptr<NetDevice> n, Ptr<const ThreeGppAntennaArrayModel> a)
{
  NS_ASSERT_MSG (m_deviceAntennaMap.find (n->GetNode ()->GetId ()) == m_deviceAntennaMap.end (), "Device is already present in the map");
  m_deviceAntennaMap.insert (std::make_pair (n->GetNode ()->GetId (), a));
}

double
ThreeGppSpectrumPropagationLossModel::GetFrequency () const
{
  DoubleValue freq;
  m_channelModel->GetAttribute ("Frequency", freq);
  return freq.Get ();
}

void
ThreeGppSpectrumPropagationLossModel::SetChannelModelAttribute (const std::string &name, const AttributeValue &value)
{
  m_channelModel->SetAttribute (name, value);
}

void
ThreeGppSpectrumPropagationLossModel::GetChannelModelAttribute (const std::string &name, AttributeValue &value) const
{
  m_channelModel->GetAttribute (name, value);
}

ThreeGppAntennaArrayModel::ComplexVector
ThreeGppSpectrumPropagationLossModel::CalcLongTerm (Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                    const ThreeGppAntennaArrayModel::ComplexVector &sW,
                                                    const ThreeGppAntennaArrayModel::ComplexVector &uW) const
{
  NS_LOG_FUNCTION (this);

  uint16_t sAntenna = static_cast<uint16_t> (sW.size ());
  uint16_t uAntenna = static_cast<uint16_t> (uW.size ());

  NS_LOG_DEBUG ("CalcLongTerm with sAntenna " << sAntenna << " uAntenna " << uAntenna);
  //store the long term part to reduce computation load
  //only the small scale fading needs to be updated if the large scale parameters and antenna weights remain unchanged.
  ThreeGppAntennaArrayModel::ComplexVector longTerm;
  //uint8_t numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());
  uint8_t numCluster;
  if (channelTypeRaytracingSM)
  {
    numCluster = params->m_delay.size ();
  }
  else
  {
    numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());
  }
  

  for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
      std::complex<double> txSum (0,0);
      for (uint16_t sIndex = 0; sIndex < sAntenna; sIndex++)
        {
          std::complex<double> rxSum (0,0);
          for (uint16_t uIndex = 0; uIndex < uAntenna; uIndex++)
            {
              if (channelTypeRaytracingSM)
              {
                rxSum = rxSum + std::conj(uW[uIndex]) * params->m_channel[uIndex][sIndex][cIndex];
              }
              else
              {
                rxSum = rxSum + uW[uIndex] * params->m_channel[uIndex][sIndex][cIndex];
              }
              
            }
          txSum = txSum + sW[sIndex] * rxSum;
        }
      longTerm.push_back (txSum);
    }
  return longTerm;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
                                                           ThreeGppAntennaArrayModel::ComplexVector longTerm,
                                                           ThreeGppAntennaArrayModel::ComplexVector txW,
                                                           ThreeGppAntennaArrayModel::ComplexVector rxW,
                                                           Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                           double speed) const
{
  NS_LOG_FUNCTION (this);

  NS_LOG_FUNCTION (this);

  //NS_LOG_INFO ("CalBeamGain: txW " << txW << std::endl << " rxW " << rxW << std::endl <<
 //               " delay " << params->m_delay << std::endl << " angle " << mara << std::endl <<
  //              " speed " << speed);


  NS_ABORT_MSG_IF (params->m_channel.size()==0, "Channel matrix is empty.");
  //NS_ABORT_MSG_IF (longTerm.size()==0, "Long-term matrix is empty.");
  NS_ABORT_MSG_IF (txW.size()==0, "Tx beamforming vector is emtpy.");
  NS_ABORT_MSG_IF (rxW.size()==0, "Rx beamforming vector is emtpy.");

  double spectrumValueCenterFrequency = txPsd->GetSpectrumModel()->Begin()->fl +
      ((txPsd->GetSpectrumModel()->End()-1)->fh - txPsd->GetSpectrumModel()->Begin()->fl)/2;

  NS_ABORT_MSG_IF (spectrumValueCenterFrequency!= GetFrequency(),
                   "Can't calculate beamforming gain for a spectrum value with that does not have the same central carrier frequency as the channel map");

  Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue> (txPsd);

  NS_ABORT_MSG_UNLESS (params->m_delay.size()==params->m_channel.at(0).at(0).size(), "the cluster number of channel and delay spread should be the same");
  NS_ABORT_MSG_UNLESS (txW.size()==params->m_channel.at(0).size() || txW.size()==params->m_channel.size(), "the tx antenna size of channel and antenna weights should be the same");
  NS_ABORT_MSG_UNLESS (rxW.size()==params->m_channel.size() || rxW.size()==params->m_channel.at(0).size(), "the rx antenna size of channel and antenna weights should be the same");
  NS_ABORT_MSG_UNLESS (params->m_angle.at(0).size()==params->m_channel.at(0).at(0).size(), "the cluster number of channel and AOA should be the same");
  NS_ABORT_MSG_UNLESS (params->m_angle.at(1).size()==params->m_channel.at(0).at(0).size(), "the cluster number of channel and ZOA should be the same");

  //channel[rx][tx][cluster]
  size_t numCluster = params->m_delay.size ();
  //the update of Doppler is simplified by only taking the center angle of each cluster in to consideration.


  Values::iterator vit = tempPsd->ValuesBegin ();
  Bands::const_iterator sbit = tempPsd->ConstBandsBegin(); // sub band iterator

  uint16_t iSubband = 0;
  Time time = Simulator::Now();
  double varTtiTime = time.GetSeconds ();
  std::complex<double> doppler;
  while (vit != tempPsd->ValuesEnd ())
    {
      std::complex<double> subsbandGain (0.0,0.0);
      if ((*vit) != 0.00)
        {
          double fsb = (*sbit).fc; // take the carrier frequency of the band for which we al calculating the gain
          for (size_t cIndex = 0; cIndex < numCluster; cIndex++) // calculate for this subband for all the clusters
            {
              double temp_delay = -2 * M_PI * fsb * (params->m_delay.at (cIndex))*(1e-9);//need to convert ns to s
              std::complex<double> delay(cos(temp_delay),sin(temp_delay));

              double f_d = speed * GetFrequency () / 3e8;
              double temp_doppler = 2 * M_PI * varTtiTime * f_d * params->m_doppler.at(cIndex);
              doppler = std::complex<double> (cos (temp_doppler), sin (temp_doppler));
              std::complex<double> smallScaleFading = /*sqrt (pathPowerLinear) *  doppler */  delay;

              subsbandGain = subsbandGain + smallScaleFading * longTerm.at (cIndex);
            }
          *vit = (*vit) * (norm (subsbandGain));
        }
      vit++;
      sbit++;
      iSubband++;
    }
  return tempPsd;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
                                                           ThreeGppAntennaArrayModel::ComplexVector longTerm,
                                                           Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                           const ns3::Vector &sSpeed, const ns3::Vector &uSpeed) const
{
  NS_LOG_FUNCTION (this);

  Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue> (txPsd);

  //channel[rx][tx][cluster]
  uint8_t numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());

  // compute the doppler term
  // NOTE the update of Doppler is simplified by only taking the center angle of
  // each cluster in to consideration.
  double slotTime = Simulator::Now ().GetSeconds ();
  ThreeGppAntennaArrayModel::ComplexVector doppler;
  for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
      //cluster angle angle[direction][n],where, direction = 0(aoa), 1(zoa).
      // TODO should I include the "alfa" term for the Doppler of delayed paths?
      double temp_doppler = 2 * M_PI * ((sin (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] * M_PI / 180) * cos (params->m_angle[MatrixBasedChannelModel::AOA_INDEX][cIndex] * M_PI / 180) * uSpeed.x
                                         + sin (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] * M_PI / 180) * sin (params->m_angle[MatrixBasedChannelModel::AOA_INDEX][cIndex] * M_PI / 180) * uSpeed.y
                                         + cos (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] * M_PI / 180) * uSpeed.z)
                                         + (sin (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] * M_PI / 180) * cos (params->m_angle[MatrixBasedChannelModel::AOD_INDEX][cIndex] * M_PI / 180) * sSpeed.x
                                         + sin (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] * M_PI / 180) * sin (params->m_angle[MatrixBasedChannelModel::AOD_INDEX][cIndex] * M_PI / 180) * sSpeed.y
                                         + cos (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] * M_PI / 180) * sSpeed.z))
        * slotTime * GetFrequency () / 3e8;
      doppler.push_back (exp (std::complex<double> (0, temp_doppler)));
    }

  // apply the doppler term and the propagation delay to the long term component
  // to obtain the beamforming gain
  auto vit = tempPsd->ValuesBegin (); // psd iterator
  auto sbit = tempPsd->ConstBandsBegin(); // band iterator
  while (vit != tempPsd->ValuesEnd ())
    {
      std::complex<double> subsbandGain (0.0,0.0);
      if ((*vit) != 0.00)
        {
          double fsb = (*sbit).fc; // center frequency of the sub-band
          for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
            {
              double delay = -2 * M_PI * fsb * (params->m_delay[cIndex]);
              subsbandGain = subsbandGain + longTerm[cIndex] * doppler[cIndex] * exp (std::complex<double> (0, delay));
            }
          *vit = (*vit) * (norm (subsbandGain));
        }
      vit++;
      sbit++;
    }
  return tempPsd;
}

ThreeGppAntennaArrayModel::ComplexVector
ThreeGppSpectrumPropagationLossModel::GetLongTerm (uint32_t aId, uint32_t bId,
                                                   Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
                                                   const ThreeGppAntennaArrayModel::ComplexVector &aW,
                                                   const ThreeGppAntennaArrayModel::ComplexVector &bW) const
{
  ThreeGppAntennaArrayModel::ComplexVector longTerm; // vector containing the long term component for each cluster

  // check if the channel matrix was generated considering a as the s-node and
  // b as the u-node or viceversa
  ThreeGppAntennaArrayModel::ComplexVector sW, uW;
  if (!channelMatrix->IsReverse (aId, bId))
  {
    sW = aW;
    uW = bW;
  }
  else
  {
    sW = bW;
    uW = aW;
  }

  // compute the long term key, the key is unique for each tx-rx pair
  uint32_t x1 = std::min (aId, bId);
  uint32_t x2 = std::max (aId, bId);
  uint32_t longTermId = MatrixBasedChannelModel::GetKey (x1, x2);

  bool update = false; // indicates whether the long term has to be updated
  bool notFound = false; // indicates if the long term has not been computed yet

  // look for the long term in the map and check if it is valid
  if (m_longTermMap.find (longTermId) != m_longTermMap.end ())
  {
    NS_LOG_DEBUG ("found the long term component in the map");
    longTerm = m_longTermMap[longTermId]->m_longTerm;

    // check if the channel matrix has been updated
    // or the s beam has been changed
    // or the u beam has been changed
    update = (m_longTermMap[longTermId]->m_channel->m_generatedTime != channelMatrix->m_generatedTime
              || m_longTermMap[longTermId]->m_sW != sW
              || m_longTermMap[longTermId]->m_uW != uW);

  }
  else
  {
    NS_LOG_DEBUG ("long term component NOT found");
    notFound = true;
  }

  if (update || notFound)
    {
      NS_LOG_DEBUG ("compute the long term");
      // compute the long term component
      longTerm = CalcLongTerm (channelMatrix, sW, uW);

      // store the long term
      Ptr<LongTerm> longTermItem = Create<LongTerm> ();
      longTermItem->m_longTerm = longTerm;
      longTermItem->m_channel = channelMatrix;
      longTermItem->m_sW = sW;
      longTermItem->m_uW = uW;

      m_longTermMap[longTermId] = longTermItem;
    }

  return longTerm;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                                    Ptr<const MobilityModel> a,
                                                                    Ptr<const MobilityModel> b) const
{
  NS_LOG_FUNCTION (this);
  uint32_t aId = a->GetObject<Node> ()->GetId (); // id of the node a
  uint32_t bId = b->GetObject<Node> ()->GetId (); // id of the node b

  uint32_t traceIndex = 0;

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);

  /*if ("ns3::ThreeGppChannelModel" == typeid(m_channelModel).name())
  {*/
    bool aIsEnb = false;
    bool bIsEnb = false;

    traceIndex = DynamicCast<ThreeGppChannelModel> (m_channelModel)->GetTraceIndex ();

    // With this method a eNB is recognized by matching the poisition from the loaded data
    NS_LOG_INFO ("size=" << enbLocations.size ());
    if (enbLocations.size () != 0)
    {
      for (unsigned int i = 0; i < enbLocations.size (); i++)
      {
        NS_LOG_INFO("enb i="<<i<<": x="<<enbLocations.at(i).x<<" y="<<enbLocations.at(i).y);
        if (a->GetPosition ().x == enbLocations.at (i).x && a->GetPosition ().y == enbLocations.at (i).y)
        {
          aIsEnb = true;
        }
        else if (b->GetPosition ().x == enbLocations.at(i).x && b->GetPosition ().y == enbLocations.at(i).y)
        {
          bIsEnb = true;
        }
      }
    }

    if ((aIsEnb && bIsEnb) || (!aIsEnb && !bIsEnb))
    {
      NS_LOG_INFO ("UE<->UE or gNB<->gNB, returning");
      if (channelTypeRaytracingSM)
      {
        *rxPsd = *rxPsd * 1e-11;
      }
      return rxPsd;
    }
    else if (aIsEnb && !bIsEnb)
    {
      b->GetObject<MobilityModel> ()->SetPosition (walkCords.at(traceIndex));
    }
    else if (!aIsEnb && bIsEnb)
    {
      a->GetObject<MobilityModel> ()->SetPosition (walkCords.at(traceIndex));
    }

    NS_ASSERT (aId != bId);
    NS_ASSERT_MSG (a->GetDistanceFrom (b) > 0.0, "The position of a and b devices cannot be the same");
  //}

  // retrieve the antenna of device a
  NS_ASSERT_MSG (m_deviceAntennaMap.find (aId) != m_deviceAntennaMap.end (), "Antenna not found for node " << aId);
  Ptr<const ThreeGppAntennaArrayModel> aAntenna = m_deviceAntennaMap.at (aId);
  NS_LOG_DEBUG ("a node " << a->GetObject<Node> () << " antenna " << aAntenna);

  // retrieve the antenna of the device b
  NS_ASSERT_MSG (m_deviceAntennaMap.find (bId) != m_deviceAntennaMap.end (), "Antenna not found for device " << bId);
  Ptr<const ThreeGppAntennaArrayModel> bAntenna = m_deviceAntennaMap.at (bId);
  NS_LOG_DEBUG ("b node " << bId << " antenna " << bAntenna);

  if (aAntenna->IsOmniTx () || bAntenna->IsOmniTx () )
    {
      NS_LOG_LOGIC ("Omni transmission, do nothing.");
      if (channelTypeRaytracingSM)
      {
        *rxPsd = *rxPsd * 1e-22;
      }
      return rxPsd;
    }

  Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix;

  if (aIsEnb && !bIsEnb)
  {
    channelMatrix = m_channelModel->GetChannel (a, b, aAntenna, bAntenna);  
  }
  else if (!aIsEnb && bIsEnb) 
  {
    channelMatrix = m_channelModel->GetChannel (a, b, aAntenna, bAntenna);
  }
  else
  {
    NS_ABORT_MSG ("Both node a and b cannot be of the same type");
  }  
  
  if (channelMatrix->m_numCluster == 0)
  {
    NS_LOG_INFO ("No RayTrace data between " << a->GetPosition () << " and " << b->GetPosition ());
    //The ryatracing channel does not have any pathloss, therefore when there is no data we need to supress the ouput signal strength
    if (channelTypeRaytracingSM)
    {
      *rxPsd = *rxPsd * 1e-22;
    }
    return rxPsd;
  }

  // get the precoding and combining vectors

  ThreeGppAntennaArrayModel::ComplexVector aW = aAntenna->GetBeamformingVector ();
  ThreeGppAntennaArrayModel::ComplexVector bW = bAntenna->GetBeamformingVector ();

  // retrieve the long term component
  ThreeGppAntennaArrayModel::ComplexVector longTerm;
  longTerm = GetLongTerm (aId, bId, channelMatrix, aW, bW);
  Ptr<SpectrumValue> bfPsd;
  
  // apply the beamforming gain
  if (channelTypeRaytracingSM)
  {
    bfPsd = CalcBeamformingGain (rxPsd, longTerm, aW, bW, channelMatrix, walkSpeed.at (traceIndex));
  }
  else
  {
    bfPsd = CalcBeamformingGain (rxPsd, longTerm, channelMatrix, a->GetVelocity (), b->GetVelocity ());
  }
  
  Ptr<SpectrumValue> bfGain = Create<SpectrumValue>((*bfPsd) / (*rxPsd));

  return bfPsd;
}

// ADDED DURING MERGING

void 
ThreeGppSpectrumPropagationLossModel::LoadAllTraceData (uint16_t ueWalkId)
{
  input_files_folder = "src/nr/model/Raytracing/";
  std::string walk_file = input_files_folder + std::to_string (ueWalkId) + "_cords.txt";
  std::string speed_file = input_files_folder + std::to_string (ueWalkId) + "_speed.txt";
  std::string los_file = input_files_folder + std::to_string (ueWalkId) + "_ped.txt";
  std::string link_file = input_files_folder + std::to_string (ueWalkId) + "_link.txt";
  std::string inventory_file;
  if (m_frequency == "60GHz")
  {
    inventory_file = input_files_folder + "Inventory.txt";
  }
  else if (m_frequency == "28GHz")
  {
    inventory_file = input_files_folder + "Inventory_28GHz.txt";
  }
  else
  {
    NS_ABORT_MSG ("Undefined central frequency band");
  }
  

  LoadEnbLocations (input_files_folder);

  if (SMGetRaySourceType () == "Inventory")
  {
    NS_LOG_INFO ("Loading the inventory trace data for all ENB");
    LoadInventory (inventory_file);
  }
  else if (SMGetRaySourceType () == "EnbTraceData")
  {
    NS_LOG_INFO ("Loading the trace data for all configured ENB");
    std::vector<Vector>::iterator it;
    for (it = enbLocations.begin (); it != enbLocations.end (); it++)
    {
      LoadEnbTraceData (*it);
    }
  }
  else
  {
    NS_ABORT_MSG ("Unidentified source for RayData");
  }

  NS_LOG_INFO ("Loading the walk and pedestrian blockage data");
  LoadWalkInfo (walk_file, speed_file);
  LoadLosInfo (los_file);
  //LoadLinkInfo (link_file);
  
  if (SMGetRaySourceType () == "Inventory")
  {
    DynamicCast<ThreeGppChannelModel> (m_channelModel)->SetSharedParams (enbLocations,
                                      walkCords, los_data, allLinkData,maxTraceIndexSM, 
                                      channelTypeRaytracingSM, "Inventory");
  }
  else if (SMGetRaySourceType () == "EnbTraceData")
  {
    DynamicCast<ThreeGppChannelModel> (m_channelModel)->SetSharedParams (enbLocations,
                                      walkCords, los_data, all_enbTraceData, maxTraceIndexSM,
                                      channelTypeRaytracingSM, "EnbTraceData");
  }
  else
  {
    NS_ABORT_MSG ("Unidentified source for RayData");
  }
  
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadEnbLocations (std::string input_folder)
{
  std::string enbFile = input_folder + "enb_locations.txt";
  NS_LOG_UNCOND ("Loading ENB locations from: " << enbFile);
  std::ifstream file1;
  file1.open (enbFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "File containing ENB locations not found");

  std::string line;
  std::string token;
  while ( (std::getline (file1, line))) //Parse each line of the file
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma separated string in a line
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    enbLocations.push_back (
            Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
    
  }
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadEnbTraceData (Vector enbLocation)
{
  NS_LOG_UNCOND ("Loading raytracer data for the ENB:" << enbLocation);

  std::string traceFileName = GetTraceFileName (enbLocation);
  std::ifstream singlefile;
  singlefile.open (traceFileName.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (singlefile.good (), "Raytracing file not found");

  std::map<Vector, Ptr<RxRayData>> all_rx_rayData;
  Ptr<RxRayData> rxData = Create<RxRayData> ();
  Vector rxPos;

  uint16_t counter = 0;
  std::string line;
  std::string token;
  while (std::getline (singlefile, line)) //Parse each line of the file
    {
      if (counter == 9)
        {
          all_rx_rayData.insert (std::make_pair (rxPos, rxData));
          rxData = Create<RxRayData> ();
          counter = 0;
        }

      doubleVector_t lineElements;
      std::istringstream stream (line);
      while (getline (stream, token, ',')) //Parse each comma separated string in a line
        {
          double sigma = 0.00;
          std::stringstream stream (token);
          stream >> sigma;
          lineElements.push_back (sigma);
        }

      switch (counter)
        {
        case 0:
          rxPos.x = lineElements.at (0);
          rxPos.y = lineElements.at (1);
          rxPos.z = lineElements.at (2);
          break;
        case 1:
          rxData->m_path = lineElements.at (0);
          break;
        case 2:
          rxData->m_delay = lineElements;
          break;
        case 3:
          rxData->m_pathloss = lineElements;
          break;
        case 4:
          rxData->m_los = lineElements;
          break;
        case 5:
          rxData->m_aodElevation = lineElements;
          break;
        case 6:
          rxData->m_aodAzimuth = lineElements;
          break;
        case 7:
          rxData->m_aoaElevation = lineElements;
          break;
        case 8:
          rxData->m_aoaAzimuth = lineElements;
          break;
        default:
          break;
        }
      counter++;
    }

  Ptr<EnbTraceData> enb_trace = Create<EnbTraceData> ();
  enb_trace->m_enbPos = enbLocation;
  enb_trace->m_rxRayData = all_rx_rayData;
  all_enbTraceData.insert (std::make_pair (enbLocation, enb_trace));
}

void
ThreeGppSpectrumPropagationLossModel::LoadWalkInfo (std::string walkInfoFile, std::string speedFile)
{
  NS_LOG_UNCOND ("Loading the walk coordinates from: " << walkInfoFile);
  std::ifstream file1;
  file1.open (walkInfoFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "Walk coordinates file not found");

  std::string line;
  std::string token;
  while (std::getline (file1, line))  //Parse each line of the file
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma separated string in a line
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    walkCords.push_back (Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
  }
  
  std::ifstream file2;
  file2.open (speedFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file2.good (), "Walk speed file not found");
  while (std::getline (file2, line)) //Parse each line of the file
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma seperated string in a line
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    
    walkSpeed.push_back (lineElements.at (0));
  }

  maxTraceIndexSM = walkCords.size ();
  NS_ASSERT_MSG (walkSpeed.size () == walkCords.size (),
                "The size of the walk coordinates file and the speed file must be the same.");
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadLosInfo (std::string los_file)
{
  NS_LOG_UNCOND ("Loading the LOS data from: " << los_file);
  std::ifstream file1;
  file1.open (los_file.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "LOSS file not found");

  std::string line;
  std::string token;
  while (std::getline (file1, line))  //Parse each line of the file
  {
    std::vector<int> lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma separated string in a line
    {
      int sigma = 0;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    
    los_data.push_back (lineElements);
  }
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadLinkInfo (std::string link_file)
{
  NS_LOG_UNCOND ("Loading the Link data from: " << link_file);
  std::ifstream file1;
  file1.open (link_file.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "Link file not founc");

  std::string line;
	std::string token;
  while (std::getline (file1, line))  //Parse each line of the file
  {
    std::vector<int> lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ',')) //Parse each comma separated sring in a line
    {
      int sigma = 0;
      std::stringstream stream (token);
      stream >>sigma;
      lineElements.push_back (sigma);
    }
    
    link_data.push_back (lineElements);
  }
  
}

// New function to load the inventory trace data for all ENB
void
ThreeGppSpectrumPropagationLossModel::LoadInventory (std::string inventory_file)
{
  NS_LOG_UNCOND ("Loading the inventory from: " << inventory_file);
	std::ifstream singlefile;
	singlefile.open (inventory_file.c_str (), std::ifstream::in);
	NS_ASSERT_MSG (singlefile.good (), "Inventory file not found");

    //std::map<Vector, Ptr<LinkData>> allLinkData;
	Ptr<LinkData> linkData = Create<LinkData> ();
	Vector rxPos;

	uint16_t counter = 0;
	std::string line;
	std::string token;
  while (std::getline (singlefile, line)) //Parse each line of the file
  {
  	if (counter == 10)
  	{
  		allLinkData.insert (std::make_pair (rxPos, linkData));
  		linkData = Create<LinkData> ();
  		counter = 0;
  	}

  	doubleVector_t lineElements;
  	std::istringstream stream (line);
      while (getline (stream, token, ',')) //Parse each comma separated string in a line
      {
      	double sigma = 0.00;
      	std::stringstream stream (token);
      	stream >> sigma;
      	lineElements.push_back (sigma);
      }

      switch (counter)
      {
      	case 0:
      	rxPos.x = lineElements.at (0);
      	rxPos.y = lineElements.at (1);
      	rxPos.z = lineElements.at (2);
      	break;
      	case 1:
      	linkData->m_path = lineElements.at(0);
      	break;
      	case 2:
      	linkData->m_los = lineElements;
      	break;
      	case 3:
      	linkData->m_txId = lineElements;
      	break;
      	case 4:
      	linkData->m_aodAzimuth = lineElements;
      	break;
      	case 5:
      	linkData->m_aodElevation = lineElements;
      	break;
      	case 6:
      	linkData->m_aoaAzimuth = lineElements;
      	break;
      	case 7:
      	linkData->m_aoaElevation = lineElements;
      	break;
      	case 8:
      	linkData->m_pathloss = lineElements;
      	break;
      	case 9:
      	linkData->m_delay = lineElements;
      	break;
      	default:
      	break;
      }
      counter++;
  }
}

Vector
ThreeGppSpectrumPropagationLossModel::GetCurrentUePosition()
{
  return walkCords.at(DynamicCast<ThreeGppChannelModel> (m_channelModel)->GetTraceIndex ());
}

void
ThreeGppSpectrumPropagationLossModel::SMSetRaySourceType (std::string type)
{
  m_smRaySourceType = type;
}

std::string
ThreeGppSpectrumPropagationLossModel::SMGetRaySourceType ()
{
  return m_smRaySourceType;
}

std::string
ThreeGppSpectrumPropagationLossModel::GetTraceFileName (Vector enbLoc)
{
  std::stringstream ss;
  ss << enbLoc.x;
  ss << enbLoc.y;
  ss << enbLoc.z;
  std::string str = input_files_folder + ss.str () + ".txt";
  return str;
}

void
ThreeGppSpectrumPropagationLossModel::SetBeamSweepState (bool beamSweepState)
{
  DynamicCast<ThreeGppChannelModel> (m_channelModel)->DoSetBeamSweepState (beamSweepState);
}

std::pair<std::pair<double, double>, std::pair<double, double>>
ThreeGppSpectrumPropagationLossModel::GetAodAoaFromCM (Vector enbLoc)
{
  return DynamicCast<ThreeGppChannelModel> (m_channelModel)->DoGetAoaAod (enbLoc);
}

/*
void 
ThreeGppSpectrumPropagationLossModel::SetSMIdealBeamformingHelper (Ptr<IdealBeamformingHelper> idealBeamformingHelper)
{
  m_SMidealBeamformingHelper = idealBeamformingHelper;
}*/

/*void
ThreeGppSpectrumPropagationLossModel::BeamSearchBeamforming (Ptr<MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
                                                             Ptr<ThreeGppAntennaArrayModel> aAntenna,
                                                             Ptr<ThreeGppAntennaArrayModel> bAntenna) const
{
  Ptr<const SpectrumModel> txSm = aAntenna->GetObject<
}*/

}  // namespace ns3
