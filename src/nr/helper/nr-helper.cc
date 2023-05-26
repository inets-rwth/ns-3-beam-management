/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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

#include "nr-helper.h"
#include <ns3/lte-rrc-protocol-ideal.h>
#include <ns3/lte-rrc-protocol-real.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-chunk-processor.h>
#include <ns3/epc-ue-nas.h>
#include <ns3/names.h>
#include <ns3/nr-rrc-protocol-ideal.h>
#include <ns3/nr-gnb-mac.h>
#include <ns3/nr-gnb-phy.h>
#include <ns3/nr-ue-phy.h>
#include <ns3/nr-ue-mac.h>
#include <ns3/nr-gnb-net-device.h>
#include <ns3/nr-ue-net-device.h>
#include <ns3/nr-ch-access-manager.h>
#include <ns3/bandwidth-part-gnb.h>
#include <ns3/bwp-manager-gnb.h>
#include <ns3/bwp-manager-ue.h>
#include <ns3/nr-rrc-protocol-ideal.h>
#include <ns3/epc-helper.h>
#include <ns3/epc-enb-application.h>
#include <ns3/epc-x2.h>
#include <ns3/nr-phy-rx-trace.h>
#include <ns3/nr-mac-rx-trace.h>
#include <ns3/nr-bearer-stats-calculator.h>
#include <ns3/bandwidth-part-ue.h>
#include <ns3/beam-manager.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/three-gpp-spectrum-propagation-loss-model.h>
#include <ns3/buildings-channel-condition-model.h>
#include <ns3/nr-mac-scheduler-tdma-rr.h>
#include <ns3/bwp-manager-algorithm.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/cc-helper.h>
#include <ns3/lte-handover-algorithm.h>
#include <ns3/component-carrier-enb.h>
#include <ns3/lte-enb-mac.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/lte-ffr-algorithm.h>

// ADDED DURING MERGING
#include <ns3/three-gpp-channel-model.h>
#include <ns3/nr-mac-scheduler.h>
#include <ns3/nr-mac-scheduler-ns3.h>

#include <algorithm>

namespace ns3 {

/* ... */
NS_LOG_COMPONENT_DEFINE ("NrHelper");

NS_OBJECT_ENSURE_REGISTERED (NrHelper);

NrHelper::NrHelper (void)
{
  NS_LOG_FUNCTION (this);
  m_channelFactory.SetTypeId (MultiModelSpectrumChannel::GetTypeId ());
  m_gnbNetDeviceFactory.SetTypeId (NrGnbNetDevice::GetTypeId ());
  m_ueNetDeviceFactory.SetTypeId (NrUeNetDevice::GetTypeId ());
  m_ueMacFactory.SetTypeId (NrUeMac::GetTypeId ());
  m_gnbMacFactory.SetTypeId (NrGnbMac::GetTypeId ());
  m_ueSpectrumFactory.SetTypeId (NrSpectrumPhy::GetTypeId ());
  m_gnbSpectrumFactory.SetTypeId (NrSpectrumPhy::GetTypeId ());
  m_uePhyFactory.SetTypeId (NrUePhy::GetTypeId ());
  m_gnbPhyFactory.SetTypeId (NrGnbPhy::GetTypeId ());
  m_ueChannelAccessManagerFactory.SetTypeId (NrAlwaysOnAccessManager::GetTypeId ());
  m_gnbChannelAccessManagerFactory.SetTypeId (NrAlwaysOnAccessManager::GetTypeId ());
  m_schedFactory.SetTypeId (NrMacSchedulerTdmaRR::GetTypeId ());
  m_ueAntennaFactory.SetTypeId (ThreeGppAntennaArrayModel::GetTypeId ());
  m_gnbAntennaFactory.SetTypeId (ThreeGppAntennaArrayModel::GetTypeId ());
  m_gnbBwpManagerAlgoFactory.SetTypeId (BwpManagerAlgorithmStatic::GetTypeId ());
  m_ueBwpManagerAlgoFactory.SetTypeId (BwpManagerAlgorithmStatic::GetTypeId ());
  m_gnbUlAmcFactory.SetTypeId (NrAmc::GetTypeId ());
  m_gnbDlAmcFactory.SetTypeId (NrAmc::GetTypeId ());
  m_lteChannelFactory.SetTypeId (MultiModelSpectrumChannel::GetTypeId ());

  m_spectrumPropagationFactory.SetTypeId (ThreeGppSpectrumPropagationLossModel::GetTypeId ());
  m_lteEnbNetDeviceFactory.SetTypeId (LteEnbNetDevice::GetTypeId ());

  // Initialization that is there just because the user can configure attribute
  // through the helper methods without making it sad that no TypeId is set.
  // When the TypeId is changed, the user-set attribute will be maintained.
  m_pathlossModelFactory.SetTypeId (ThreeGppPropagationLossModel::GetTypeId ());
  m_channelConditionModelFactory.SetTypeId (ThreeGppChannelConditionModel::GetTypeId ());

  Config::SetDefault ("ns3::EpsBearer::Release", UintegerValue (15));

  m_phyStats = CreateObject<NrPhyRxTrace> ();
}

NrHelper::~NrHelper (void)
{
  NS_LOG_FUNCTION (this);
}

TypeId
NrHelper::GetTypeId (void)
{
  static TypeId
    tid =
    TypeId ("ns3::NrHelper")
    .SetParent<Object> ()
    .AddConstructor<NrHelper> ()
    .AddAttribute ("ChannelModel",
                   "The type of MIMO channel model to be used. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::SpectrumPropagationLossModel.",
                   StringValue ("ns3::ThreeGppSpectrumPropagationLossModel"),
                   MakeStringAccessor (&NrHelper::SetChannelModelType),
                   MakeStringChecker ())
    .AddAttribute ("HarqEnabled",
                   "Enable Hybrid ARQ",
                   BooleanValue (true),
                   MakeBooleanAccessor (&NrHelper::m_harqEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("LteHandoverAlgorithm",
                   "The type of handover algorithm to be used for LTE eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::LteHandoverAlgorithm.",
                   StringValue ("ns3::A2A4RsrqHandoverAlgorithm"),
                   MakeStringAccessor (&NrHelper::SetLteHandoverAlgorithmType,
                                       &NrHelper::GetLteHandoverAlgorithmType),
                   MakeStringChecker ())
    .AddAttribute ("LtePathlossModel",
                   "The type of pathloss model to be used for the 2 LTE channels. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::PropagationLossModel.",
                   StringValue ("ns3::JakesPropagationLossModel"),
                   MakeStringAccessor (&NrHelper::SetLtePathlossModelType),
                   MakeStringChecker ())
    .AddAttribute ("LteScheduler",
                   "The type of scheduler to be used for LTE eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::FfMacScheduler.",
                   StringValue ("ns3::PfFfMacScheduler"),
                   MakeStringAccessor (&NrHelper::SetLteSchedulerType),
                   MakeStringChecker ())
    .AddAttribute ("LteFfrAlgorithm",
                   "The type of FFR algorithm to be used for LTE eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting from ns3::LteFfrAlgorithm.",
                   StringValue ("ns3::LteFrNoOpAlgorithm"),
                   MakeStringAccessor (&NrHelper::SetLteFfrAlgorithmType),
                   MakeStringChecker ())
    .AddAttribute ("LteEnbComponentCarrierManager",
                   "The type of Component Carrier Manager to be used for eNBs. "
                   "The allowed values for this attributes are the type names "
                   "of any class inheriting ns3::LteEnbComponentCarrierManager.",
                   StringValue ("ns3::NoOpComponentCarrierManager"),
                   MakeStringAccessor (&NrHelper::SetLteEnbComponentCarrierManagerType),
                   MakeStringChecker ())
    .AddAttribute ("AdaptiveBeamforming",
                   "Boolean which indicates whether adaptive or periodic beamforming is performed",
                   BooleanValue (true),
                   MakeBooleanAccessor (&NrHelper::m_adaptiveBF),
                   MakeBooleanChecker ())
    .AddAttribute ("RealisticIA",
                   "Boolean which indicates whether realistic IA scheme will be employed or not",
                   BooleanValue (false),
                   MakeBooleanAccessor (&NrHelper::m_realisticIA),
                   MakeBooleanChecker ())
    .AddAttribute ("RadioLinkMonitoring",
                   "Boolean which indicates whether radio link monitoring will be performed",
                   BooleanValue (false),
                   MakeBooleanAccessor (&NrHelper::m_rlmOn),
                   MakeBooleanChecker ())
    .AddAttribute ("NumberOfRLMDirections",
                   "Number of Beam Pair Links that will be monitored by RLM",
                   UintegerValue (8),
                   MakeUintegerAccessor (&NrHelper::m_noOfBeamsTbRLM),
                   MakeUintegerChecker<uint8_t> (0, 1000))
    .AddAttribute ("NumberOfRLMBeamsTbReported",
                   "Number of Beam Pair Links that will be reported",
                  UintegerValue (4),
                  MakeUintegerAccessor (&NrHelper::m_noOfBeamsTbReported),
                  MakeUintegerChecker<uint8_t> (0,1000))
    .AddAttribute ("NumberOfSSBsPerSlot",
                   "Number of SS Blocks within a single slot",
                   UintegerValue(2),
                   MakeUintegerAccessor (&NrHelper::m_noOfSSBsPerSlot),
                   MakeUintegerChecker<uint8_t> (0,10))
    ;
  return tid;
}

void
NrHelper::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_phyStats = nullptr;
  //m_bwpConfiguration.clear ();
  m_lteComponentCarrierPhyParams.clear ();
  Object::DoDispose ();
}

void
NrHelper::DoInitialize ()
{
  NS_LOG_FUNCTION (this);

  //aborts if the channel model isn't 3GPP Channel
  //NS_ABORT_MSG_IF (m_channelModelType != "ns3::ThreeGppSpectrumPropagationLossModel", "Cannot set a different type of channel");

  m_noOfLteCcs = 1; //Hardcoded since LTE-eNB is only for coordination not connection
  
  if (m_lteComponentCarrierPhyParams.size () == 0)
  {
    // create the map of LTE component carriers
    Ptr<CcHelper> lteCcHelper = CreateObject<CcHelper> ();
    lteCcHelper->SetNumberOfComponentCarriers (1);
    lteCcHelper->SetUlEarfcn (18100);
    lteCcHelper->SetDlEarfcn (100);
    lteCcHelper->SetDlBandwidth (100);
    lteCcHelper->SetUlBandwidth (100);
    std::map<uint8_t, ComponentCarrier > lteCcMap = lteCcHelper->EquallySpacedCcs ();
    lteCcMap.at (0).SetAsPrimary (true);
    this->SetLteCcPhyParams (lteCcMap);
  }

  LteChannelModelInitialization ();         // lte channel initialization

  Object::DoInitialize ();
}

typedef std::function<void (ObjectFactory *, ObjectFactory *)> InitPathLossFn;

static void
InitRma (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppRmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (ThreeGppRmaChannelConditionModel::GetTypeId ());
}

static void
InitRma_LoS (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppRmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (AlwaysLosChannelConditionModel::GetTypeId ());
}

static void
InitRma_nLoS (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppRmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (NeverLosChannelConditionModel::GetTypeId ());
}

static void
InitUma (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (ThreeGppUmaChannelConditionModel::GetTypeId ());
}

static void
InitUma_LoS (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (AlwaysLosChannelConditionModel::GetTypeId ());
}

static void
InitUma_nLoS (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (NeverLosChannelConditionModel::GetTypeId ());
}

static void
InitUmi (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmiStreetCanyonPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (ThreeGppUmiStreetCanyonChannelConditionModel::GetTypeId ());
}

static void
InitUmi_LoS (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmiStreetCanyonPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (AlwaysLosChannelConditionModel::GetTypeId ());
}

static void
InitUmi_nLoS (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmiStreetCanyonPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (NeverLosChannelConditionModel::GetTypeId ());
}

static void
InitIndoorOpen (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppIndoorOfficePropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (ThreeGppIndoorOpenOfficeChannelConditionModel::GetTypeId ());
}

static void
InitIndoorMixed (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppIndoorOfficePropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (ThreeGppIndoorMixedOfficeChannelConditionModel::GetTypeId ());
}

static void
InitUmaBuildings (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmaPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (BuildingsChannelConditionModel::GetTypeId ());
}

static void
InitUmiBuildings (ObjectFactory *pathlossModelFactory, ObjectFactory *channelConditionModelFactory)
{
  pathlossModelFactory->SetTypeId (ThreeGppUmiStreetCanyonPropagationLossModel::GetTypeId ());
  channelConditionModelFactory->SetTypeId (BuildingsChannelConditionModel::GetTypeId ());
}

void 
NrHelper::LteChannelModelInitialization (void)
{
  NS_LOG_FUNCTION (this);
  // setup of LTE channels & related
  m_downlinkChannel = m_lteChannelFactory.Create<SpectrumChannel> ();
  m_uplinkChannel = m_lteChannelFactory.Create<SpectrumChannel> ();
  m_downlinkPathlossModel = m_dlPathlossModelFactory.Create ();
  Ptr<SpectrumPropagationLossModel> dlSplm = m_downlinkPathlossModel->GetObject<SpectrumPropagationLossModel> ();
  if (dlSplm != 0)
  {
    NS_LOG_LOGIC (this << " using a SpectrumPropagationLossModel in DL");
    m_downlinkChannel->AddSpectrumPropagationLossModel (dlSplm);
  }
  else
  {
    NS_LOG_LOGIC (this << " using a PropagationLossModel in DL");
    Ptr<PropagationLossModel> dlPlm = m_downlinkPathlossModel->GetObject<PropagationLossModel> ();
    NS_ASSERT_MSG (dlPlm != 0, " " << m_downlinkPathlossModel << " is neither PropagationLossModel nor SpectrumPropagationLossModel");
    m_downlinkChannel->AddPropagationLossModel (dlPlm);
  }

  m_uplinkPathlossModel = m_ulPathlossModelFactory.Create ();
  Ptr<SpectrumPropagationLossModel> ulSplm = m_uplinkPathlossModel->GetObject<SpectrumPropagationLossModel> ();
  if (ulSplm != 0)
    {
      NS_LOG_LOGIC (this << " using a SpectrumPropagationLossModel in UL");
      m_uplinkChannel->AddSpectrumPropagationLossModel (ulSplm);
    }
  else
    {
      NS_LOG_LOGIC (this << " using a PropagationLossModel in UL");
      Ptr<PropagationLossModel> ulPlm = m_uplinkPathlossModel->GetObject<PropagationLossModel> ();
      NS_ASSERT_MSG (ulPlm != 0, " " << m_uplinkPathlossModel << " is neither PropagationLossModel nor SpectrumPropagationLossModel");
      m_uplinkChannel->AddPropagationLossModel (ulPlm);
    }
    // TODO consider if adding LTE fading
  // TODO add mac & phy LTE stats  
}

void
NrHelper::SetLteCcPhyParams ( std::map<uint8_t, ComponentCarrier> ccMapParams)
{
  NS_LOG_FUNCTION (this);
  m_lteComponentCarrierPhyParams = ccMapParams;
}

void
NrHelper::SetLtePathlossModelType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_dlPathlossModelFactory = ObjectFactory ();
  m_dlPathlossModelFactory.SetTypeId (type);
  m_ulPathlossModelFactory = ObjectFactory ();
  m_ulPathlossModelFactory.SetTypeId (type);
}

void 
NrHelper::SetLteSchedulerType (std::string type)
{
  NS_LOG_FUNCTION (this << type );
  m_lteSchedulerFactory = ObjectFactory ();
  m_lteSchedulerFactory.SetTypeId (type);
}

void
NrHelper::SetChannelModelType (std::string type)
{
  NS_LOG_FUNCTION (this <<type);
  m_channelModelType = type;
}

void 
NrHelper::SetLteFfrAlgorithmType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_lteFfrAlgorithmFactory = ObjectFactory ();
  m_lteFfrAlgorithmFactory.SetTypeId (type);
}

void
NrHelper::SetLteEnbComponentCarrierManagerType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_lteEnbComponentCarrierManagerFactory = ObjectFactory ();
  m_lteEnbComponentCarrierManagerFactory.SetTypeId (type);
}

void
NrHelper::InitializeOperationBand (OperationBandInfo *band, uint8_t flags)
{
  NS_LOG_FUNCTION (this);

  static std::unordered_map<BandwidthPartInfo::Scenario, InitPathLossFn, std::hash<int>> initLookupTable
  {
    {BandwidthPartInfo::RMa, std::bind (&InitRma, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::RMa_LoS, std::bind (&InitRma_LoS, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::RMa_nLoS, std::bind (&InitRma_nLoS, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMa, std::bind (&InitUma, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMa_LoS, std::bind (&InitUma_LoS, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMa_nLoS, std::bind (&InitUma_nLoS, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMi_StreetCanyon, std::bind (&InitUmi, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMi_StreetCanyon_LoS, std::bind (&InitUmi_LoS, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMi_StreetCanyon_nLoS, std::bind (&InitUmi_nLoS, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::InH_OfficeOpen, std::bind (&InitIndoorOpen, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::InH_OfficeMixed, std::bind (&InitIndoorMixed, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMa_Buildings, std::bind (&InitUmaBuildings, std::placeholders::_1, std::placeholders::_2)},
    {BandwidthPartInfo::UMi_Buildings, std::bind (&InitUmiBuildings, std::placeholders::_1, std::placeholders::_2)},
  };

  // Iterate over all CCs, and instantiate the channel and propagation model
  for (const auto & cc : band->m_cc)
    {
      for (const auto & bwp : cc->m_bwp)
        {
          // Initialize the type ID of the factories by calling the relevant
          // static function defined above and stored inside the lookup table
          initLookupTable.at (bwp->m_scenario) (&m_pathlossModelFactory, &m_channelConditionModelFactory);

          auto channelConditionModel  = m_channelConditionModelFactory.Create<ChannelConditionModel>();

          if (bwp->m_propagation == nullptr && flags & INIT_PROPAGATION)
            {

              bwp->m_propagation = m_pathlossModelFactory.Create <ThreeGppPropagationLossModel> ();
              bwp->m_propagation->SetAttributeFailSafe ("Frequency", DoubleValue (bwp->m_centralFrequency));
              DynamicCast<ThreeGppPropagationLossModel> (bwp->m_propagation)->SetChannelConditionModel (channelConditionModel);
            }

          if (bwp->m_3gppChannel == nullptr && flags & INIT_FADING)
            {
              bwp->m_3gppChannel = m_spectrumPropagationFactory.Create<ThreeGppSpectrumPropagationLossModel>();
              DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel)->SetChannelModelAttribute ("Frequency", DoubleValue (bwp->m_centralFrequency));
              DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel)->SetChannelModelAttribute ("Scenario", StringValue (bwp->GetScenario ()));
              DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel)->SetChannelModelAttribute ("ChannelConditionModel", PointerValue (channelConditionModel));

              m_raytracing[cc->m_ccId][bwp->m_bwpId] = DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel);
            }

          if (bwp->m_channel == nullptr && flags & INIT_CHANNEL)
            {
              bwp->m_channel = m_channelFactory.Create<SpectrumChannel> ();
              bwp->m_channel->AddPropagationLossModel (bwp->m_propagation);
              bwp->m_channel->AddSpectrumPropagationLossModel (DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel));
            }
            /*
            m_raytracing[bwp->m_bwpId] = m_spectrumPropagationFactory.Create<ThreeGppSpectrumPropagationLossModel> ();
            m_raytracing[bwp->m_bwpId]->SetChannelModelAttribute ("Frequency", DoubleValue (bwp->m_centralFrequency));
            m_raytracing[bwp->m_bwpId]->SetChannelModelAttribute ("Scenario", StringValue (bwp->GetScenario ()));
            m_raytracing[bwp->m_bwpId]->SetChannelModelAttribute ("ChannelConditionModel", PointerValue (channelConditionModel));*/
        }
    }
}

uint32_t
NrHelper::GetNumberBwp (const Ptr<const NetDevice> &gnbDevice)
{
  NS_LOG_FUNCTION (gnbDevice);
  Ptr<const NrGnbNetDevice> netDevice = DynamicCast<const NrGnbNetDevice> (gnbDevice);
  if (netDevice == nullptr)
    {
      return 0;
    }
  return netDevice->GetCcMapSize ();
}

Ptr<NrGnbPhy>
NrHelper::GetGnbPhy (const Ptr<NetDevice> &gnbDevice, uint32_t bwpIndex)
{
  NS_LOG_FUNCTION (gnbDevice << bwpIndex);
  NS_ASSERT(bwpIndex < UINT8_MAX);
  Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice> (gnbDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }
  return netDevice->GetPhy (static_cast<uint8_t> (bwpIndex));
}

Ptr<NrGnbMac>
NrHelper::GetGnbMac (const Ptr<NetDevice> &gnbDevice, uint32_t bwpIndex)
{
  NS_LOG_FUNCTION (gnbDevice << bwpIndex);
  NS_ASSERT(bwpIndex < UINT8_MAX);
  Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice> (gnbDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }
  return netDevice->GetMac (static_cast<uint8_t> (bwpIndex));
}

Ptr<NrUeMac>
NrHelper::GetUeMac(const Ptr<NetDevice> &ueDevice, uint32_t bwpIndex)
{
  NS_LOG_FUNCTION (ueDevice << bwpIndex);
  NS_ASSERT(bwpIndex < UINT8_MAX);
  Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice> (ueDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }
  return netDevice->GetMac (static_cast<uint8_t> (bwpIndex));
}

Ptr<NrUePhy>
NrHelper::GetUePhy(const Ptr<NetDevice> &ueDevice, uint32_t bwpIndex)
{
  NS_LOG_FUNCTION (ueDevice << bwpIndex);
  NS_ASSERT(bwpIndex < UINT8_MAX);
  Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice> (ueDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }
  return netDevice->GetPhy (static_cast<uint8_t> (bwpIndex));
}

Ptr<BwpManagerGnb>
NrHelper::GetBwpManagerGnb(const Ptr<NetDevice> &gnbDevice)
{
  NS_LOG_FUNCTION (gnbDevice);

  Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice> (gnbDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }

  return netDevice->GetBwpManager ();
}

Ptr<BwpManagerUe>
NrHelper::GetBwpManagerUe(const Ptr<NetDevice> &ueDevice)
{
  NS_LOG_FUNCTION (ueDevice);

  Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice> (ueDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }

  return netDevice->GetBwpManager ();
}

Ptr<NrMacScheduler>
NrHelper::GetScheduler(const Ptr<NetDevice> &gnbDevice, uint32_t bwpIndex)
{
  NS_LOG_FUNCTION (gnbDevice << bwpIndex);

  Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice> (gnbDevice);
  if (netDevice == nullptr)
    {
      return nullptr;
    }

  return netDevice->GetScheduler (bwpIndex);
}

void
NrHelper::SetHarqEnabled (bool harqEnabled)
{
  m_harqEnabled = harqEnabled;
}

bool
NrHelper::GetHarqEnabled ()
{
  return m_harqEnabled;
}

void
NrHelper::SetSnrTest (bool snrTest)
{
  m_snrTest = snrTest;
}

bool
NrHelper::GetSnrTest ()
{
  return m_snrTest;
}

NetDeviceContainer
NrHelper::InstallUeDevice (const NodeContainer &c,
                               const std::vector<std::reference_wrapper<BandwidthPartInfoPtr> > &allBwps)
{
  NS_LOG_FUNCTION (this);
  Initialize ();    // Run DoInitialize (), if necessary
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<NetDevice> device = InstallSingleUeDevice (node, allBwps);
      device->SetAddress (Mac48Address::Allocate ());
      devices.Add (device);
    }
  return devices;

}

NetDeviceContainer
NrHelper::InstallGnbDevice (const NodeContainer & c,
                                const std::vector<std::reference_wrapper<BandwidthPartInfoPtr> > allBwps)
{
  NS_LOG_FUNCTION (this);
  Initialize ();    // Run DoInitialize (), if necessary
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<NetDevice> device = InstallSingleGnbDevice (node, allBwps);
      device->SetAddress (Mac48Address::Allocate ());
      devices.Add (device);
    }
  return devices;
}

NetDeviceContainer
NrHelper::InstallLteEnbDevice (NodeContainer c)
{
  NS_LOG_FUNCTION (this);
  Initialize ();        // Run DoInitialize (), if necessary
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
  {
    Ptr<Node> node = *i;
    Ptr<NetDevice> device = InstallSingleLteEnbDevice (node);
    device->SetAddress (Mac64Address::Allocate ());
    devices.Add (device);
  }
  return devices;
}

Ptr<NrUeMac>
NrHelper::CreateUeMac () const
{
  NS_LOG_FUNCTION (this);
  Ptr<NrUeMac> mac = m_ueMacFactory.Create <NrUeMac> ();
  return mac;
}

Ptr<NrUePhy>
NrHelper::CreateUePhy (const Ptr<Node> &n, const std::unique_ptr<BandwidthPartInfo> &bwp,
                           const Ptr<NrUeNetDevice> &dev,
                           const NrSpectrumPhy::NrPhyDlHarqFeedbackCallback &dlHarqCallback,
                           const NrSpectrumPhy::NrPhyRxCtrlEndOkCallback &phyRxCtrlCallback)
{
  NS_LOG_FUNCTION (this);

  Ptr<NrSpectrumPhy> channelPhy = m_ueSpectrumFactory.Create <NrSpectrumPhy> ();
  Ptr<NrUePhy> phy = m_uePhyFactory.Create <NrUePhy> ();
  Ptr<NrHarqPhy> harq = Create<NrHarqPhy> ();

  Ptr<ThreeGppAntennaArrayModel> antenna = m_ueAntennaFactory.Create <ThreeGppAntennaArrayModel> ();
  antenna->SetAttribute ("HorizontalBeamStep", DoubleValue(phy->GetUeHorizontalAngleStep ()));
  antenna->SetAttribute ("VerticalBeamStep", DoubleValue (phy->GetUeVerticalAngleStep ()));

  ///IMPLEMENTATION OF THE PREVIOUS VERSION
  ///IN THIS VERSION, IT IS ALREADY DONE WITHIN THE INITIALIZEOPERATIONBAND
  ///WHEN WE TRY TO DO IT HERE AS WELL, WE CODE GENERATES ERROR IN STARTTXCONTROLFRAME
  /*if (m_channelModelType == "ns3::ThreeGppChannelModel")
  {
    //m_raytracing.at (bwp->m_bwpId)->AddDevice (dev, antenna);
    bwp->m_3gppChannel->AddDevice (dev, antenna);
    phy->AddSpectrumPropagationLossModel (m_raytracing.at (bwp->m_bwpId));
    phy->AddPropagationLossModel (bwp->m_propagation);
  }*/

  phy->SetPhyIdealBeamformingHelper (m_idealBeamformingHelper);

  DoubleValue frequency;
  bool res = bwp->m_propagation->GetAttributeFailSafe ("Frequency", frequency);
  NS_ASSERT_MSG (res, "Propagation model without Frequency attribute");
  phy->InstallCentralFrequency (frequency.Get ());

  phy->ScheduleStartEventLoop (n->GetId (), 0, 0, 0);

  Ptr<NrChAccessManager> cam = DynamicCast<NrChAccessManager> (m_ueChannelAccessManagerFactory.Create ());
  cam->SetNrSpectrumPhy (channelPhy);
  phy->SetCam (cam);

  channelPhy->InstallHarqPhyModule (harq);

  Ptr<LteChunkProcessor> pData = Create<LteChunkProcessor> ();
  pData->AddCallback (MakeCallback (&NrUePhy::GenerateDlCqiReport, phy));
  pData->AddCallback (MakeCallback (&NrSpectrumPhy::UpdateSinrPerceived, channelPhy));
  channelPhy->AddDataSinrChunkProcessor (pData);

  if (m_harqEnabled)
    {
      channelPhy->SetPhyDlHarqFeedbackCallback (dlHarqCallback);
    }

  NS_ASSERT (bwp->m_channel != nullptr);
  channelPhy->SetChannel (bwp->m_channel);
  channelPhy->InstallPhy (phy);

  Ptr<MobilityModel> mm = n->GetObject<MobilityModel> ();
  NS_ASSERT_MSG (mm, "MobilityModel needs to be set on node before calling NrHelper::InstallUeDevice ()");
  channelPhy->SetMobility (mm);

  channelPhy->SetPhyRxDataEndOkCallback (MakeCallback (&NrUePhy::PhyDataPacketReceived, phy));
  channelPhy->SetPhyRxCtrlEndOkCallback (phyRxCtrlCallback);

  phy->InstallSpectrumPhy (channelPhy);
  phy->InstallAntenna (antenna);
  

  if (m_realisticIA)
  {
    bwp->m_channel->AddRx (channelPhy);
  }
  
  phy->DoSetInitialIAState (m_realisticIA);  

  // TODO: If antenna changes, this will be broken
  auto channel = DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel);
  if (channel)
    {
      channel->AddDevice (dev, phy->GetSpectrumPhy()->GetAntennaArray());
      phy->AddSpectrumPropagationLossModel (DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel));
      phy->AddPropagationLossModel (bwp->m_propagation);
    }


  return phy;
}

Ptr<NetDevice>
NrHelper::InstallSingleUeDevice (const Ptr<Node> &n,
                                     const std::vector<std::reference_wrapper<BandwidthPartInfoPtr> > allBwps)
{
  NS_LOG_FUNCTION (this);

  Ptr<NrUeNetDevice> dev = m_ueNetDeviceFactory.Create<NrUeNetDevice> ();
  dev->SetNode (n);

  std::map<uint8_t, Ptr<BandwidthPartUe> > ueCcMap;
  uint16_t beamVectorMapSize;

  // Create, for each ue, its bandwidth parts
  for (uint32_t bwpId = 0; bwpId < allBwps.size (); ++bwpId)
    {
      Ptr <BandwidthPartUe> cc =  CreateObject<BandwidthPartUe> ();
      double bwInKhz = allBwps[bwpId].get()->m_channelBandwidth / 1000.0;
      NS_ABORT_MSG_IF (bwInKhz/100.0 > 65535.0, "A bandwidth of " << bwInKhz/100.0 << " kHz cannot be represented");
      cc->SetUlBandwidth (static_cast<uint16_t> (bwInKhz / 100));
      cc->SetDlBandwidth (static_cast<uint16_t> (bwInKhz / 100));
      cc->SetDlEarfcn (bwpId + 1); // Used for nothing..
      cc->SetUlEarfcn (bwpId + 1); // Used for nothing..

      auto mac = CreateUeMac ();
      cc->SetMac (mac);

      auto phy = CreateUePhy (n, allBwps[bwpId].get(),
                              dev, MakeCallback (&NrUeNetDevice::EnqueueDlHarqFeedback, dev),
                              std::bind (&NrUeNetDevice::RouteIngoingCtrlMsgs, dev,
                                         std::placeholders::_1, bwpId));
      phy->SetBwpId (bwpId);
      phy->SetDevice (dev);
      phy->GetSpectrumPhy ()->SetDevice (dev);

      if (m_realisticIA)
      {
        phy->GetSpectrumPhy ()->SetIAState (true);
      }      
      else
      {
        phy->GetSpectrumPhy ()->SetIAState (false);
      }
      
      phy->SetNoOfBeamsTbRLM (m_noOfBeamsTbRLM);
      phy->SetNoOfBeamsTbReported (m_noOfBeamsTbReported);
      phy->SetNoOfSSBsPerSlot (m_noOfSSBsPerSlot);
      beamVectorMapSize = phy->DoGenerateBeamVectorMap().size ();
      phy->m_beamSweepTimer = TimeValue (MilliSeconds(double(beamVectorMapSize) * 20.0 * 1.25));
      phy->SetAdaptiveBFProperty (m_adaptiveBF);
      phy->m_realisticIA = m_realisticIA; 
      phy->m_rlmOn = m_rlmOn;
      phy->SetPHYEpcHelper (m_epcHelper);

      cc->SetPhy (phy);

      if (bwpId == 0)
        {
          cc->SetAsPrimary (true);
        }
      else
        {
          cc->SetAsPrimary (false);
        }

      ueCcMap.insert (std::make_pair (bwpId, cc));
    }

  TimeValue beamSweepTimer = TimeValue (MilliSeconds(20.0 * double(beamVectorMapSize) * 1.25));
  Config::SetDefault ("ns3::LteUeRrc::BeamSweepTimer", beamSweepTimer);
  Config::SetDefault ("ns3::LteUeRrc::StartBeamSweepTimer", beamSweepTimer);
  Config::SetDefault ("ns3::LteEnbRrc::BeamSweepTimeoutDuration", beamSweepTimer);

  Ptr<LteUeComponentCarrierManager> ccmUe = DynamicCast<LteUeComponentCarrierManager> (CreateObject <BwpManagerUe> ());
  DynamicCast<BwpManagerUe> (ccmUe)->SetBwpManagerAlgorithm (m_ueBwpManagerAlgoFactory.Create <BwpManagerAlgorithm> ());

  Ptr<LteUeRrc> rrc = CreateObject<LteUeRrc> ();
  rrc->m_numberOfComponentCarriers = ueCcMap.size ();
  // run intializeSap to create the proper number of sap provider/users
  rrc->InitializeSap ();
  rrc->SetLteMacSapProvider (ccmUe->GetLteMacSapProvider ());
  // setting ComponentCarrierManager SAP
  rrc->SetLteCcmRrcSapProvider (ccmUe->GetLteCcmRrcSapProvider ());
  ccmUe->SetLteCcmRrcSapUser (rrc->GetLteCcmRrcSapUser ());
  ccmUe->SetNumberOfComponentCarriers (ueCcMap.size ());

  bool useIdealRrc = true;
  if (useIdealRrc)
    {
      Ptr<nrUeRrcProtocolIdeal> rrcProtocol = CreateObject<nrUeRrcProtocolIdeal> ();
      rrcProtocol->SetUeRrc (rrc);
      rrc->AggregateObject (rrcProtocol);
      rrcProtocol->SetLteUeRrcSapProvider (rrc->GetLteUeRrcSapProvider ());
      rrc->SetLteUeRrcSapUser (rrcProtocol->GetLteUeRrcSapUser ());
    }
  else
    {
      Ptr<LteUeRrcProtocolReal> rrcProtocol = CreateObject<LteUeRrcProtocolReal> ();
      rrcProtocol->SetUeRrc (rrc);
      rrc->AggregateObject (rrcProtocol);
      rrcProtocol->SetLteUeRrcSapProvider (rrc->GetLteUeRrcSapProvider ());
      rrc->SetLteUeRrcSapUser (rrcProtocol->GetLteUeRrcSapUser ());
    }

  if (m_epcHelper != nullptr)
    {
      rrc->SetUseRlcSm (false);
    }
  else
    {
      rrc->SetUseRlcSm (true);
    }
  Ptr<EpcUeNas> nas = CreateObject<EpcUeNas> ();

  nas->SetAsSapProvider (rrc->GetAsSapProvider ());
  nas->SetDevice (dev);
  nas->SetForwardUpCallback (MakeCallback (&NrUeNetDevice::Receive, dev));

  rrc->SetAsSapUser (nas->GetAsSapUser ());
  rrc->m_rlmOn = m_rlmOn;

  for (auto it = ueCcMap.begin (); it != ueCcMap.end (); ++it)
    {
      rrc->SetLteUeCmacSapProvider (it->second->GetMac ()->GetUeCmacSapProvider (), it->first);
      it->second->GetMac ()->SetUeCmacSapUser (rrc->GetLteUeCmacSapUser (it->first));

      it->second->GetPhy ()->SetUeCphySapUser (rrc->GetLteUeCphySapUser ());
      rrc->SetLteUeCphySapProvider (it->second->GetPhy ()->GetUeCphySapProvider (), it->first);

      it->second->GetPhy ()->SetPhySapUser (it->second->GetMac ()->GetPhySapUser ());
      it->second->GetMac ()->SetPhySapProvider (it->second->GetPhy ()->GetPhySapProvider ());

      bool ccmTest = ccmUe->SetComponentCarrierMacSapProviders (it->first,
                                                                it->second->GetMac ()->GetUeMacSapProvider ());

      if (ccmTest == false)
        {
          NS_FATAL_ERROR ("Error in SetComponentCarrierMacSapProviders");
        }
    }

  NS_ABORT_MSG_IF (m_imsiCounter >= 0xFFFFFFFF, "max num UEs exceeded");
  uint64_t imsi = ++m_imsiCounter;

  dev->SetAttribute ("Imsi", UintegerValue (imsi));
  dev->SetCcMap (ueCcMap);
  dev->SetAttribute ("nrUeRrc", PointerValue (rrc));
  dev->SetAttribute ("EpcUeNas", PointerValue (nas));
  dev->SetAttribute ("LteUeComponentCarrierManager", PointerValue (ccmUe));

  n->AddDevice (dev);


  if (m_epcHelper != nullptr)
    {
      m_epcHelper->AddUe (dev, dev->GetImsi ());
    }

  dev->Initialize ();

  return dev;
}

Ptr<NrGnbPhy>
NrHelper::CreateGnbPhy (const Ptr<Node> &n, const std::unique_ptr<BandwidthPartInfo> &bwp,
                            const Ptr<NrGnbNetDevice> &dev,
                            const NrSpectrumPhy::NrPhyRxCtrlEndOkCallback &phyEndCtrlCallback)
{
  NS_LOG_FUNCTION (this);

  Ptr<NrSpectrumPhy> channelPhy = m_gnbSpectrumFactory.Create <NrSpectrumPhy> ();
  Ptr<NrGnbPhy> phy = m_gnbPhyFactory.Create <NrGnbPhy> ();
  Ptr<ThreeGppAntennaArrayModel> antenna = m_gnbAntennaFactory.Create <ThreeGppAntennaArrayModel> ();
  antenna->SetAttribute ("HorizontalBeamStep", DoubleValue (phy->GetGnbHorizontalAngleStep ()));
  antenna->SetAttribute ("VerticalBeamStep", DoubleValue (phy->GetGnbVerticalAngleStep ()));

  ///IMPLEMENTATION OF THE PREVIOUS VERSION
  ///IN THIS VERSION, IT IS ALREADY DONE WITHIN THE INITIALIZEOPERATIONBAND
  ///WHEN WE TRY TO DO IT HERE AS WELL, WE CODE GENERATES ERROR IN STARTTXCONTROLFRAME

  /*if (m_channelModelType == "ns3::ThreeGppChannelModel")
  {
    m_raytracing.at (bwp->m_bwpId)->AddDevice (dev, antenna);
    phy->AddSpectrumPropagationLossModel (m_raytracing.at (bwp->m_bwpId));
    phy->AddPropagationLossModel (bwp->m_propagation);
  }*/

  phy->SetPhyIdealBeamformingHelper (m_idealBeamformingHelper);

  DoubleValue frequency;
  bool res = bwp->m_propagation->GetAttributeFailSafe ("Frequency", frequency);
  NS_ASSERT_MSG (res, "Propagation model without Frequency attribute");
  phy->InstallCentralFrequency (frequency.Get ());

  phy->ScheduleStartEventLoop (n->GetId (), 0, 0, 0);

  // PHY <--> CAM
  Ptr<NrChAccessManager> cam = DynamicCast<NrChAccessManager> (m_gnbChannelAccessManagerFactory.Create ());
  cam->SetNrSpectrumPhy (channelPhy);
  phy->SetCam (cam);

  Ptr<NrHarqPhy> harq = Create<NrHarqPhy> ();
  channelPhy->InstallHarqPhyModule (harq);

  Ptr<LteChunkProcessor> pData = Create<LteChunkProcessor> ();
  if (!m_snrTest)
    {
      pData->AddCallback (MakeCallback (&NrGnbPhy::GenerateDataCqiReport, phy));
      pData->AddCallback (MakeCallback (&NrSpectrumPhy::UpdateSinrPerceived, channelPhy));
    }
  channelPhy->AddDataSinrChunkProcessor (pData);

  phy->SetDevice (dev);

  channelPhy->SetChannel (bwp->m_channel);
  channelPhy->InstallPhy (phy);

  Ptr<MobilityModel> mm = n->GetObject<MobilityModel> ();
  NS_ASSERT_MSG (mm, "MobilityModel needs to be set on node before calling NrHelper::InstallEnbDevice ()");
  channelPhy->SetMobility (mm);

  channelPhy->SetDevice (dev);
  channelPhy->SetPhyRxDataEndOkCallback (MakeCallback (&NrGnbPhy::PhyDataPacketReceived, phy));
  channelPhy->SetPhyRxCtrlEndOkCallback (phyEndCtrlCallback);
  channelPhy->SetPhyUlHarqFeedbackCallback (MakeCallback (&NrGnbPhy::ReportUlHarqFeedback, phy));

  phy->InstallSpectrumPhy (channelPhy);
  phy->InstallAntenna (antenna);

  bwp->m_channel->AddRx (channelPhy);
  auto channel = DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel);
  // TODO: NOTE: if changing the Antenna Array, this will broke
  if (channel)
    {
      channel->AddDevice (dev, phy->GetSpectrumPhy()->GetAntennaArray());
      phy->AddSpectrumPropagationLossModel (DynamicCast<ThreeGppSpectrumPropagationLossModel> (bwp->m_3gppChannel));
      phy->AddPropagationLossModel (bwp->m_propagation);
    }

  return phy;
}

Ptr<NrGnbMac>
NrHelper::CreateGnbMac ()
{
  NS_LOG_FUNCTION (this);

  Ptr<NrGnbMac> mac = m_gnbMacFactory.Create <NrGnbMac> ();
  return mac;
}

Ptr<NrMacScheduler>
NrHelper::CreateGnbSched ()
{
  NS_LOG_FUNCTION (this);

  auto sched = m_schedFactory.Create <NrMacSchedulerNs3> ();
  auto dlAmc = m_gnbDlAmcFactory.Create <NrAmc> ();
  auto ulAmc = m_gnbUlAmcFactory.Create <NrAmc> ();

  sched->InstallDlAmc (dlAmc);
  sched->InstallUlAmc (ulAmc);

  return sched;
}

Ptr<NetDevice>
NrHelper::InstallSingleGnbDevice (const Ptr<Node> &n,
                                      const std::vector<std::reference_wrapper<BandwidthPartInfoPtr> > allBwps)
{
  NS_ABORT_MSG_IF (m_cellIdCounter == 65535, "max num gNBs exceeded");

  Ptr<NrGnbNetDevice> dev = m_gnbNetDeviceFactory.Create<NrGnbNetDevice> ();

  NS_LOG_DEBUG ("Creating gNB, cellId = " << m_cellIdCounter);
  uint16_t cellId = m_cellIdCounter;

  dev->SetCellId (cellId);
  dev->SetNode (n);

  // create component carrier map for this gNB device
  std::map<uint8_t,Ptr<BandwidthPartGnb> > ccMap;

  for (uint32_t bwpId = 0; bwpId < allBwps.size (); ++bwpId)
    {
      NS_LOG_DEBUG ("Creating BandwidthPart, id = " << bwpId);
      //AddBandwidthPart(bwpId, allBwps[bwpId]);
      Ptr <BandwidthPartGnb> cc =  CreateObject<BandwidthPartGnb> ();
      double bwInKhz = allBwps[bwpId].get()->m_channelBandwidth / 1000.0;
      NS_ABORT_MSG_IF (bwInKhz/100.0 > 65535.0, "A bandwidth of " << bwInKhz/100.0 << " kHz cannot be represented");

      cc->SetUlBandwidth (static_cast<uint16_t> (bwInKhz / 100));
      cc->SetDlBandwidth (static_cast<uint16_t> (bwInKhz / 100));
      cc->SetDlEarfcn (bwpId + 1); // Argh... handover not working
      cc->SetUlEarfcn (bwpId + 1); // Argh... handover not working
      cc->SetCellId (m_cellIdCounter++);

      auto phy = CreateGnbPhy (n, allBwps[bwpId].get(), dev,
                               std::bind (&NrGnbNetDevice::RouteIngoingCtrlMsgs,
                                          dev, std::placeholders::_1, bwpId));
      phy->SetBwpId (bwpId);
      phy->SetAdaptiveBFProperty(m_adaptiveBF);
      phy->m_realisticIA = m_realisticIA; 
      phy->m_rlmOn = m_rlmOn;
      phy->SetNoOfBeamsTbRLM (m_noOfBeamsTbRLM);
      phy->SetNoOfBeamsTbReported (m_noOfBeamsTbReported);
      phy->SetNoOfSSBsPerSlot (m_noOfSSBsPerSlot);
      phy->SetPHYEpcHelper (m_epcHelper);
      
      cc->SetPhy (phy);

      auto mac = CreateGnbMac ();
      cc->SetMac (mac);
      phy->GetCam ()->SetNrGnbMac (mac);



      auto sched = CreateGnbSched ();
      cc->SetNrMacScheduler (sched);

      if (bwpId == 0)
        {
          cc->SetAsPrimary (true);
        }
      else
        {
          cc->SetAsPrimary (false);
        }

      ccMap.insert (std::make_pair (bwpId, cc));
    }

  Ptr<LteEnbRrc> rrc = CreateObject<LteEnbRrc> ();
  Ptr<LteEnbComponentCarrierManager> ccmEnbManager = DynamicCast<LteEnbComponentCarrierManager> (CreateObject<BwpManagerGnb> ());
  DynamicCast<BwpManagerGnb> (ccmEnbManager)->SetBwpManagerAlgorithm (m_gnbBwpManagerAlgoFactory.Create <BwpManagerAlgorithm> ());

  // Convert Enb carrier map to only PhyConf map
  // we want to make RRC to be generic, to be able to work with any type of carriers, not only strictly LTE carriers
  std::map < uint8_t, Ptr<ComponentCarrierBaseStation> > ccPhyConfMap;
  for (const auto &i : ccMap)
    {
      Ptr<ComponentCarrierBaseStation> c = i.second;
      ccPhyConfMap.insert (std::make_pair (i.first,c));
    }

  //ComponentCarrierManager SAP
  rrc->SetLteCcmRrcSapProvider (ccmEnbManager->GetLteCcmRrcSapProvider ());
  ccmEnbManager->SetLteCcmRrcSapUser (rrc->GetLteCcmRrcSapUser ());
  // Set number of component carriers. Note: eNB CCM would also set the
  // number of component carriers in eNB RRC

  ccmEnbManager->SetNumberOfComponentCarriers (ccMap.size ());
  rrc->ConfigureCarriers (ccPhyConfMap);

  rrc->m_realisticIA = m_realisticIA;

  //nr module currently uses only RRC ideal mode
  bool useIdealRrc = true;

  if (useIdealRrc)
    {
      Ptr<NrGnbRrcProtocolIdeal> rrcProtocol = CreateObject<NrGnbRrcProtocolIdeal> ();
      rrcProtocol->SetLteEnbRrcSapProvider (rrc->GetLteEnbRrcSapProvider ());
      rrc->SetLteEnbRrcSapUser (rrcProtocol->GetLteEnbRrcSapUser ());
      rrc->AggregateObject (rrcProtocol);
    }
  else
    {
      Ptr<LteEnbRrcProtocolReal> rrcProtocol = CreateObject<LteEnbRrcProtocolReal> ();
      rrcProtocol->SetLteEnbRrcSapProvider (rrc->GetLteEnbRrcSapProvider ());
      rrc->SetLteEnbRrcSapUser (rrcProtocol->GetLteEnbRrcSapUser ());
      rrc->AggregateObject (rrcProtocol);
    }

  if (m_epcHelper != nullptr)
    {
      EnumValue epsBearerToRlcMapping;
      rrc->GetAttribute ("EpsBearerToRlcMapping", epsBearerToRlcMapping);
      // it does not make sense to use RLC/SM when also using the EPC
      if (epsBearerToRlcMapping.Get () == LteEnbRrc::RLC_SM_ALWAYS)
        {
          rrc->SetAttribute ("EpsBearerToRlcMapping", EnumValue (LteEnbRrc::RLC_UM_ALWAYS));
        }
    }

  // This RRC attribute is used to connect each new RLC instance with the MAC layer
  // (for function such as TransmitPdu, ReportBufferStatusReport).
  // Since in this new architecture, the component carrier manager acts a proxy, it
  // will have its own LteMacSapProvider interface, RLC will see it as through original MAC
  // interface LteMacSapProvider, but the function call will go now through LteEnbComponentCarrierManager
  // instance that needs to implement functions of this interface, and its task will be to
  // forward these calls to the specific MAC of some of the instances of component carriers. This
  // decision will depend on the specific implementation of the component carrier manager.
  rrc->SetLteMacSapProvider (ccmEnbManager->GetLteMacSapProvider ());
  rrc->SetForwardUpCallback (MakeCallback (&NrGnbNetDevice::Receive, dev));

  for (auto it = ccMap.begin (); it != ccMap.end (); ++it)
    {
      it->second->GetPhy ()->SetEnbCphySapUser (rrc->GetLteEnbCphySapUser (it->first));
      rrc->SetLteEnbCphySapProvider (it->second->GetPhy ()->GetEnbCphySapProvider (), it->first);

      rrc->SetLteEnbCmacSapProvider (it->second->GetMac ()->GetEnbCmacSapProvider (),it->first );
      it->second->GetMac ()->SetEnbCmacSapUser (rrc->GetLteEnbCmacSapUser (it->first));

      // PHY <--> MAC SAP
      it->second->GetPhy ()->SetPhySapUser (it->second->GetMac ()->GetPhySapUser ());
      it->second->GetMac ()->SetPhySapProvider (it->second->GetPhy ()->GetPhySapProvider ());
      // PHY <--> MAC SAP END

      //Scheduler SAP
      it->second->GetMac ()->SetNrMacSchedSapProvider (it->second->GetScheduler ()->GetMacSchedSapProvider ());
      it->second->GetMac ()->SetNrMacCschedSapProvider (it->second->GetScheduler ()->GetMacCschedSapProvider ());

      it->second->GetScheduler ()->SetMacSchedSapUser (it->second->GetMac ()->GetNrMacSchedSapUser ());
      it->second->GetScheduler ()->SetMacCschedSapUser (it->second->GetMac ()->GetNrMacCschedSapUser ());
      // Scheduler SAP END

      it->second->GetMac ()->SetLteCcmMacSapUser (ccmEnbManager->GetLteCcmMacSapUser ());
      it->second->GetMac ()->m_realisticIA = m_realisticIA;
      it->second->GetMac ()->m_rlmON = m_rlmOn;
      ccmEnbManager->SetCcmMacSapProviders (it->first, it->second->GetMac ()->GetLteCcmMacSapProvider ());

      // insert the pointer to the LteMacSapProvider interface of the MAC layer of the specific component carrier
      ccmEnbManager->SetMacSapProvider (it->first, it->second->GetMac ()->GetMacSapProvider ());
    }


  dev->SetAttribute ("LteEnbComponentCarrierManager", PointerValue (ccmEnbManager));
  dev->SetCcMap (ccMap);
  dev->SetAttribute ("LteEnbRrc", PointerValue (rrc));
  dev->Initialize ();

  n->AddDevice (dev);

  if (m_epcHelper != nullptr)
    {
      NS_LOG_INFO ("adding this eNB to the EPC");
      m_epcHelper->AddEnb (n, dev, dev->GetCellId ());
      Ptr<EpcEnbApplication> enbApp = n->GetApplication (0)->GetObject<EpcEnbApplication> ();
      NS_ASSERT_MSG (enbApp != nullptr, "cannot retrieve EpcEnbApplication");

      // S1 SAPs
      rrc->SetS1SapProvider (enbApp->GetS1SapProvider ());
      enbApp->SetS1SapUser (rrc->GetS1SapUser ());

      // X2 SAPs
      Ptr<EpcX2> x2 = n->GetObject<EpcX2> ();
      x2->SetEpcX2SapUser (rrc->GetEpcX2SapUser ());
      rrc->SetEpcX2SapProvider (x2->GetEpcX2SapProvider ());
    }

  return dev;
}

Ptr<NetDevice>
NrHelper::InstallSingleLteEnbDevice (Ptr<Node> n)
{
  uint16_t cellId = m_cellIdCounter;      // \todo Remove, eNB has no cell ID
  
  Ptr<LteEnbNetDevice> dev = m_lteEnbNetDeviceFactory.Create<LteEnbNetDevice> ();
  Ptr<LteHandoverAlgorithm> handoverAlgorithm = m_lteHandoverAlgorithmFactory.Create<LteHandoverAlgorithm> ();

  NS_ASSERT_MSG (m_lteComponentCarrierPhyParams.size () != 0, "Cannot create enb ccm map.");
  //create component carrier map for this eNb device
  std::map<uint8_t,Ptr<ComponentCarrierEnb> > ccMap;
  for (std::map<uint8_t, ComponentCarrier>::iterator it = m_lteComponentCarrierPhyParams.begin (); it !=m_lteComponentCarrierPhyParams.end (); ++it)
  {
    Ptr <ComponentCarrierEnb> cc = CreateObject<ComponentCarrierEnb> ();
    cc->SetUlBandwidth (it->second.GetUlBandwidth ());
    cc->SetDlBandwidth (it->second.GetDlBandwidth ());
    cc->SetDlEarfcn (it->second.GetDlEarfcn ());
    cc->SetUlEarfcn (it->second.GetUlEarfcn ());
    cc->SetAsPrimary (it->second.IsPrimary ());
    NS_ABORT_MSG_IF (m_cellIdCounter == 65535, "nax bun cells exceeded");
    cc->SetCellId (m_cellIdCounter++);
    ccMap [it->first] = cc;
  }
  NS_ASSERT (ccMap.size () == m_noOfLteCcs);

  for (std::map<uint8_t,Ptr<ComponentCarrierEnb> >::iterator it = ccMap.begin (); it != ccMap.end(); ++it)
  {
    NS_LOG_DEBUG (this << "component carrier map size " << (uint16_t) ccMap.size ());
    Ptr<LteSpectrumPhy> dlPhy = CreateObject<LteSpectrumPhy> ();
    Ptr<LteSpectrumPhy> ulPhy = CreateObject<LteSpectrumPhy> ();
    Ptr<LteEnbPhy> phy = CreateObject<LteEnbPhy> (ulPhy, dlPhy);

    Ptr<LteHarqPhy> harq = Create<LteHarqPhy> ();
    dlPhy->SetHarqPhyModule (harq);
    ulPhy->SetHarqPhyModule (harq);
    phy->SetHarqPhyModule (harq);

    Ptr<LteChunkProcessor> pCtrl = Create<LteChunkProcessor> ();
    pCtrl->AddCallback (MakeCallback (&LteEnbPhy::GenerateCtrlCqiReport, phy));
    ulPhy->AddCtrlSinrChunkProcessor (pCtrl);   // for evaluating SRS UL-CQI

    Ptr<LteChunkProcessor> pData = Create<LteChunkProcessor> ();
    pData->AddCallback (MakeCallback (&LteEnbPhy::GenerateDataCqiReport, phy));
    pData->AddCallback (MakeCallback (&LteSpectrumPhy::UpdateSinrPerceived, ulPhy));
    ulPhy->AddDataSinrChunkProcessor (pData);   // for evaluating PUSCH UL-CQI

    Ptr<LteChunkProcessor> pInterf = Create<LteChunkProcessor> ();
    pInterf->AddCallback (MakeCallback (&LteEnbPhy::ReportInterference, phy));
    ulPhy->AddInterferenceDataChunkProcessor (pInterf);   // for interference power tracing

    dlPhy->SetChannel (m_downlinkChannel);
    ulPhy->SetChannel (m_uplinkChannel);

    Ptr<MobilityModel> mm = n->GetObject<MobilityModel> ();
    NS_ASSERT_MSG (mm, "MobilityModel needs to be set on node before calling LteHelper::InstallEnbDevice ()");
    dlPhy->SetMobility (mm);
    ulPhy->SetMobility (mm);

    /*Ptr<AntennaModel> antenna = (m_lteEnbAntennaModelFactory.Create ())->GetObject<AntennaModel> ();
    NS_ASSERT_MSG (antenna, "error in creating the AntennaModel object");
    dlPhy->SetAntenna (antenna);
    ulPhy->SetAntenna (antenna);*/

    Ptr<LteEnbMac> mac = CreateObject<LteEnbMac> ();
    Ptr<FfMacScheduler> sched = m_lteSchedulerFactory.Create<FfMacScheduler> ();
    Ptr<LteFfrAlgorithm> ffrAlgorithm = m_lteFfrAlgorithmFactory.Create<LteFfrAlgorithm> ();
    it->second->SetMac (mac);
    it->second->SetFfMacScheduler (sched);
    it->second->SetFfrAlgorithm (ffrAlgorithm);

    it->second->SetPhy (phy);
  }

  // Convert Enb carrier map to only PhyConf map
  // we want to make RRC to be generic, to be able to work with any type of carriers, not only strictly LTE carriers
  std::map < uint8_t, Ptr<ComponentCarrierBaseStation> > ccPhyConfMap;
  for (const auto &i : ccMap)
  {
    Ptr<ComponentCarrierBaseStation> c = i.second;
    ccPhyConfMap.insert (std::pair<uint8_t, Ptr<ComponentCarrierBaseStation> > (i.first, c));
  }

  Ptr<LteEnbRrc> rrc = CreateObject<LteEnbRrc> ();
  Ptr<LteEnbComponentCarrierManager> ccmEnbManager = m_lteEnbComponentCarrierManagerFactory.Create<LteEnbComponentCarrierManager> ();

  //ComponentCarrierManager SAP
  rrc->SetLteCcmRrcSapProvider (ccmEnbManager->GetLteCcmRrcSapProvider ());
  ccmEnbManager->SetLteCcmRrcSapUser (rrc->GetLteCcmRrcSapUser ());
  ccmEnbManager->SetNumberOfComponentCarriers (m_noOfLteCcs);
  //ccmEnbManager->SetRrc (rrc); //deleted from LTE module by CTTC

  rrc->ConfigureCarriers (ccPhyConfMap);

  rrc->m_realisticIA = m_realisticIA;
  rrc->m_rlmOn = m_rlmOn;

  Ptr<LteEnbRrcProtocolReal> rrcProtocol = CreateObject<LteEnbRrcProtocolReal> ();
  rrcProtocol->SetLteEnbRrcSapProvider (rrc->GetLteEnbRrcSapProvider ());
  rrc->SetLteEnbRrcSapUser (rrcProtocol->GetLteEnbRrcSapUser ());
  rrc->AggregateObject (rrcProtocol);
  rrcProtocol->SetCellId (cellId);

  if (m_epcHelper != 0)
  {
    EnumValue epsBearerToRlcMapping;
    rrc->GetAttribute ("EpsBearerToRlcMapping", epsBearerToRlcMapping);
    rrc->SetAttribute ("EpsBearerToRlcMapping", EnumValue (LteEnbRrc::RLC_AM_ALWAYS));
  }

  rrc->SetLteHandoverManagementSapProvider (handoverAlgorithm->GetLteHandoverManagementSapProvider ());
  handoverAlgorithm->SetLteHandoverManagementSapUser (rrc->GetLteHandoverManagementSapUser ());

  // This RRC attribute is used to connect each new RLC instance with the MAC layer
  // (for function such as TransmitPdu, ReportBufferStatusReport).
  // Since in this new architecture, the component carrier manager acts a proxy, it
  // will have its own LteMacSapProvider interface, RLC will see it as through original MAC
  // interface LteMacSapProvider, but the function call will go now through LteEnbComponentCarrierManager
  // instance that needs to implement functions of this interface, and its task will be to
  // forward these calls to the specific MAC of some of the instances of component carriers. This
  // decision will depend on the specific implementation of the component carrier manager.
  rrc->SetLteMacSapProvider (ccmEnbManager->GetLteMacSapProvider ());

  bool ccmTest;
  for (std::map<uint8_t,Ptr<ComponentCarrierEnb> >::iterator it = ccMap.begin (); it!= ccMap.end (); ++it)
  {
    it->second->GetPhy ()->SetLteEnbCphySapUser (rrc->GetLteEnbCphySapUser (it->first));
    rrc->SetLteEnbCphySapProvider (it->second->GetPhy ()->GetLteEnbCphySapProvider (), it->first);

    rrc->SetLteEnbCmacSapProvider (it->second->GetMac ()->GetLteEnbCmacSapProvider (), it->first);
    it->second->GetMac ()->SetLteEnbCmacSapUser (rrc->GetLteEnbCmacSapUser (it->first));

    it->second->GetPhy ()->SetComponentCarrierId (it->first);
    it->second->GetMac ()->SetComponentCarrierId (it->first);
    //FFR SAP
    it->second->GetFfMacScheduler ()->SetLteFfrSapProvider (it->second->GetFfrAlgorithm ()->GetLteFfrSapProvider ());
    it->second->GetFfrAlgorithm ()->SetLteFfrSapUser (it->second->GetFfMacScheduler ()->GetLteFfrSapUser ());
    rrc->SetLteFfrRrcSapProvider (it->second->GetFfrAlgorithm ()->GetLteFfrRrcSapProvider (), it->first);
    it->second->GetFfrAlgorithm ()->SetLteFfrRrcSapUser (rrc->GetLteFfrRrcSapUser (it->first));
    //FFR SAP END

    // PHY <--> MAC SAP
    it->second->GetPhy ()->SetLteEnbPhySapUser (it->second->GetMac ()->GetLteEnbPhySapUser ());
    it->second->GetMac ()->SetLteEnbPhySapProvider (it->second->GetPhy ()->GetLteEnbPhySapProvider ());
    // PHY <--> MAC SAP END

    //Scheduler SAP
    it->second->GetMac ()->SetFfMacSchedSapProvider (it->second->GetFfMacScheduler ()->GetFfMacSchedSapProvider ());
    it->second->GetMac ()->SetFfMacCschedSapProvider (it->second->GetFfMacScheduler ()->GetFfMacCschedSapProvider ());

    it->second->GetFfMacScheduler ()->SetFfMacSchedSapUser (it->second->GetMac ()->GetFfMacSchedSapUser ());
    it->second->GetFfMacScheduler ()->SetFfMacCschedSapUser (it->second->GetMac ()->GetFfMacCschedSapUser ());
    // Scheduler SAP END

    it->second->GetMac ()->SetLteCcmMacSapUser (ccmEnbManager->GetLteCcmMacSapUser ());
    ccmEnbManager->SetCcmMacSapProviders (it->first, it->second->GetMac ()->GetLteCcmMacSapProvider ());

    // insert the pointer to the LteMacSapProvider interface of the MAC layer of the specific component carrier
    ccmTest = ccmEnbManager->SetMacSapProvider (it->first, it->second->GetMac ()->GetLteMacSapProvider ());

    if (ccmTest == false)
      {
        NS_FATAL_ERROR ("Error in SetComponentCarrierMacSapProviders");
      }
  }

  dev->SetNode (n);
  dev->SetAttribute ("CellId", UintegerValue (cellId));
  dev->SetAttribute ("LteEnbComponentCarrierManager", PointerValue (ccmEnbManager));
  dev->SetCcMap (ccPhyConfMap);
  std::map<uint8_t,Ptr<ComponentCarrierEnb> >::iterator it = ccMap.begin ();
  dev->SetAttribute ("LteEnbRrc", PointerValue (rrc));
  dev->SetAttribute ("LteHandoverAlgorithm", PointerValue (handoverAlgorithm));
  dev->SetAttribute ("LteFfrAlgorithm", PointerValue (it->second->GetFfrAlgorithm ()));

  for (it = ccMap.begin (); it != ccMap.end (); ++it)
  {
    Ptr<LteEnbPhy> ccPhy = it->second->GetPhy ();
    ccPhy->SetDevice (dev);
    ccPhy->GetUlSpectrumPhy ()->SetDevice (dev);
    ccPhy->GetDlSpectrumPhy ()->SetDevice (dev);
    ccPhy->GetUlSpectrumPhy ()->SetLtePhyRxDataEndOkCallback (MakeCallback (&LteEnbPhy::PhyPduReceived, ccPhy));
    ccPhy->GetUlSpectrumPhy ()->SetLtePhyRxCtrlEndOkCallback (MakeCallback (&LteEnbPhy::ReceiveLteControlMessageList, ccPhy));
    ccPhy->GetUlSpectrumPhy ()->SetLtePhyUlHarqFeedbackCallback (MakeCallback (&LteEnbPhy::ReportUlHarqFeedback, ccPhy));
    NS_LOG_LOGIC ("set the propagation model frequencies");
  } //end for
  rrc->SetForwardUpCallback (MakeCallback (&LteEnbNetDevice::Receive, dev));
  dev->Initialize ();
  n->AddDevice (dev);

  for (it = ccMap.begin (); it != ccMap.end (); ++it)
  {
    m_uplinkChannel->AddRx (it->second->GetPhy ()->GetUlSpectrumPhy ());
  }

  if (m_epcHelper != 0)
  {
    NS_LOG_INFO ("adding this eNB to the EPC");
    m_epcHelper->AddEnb (n, dev, dev->GetCellId ());
    Ptr<EpcEnbApplication> enbApp = n->GetApplication (0)->GetObject<EpcEnbApplication> ();
    NS_ASSERT_MSG (enbApp != 0, "cannot retrieve EpcEnbApplication");

    // S1 SAPs
    rrc->SetS1SapProvider (enbApp->GetS1SapProvider ());
    enbApp->SetS1SapUser (rrc->GetS1SapUser ());

    // X2 SAPs
    Ptr<EpcX2> x2 = n->GetObject<EpcX2> ();
    x2->SetEpcX2SapUser (rrc->GetEpcX2SapUser ());
    rrc->SetEpcX2SapProvider (x2->GetEpcX2SapProvider ());
    //rrc->SetEpcX2PdcpProvider (x2->GetEpcX2PdcpProvider ());
  }

  return dev;
}

void
NrHelper::AttachToClosestEnb (NetDeviceContainer ueDevices, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (this);

  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); i++)
    {
      AttachToClosestEnb (*i, enbDevices);
    }
}

void
NrHelper::AttachToClosestEnb (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (enbDevices.GetN () > 0, "empty enb device container");
  Vector uepos = ueDevice->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
  double minDistance = std::numeric_limits<double>::infinity ();
  Ptr<NetDevice> closestEnbDevice;
  for (NetDeviceContainer::Iterator i = enbDevices.Begin (); i != enbDevices.End (); ++i)
    {
      Vector enbpos = (*i)->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double distance = CalculateDistance (uepos, enbpos);
      if (distance < minDistance)
        {
          minDistance = distance;
          closestEnbDevice = *i;
        }
    }
  NS_ASSERT (closestEnbDevice != 0);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Ptr<NrUeNetDevice> ueNetDev = ueDevice->GetObject<NrUeNetDevice> ();

  for (NetDeviceContainer::Iterator i = enbDevices.Begin (); i != enbDevices.End (); ++i)
  {
    Ptr<NrGnbNetDevice> gnbNetDev = (*i)->GetObject<NrGnbNetDevice> ();

    std::map<uint8_t, Ptr<BandwidthPartGnb> > gnbBwpMap = gnbNetDev->GetCcMap ();
    for (std::map<uint8_t, Ptr<BandwidthPartGnb>>::iterator itGnb = gnbBwpMap.begin (); itGnb != gnbBwpMap.end (); ++itGnb)
    {
      uint16_t nrCellId = itGnb->second->GetCellId ();
      itGnb->second->GetPhy ()->RegisterUe (ueNetDev->GetImsi (), ueNetDev);
      //Register to other GNB using BWP ID as well
      itGnb->second->GetPhy ()->GetBwpId ();

      std::map<uint8_t, Ptr<BandwidthPartUe>> ueCcMap = ueNetDev->GetCcMap ();
      for (std::map<uint8_t, Ptr<BandwidthPartUe>>::iterator itUe = ueCcMap.begin (); itUe != ueCcMap.end (); ++itUe)
      {
        itUe->second->GetPhy ()->RegisterOtherEnb (nrCellId, gnbNetDev);
      }
      NS_LOG_INFO ("nrCellId " << nrCellId);
    }
    if (!m_realisticIA)
    {
      m_idealBeamformingHelper->AddBeamformingTask (gnbNetDev, ueNetDev);
    }    
    gnbNetDev->GetPhy (0)->GetBeamManager ()->SetSector (0,50);
    ueNetDev->GetPhy (0)->GetBeamManager ()->SetSector (0,70);
  }
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if (!m_realisticIA)  ///IF realistic IA is employed, dont Attach to the ENB /* CHANGED DUE TO PROBLEMS WITH TCP !!!
    {
      AttachToEnb (ueDevice, closestEnbDevice);
    }
  
}


void
NrHelper::AttachToEnb (const Ptr<NetDevice> &ueDevice,
                           const Ptr<NetDevice> &gnbDevice)
{
  Ptr<NrGnbNetDevice> enbNetDev = gnbDevice->GetObject<NrGnbNetDevice> ();
  Ptr<NrUeNetDevice> ueNetDev = ueDevice->GetObject<NrUeNetDevice> ();

  NS_ABORT_IF (enbNetDev == nullptr || ueNetDev == nullptr);

  for (uint32_t i = 0; i < enbNetDev->GetCcMapSize (); ++i)
    {
      enbNetDev->GetPhy(i)->RegisterUe (ueNetDev->GetImsi (), ueNetDev);
      ueNetDev->GetPhy (i)->RegisterToEnb (enbNetDev->GetBwpId (i));
      ueNetDev->GetPhy (i)->SetDlAmc (
            DynamicCast<NrMacSchedulerNs3> (enbNetDev->GetScheduler (i))->GetDlAmc ());
      ueNetDev->GetPhy (i)->SetDlCtrlSyms (enbNetDev->GetMac(i)->GetDlCtrlSyms ());
      ueNetDev->GetPhy (i)->SetUlCtrlSyms (enbNetDev->GetMac(i)->GetUlCtrlSyms ());
      ueNetDev->GetPhy (i)->SetNumRbPerRbg (enbNetDev->GetMac(i)->GetNumRbPerRbg());
      ueNetDev->GetPhy (i)->SetRbOverhead (enbNetDev->GetPhy (i)->GetRbOverhead ());
      ueNetDev->GetPhy (i)->SetSymbolsPerSlot (enbNetDev->GetPhy (i)->GetSymbolsPerSlot ());
      ueNetDev->GetPhy (i)->SetNumerology (enbNetDev->GetPhy(i)->GetNumerology ());
      ueNetDev->GetPhy (i)->SetPattern (enbNetDev->GetPhy (i)->GetPattern ());
      Ptr<EpcUeNas> ueNas = ueNetDev->GetNas ();
      ueNas->Connect (enbNetDev->GetBwpId (i), enbNetDev->GetEarfcn (i));
    }

  if (m_epcHelper != nullptr)
    {
      // activate default EPS bearer
      m_epcHelper->ActivateEpsBearer (ueDevice, ueNetDev->GetImsi (), EpcTft::Default (), EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT));
    }

  // tricks needed for the simplified LTE-only simulations
  //if (m_epcHelper == 0)
  //{
  ueNetDev->SetTargetEnb (enbNetDev);
  //}
}

void 
NrHelper::AttachToLteCoordinator (const Ptr<NetDevice> &ueDevice,
                                    const Ptr<NetDevice> &enbDevice)
{
  Ptr<LteEnbNetDevice> enbNetDev = enbDevice->GetObject<LteEnbNetDevice> ();
  Ptr<NrUeNetDevice> ueNetDev = ueDevice->GetObject<NrUeNetDevice> ();

  NS_ABORT_IF (enbNetDev == nullptr || ueNetDev == nullptr);

  uint8_t lteRnti = enbNetDev->GetRrc ()->AddUe (UeManager::CONNECTION_LTE_COORDINATOR, 0);
  enbNetDev->GetRrc ()->SetupUeFromHelper (lteRnti);
  ueNetDev->GetRrc ()->SetLteEnbRrcSapProviderFromHelper ();  
}


uint8_t
NrHelper::ActivateDedicatedEpsBearer (NetDeviceContainer ueDevices, EpsBearer bearer, Ptr<EpcTft> tft)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      uint8_t bearerId = ActivateDedicatedEpsBearer (*i, bearer, tft);
      return bearerId;
    }
  return 0;
}


uint8_t
NrHelper::ActivateDedicatedEpsBearer (Ptr<NetDevice> ueDevice, EpsBearer bearer, Ptr<EpcTft> tft)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG (m_epcHelper != 0, "dedicated EPS bearers cannot be set up when the EPC is not used");

  uint64_t imsi = ueDevice->GetObject<NrUeNetDevice> ()->GetImsi ();
  uint8_t bearerId = m_epcHelper->ActivateEpsBearer (ueDevice, imsi, tft, bearer);
  return bearerId;
}

void
NrHelper::DeActivateDedicatedEpsBearer (Ptr<NetDevice> ueDevice,Ptr<NetDevice> enbDevice, uint8_t bearerId)
{
  NS_LOG_FUNCTION (this << ueDevice << bearerId);
  NS_ASSERT_MSG (m_epcHelper != nullptr, "Dedicated EPS bearers cannot be de-activated when the EPC is not used");
  NS_ASSERT_MSG (bearerId != 1, "Default bearer cannot be de-activated until and unless and UE is released");

  DoDeActivateDedicatedEpsBearer (ueDevice, enbDevice, bearerId);
}

void
NrHelper::SetUeMacAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueMacFactory.Set (n, v);
}

void
NrHelper::SetGnbMacAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbMacFactory.Set (n, v);
}

void
NrHelper::SetGnbSpectrumAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbSpectrumFactory.Set (n, v);
}

void
NrHelper::SetUeSpectrumAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueSpectrumFactory.Set (n, v);
}

void
NrHelper::SetUeChannelAccessManagerAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueChannelAccessManagerFactory.Set (n, v);
}

void
NrHelper::SetGnbChannelAccessManagerAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbChannelAccessManagerFactory.Set (n, v);
}

void
NrHelper::SetSchedulerAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_schedFactory.Set (n, v);
}

void
NrHelper::SetUePhyAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_uePhyFactory.Set (n, v);
}

void
NrHelper::SetGnbPhyAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbPhyFactory.Set (n, v);
}

void
NrHelper::SetUeAntennaAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueAntennaFactory.Set (n, v);
}

void
NrHelper::SetGnbAntennaAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbAntennaFactory.Set (n, v);
}

void
NrHelper::SetUeChannelAccessManagerTypeId (const TypeId &typeId)
{
  NS_LOG_FUNCTION (this);
  m_ueChannelAccessManagerFactory.SetTypeId (typeId);
}

void
NrHelper::SetGnbChannelAccessManagerTypeId (const TypeId &typeId)
{
  NS_LOG_FUNCTION (this);
  m_gnbChannelAccessManagerFactory.SetTypeId (typeId);
}

void
NrHelper::SetSchedulerTypeId (const TypeId &typeId)
{
  NS_LOG_FUNCTION (this);
  m_schedFactory.SetTypeId (typeId);
}

void
NrHelper::SetUeBwpManagerAlgorithmTypeId (const TypeId &typeId)
{

  NS_LOG_FUNCTION (this);
  m_ueBwpManagerAlgoFactory.SetTypeId (typeId);
}

void
NrHelper::SetUeBwpManagerAlgorithmAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_ueBwpManagerAlgoFactory.Set (n, v);
}

void
NrHelper::SetChannelConditionModelAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_channelConditionModelFactory.Set (n, v);
}

void
NrHelper::SetPathlossAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_pathlossModelFactory.Set (n, v);
}

void
NrHelper::SetGnbDlAmcAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbDlAmcFactory.Set (n, v);
}

void
NrHelper::SetGnbUlAmcAttribute (const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbUlAmcFactory.Set (n, v);
}

void
NrHelper::SetUlErrorModel(const std::string &errorModelTypeId)
{
  NS_LOG_FUNCTION (this);

  SetGnbUlAmcAttribute ("ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModelTypeId)));
  SetGnbSpectrumAttribute ("ErrorModelType", TypeIdValue (TypeId::LookupByName(errorModelTypeId)));
}

void
NrHelper::SetDlErrorModel(const std::string &errorModelTypeId)
{
  NS_LOG_FUNCTION (this);

  SetGnbDlAmcAttribute ("ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModelTypeId)));
  SetUeSpectrumAttribute ("ErrorModelType", TypeIdValue (TypeId::LookupByName(errorModelTypeId)));
}

void
NrHelper::SetGnbBwpManagerAlgorithmTypeId (const TypeId &typeId)
{
  NS_LOG_FUNCTION (this);
  m_gnbBwpManagerAlgoFactory.SetTypeId (typeId);
}

void NrHelper::SetGnbBwpManagerAlgorithmAttribute(const std::string &n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this);
  m_gnbBwpManagerAlgoFactory.Set (n, v);
}

void
NrHelper::DoDeActivateDedicatedEpsBearer (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice, uint8_t bearerId)
{
  NS_LOG_FUNCTION (this << ueDevice << bearerId);

  //Extract IMSI and rnti
  uint64_t imsi = ueDevice->GetObject<NrUeNetDevice> ()->GetImsi ();
  uint16_t rnti = ueDevice->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ();


  Ptr<LteEnbRrc> enbRrc = enbDevice->GetObject<NrGnbNetDevice> ()->GetRrc ();

  enbRrc->DoSendReleaseDataRadioBearer (imsi,rnti,bearerId);
}


void
NrHelper::SetEpcHelper (Ptr<EpcHelper> epcHelper)
{
  m_epcHelper = epcHelper;
}

void
NrHelper::SetIdealBeamformingHelper (Ptr<IdealBeamformingHelper> idealBeamformingHelper)
{
  m_idealBeamformingHelper = idealBeamformingHelper;
  m_idealBeamformingHelper->Initialize();
}

class NrDrbActivator : public SimpleRefCount<NrDrbActivator>
{
public:
  NrDrbActivator (Ptr<NetDevice> ueDevice, EpsBearer bearer);
  static void ActivateCallback (Ptr<NrDrbActivator> a, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);
  void ActivateDrb (uint64_t imsi, uint16_t cellId, uint16_t rnti);
private:
  bool m_active;
  Ptr<NetDevice> m_ueDevice;
  EpsBearer m_bearer;
  uint64_t m_imsi;
};

NrDrbActivator::NrDrbActivator (Ptr<NetDevice> ueDevice, EpsBearer bearer)
  : m_active (false),
  m_ueDevice (ueDevice),
  m_bearer (bearer),
  m_imsi (m_ueDevice->GetObject< NrUeNetDevice> ()->GetImsi ())
{
}

void
NrDrbActivator::ActivateCallback (Ptr<NrDrbActivator> a, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (a << context << imsi << cellId << rnti);
  a->ActivateDrb (imsi, cellId, rnti);
}

void
NrDrbActivator::ActivateDrb (uint64_t imsi, uint16_t cellId, uint16_t rnti)
{

  NS_LOG_FUNCTION (this << imsi << cellId << rnti << m_active);
  if ((!m_active) && (imsi == m_imsi))
    {
      Ptr<LteUeRrc> ueRrc = m_ueDevice->GetObject<NrUeNetDevice> ()->GetRrc ();
      NS_ASSERT (ueRrc->GetState () == LteUeRrc::CONNECTED_NORMALLY);
      uint16_t rnti = ueRrc->GetRnti ();
      Ptr<const NrGnbNetDevice> enbLteDevice = m_ueDevice->GetObject<NrUeNetDevice> ()->GetTargetEnb ();
      Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetObject<NrGnbNetDevice> ()->GetRrc ();
      NS_ASSERT (ueRrc->GetCellId () == enbLteDevice->GetCellId ());
      Ptr<UeManager> ueManager = enbRrc->GetUeManager (rnti);
      NS_ASSERT (ueManager->GetState () == UeManager::CONNECTED_NORMALLY
                 || ueManager->GetState () == UeManager::CONNECTION_RECONFIGURATION);
      EpcEnbS1SapUser::DataRadioBearerSetupRequestParameters params;
      params.rnti = rnti;
      params.bearer = m_bearer;
      params.bearerId = 0;
      params.gtpTeid = 0;   // don't care
      enbRrc->GetS1SapUser ()->DataRadioBearerSetupRequest (params);
      m_active = true;
    }
}
void
NrHelper::ActivateDataRadioBearer (NetDeviceContainer ueDevices, EpsBearer bearer)
{
  NS_LOG_FUNCTION (this);
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); ++i)
    {
      ActivateDataRadioBearer (*i, bearer);
    }
}
void
NrHelper::ActivateDataRadioBearer (Ptr<NetDevice> ueDevice, EpsBearer bearer)
{
  NS_LOG_FUNCTION (this << ueDevice);
  NS_ASSERT_MSG (m_epcHelper == 0, "this method must not be used when the EPC is being used");

  // Normally it is the EPC that takes care of activating DRBs
  // when the UE gets connected. When the EPC is not used, we achieve
  // the same behavior by hooking a dedicated DRB activation function
  // to the Enb RRC Connection Established trace source


  Ptr<const NrGnbNetDevice> enbnrDevice = ueDevice->GetObject<NrUeNetDevice> ()->GetTargetEnb ();

  std::ostringstream path;
  path << "/NodeList/" << enbnrDevice->GetNode ()->GetId ()
       << "/DeviceList/" << enbnrDevice->GetIfIndex ()
       << "/LteEnbRrc/ConnectionEstablished";
  Ptr<NrDrbActivator> arg = Create<NrDrbActivator> (ueDevice, bearer);
  Config::Connect (path.str (), MakeBoundCallback (&NrDrbActivator::ActivateCallback, arg));
}


void
NrHelper::EnableTraces (void)
{
  EnableDlPhyTrace ();
  EnableUlPhyTrace ();
  //EnableEnbPacketCountTrace ();
  //EnableUePacketCountTrace ();
  EnableTransportBlockTrace ();
  EnableRlcTraces ();
  //EnablePdcpTraces ();
  //EnableGnbPhyCtrlMsgsTraces ();
  //EnableUePhyCtrlMsgsTraces ();
  //EnableGnbMacCtrlMsgsTraces ();
  //EnableUeMacCtrlMsgsTraces ();
  EnableMacThroughputTraces ();
  EnableBeamSweepTrace ();
  
}

void
NrHelper::EnableBeamSweepTrace (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/BeamSweepTrace",
                   MakeBoundCallback (&NrPhyRxTrace::BeamSweepTraceCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/BeamSweepTrace",
                   MakeBoundCallback (&NrPhyRxTrace::BeamSweepTraceCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/BeamSweepInitialization",
                   MakeBoundCallback (&NrPhyRxTrace::BeamSweepTraceCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/BeamSweepTrace",
                   MakeBoundCallback (&NrPhyRxTrace::BeamSweepTraceCallback, m_phyStats));
}

void
NrHelper::EnableDlPhyTrace (void)
{
  //NS_LOG_FUNCTION_NOARGS ();
  //Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportCurrentCellRsrpSinr",
  //                 MakeBoundCallback (&NrPhyRxTrace::ReportCurrentCellRsrpSinrCallback, m_phyStats));

  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/SpectrumPhy/RxPacketTraceUe",
                   MakeBoundCallback (&NrPhyRxTrace::RxPacketTraceUeCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/SpectrumPhy/RxPacketTraceUe",
                   MakeBoundCallback (&NrPhyRxTrace::RxNewTraceUeCallback, m_phyStats));
}

void
NrHelper::EnableGnbPhyCtrlMsgsTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/GnbPhyRxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrPhyRxTrace::RxedGnbPhyCtrlMsgsCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/GnbPhyTxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrPhyRxTrace::TxedGnbPhyCtrlMsgsCallback, m_phyStats));
}

void
NrHelper::EnableGnbMacCtrlMsgsTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/GnbMacRxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrMacRxTrace::RxedGnbMacCtrlMsgsCallback, m_macStats));

  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/GnbMacTxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrMacRxTrace::TxedGnbMacCtrlMsgsCallback, m_macStats));
}

void
NrHelper::EnableUePhyCtrlMsgsTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyRxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrPhyRxTrace::RxedUePhyCtrlMsgsCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyTxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrPhyRxTrace::TxedUePhyCtrlMsgsCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyRxedDlDciTrace",
                   MakeBoundCallback (&NrPhyRxTrace::RxedUePhyDlDciCallback, m_phyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyTxedHarqFeedbackTrace",
                   MakeBoundCallback (&NrPhyRxTrace::TxedUePhyHarqFeedbackCallback, m_phyStats));
}

void
NrHelper::EnableUeMacCtrlMsgsTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacRxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrMacRxTrace::RxedUeMacCtrlMsgsCallback, m_macStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacTxedCtrlMsgsTrace",
                   MakeBoundCallback (&NrMacRxTrace::TxedUeMacCtrlMsgsCallback, m_macStats));
}

void
NrHelper::EnableUlPhyTrace (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/SpectrumPhy/RxPacketTraceEnb",
                   MakeBoundCallback (&NrPhyRxTrace::RxPacketTraceEnbCallback, m_phyStats));
}

void
NrHelper::EnableEnbPacketCountTrace ()
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/SpectrumPhy/ReportEnbTxRxPacketCount",
                   MakeBoundCallback (&NrPhyRxTrace::ReportPacketCountEnbCallback, m_phyStats));

}

void
NrHelper::EnableUePacketCountTrace ()
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/SpectrumPhy/ReportUeTxRxPacketCount",
                   MakeBoundCallback (&NrPhyRxTrace::ReportPacketCountUeCallback, m_phyStats));

}

void
NrHelper::EnableTransportBlockTrace ()
{
  NS_LOG_FUNCTION_NOARGS ();
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportDownlinkTbSize",
                   MakeBoundCallback (&NrPhyRxTrace::ReportDownLinkTBSize, m_phyStats));
}


void
NrHelper::EnableRlcTraces (void)
{
  NS_ASSERT_MSG (m_rlcStats == 0, "please make sure that NrHelper::EnableRlcTraces is called at most once");
  m_rlcStats = CreateObject<NrBearerStatsCalculator> ("RLC");
  m_radioBearerStatsConnector.EnableRlcStats (m_rlcStats);
}

Ptr<NrBearerStatsCalculator>
NrHelper::GetRlcStats (void)
{
  return m_rlcStats;
}

void
NrHelper::EnablePdcpTraces (void)
{
  NS_ASSERT_MSG (m_pdcpStats == 0, "please make sure that NrHelper::EnablePdcpTraces is called at most once");
  m_pdcpStats = CreateObject<NrBearerStatsCalculator> ("PDCP");
  m_radioBearerStatsConnector.EnablePdcpStats (m_pdcpStats);
}

Ptr<NrBearerStatsCalculator>
NrHelper::GetPdcpStats (void)
{
  return m_pdcpStats;
}

// ADDED DURING MERGING
std::string
NrHelper::GetLteHandoverAlgorithmType () const
{
  return m_lteHandoverAlgorithmFactory.GetTypeId ().GetName ();
}

void
NrHelper::SetLteHandoverAlgorithmType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_lteHandoverAlgorithmFactory = ObjectFactory ();
  m_lteHandoverAlgorithmFactory.SetTypeId (type);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//substituted m_componentCarrierPhyParams with m_bwpConfiguration
void
NrHelper::RayTraceModelLoadData (OperationBandInfo *band, uint16_t ueWalkId)
{
  //NS_LOG_UNCOND("Loading RayTraceModelLoadData. m_bwpConfiguration size= " << m_bwpConfiguration.size ());
  /*for (std::unordered_map<uint32_t, BandwidthPartInfo>::iterator it = m_bwpConfiguration.begin (); it != m_bwpConfiguration.end (); ++it)
  {
    NS_LOG_UNCOND ("raytrace data for i=" << it->first);
    m_raytracing [it->first]->LoadAllTraceData (ueWalkId);
  }*/

  /*if (band->m_cc.size () == 1)
  {
    auto & compcarrier = band->m_cc.at(0);

    for (const auto & bwp : compcarrier->m_bwp)
    {
      m_raytracing [bwp->m_bwpId]->LoadAllTraceData (ueWalkId);
    }
  } */ 

  for (const auto & cc : band ->m_cc)
  {
    for (const auto & bwp : cc->m_bwp)
    {
      m_raytracing [cc->m_ccId][bwp->m_bwpId]->LoadAllTraceData (ueWalkId);
    }
  }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void
NrHelper::AddBandwidthPart (uint32_t id, const BandwidthPartInfoPtr &bwpRepr)
{
  NS_LOG_UNCOND (this);
  auto it = m_bwpConfiguration.find (id);
  if (it != m_bwpConfiguration.end ())
  {
    NS_FATAL_ERROR ("Bad BWP configuration: You already configured bwp id " << id);
  }
  
  NS_ASSERT (id == bwpRepr->m_bwpId);
  m_bwpConfiguration.emplace (std::make_pair (id, *bwpRepr));
}

// ADDED DURING MERGING

void
NrHelper::EnableMacThroughputTraces (void)
{
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacRxedPacketTrace",
                   MakeBoundCallback (&NrPhyRxTrace::MacRxUe, m_phyStats));
}

void
NrHelper::HandoverRequest (Time hoTime, Ptr<NetDevice> ueDev, Ptr<NetDevice> sourceEnbDev, Ptr<NetDevice> targetEnbDev)
{
  NS_LOG_FUNCTION (this << ueDev << sourceEnbDev << targetEnbDev);
  NS_ASSERT_MSG (m_epcHelper, "Handover requires the use of the EPC - did you forget to call LteHelper::SetEpcHelper () ?");
  uint16_t targetCellId = targetEnbDev->GetObject<NrGnbNetDevice> ()->GetCellId ();
  Simulator::Schedule (hoTime, &NrHelper::DoHandoverRequest, this, ueDev, sourceEnbDev, targetCellId);
}

void
NrHelper::DoHandoverRequest (Ptr<NetDevice> ueDev, Ptr<NetDevice> sourceEnbDev, uint16_t targetCellId)
{
  NS_LOG_FUNCTION (this << ueDev << sourceEnbDev << targetCellId);

  Ptr<LteEnbRrc> sourceRrc = sourceEnbDev->GetObject<NrGnbNetDevice> ()->GetRrc ();
  uint16_t rnti = ueDev->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ();
  sourceRrc->SendHandoverRequest (rnti, targetCellId);
}

void 
NrHelper::SetHandoverAlgorithmType (std::string type)
{
  NS_LOG_FUNCTION (this << type);
  m_handoverAlgorithmFactory = ObjectFactory ();
  m_handoverAlgorithmFactory.SetTypeId (type);
}

void
NrHelper::SetHandoverAlgorithmAttribute (std::string n, const AttributeValue &v)
{
  NS_LOG_FUNCTION (this << n);
  m_handoverAlgorithmFactory.Set (n, v);
}

} // namespace ns3

