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
 */

#define NS_LOG_APPEND_CONTEXT                                            \
  do                                                                     \
    {                                                                    \
      std::clog << " [ CellId " << GetCellId() << ", bwpId "             \
                << GetBwpId () << "] ";                                  \
    }                                                                    \
  while (false);

#include <ns3/log.h>
#include <ns3/lte-radio-bearer-tag.h>
#include <ns3/node.h>
#include <algorithm>
#include <functional>
#include <string>
#include <unordered_set>

#include "nr-gnb-phy.h"
#include "nr-ue-phy.h"
#include "nr-net-device.h"
#include "nr-ue-net-device.h"
#include "nr-gnb-net-device.h"
#include "nr-radio-bearer-tag.h"
#include "nr-ch-access-manager.h"

#include <ns3/node-list.h>
#include <ns3/node.h>
#include <ns3/pointer.h>
#include <ns3/double.h>
#include <ns3/boolean.h>
#include "beam-manager.h"

// Headers added during merging
#include <ns3/nr-spectrum-value-helper.h>
#include <ns3/three-gpp-channel-model.h>
#include <ns3/three-gpp-spectrum-propagation-loss-model.h>

std::vector<bool> performBF(11);

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrGnbPhy");

NS_OBJECT_ENSURE_REGISTERED (NrGnbPhy);

NrGnbPhy::NrGnbPhy ():
    m_n0Delay (0),
    m_n1Delay (4)
{
  NS_LOG_FUNCTION (this);
  m_enbCphySapProvider = new MemberLteEnbCphySapProvider<NrGnbPhy> (this);
  IAisPerformed = true;
/*
  if (m_beamformingPeriodicity != MilliSeconds (0))
  {
    // UNCOMMENT IF UNNECESSARY
    //m_beamformingTimer = Simulator::Schedule (m_beamformingPeriodicity, m_phyIdealBeamformingHelper->ExpireBeamformingTimer, this);
  }

  Simulator::Schedule (MilliSeconds (100), &NrGnbPhy::UpdateUeSinrEstimate, this);
  performBF[GetCellId ()] = true;

  NS_LOG_UNCOND("NrGnbPhy initialized");*/
}

NrGnbPhy::~NrGnbPhy ()
{
}


void
NrGnbPhy::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete m_enbCphySapProvider;
  NrPhy::DoDispose ();
}

TypeId
NrGnbPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrGnbPhy")
    .SetParent<NrPhy> ()
    .AddConstructor<NrGnbPhy> ()
    .AddAttribute ("RbOverhead",
                   "Overhead when calculating the usable RB number",
                   DoubleValue (0.04),
                   MakeDoubleAccessor (&NrGnbPhy::SetRbOverhead,
                                       &NrGnbPhy::GetRbOverhead),
                   MakeDoubleChecker <double> (0, 0.5))
    .AddAttribute ("TxPower",
                   "Transmission power in dBm",
                   DoubleValue (30.0),
                   MakeDoubleAccessor (&NrGnbPhy::SetTxPower,
                                       &NrGnbPhy::GetTxPower),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("NoiseFigure",
                   "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                   " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                   "\"the difference in decibels (dB) between"
                   " the noise output of the actual receiver to the noise output of an "
                   " ideal receiver with the same overall gain and bandwidth when the receivers "
                   " are connected to sources at the standard noise temperature T0.\" "
                   "In this model, we consider T0 = 290K.",
                   DoubleValue (5.0),
                   MakeDoubleAccessor (&NrPhy::SetNoiseFigure,
                                       &NrPhy::GetNoiseFigure),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SpectrumPhy",
                   "The downlink NrSpectrumPhy associated to this NrPhy",
                   TypeId::ATTR_GET,
                   PointerValue (),
                   MakePointerAccessor (&NrPhy::GetSpectrumPhy),
                   MakePointerChecker <NrSpectrumPhy> ())
    .AddTraceSource ("UlSinrTrace",
                     "UL SINR statistics.",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_ulSinrTrace),
                     "ns3::UlSinr::TracedCallback")
    .AddTraceSource ("GnbPhyRxedCtrlMsgsTrace",
                     "Enb PHY Rxed Control Messages Traces.",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_phyRxedCtrlMsgsTrace),
                     "ns3::NrPhyRxTrace::RxedGnbPhyCtrlMsgsTracedCallback")
    .AddTraceSource ("GnbPhyTxedCtrlMsgsTrace",
                     "Enb PHY Txed Control Messages Traces.",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_phyTxedCtrlMsgsTrace),
                     "ns3::NrPhyRxTrace::TxedGnbPhyCtrlMsgsTracedCallback")
    .AddAttribute ("N0Delay",
                   "Minimum processing delay needed to decode DL DCI and decode DL data",
                    UintegerValue (0),
                    MakeUintegerAccessor (&NrGnbPhy::SetN0Delay,
                                          &NrGnbPhy::GetN0Delay),
                    MakeUintegerChecker<uint32_t> (0, 1))
    .AddAttribute ("N1Delay",
                   "Minimum processing delay (UE side) from the end of DL Data reception to "
                   "the earliest possible start of the corresponding ACK/NACK transmission",
                    UintegerValue (2),
                    MakeUintegerAccessor (&NrGnbPhy::SetN1Delay,
                                          &NrGnbPhy::GetN1Delay),
                    MakeUintegerChecker<uint32_t> (0, 4))
    .AddAttribute ("N2Delay",
                   "Minimum processing delay needed to decode UL DCI and prepare UL data",
                   UintegerValue (2),
                   MakeUintegerAccessor (&NrGnbPhy::SetN2Delay,
                                         &NrGnbPhy::GetN2Delay),
                   MakeUintegerChecker<uint32_t> (0, 4))
    .AddAttribute ("TbDecodeLatency",
                   "Transport block decode latency",
                    TimeValue (MicroSeconds (100)),
                    MakeTimeAccessor (&NrPhy::SetTbDecodeLatency,
                                      &NrPhy::GetTbDecodeLatency),
                    MakeTimeChecker ())
    .AddAttribute ("Numerology",
                   "The 3GPP numerology to be used",
                   UintegerValue (0),
                   MakeUintegerAccessor (&NrPhy::SetNumerology,
                                         &NrPhy::GetNumerology),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("SymbolsPerSlot",
                   "Number of symbols in one slot",
                   UintegerValue (14),
                   MakeUintegerAccessor (&NrPhy::SetSymbolsPerSlot,
                                         &NrPhy::GetSymbolsPerSlot),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("Pattern",
                   "The slot pattern",
                   StringValue ("F|F|F|F|F|F|F|F|F|F|"),
                   MakeStringAccessor (&NrGnbPhy::SetPattern,
                                       &NrGnbPhy::GetPattern),
                   MakeStringChecker ())
    .AddTraceSource ("SlotDataStats",
                     "Data statistics for the current slot: SfnSf, active UE, used RE, "
                     "used symbols, available RBs, available symbols, bwp ID, cell ID",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_phySlotDataStats),
                     "ns3::NrGnbPhy::SlotStatsTracedCallback")
    .AddTraceSource ("SlotCtrlStats",
                     "Ctrl statistics for the current slot: SfnSf, active UE, used RE, "
                     "used symbols, available RBs, available symbols, bwp ID, cell ID",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_phySlotCtrlStats),
                     "ns3::NrGnbPhy::SlotStatsTracedCallback")
    .AddTraceSource ("RBDataStats",
                     "Resource Block used for data: SfnSf, symbol, RB PHY map, bwp ID, cell ID",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_rbStatistics),
                     "ns3::NrGnbPhy::RBStatsTracedCallback")
    .AddTraceSource ("BeamSweepTrace",
                     "trace fired when a beam refinement has been initiated from GNB PHY",
                     MakeTraceSourceAccessor (&NrGnbPhy::m_beamSweepTrace),
                     "ns3::BeamSweepTraceParams::TracedCallback")
    .AddAttribute ("BeamformingPeriodicity",
                   "Interval between beamforming phases",
                   TimeValue (MilliSeconds (100)),
                   MakeTimeAccessor (&NrGnbPhy::m_beamformingPeriodicity),
                   MakeTimeChecker())
    .AddAttribute ("UpdateSinrEstimatePeriod",
                   "Period (in microseconds) of update of SINR estimate of all the UE",
                   DoubleValue (1600),     //TODO considering refactoring in MmWavePhyMacCommon
                   MakeDoubleAccessor (&NrGnbPhy::m_updateSinrPeriod),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UpdateUeSinrEstimatePeriod",
                   "Period (in ms) of reporting of SINR estimate of all the UE",
                   DoubleValue (25.6),
                   MakeDoubleAccessor (&NrGnbPhy::m_ueUpdateSinrPeriod),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ApplyBeamformingDelay",
                   "If true, triggers delays on adaptive BF, else ideak timeless BF",
                    BooleanValue (true),
                    MakeBooleanAccessor (&NrGnbPhy::m_BFdelay),
                    MakeBooleanChecker ())     
    .AddAttribute ("OmniNrFallback",
                   "If true, omni-directional mmWave data transmission while BF, else the scheduler blocks all communication",
                   BooleanValue(true),
                   MakeBooleanAccessor (&NrGnbPhy::m_omniFallback),
                   MakeBooleanChecker ())
    .AddAttribute ("BeamTrainingDelay",
                   "Delay in ms to simulate beam training",
                   DoubleValue (5),
                   MakeDoubleAccessor (&NrGnbPhy::m_BFtrainingDelay),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("IADelay",
                   "Delay in ms to simulate IA",
                   DoubleValue (5250),
                   MakeDoubleAccessor (&NrGnbPhy::m_IAdelay),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("GnbElevationAngleStep",
                   "Angle step that will be used for sweep at elevation",
                   DoubleValue (20),
                   MakeDoubleAccessor (&NrGnbPhy::SetGnbVerticalAngleStep),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("GnbHorizontalAngleStep",
                   "Angle step that will be used for sweeo at azimuth",
                   DoubleValue (9),
                   MakeDoubleAccessor (&NrGnbPhy::SetGnbHorizontalAngleStep),
                   MakeDoubleChecker<double> (1.0, 90.0))
    ;
  return tid;

}

void
NrGnbPhy::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  //Some stuff might be missing from the previous version, that is because
  //only the operations with dependencies are included here

  //if (m_beamformingPeriodicity != MilliSeconds (0))
  //{
    // UNCOMMENT IF UNNECESSARY
    //m_beamformingTimer = Simulator::Schedule (m_beamformingPeriodicity, m_phyIdealBeamformingHelper->ExpireBeamformingTimer, this);
  //}

  Simulator::Schedule (MilliSeconds (100), &NrGnbPhy::UpdateUeSinrEstimate, this);
  performBF[GetCellId ()] = true;
  NrPhy::DoInitialize ();
}

uint32_t
NrGnbPhy::GetNumRbPerRbg () const
{
  return m_phySapUser->GetNumRbPerRbg();
}

uint32_t
NrGnbPhy::GetChannelBandwidth () const
{
  // m_channelBandwidth is in kHz * 100
  return m_channelBandwidth * 1000 * 100;
}

const SfnSf &
NrGnbPhy::GetCurrentSfnSf () const
{
  return m_currentSlot;
}

/**
 * \brief An intelligent way to calculate the modulo
 * \param n Number
 * \param m Modulo
 * \return n+=m until n < 0
 */
static uint32_t modulo (int n, uint32_t m)
{
  if (n >= 0)
    {
      return static_cast<uint32_t> (n) % m;
    }
  else
    {
      while (n < 0) {
          n += m;
        }
      return static_cast<uint32_t> (n);
    }
}

/**
 * \brief Return the slot in which the DL HARQ Feedback should be sent, according to the parameter N1
 * \param pattern The TDD pattern
 * \param pos The position of the data inside the pattern for which we want to find where the feedback should be sent
 * \param n1 The N1 parameter
 * \return k1 (after how many slots the DL HARQ Feedback should be sent)
 *
 * Please note that for the LTE TDD case, although the calculation follows the
 * logic of Table 10.1-1 of TS 36.213, some configurations are simplified in order
 * to avoid having a table from where we take the K1 values. In particular, for
 * configurations 3, 4 and 6 (starting form 0), the specification splits the
 * HARQ feedbacks among all UL subframes in an equal (as much as possible) manner.
 * This tactic is ommitted in this implementation.
 */
static int32_t
ReturnHarqSlot (const std::vector<LteNrTddSlotType> &pattern, uint32_t pos, uint32_t n1)
{
  int32_t k1 = static_cast<int32_t> (n1);

  uint32_t index = modulo (static_cast<int> (pos) + k1, static_cast<uint32_t> (pattern.size ()));

  while (pattern[index] < LteNrTddSlotType::F)
    {
      k1++;
      index = modulo (static_cast<int> (pos) + k1, static_cast<uint32_t> (pattern.size ()));
      NS_ASSERT (index < pattern.size ());
    }

  return k1;
}

struct DciKPair
{
  uint32_t indexDci {0};
  uint32_t k {0};
};

/**
 * \brief Return the slot in which the DCI should be send, according to the parameter n,
 * along with the number of slots required to add to the current slot to get the slot of DCI (k0/k2)
 * \param pattern The TDD pattern
 * \param pos The position inside the pattern for which we want to check where the DCI should be sent
 * \param n The N parameter (equal to N0 or N2, depending if it is DL or UL)
 * \return The slot position in which the DCI for the position specified should be sent and the k0/k2
 */
static DciKPair
ReturnDciSlot (const std::vector<LteNrTddSlotType> &pattern, uint32_t pos, uint32_t n)
{
  DciKPair ret;
  ret.k = n;
  ret.indexDci = modulo (static_cast<int> (pos) - static_cast<int> (ret.k),
                           static_cast<uint32_t> (pattern.size ()));

  while (pattern[ret.indexDci] > LteNrTddSlotType::F)
    {
      ret.k++;
      ret.indexDci = modulo (static_cast<int> (pos) - static_cast<int> (ret.k),
                      static_cast<uint32_t> (pattern.size ()));
      NS_ASSERT (ret.indexDci < pattern.size ());
    }

  return ret;
}

/**
 * \brief Generates the map tosendDl/Ul that holds the information of the DCI Slot and the
 * corresponding k0/k2 value, and the generateDl/Ul that includes the L1L2CtrlLatency.
 * \param pattern The TDD pattern
 * \param pattern The pattern to analyze
 * \param toSend The structure toSendDl/tosendUl to fill
 * \param generate The structure generateDl/generateUl to fill
 * \param pos The position inside the pattern for which we want to check where the DCI should be sent
 * \param n The N parameter (equal to N0 or N2, depending if it is DL or UL)
 * \param l1l2CtrlLatency L1L2CtrlLatency of the system
 */
static void GenerateDciMaps (const std::vector<LteNrTddSlotType> &pattern,
                             std::map<uint32_t, std::vector<uint32_t>> *toSend,
                             std::map<uint32_t, std::vector<uint32_t>> *generate,
                             uint32_t pos, uint32_t n, uint32_t l1l2CtrlLatency)
{
  auto dciSlot = ReturnDciSlot (pattern, pos, n);
  uint32_t indexGen = modulo (static_cast<int>(dciSlot.indexDci) - static_cast<int> (l1l2CtrlLatency),
                              static_cast<uint32_t> (pattern.size ()));
  uint32_t kWithCtrlLatency = static_cast<uint32_t> (dciSlot.k) + l1l2CtrlLatency;

  (*toSend)[dciSlot.indexDci].push_back(static_cast<uint32_t> (dciSlot.k));
  (*generate)[indexGen].push_back (kWithCtrlLatency);
}

void
NrGnbPhy::GenerateStructuresFromPattern (const std::vector<LteNrTddSlotType> &pattern,
                                             std::map<uint32_t, std::vector<uint32_t>> *toSendDl,
                                             std::map<uint32_t, std::vector<uint32_t>> *toSendUl,
                                             std::map<uint32_t, std::vector<uint32_t>> *generateDl,
                                             std::map<uint32_t, std::vector<uint32_t>> *generateUl,
                                             std::map<uint32_t, uint32_t> *dlHarqfbPosition,
                                             uint32_t n0, uint32_t n2, uint32_t n1, uint32_t l1l2CtrlLatency)
{
  const uint32_t n = static_cast<uint32_t> (pattern.size ());

  // Create a pattern that is all F.
  std::vector<LteNrTddSlotType> fddGenerationPattern;
  fddGenerationPattern.resize (pattern.size (), LteNrTddSlotType::F);

  /* if we have to generate structs for a TDD pattern, then use the input pattern.
   * Otherwise, pass to the gen functions a pattern which is all F (therefore, the
   * the function will think that they will be able to transmit or
   * receive things following n0, n1, n2, that is what happen in FDD, just in
   * another band..
   */

  const std::vector<LteNrTddSlotType> *generationPattern;

  if (IsTdd (pattern))
    {
      generationPattern = &pattern;
    }
  else
    {
      generationPattern = &fddGenerationPattern;
    }

  for (uint32_t i = 0; i < n; i++)
    {
      if ((*generationPattern)[i] == LteNrTddSlotType::UL)
        {
          GenerateDciMaps (*generationPattern, toSendUl, generateUl, i, n2, l1l2CtrlLatency);
        }
      else if ((*generationPattern)[i] == LteNrTddSlotType::DL || pattern[i] == LteNrTddSlotType::S)
        {
          GenerateDciMaps (*generationPattern, toSendDl, generateDl, i, n0, l1l2CtrlLatency);

          int32_t k1 = ReturnHarqSlot (*generationPattern, i, n1);
          (*dlHarqfbPosition).insert (std::make_pair (i, k1));
        }
      else if ((*generationPattern)[i] == LteNrTddSlotType::F)
        {
          GenerateDciMaps (*generationPattern, toSendDl, generateDl, i, n0, l1l2CtrlLatency);
          GenerateDciMaps (*generationPattern, toSendUl, generateUl, i, n2, l1l2CtrlLatency);

          int32_t k1 = ReturnHarqSlot (*generationPattern, i, n1);
          (*dlHarqfbPosition).insert (std::make_pair (i, k1));
        }
    }

  /*
   * Now, if the input pattern is for FDD, remove the elements in the
   * opposite generate* structures: in the end, we don't want to generate DL
   * for a FDD-UL band, right?
   *
   * But.. maintain the toSend structures, as they will be used to send
   * feedback or other messages, like DCI.
   */

  if (! IsTdd (pattern))
    {
      if (HasUlSlot (pattern))
        {
          generateDl->clear ();
        }
      else
        {
          generateUl->clear();
        }
    }

  for (auto & list : (*generateUl))
    {
      std::sort (list.second.begin (), list.second.end ());
    }

  for (auto & list : (*generateDl))
    {
      std::sort (list.second.begin (), list.second.end ());
    }
}

void
NrGnbPhy::PushDlAllocation (const SfnSf &sfnSf) const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_phySapUser);

  auto dci = m_phySapUser->GetDlCtrlDci ();
  VarTtiAllocInfo dlCtrlVarTti (dci);

  SlotAllocInfo slotAllocInfo = SlotAllocInfo (sfnSf);

  slotAllocInfo.m_numSymAlloc = dlCtrlVarTti.m_dci->m_numSym;
  slotAllocInfo.m_type = SlotAllocInfo::DL;
  slotAllocInfo.m_varTtiAllocInfo.emplace_back (dlCtrlVarTti);

  m_phySapProvider->SetSlotAllocInfo (slotAllocInfo);
}

void
NrGnbPhy::PushUlAllocation (const SfnSf &sfnSf) const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_phySapUser);

  auto dci = m_phySapUser->GetUlCtrlDci ();
  VarTtiAllocInfo ulCtrlVarTti (dci);

  SlotAllocInfo slotAllocInfo = SlotAllocInfo (sfnSf);

  slotAllocInfo.m_numSymAlloc = ulCtrlVarTti.m_dci->m_numSym;
  slotAllocInfo.m_type = SlotAllocInfo::UL;
  slotAllocInfo.m_varTtiAllocInfo.emplace_back (ulCtrlVarTti);

  m_phySapProvider->SetSlotAllocInfo (slotAllocInfo);
}

void
NrGnbPhy::SetTddPattern (const std::vector<LteNrTddSlotType> &pattern)
{
  NS_LOG_FUNCTION (this);

  std::stringstream ss;

  for (const auto & v : pattern)
    {
      ss << v << "|";
    }
  NS_LOG_INFO ("Set pattern : " << ss.str ());

  m_tddPattern = pattern;

  m_generateDl.clear ();
  m_generateUl.clear ();
  m_toSendDl.clear ();
  m_toSendUl.clear ();
  m_dlHarqfbPosition.clear ();

  GenerateStructuresFromPattern (pattern, &m_toSendDl, &m_toSendUl,
                                 &m_generateDl, &m_generateUl,
                                 &m_dlHarqfbPosition, 0,
                                 GetN2Delay (), GetN1Delay (),
                                 GetL1L2CtrlLatency ());
}

void
NrGnbPhy::ScheduleStartEventLoop (uint32_t nodeId, uint16_t frame, uint8_t subframe, uint16_t slot)
{
  NS_LOG_FUNCTION (this);
  Simulator::ScheduleWithContext (nodeId, MilliSeconds (0),
                                  &NrGnbPhy::StartEventLoop, this, frame, subframe, slot);
}

void
NrGnbPhy::StartEventLoop (uint16_t frame, uint8_t subframe, uint16_t slot)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("PHY starting. Configuration: "  << std::endl <<
                "\t TxPower: " << m_txPower << " dB" << std::endl <<
                "\t NoiseFigure: " << m_noiseFigure << std::endl <<
                "\t N0: " << m_n0Delay << std::endl <<
                "\t N1: " << m_n1Delay << std::endl <<
                "\t N2: " << m_n2Delay << std::endl <<
                "\t TbDecodeLatency: " << GetTbDecodeLatency ().GetMicroSeconds () << " us " << std::endl <<
                "\t Numerology: " << GetNumerology () << std::endl <<
                "\t SymbolsPerSlot: " << GetSymbolsPerSlot () << std::endl <<
                "\t Pattern: " << GetPattern () << std::endl <<
                "Attached to physical channel: " << std::endl <<
                "\t Channel bandwidth: " << GetChannelBandwidth () << " Hz" << std::endl <<
                "\t Channel central freq: " << GetCentralFrequency() << " Hz" << std::endl <<
                "\t Num. RB: " << GetRbNum ());
  SfnSf startSlot (frame, subframe, slot, GetNumerology ());
  InitializeMessageList ();
  StartSlot (startSlot);
}

void
NrGnbPhy::SetEnbCphySapUser (LteEnbCphySapUser* s)
{
  NS_LOG_FUNCTION (this);
  m_enbCphySapUser = s;
}

LteEnbCphySapProvider*
NrGnbPhy::GetEnbCphySapProvider ()
{
  NS_LOG_FUNCTION (this);
  return m_enbCphySapProvider;
}

uint32_t
NrGnbPhy::GetN0Delay (void) const
{
  return m_n0Delay;
}

uint32_t
NrGnbPhy::GetN1Delay (void) const
{
  return m_n1Delay;
}

uint32_t
NrGnbPhy::GetN2Delay () const
{
  return m_n2Delay;
}

void
NrGnbPhy::SetN0Delay (uint32_t delay)
{
  m_n0Delay = delay;
  SetTddPattern (m_tddPattern); // Update the generate/send structures
}

void
NrGnbPhy::SetN1Delay (uint32_t delay)
{
  m_n1Delay = delay;
  SetTddPattern (m_tddPattern); // Update the generate/send structures
}

void
NrGnbPhy::SetN2Delay (uint32_t delay)
{
  m_n2Delay = delay;
  SetTddPattern (m_tddPattern); // Update the generate/send structures
}

BeamId 
NrGnbPhy::GetBeamId (uint16_t rnti) const
{
  for (uint8_t i = 0; i < m_deviceMap.size (); i++)
    {
      Ptr<NrUeNetDevice> ueDev = DynamicCast < NrUeNetDevice > (m_deviceMap.at (i));
      uint64_t ueRnti = (DynamicCast<NrUePhy>(ueDev->GetPhy (0)))->GetRnti ();

      if (ueRnti == rnti)
        {
          return m_beamManager->GetBeamId (m_deviceMap.at(i));
        }
    }
  return BeamId (0,0.0);
}

void
NrGnbPhy::SetCam (const Ptr<NrChAccessManager> &cam)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (cam != nullptr);
  m_cam = cam;
  m_cam->SetAccessGrantedCallback (std::bind (&NrGnbPhy::ChannelAccessGranted, this,
                                              std::placeholders::_1));
  m_cam->SetAccessDeniedCallback (std::bind (&NrGnbPhy::ChannelAccessLost, this));
}

Ptr<NrChAccessManager>
NrGnbPhy::GetCam() const
{
  NS_LOG_FUNCTION (this);
  return m_cam;
}

void
NrGnbPhy::SetTxPower (double pow)
{
  m_txPower = pow;
}
double
NrGnbPhy::GetTxPower () const
{
  return m_txPower;
}

void
NrGnbPhy::SetSubChannels (const std::vector<int> &rbIndexVector)
{
  m_listOfSubchannels = rbIndexVector;
  Ptr<SpectrumValue> txPsd = GetTxPowerSpectralDensity (rbIndexVector);
  NS_ASSERT (txPsd);
  m_spectrumPhy->SetTxPowerSpectralDensity (txPsd);
}

// Added during merging
void 
NrGnbPhy::UpdateUeSinrEstimate ()
{
  if (!IAisPerformed || m_omniFallback)
  {
    m_sinrMap.clear ();
    m_rxPsdMap.clear ();

    Ptr<SpectrumValue> noisePsd = NrSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_noiseFigure, GetSpectrumModel ());
    Ptr<SpectrumValue> totalReceivedPsd = Create <SpectrumValue> (SpectrumValue (noisePsd->GetSpectrumModel ()));

    for (std::map<uint64_t, Ptr<NetDevice> >::iterator ue = m_ueAttachedImsiMap.begin (); ue != m_ueAttachedImsiMap.end (); ++ue)
    {
      // distinguish between MC and NrNetDevice
      Ptr<NrUeNetDevice> ueNetDevice = DynamicCast<NrUeNetDevice> (ue->second);
      Ptr<NrUePhy> uePhy;

      m_rxPsdMap[ue->first] = CalculateRxPsdToUe (ueNetDevice, true);
      *totalReceivedPsd += *m_rxPsdMap[ue->first];
      BeamId beam = m_beamManager->GetBeamId (ueNetDevice);
    }
      
    for (std::map<uint64_t, Ptr<SpectrumValue> >::iterator ue = m_rxPsdMap.begin (); ue != m_rxPsdMap.end (); ++ue)
    {
      SpectrumValue interference = *totalReceivedPsd - *(ue->second);
      NS_LOG_LOGIC ("interference " << interference);
      SpectrumValue sinr = *(ue->second) /(*noisePsd);      // *interference
      // we consider the SNR only!
      NS_LOG_LOGIC ("sinr " << sinr);

      double sinrAvg = Sum (sinr) / (sinr.GetSpectrumModel ()->GetNumBands ());

      NS_LOG_DEBUG ("Time " << Simulator::Now ().GetSeconds () << " CellId " << GetCellId () << " UE " << ue->first << "Average SINR " << 10 * std::log10 (sinrAvg));
      m_sinrMap[ue->first] = sinrAvg;

      Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(m_ueAttachedImsiMap.find(ue->first)->second);
      //Check if this is the eNB the Ue is connected to
      /*if ((m_netDevice == ueDev->GetTargetEnb () && m_adaptiveBF))
      {
        long double currentSinrDb = 10 * std::log10(m_sinrMap[ue->first]);
        long double prevSinrDb = 10 * std::log10(m_prevSinrMap[ue->first]);
        long double sinrDiff = prevSinrDb - currentSinrDb;
        //if((currentSinrDb < 20 || sinrDiff > 10) && sinrDiff != 0)
        if (currentSinrDb < 10 && sinrDiff != 0)
        {
          Time delay;
          Time BRreactiveness = MilliSeconds ((uint64_t)40.56);
          if (currentSinrDb < 0)
          {
            //The Sinr triggers a full IA with the delay of 42.6072 ms
            //Value see Giordani, "Standalone and Non-Standalone Beam Management for 3gpp Nr at mmWave
            delay = MilliSeconds(m_IAdelay) + BRreactiveness;            
          }
          else
          {
            if (m_adaptiveBF)
            {
              m_phyIdealBeamformingHelper->AddBeamformingTask (DynamicCast<NrGnbNetDevice> (m_netDevice), ueDev);
            }
            //The Sinr triggers beam training, since it's below 10 dB but not low enough for needing IA
            delay = MilliSeconds(m_BFtrainingDelay) + BRreactiveness; //not able to find results 

            BeamSweepTraceParams params;
            params.m_beamSweepOrigin = BeamSweepTraceParams::UE_INITIATED_BEAM_REFINEMENT;
            params.currentCell = ueDev->GetTargetEnb ()->GetCellId ();
            params.snrBeforeSweep = currentSinrDb;
            m_beamSweepTrace (params);
          }
          if (m_BFdelay)
          {
            if (!m_omniFallback)
            {
              //COMMENTED OUT DUE TO TWO REASONS:
              //1)INCLUSION OF NRMACSCHEDULER/NRMACSCHEDULERNS3 CAUSES PROBLEMS
              //2)WE ARE GOING TO REPLACE IAdelay WITH A MORE REALISTIC FUNCTION ANYWAYS
              Simulator::ScheduleNow(&NrMacSchedulerNs3::IAdelay, m_sched, delay);
            }
            
            IAdelay (delay);
          }
          NS_LOG_UNCOND("Perform Beamforming: IA, delay of " << delay.GetMilliSeconds() << "ms");
        }
      }*/
      m_prevSinrMap[ue->first] = sinrAvg;
    }


    if (m_roundFromLastUeSinrUpdate >= (m_ueUpdateSinrPeriod / m_updateSinrPeriod))
    {
      m_roundFromLastUeSinrUpdate = 0;
      for (std::map<uint64_t, Ptr<NetDevice> >::iterator ue = m_ueAttachedImsiMap.begin (); ue != m_ueAttachedImsiMap.end (); ++ue)
      {
        // distinguish between MC and NrNetDevice
        Ptr<NrUeNetDevice> ueNetDevice = DynamicCast<NrUeNetDevice> (ue->second);
        Ptr<NrUePhy> uePhy;
        if (ueNetDevice != 0)
        {
          uePhy = DynamicCast<NrUePhy> (ueNetDevice->GetPhy (0));
        }
        uePhy->UpdateSinrEstimate (GetCellId(), m_sinrMap.find (ue->first)->second);
      }
    }
    else
    {
      m_roundFromLastUeSinrUpdate++;
    }

    LteEnbCphySapUser::UeAssociatedSinrInfo info;
    info.ueImsiSinrMap = m_sinrMap;
    info.componentCarrierId = GetBwpId ();
    m_enbCphySapUser->UpdateUeSinrEstimate (info);
    
  }
  Simulator::Schedule (MicroSeconds (m_updateSinrPeriod), & NrGnbPhy::UpdateUeSinrEstimate, this);  // recall after m_updateSinrPeriod microseconds
}

void
NrGnbPhy::IAdelay (Time delay)
{
  if (!IAisPerformed)
  {
    NS_LOG_UNCOND ("BF update started at: " << Simulator::Now().GetSeconds ());
    IAisPerformed = true;
    Simulator::Schedule (delay, &NrGnbPhy::ResetIAdelay, this);
  }
}

void
NrGnbPhy::ResetIAdelay()
{
  if (IAisPerformed)
  {
    NS_LOG_UNCOND ("BF update finished at: " << Simulator::Now().GetSeconds());
    for (size_t i = 0; i < performBF.size (); i++)
    {
      performBF[i] = true;
    }
    IAisPerformed = false;
  }
}

void
NrGnbPhy::QueueMib ()
{
  NS_LOG_FUNCTION (this);
  LteRrcSap::MasterInformationBlock mib;
  mib.dlBandwidth = m_channelBandwidth;
  mib.systemFrameNumber = 1;
  Ptr<NrMibMessage> mibMsg = Create<NrMibMessage> ();
  mibMsg->SetSourceBwp (GetBwpId ());
  mibMsg->SetMib (mib);
  EnqueueCtrlMsgNow (mibMsg);
}

void NrGnbPhy::QueueSib ()
{
  NS_LOG_FUNCTION (this);
  Ptr<NrSib1Message> msg = Create<NrSib1Message> ();
  msg->SetSib1 (m_sib1);
  msg->SetSourceBwp (GetBwpId ());
  EnqueueCtrlMsgNow (msg);
}

void
NrGnbPhy::QueueSSB (bool pushFront)
{
  NS_LOG_FUNCTION (this);

  auto cellId = GetCellId ();
  
  Ptr<NrPssMessage> pssMsg = Create<NrPssMessage> ();
  pssMsg->SetCellId (cellId);
  pssMsg->SetSourceBwp (GetBwpId ());
  if (m_currSymStart == 2)
  {
    pssMsg->SetSymbolOffset (0);
  }
  else if (m_currSymStart == 8)
  {
    pssMsg->SetSymbolOffset (1);
  }
  
  if (pushFront)
  {
    m_ctrlMsgs.push_front (pssMsg);
  }
  else
  {
    m_ctrlMsgs.push_back(pssMsg);
  }
}

void 
NrGnbPhy::QueueCSIRS (bool pushFront, uint64_t imsi)
{
  NS_LOG_FUNCTION (this);

  Ptr<NrCSIRSMessage> csiRSMsg = Create<NrCSIRSMessage> ();
  csiRSMsg->SetCellId (GetCellId ());
  NS_ASSERT_MSG (m_ueAttached.find (imsi) != m_ueAttached.end (), "UE with IMSI " << imsi << " could not be found");
  csiRSMsg->SetSourceBwp (GetBwpId ());

  if (pushFront)
  {
    m_ctrlMsgs.push_front (csiRSMsg);
  }
  else
  {
    m_ctrlMsgs.push_back (csiRSMsg);
  }
  
}

void
NrGnbPhy::CallMacForSlotIndication (const SfnSf &currentSlot)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (!m_generateDl.empty() || !m_generateUl.empty());

  m_phySapUser->SetCurrentSfn (currentSlot);

  uint64_t currentSlotN = currentSlot.Normalize () % m_tddPattern.size ();

  NS_LOG_INFO ("Start Slot " << currentSlot << ". In position " <<
               currentSlotN << " there is a slot of type " <<
               m_tddPattern[currentSlotN]);

  for (const auto & k2WithLatency : m_generateUl[currentSlotN])
    {
      SfnSf targetSlot = currentSlot;
      targetSlot.Add (k2WithLatency);

      uint64_t pos = targetSlot.Normalize () % m_tddPattern.size ();

      NS_LOG_INFO (" in slot " << currentSlot << " generate UL for " <<
                    targetSlot << " which is of type " << m_tddPattern[pos]);

      m_phySapUser->SlotUlIndication (targetSlot, m_tddPattern[pos]);
      
    }

  for (const auto & k0WithLatency : m_generateDl[currentSlotN])
    {
      SfnSf targetSlot = currentSlot;
      targetSlot.Add (k0WithLatency);

      uint64_t pos = targetSlot.Normalize () % m_tddPattern.size ();

      NS_LOG_INFO (" in slot " << currentSlot << " generate DL for " <<
                     targetSlot << " which is of type " << m_tddPattern[pos]);

      m_phySapUser->SlotDlIndication (targetSlot, m_tddPattern[pos]);
    }
}

void
NrGnbPhy::StartSlot (const SfnSf &startSlot)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_channelStatus != TO_LOSE);

  m_currentSlot = startSlot;
  m_lastSlotStart = Simulator::Now ();

  Simulator::Schedule (GetSlotPeriod (), &NrGnbPhy::EndSlot, this);

  // update the current slot allocation; if empty (e.g., at the beginning of simu)
  // then insert a dummy allocation, without anything.
  if (SlotAllocInfoExists (m_currentSlot))
    {
      m_currSlotAllocInfo = RetrieveSlotAllocInfo (m_currentSlot);
    }
  else
    {
      NS_LOG_WARN ("No allocation for the current slot. Using an empty one");
      m_currSlotAllocInfo = SlotAllocInfo (m_currentSlot);
    }

  if (m_isPrimary)
    {
      if (m_currentSlot.GetSlot () == 0)
        {
          if (m_currentSlot.GetSubframe () == 0)   //send MIB at the beginning of each frame
            {
              QueueMib ();
            }
          else if (m_currentSlot.GetSubframe () == 5)   // send SIB at beginning of second half-frame
            {
              QueueSib ();
            }
        }
      /*if (m_phySapUser->IsSSBRequired (GetCurrentSfnSf ()))
      {
        QueueSSB ();
      }*/
    }

  if (m_channelStatus == GRANTED)
    {
      NS_LOG_INFO ("Channel granted");
      CallMacForSlotIndication (m_currentSlot);
      DoStartSlot ();
    }
  else
    {
      bool hasUlDci = false;
      SfnSf ulSfn = m_currentSlot;
      ulSfn.Add (GetN2Delay ());

      if (GetN2Delay () > 0)
      {
        if (SlotAllocInfoExists (ulSfn))
          {
            SlotAllocInfo & ulSlot = PeekSlotAllocInfo (ulSfn);
            hasUlDci = ulSlot.ContainsDataAllocation ();
          }
      }
      // If there is a DL CTRL, try to obtain the channel to transmit it;
      // because, even if right now there isn't any message, maybe they
      // will come from another BWP.
      if (m_currSlotAllocInfo.ContainsDataAllocation () || m_currSlotAllocInfo.ContainsDlCtrlAllocation () || hasUlDci)
        {
          // Request the channel access
          if (m_channelStatus == NONE)
            {
              NS_LOG_INFO ("Channel not granted, request the channel");
              m_channelStatus = REQUESTED; // This goes always before RequestAccess()
              m_cam->RequestAccess ();
              if (m_channelStatus == GRANTED)
                {
                  // Repetition but we can have a CAM that gives the channel
                  // instantaneously
                  NS_LOG_INFO ("Channel granted; asking MAC for SlotIndication for the future and then start the slot");
                  CallMacForSlotIndication (m_currentSlot);
                  DoStartSlot ();
                  return; // Exit without calling anything else
                }
            }
          // If the channel was not granted, queue back the allocation,
          // without calling the MAC for a new slot
          auto slotAllocCopy = m_currSlotAllocInfo;
          auto newSfnSf = slotAllocCopy.m_sfnSf;
          newSfnSf.Add (1);
          NS_LOG_INFO ("Queueing allocation in front for " << newSfnSf );
          if (m_currSlotAllocInfo.ContainsDataAllocation ())
            {
              NS_LOG_INFO ("Reason: Current slot allocation has data");
            }
          else
            {
              NS_LOG_INFO ("Reason: CTRL message list is not empty");
            }

          PushFrontSlotAllocInfo (newSfnSf, slotAllocCopy);
        }
      else
        {
          // It's an empty slot; ask the MAC for a new one (maybe a new data will arrive..)
          // and just let the current one go away
          NS_LOG_INFO ("Empty slot, but asking MAC for SlotIndication for the future, maybe there will be data");
          CallMacForSlotIndication (m_currentSlot);
        }
      // If we have the UL CTRL, then schedule it (we are listening, so
      // we don't need the channel.

      if (m_currSlotAllocInfo.m_varTtiAllocInfo.size() > 0)
        {
          for (const auto & alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
            {
              if (alloc.m_dci->m_type == DciInfoElementTdma::CTRL && alloc.m_dci->m_format == DciInfoElementTdma::UL)
                {
                  Time start = GetSymbolPeriod () * alloc.m_dci->m_symStart;
                  NS_LOG_INFO ("Schedule UL CTRL at " << start);
                  Simulator::Schedule (start, &NrGnbPhy::UlCtrl, this, alloc.m_dci);
                }
            }
        }
    }
}


void
NrGnbPhy::DoCheckOrReleaseChannel ()
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_channelStatus == GRANTED);
  // The channel is granted, we have to check if we maintain it for the next
  // slot or we have to release it.

  // Assuming the scheduler assign contiguos symbol
  uint8_t lastDlSymbol = 0;
  for (auto & dci : m_currSlotAllocInfo.m_varTtiAllocInfo)
    {
      if (dci.m_dci->m_type == DciInfoElementTdma::DATA && dci.m_dci->m_format == DciInfoElementTdma::DL)
        {
          lastDlSymbol = std::max (lastDlSymbol,
                                   static_cast<uint8_t> (dci.m_dci->m_symStart + dci.m_dci->m_numSym));
        }
    }

  Time lastDataTime = GetSymbolPeriod () * lastDlSymbol;

  if (GetSlotPeriod () - lastDataTime > MicroSeconds (25))
    {
      NS_LOG_LOGIC ("Last symbol of data: " << +lastDlSymbol << ", to the end of slot we still have " <<
                   (GetSlotPeriod () - lastDataTime).GetMicroSeconds() <<
                   " us, so we're going to lose the channel");
      m_channelStatus = TO_LOSE;
    }
  else
    {
      NS_LOG_LOGIC ("Last symbol of data: " << +lastDlSymbol << ", to the end of slot we still have " <<
                   (GetSlotPeriod () - lastDataTime).GetMicroSeconds() <<
                   " us, so we're NOT going to lose the channel");
    }
}

void
NrGnbPhy::RetrievePrepareEncodeCtrlMsgs ()
{
  NS_LOG_FUNCTION (this);
  auto ctrlMsgs = PopCurrentSlotCtrlMsgs ();
  ctrlMsgs.merge (RetrieveMsgsFromDCIs (m_currentSlot));

  if (m_netDevice != nullptr)
    {
      DynamicCast<NrGnbNetDevice> (m_netDevice)->RouteOutgoingCtrlMsgs (ctrlMsgs, GetBwpId ());
    }
  else
    {
      // No netDevice (that could happen in tests) so just redirect them to us
      for (const auto & msg : ctrlMsgs)
        {
          EncodeCtrlMsg (msg);
        }
    }
}

void
NrGnbPhy::GenerateAllocationStatistics (const SlotAllocInfo &allocInfo) const
{
  NS_LOG_FUNCTION (this);
  std::unordered_set<uint16_t> activeUe;
  uint32_t availRb = GetRbNum ();
  uint32_t dataReg = 0;
  uint32_t ctrlReg = 0;
  uint32_t dataSym = 0;
  uint32_t ctrlSym = 0;

  int lastSymStart = -1;
  uint32_t symUsed = 0;

  for (const auto & allocation : allocInfo.m_varTtiAllocInfo)
    {
      uint32_t rbg = std::count (allocation.m_dci->m_rbgBitmask.begin (),
                                 allocation.m_dci->m_rbgBitmask.end (), 1);

      // First: Store the RNTI of the UE in the active list
      if (allocation.m_dci->m_rnti != 0)
        {
          activeUe.insert (allocation.m_dci->m_rnti);
        }

      NS_ASSERT (lastSymStart <= allocation.m_dci->m_symStart);

      auto rbgUsed = (rbg * GetNumRbPerRbg ()) * allocation.m_dci->m_numSym;
      if (allocation.m_dci->m_type == DciInfoElementTdma::DATA)
        {
          dataReg += rbgUsed;
        }
      else
        {
          ctrlReg += rbgUsed;
        }

      if (lastSymStart != allocation.m_dci->m_symStart)
        {
          symUsed += allocation.m_dci->m_numSym;

          if (allocation.m_dci->m_type == DciInfoElementTdma::DATA)
            {
              dataSym += allocation.m_dci->m_numSym;
            }
          else
            {
              ctrlSym += allocation.m_dci->m_numSym;
            }
        }

      lastSymStart = allocation.m_dci->m_symStart;
    }

  /*NS_ASSERT_MSG (symUsed == allocInfo.m_numSymAlloc,
                 "Allocated " << +allocInfo.m_numSymAlloc << " but only " << symUsed << " written in stats");*/

  m_phySlotDataStats (allocInfo.m_sfnSf, activeUe.size (), dataReg, dataSym,
                      availRb, GetSymbolsPerSlot () - ctrlSym, GetBwpId (), GetCellId ());
  m_phySlotCtrlStats (allocInfo.m_sfnSf, activeUe.size (), ctrlReg, ctrlSym,
                      availRb, GetSymbolsPerSlot () - dataSym, GetBwpId (), GetCellId ());
}

void
NrGnbPhy::DoStartSlot ()
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_ctrlMsgs.size () == 0); // This assert has to be re-evaluated for NR-U.
                                       // We can have messages before we weren't able to tx them before.

  uint64_t currentSlotN = m_currentSlot.Normalize () % m_tddPattern.size ();;

  NS_LOG_DEBUG ("Start Slot " << m_currentSlot << " of type " << m_tddPattern[currentSlotN]);

  GenerateAllocationStatistics (m_currSlotAllocInfo);

  if (m_currSlotAllocInfo.m_varTtiAllocInfo.size () == 0)
    {
      return;
    }

  NS_LOG_INFO ("Allocations of the current slot: " << std::endl << m_currSlotAllocInfo);

  DoCheckOrReleaseChannel ();

  RetrievePrepareEncodeCtrlMsgs ();

  PrepareRbgAllocationMap (m_currSlotAllocInfo.m_varTtiAllocInfo);

  FillTheEvent ();
}

void
NrGnbPhy::PrepareRbgAllocationMap (const std::deque<VarTtiAllocInfo> &allocations)
{
  NS_LOG_FUNCTION (this);

  // Start with a clean RBG allocation bitmask
  m_rbgAllocationPerSym.clear ();

  // Create RBG map to know where to put power in DL
  for (const auto & allocation : allocations)
    {
      if (allocation.m_dci->m_type != DciInfoElementTdma::CTRL)
        {
          if (allocation.m_dci->m_format == DciInfoElementTdma::DL)
            {
              // In m_rbgAllocationPerSym, store only the DL RBG set to 1:
              // these will used to put power
              StoreRBGAllocation (&m_rbgAllocationPerSym, allocation.m_dci);
            }

          // For statistics, store UL/DL allocations
          StoreRBGAllocation (&m_rbgAllocationPerSymDataStat, allocation.m_dci);
        }
    }

  for (const auto & s : m_rbgAllocationPerSymDataStat)
    {
      auto & rbgAllocation = s.second;
      m_rbStatistics (m_currentSlot, s.first, FromRBGBitmaskToRBAssignment (rbgAllocation),
                      GetBwpId (), GetCellId ());
    }

  m_rbgAllocationPerSymDataStat.clear ();
}

void
NrGnbPhy::FillTheEvent ()
{
  NS_LOG_FUNCTION (this);

  uint8_t lastSymStart = 0;
  bool useNextAllocationSameSymbol = true;
  for (const auto & allocation : m_currSlotAllocInfo.m_varTtiAllocInfo)
    {
      NS_ASSERT (lastSymStart <= allocation.m_dci->m_symStart);
     
      if (lastSymStart == allocation.m_dci->m_symStart && !useNextAllocationSameSymbol)
        {
          NS_LOG_INFO ("Ignored allocation " << *(allocation.m_dci) << " for OFDMA DL trick");
          continue;
        }
      else
        {
          useNextAllocationSameSymbol = true;
        }

      auto varTtiStart = GetSymbolPeriod () * allocation.m_dci->m_symStart;
      Simulator::Schedule (varTtiStart, &NrGnbPhy::StartVarTti, this, allocation.m_dci);
      lastSymStart = allocation.m_dci->m_symStart;

      // If the allocation is DL, then don't schedule anything that is in the
      // same symbol (see OFDMA DL trick documentation)
      if (allocation.m_dci->m_format == DciInfoElementTdma::DL)
        {
          useNextAllocationSameSymbol = false;
        }

      NS_LOG_INFO ("Scheduled allocation " << *(allocation.m_dci) << " at " << varTtiStart);
    }

  m_currSlotAllocInfo.m_varTtiAllocInfo.clear ();
}

void
NrGnbPhy::StoreRBGAllocation (std::unordered_map<uint8_t, std::vector<uint8_t> > *map,
                              const std::shared_ptr<DciInfoElementTdma> &dci) const
{
  NS_LOG_FUNCTION (this);

  auto itAlloc = map->find (dci->m_symStart);
  if (itAlloc == map->end ())
    {
      itAlloc = map->insert (std::make_pair (dci->m_symStart, dci->m_rbgBitmask)).first;
    }
  else
    {
      auto & existingRBGBitmask = itAlloc->second;
      NS_ASSERT (existingRBGBitmask.size () == dci->m_rbgBitmask.size ());
      for (uint32_t i = 0; i < existingRBGBitmask.size (); ++i)
        {
          existingRBGBitmask.at (i) = existingRBGBitmask.at (i) | dci->m_rbgBitmask.at (i);
        }
    }
}

void
NrGnbPhy::ExpireBeamformingTimer ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("Beamforming timer expired; programming a beamforming");
  m_performBeamforming = true;
  m_beamformingTimer = Simulator::Schedule (m_beamformingPeriodicity,
                                            &NrGnbPhy::ExpireBeamformingTimer, this);
}

std::list <Ptr<NrControlMessage>>
NrGnbPhy::RetrieveDciFromAllocation (const SlotAllocInfo &alloc,
                                         const DciInfoElementTdma::DciFormat &format,
                                         uint32_t kDelay, uint32_t k1Delay)
{
  NS_LOG_FUNCTION(this);
  std::list <Ptr<NrControlMessage>> ctrlMsgs;

  for (const auto & dlAlloc : alloc.m_varTtiAllocInfo)
    {
      if (dlAlloc.m_dci->m_type != DciInfoElementTdma::CTRL &&
          dlAlloc.m_dci->m_format == format)
        {
          auto & dciElem = dlAlloc.m_dci;
          NS_ASSERT (dciElem->m_format == format);
          NS_ASSERT (dciElem->m_tbSize > 0);
          NS_ASSERT_MSG (dciElem->m_symStart + dciElem->m_numSym <= GetSymbolsPerSlot (),
                         "symStart: " << static_cast<uint32_t> (dciElem->m_symStart) <<
                         " numSym: " << static_cast<uint32_t> (dciElem->m_numSym) <<
                         " symPerSlot: " << static_cast<uint32_t> (GetSymbolsPerSlot ()));

          NS_LOG_INFO ("Send DCI to " << dciElem->m_rnti << " from sym " <<
                         +dciElem->m_symStart << " to " << +dciElem->m_symStart + dciElem->m_numSym);

          Ptr<NrControlMessage> msg;

          if (dciElem->m_format == DciInfoElementTdma::DL)
            {
              Ptr<NrDlDciMessage> dciMsg = Create<NrDlDciMessage> (dciElem);

              dciMsg->SetSourceBwp (GetBwpId ());
              dciMsg->SetKDelay (kDelay);
              dciMsg->SetK1Delay (k1Delay);
              msg = dciMsg;
            }
          else
            {
              Ptr<NrUlDciMessage> dciMsg = Create<NrUlDciMessage> (dciElem);

              dciMsg->SetSourceBwp (GetBwpId ());
              dciMsg->SetKDelay (kDelay);
              msg = dciMsg;
            }

          ctrlMsgs.push_back (msg);
        }
    }

  return ctrlMsgs;
}

std::list <Ptr<NrControlMessage> >
NrGnbPhy::RetrieveMsgsFromDCIs (const SfnSf &currentSlot)
{
  std::list <Ptr<NrControlMessage> > ctrlMsgs;
  uint64_t currentSlotN = currentSlot.Normalize () % m_tddPattern.size ();

  uint32_t k1delay = m_dlHarqfbPosition[currentSlotN];

  // TODO: copy paste :(
  for (const auto & k0delay : m_toSendDl[currentSlotN])
    {
      SfnSf targetSlot = currentSlot;

      targetSlot.Add (k0delay);

      if (targetSlot == currentSlot)
        {
          NS_LOG_INFO (" in slot " << currentSlot << " send DL DCI for the same slot");

          ctrlMsgs.merge (RetrieveDciFromAllocation (m_currSlotAllocInfo,
                                                     DciInfoElementTdma::DL, k0delay, k1delay));
        }
      else if (SlotAllocInfoExists (targetSlot))
        {
          NS_LOG_INFO (" in slot " << currentSlot << " send DL DCI for " <<
                         targetSlot);

          ctrlMsgs.merge (RetrieveDciFromAllocation (PeekSlotAllocInfo(targetSlot),
                                                     DciInfoElementTdma::DL, k0delay, k1delay));
        }
      else
        {
          NS_LOG_INFO ("No allocation found for slot " << targetSlot);
        }
    }

  for (const auto & k2delay : m_toSendUl[currentSlotN])
    {
      SfnSf targetSlot = currentSlot;

      targetSlot.Add (k2delay);

      if (targetSlot == currentSlot)
        {
          NS_LOG_INFO (" in slot " << currentSlot << " send UL DCI for the same slot");

          ctrlMsgs.merge (RetrieveDciFromAllocation (m_currSlotAllocInfo,
                                                     DciInfoElementTdma::UL, k2delay, k1delay));
        }
      else if (SlotAllocInfoExists (targetSlot))
        {
          NS_LOG_INFO (" in slot " << currentSlot << " send UL DCI for " <<
                         targetSlot);

          ctrlMsgs.merge (RetrieveDciFromAllocation (PeekSlotAllocInfo(targetSlot),
                                                     DciInfoElementTdma::UL, k2delay, k1delay));
        }
      else
        {
          NS_LOG_INFO ("No allocation found for slot " << targetSlot);
        }
    }

  return ctrlMsgs;
}

Time
NrGnbPhy::DlCtrl (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_DEBUG ("Starting DL CTRL TTI at symbol " << +m_currSymStart <<
                " to " << +m_currSymStart + dci->m_numSym);
  uint16_t currentCellId = GetCellId ();
  if ((performBF[currentCellId] || m_performBeamforming))
  {
    m_performBeamforming = false;
    performBF[currentCellId] = false;
    for (const auto & dev: m_deviceMap)
    {
      //m_doBeamforming (m_netDevice, dev);
      //m_phyIdealBeamformingHelper->AddBeamformingTask (DynamicCast<NrGnbNetDevice> (m_netDevice), dev);
      
      NS_LOG_UNCOND("Beamforming performed at "<< Simulator::Now().GetSeconds() << " to cellID: " << currentCellId);
      //Chnaging the current ue beam back to the registered eNB
      Ptr<NrUeNetDevice> ueNet = DynamicCast<NrUeNetDevice> (dev);
      if (ueNet->GetTargetEnb () != 0 && m_adaptiveBF)
      {
        //m_doBeamforming (ueNet->GetTargetEnb (), dev);
        //m_phyIdealBeamformingHelper->AddBeamformingTask (ConstCast<NrGnbNetDevice> (ueNet->GetTargetEnb ()), dev);
      }
    }
  }

  // TX control period
  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;
  std::pair<bool, std::pair<uint64_t, uint8_t>> boolImsiIndexPair = {false, {0, 0}};

  if (m_realisticIA)
  {
    if (m_phySapUser->IsSSBRequired (GetCurrentSfnSf ()) && 
     (m_currSymStart == 2 || m_currSymStart == 8))
      {
        if (!(m_currSymStart == 8 && 
            GetCurrentSfnSf ().GetSlot () == 7 &&
            GetCurrentSfnSf ().GetSubframe () == 3))
        {
          if (m_ctrlMsgs.size() == 0)
            {
              QueueSSB (false); 
            }
          else
            {
              QueueSSB (true);
            }
        }
        
        
      }
    
    if (m_rlmOn && !IAisPerformed)
    {
      boolImsiIndexPair = m_phySapUser->IsCSIRSRequired (GetCurrentSfnSf ());
      if (boolImsiIndexPair.first &&
         (m_currSymStart == 1) &&
          m_ueAttached.find (boolImsiIndexPair.second.first) != m_ueAttached.end ())
        {
          if (m_ctrlMsgs.size () == 0)
          {
            QueueCSIRS (false, boolImsiIndexPair.second.first);
          }
          else
          {
            QueueCSIRS (true, boolImsiIndexPair.second.first);
          }
          
        }
    }
  }

  

  // The function that is filling m_ctrlMsgs is NrPhy::encodeCtrlMsgs
  if (m_ctrlMsgs.size () > 0)
    {
      auto pssReceived = false;
      auto csiReceived = false;
      auto csiInfoPair = std::pair<bool, std::pair<uint64_t, uint8_t>> (false,{0,0});
      NS_LOG_INFO ("ENB TXing DL CTRL with " << m_ctrlMsgs.size () << " msgs, frame " << m_currentSlot <<
                    " symbols "  << static_cast<uint32_t> (dci->m_symStart) <<
                    "-" << static_cast<uint32_t> (dci->m_symStart + dci->m_numSym - 1) <<
                    " start " << Simulator::Now () <<
                    " end " << Simulator::Now () + varTtiPeriod - NanoSeconds (1.0));

      Ptr<NrCSIRSMessage> csiRSMsg;
      Ptr<NrPssMessage> pssMsg;

      if (IAisPerformed)
      {
        for (std::map<uint64_t, Ptr<NetDevice>>::iterator ue = m_ueAttachedImsiMap.begin (); 
                                                          ue != m_ueAttachedImsiMap.end ();
                                                          ++ue)
        {
          Ptr<NrUeNetDevice> ueNetDev = DynamicCast<NrUeNetDevice> (ue->second);
          ueNetDev->GetPhy (0)->AdjustAntennaForBeamSweep ();
        }
      }
      if (!IAisPerformed && (m_deviceMap.size() != 0))
      {
        for (std::map<uint64_t, Ptr<NetDevice>>::iterator ue = m_ueAttachedImsiMap.begin (); 
                                                          ue != m_ueAttachedImsiMap.end ();
                                                          ++ue)
        {
          Ptr<NrUeNetDevice> ueNetDev = DynamicCast<NrUeNetDevice>(ue->second);
          ueNetDev->GetPhy (0)->AdjustAntennaForBeamSweep ();
        }
      }
      

      for (auto ctrlIt = m_ctrlMsgs.begin (); ctrlIt != m_ctrlMsgs.end (); ++ctrlIt)
        {
          Ptr<NrControlMessage> msg = (*ctrlIt);
          m_phyTxedCtrlMsgsTrace (m_currentSlot, GetCellId (), dci->m_rnti, GetBwpId (), msg);

          if (msg->GetMessageType () == NrControlMessage::PSS)
            {
              pssMsg = DynamicCast<NrPssMessage> (msg);
              pssReceived = true;
            }
          else if (msg->GetMessageType () == NrControlMessage::CSI_RS)
            {
              csiRSMsg = DynamicCast<NrCSIRSMessage> (msg);
              csiReceived = true;
            }
          
        }

      SfnSf currentSfnSf = GetCurrentSfnSf ();
      SfnSfKey sfnSfKey = std::pair<uint8_t, std::pair<uint8_t, uint8_t>> (currentSfnSf.GetFrame () % (uint8_t)2,
                          std::pair<uint8_t, uint8_t> (currentSfnSf.GetSubframe (), currentSfnSf.GetSlot ()));
      if (pssReceived && m_phySapUser->IsSSBRequired (currentSfnSf))
      {
        if (GetCellId () == 10)
        {
          m_beamManager->SetSector (dci->ssbBeamId.GetSector (), dci->ssbBeamId.GetElevation ());
        }
        else
        {
          m_beamManager->SetSector (dci->ssbBeamId.GetSector (), dci->ssbBeamId.GetElevation ());
        }
        
        if (IAisPerformed)
        {
          Ptr<SpectrumValue> noisePsd = NrSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_noiseFigure, GetSpectrumModel ());
          Ptr<SpectrumValue> totalReceivedPsd = Create <SpectrumValue> (SpectrumValue (noisePsd->GetSpectrumModel ()));

          for (std::map<uint64_t, Ptr<NetDevice> >::iterator ue = m_ueAttachedImsiMap.begin (); ue != m_ueAttachedImsiMap.end (); ++ue)
          {
            Ptr<NrUeNetDevice> ueNetDevice = DynamicCast<NrUeNetDevice> (ue->second);
            pssMsg->SetDestinationImsi (ue->first);
          }
        }
        
      }
      
      if (m_deviceMap.size () != 0)
      {
        if (csiReceived && boolImsiIndexPair.first)
        {
          BeamId currentBeam = m_beamsTbRLM.at (boolImsiIndexPair.second.second);

          m_sfnBeamIdMap.insert ({sfnSfKey,currentBeam});
          m_beamManager->SetSector (currentBeam.GetSector (),
                                    currentBeam.GetElevation ());
          Ptr<SpectrumValue> rxPsd = CalculateRxPsdToUe (
                                  DynamicCast<NrUeNetDevice> (m_ueAttachedImsiMap.at(boolImsiIndexPair.second.first)),
                                  false);
          Ptr<SpectrumValue> noisePsd = NrSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_noiseFigure, GetSpectrumModel ());
          auto snrAvg = 0.0;
          SpectrumValue sinr = (*rxPsd)/(*noisePsd);

          snrAvg = Sum (sinr) / (sinr.GetSpectrumModel ()->GetNumBands ());
          csiRSMsg->SetSnrAvg (snrAvg);
          csiRSMsg->SetTXBeamId (currentBeam.GetSector (), currentBeam.GetElevation ());

          Simulator::Schedule (GetSymbolPeriod () * 2 - NanoSeconds (1.0),
                              &BeamManager::ChangeBeamformingVector,
                              m_beamManager,
                              m_deviceMap.at(0));
        }
      }
      
      SendCtrlChannels (varTtiPeriod - NanoSeconds (1.0)); // -1 ns ensures control ends before data period
    }
  else
    {
      NS_LOG_INFO ("No messages to send, skipping");
    }

  return varTtiPeriod;
}

Time
NrGnbPhy::UlCtrl(const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_DEBUG ("Starting UL CTRL TTI at symbol " << +m_currSymStart <<
                " to " << +m_currSymStart + dci->m_numSym);

  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;

  NS_LOG_INFO ("ENB RXng UL CTRL frame " << m_currentSlot <<
                " symbols "  << static_cast<uint32_t> (dci->m_symStart) <<
                "-" << static_cast<uint32_t> (dci->m_symStart + dci->m_numSym - 1) <<
                " start " << Simulator::Now () <<
                " end " << Simulator::Now () + varTtiPeriod);
  return varTtiPeriod;
}

Time
NrGnbPhy::DlData (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("Starting DL DATA TTI at symbol " << +m_currSymStart <<
                " to " << +m_currSymStart + dci->m_numSym);

  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;

  Ptr<PacketBurst> pktBurst = GetPacketBurst (m_currentSlot, dci->m_symStart);
  if (!pktBurst || pktBurst->GetNPackets () == 0)
    {
      // sometimes the UE will be scheduled when no data is queued.
      // In this case, don't send anything, don't put power... don't do nothing!
      return varTtiPeriod;
    }

  NS_LOG_INFO ("ENB TXing DL DATA frame " << m_currentSlot <<
                " symbols "  << static_cast<uint32_t> (dci->m_symStart) <<
                "-" << static_cast<uint32_t> (dci->m_symStart + dci->m_numSym - 1) <<
                " start " << Simulator::Now () + NanoSeconds (1) <<
                " end " << Simulator::Now () + varTtiPeriod - NanoSeconds (2.0));

  Simulator::Schedule (NanoSeconds (1.0), &NrGnbPhy::SendDataChannels, this,
                       pktBurst, varTtiPeriod - NanoSeconds (2.0), dci);

  return varTtiPeriod;
}

Time
NrGnbPhy::UlData(const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_INFO (this);

  NS_LOG_DEBUG ("Starting UL DATA TTI at symbol " << +m_currSymStart <<
                " to " << +m_currSymStart + dci->m_numSym);

  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;

  m_spectrumPhy->AddExpectedTb (dci->m_rnti, dci->m_ndi, dci->m_tbSize, dci->m_mcs,
                                FromRBGBitmaskToRBAssignment (dci->m_rbgBitmask),
                                dci->m_harqProcess, dci->m_rv, false,
                                dci->m_symStart, dci->m_numSym, m_currentSlot);

  bool found = false;
  for (uint8_t i = 0; i < m_deviceMap.size (); i++)
    {
      Ptr<NrUeNetDevice> ueDev = DynamicCast < NrUeNetDevice > (m_deviceMap.at (i));
      uint64_t ueRnti = (DynamicCast<NrUePhy>(ueDev->GetPhy (0)))->GetRnti ();
      if (dci->m_rnti == ueRnti)
        {
          NS_ABORT_MSG_IF(m_beamManager == nullptr, "Beam manager not initialized");
          // Even if we change the beamforming vector, we hope that the scheduler
          // has scheduled UEs within the same beam (and, therefore, have the same
          // beamforming vector)
          m_beamManager->ChangeBeamformingVector (m_deviceMap.at (i)); //assume the control signal is omni
          found = true;
          break;
        }
    }
  NS_ASSERT (found);

  NS_LOG_INFO ("ENB RXing UL DATA frame " << m_currentSlot <<
                " symbols "  << static_cast<uint32_t> (dci->m_symStart) <<
                "-" << static_cast<uint32_t> (dci->m_symStart + dci->m_numSym - 1) <<
                " start " << Simulator::Now () <<
                " end " << Simulator::Now () + varTtiPeriod);
  return varTtiPeriod;
}

void
NrGnbPhy::StartVarTti (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  NS_ABORT_MSG_IF(m_beamManager == nullptr, "Beam manager not initialized");
  //m_beamManager->ChangeToOmniTx (); //assume the control signal is omni
  m_currSymStart = dci->m_symStart;

  Time varTtiPeriod;

  NS_ASSERT (dci->m_type != DciInfoElementTdma::CTRL_DATA);

  if (dci->m_type == DciInfoElementTdma::CTRL)
    {
      if (dci->m_format == DciInfoElementTdma::DL)
        {
          varTtiPeriod = DlCtrl (dci);
        }
      else if (dci->m_format == DciInfoElementTdma::UL)
        {
          varTtiPeriod = UlCtrl (dci);
        }
    }
  else  if (dci->m_type == DciInfoElementTdma::DATA)
    {
      if (dci->m_format == DciInfoElementTdma::DL)
        {
          varTtiPeriod = DlData (dci);
        }
      else if (dci->m_format == DciInfoElementTdma::UL)
        {
          varTtiPeriod = UlData (dci);
        }
    }

  Simulator::Schedule (varTtiPeriod, &NrGnbPhy::EndVarTti, this, dci);
}

void
NrGnbPhy::EndVarTti (const std::shared_ptr<DciInfoElementTdma> &lastDci)
{
  NS_LOG_FUNCTION (this << Simulator::Now ().GetSeconds ());

  NS_LOG_DEBUG ("DCI started at symbol " << static_cast<uint32_t> (lastDci->m_symStart) <<
                " which lasted for " << static_cast<uint32_t> (lastDci->m_numSym) <<
                " symbols finished");
}

void
NrGnbPhy::EndSlot (void)
{
  NS_LOG_FUNCTION (this);

  Time slotStart = m_lastSlotStart + GetSlotPeriod () - Simulator::Now ();

  if (m_channelStatus == TO_LOSE)
    {
      NS_LOG_INFO ("Release the channel because we did not have any data to maintain the grant");
      m_channelStatus = NONE;
      m_channelLostTimer.Cancel ();
    }

  NS_LOG_DEBUG ("Slot started at " << m_lastSlotStart << " ended");
  m_currentSlot.Add (1);
  Simulator::Schedule (slotStart, &NrGnbPhy::StartSlot, this, m_currentSlot);
}

void
NrGnbPhy::SendDataChannels (const Ptr<PacketBurst> &pb, const Time &varTtiPeriod,
                                const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);
  // update beamforming vectors (currently supports 1 user only)

  if (IAisPerformed && m_omniFallback)
  {
    m_beamManager->ChangeToOmniTx ();
    for (uint8_t i = 0; i < m_deviceMap.size (); i++)
    {
      Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (m_deviceMap.at (i));
      if (ueDev->GetTargetEnb () != 0)
      {
        //NS_LOG_UNCOND ("Change Beamforming Vector");
        Ptr<NrUePhy> uePhy = DynamicCast<NrUePhy> (ueDev->GetPhy (0));
        uePhy->GetBeamManager ()->ChangeToOmniTx ();
        /*Ptr<ThreeGppAntennaArrayModel> ueAntennaArray = DynamicCast<ThreeGppAntennaArrayModel> (uePhy->GetSpectrumPhy ()->GetRxAntenna ());
        ueAntennaArray->ChangeToOmniTx ();*/ 
      }
    }
  }
  else
  {
    bool found = false;
    uint16_t ueCellId = 0;
    uint64_t ueRnti = 0;
    for (uint8_t i = 0; i < m_deviceMap.size (); i++)
      {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (m_deviceMap.at (i));
        ueRnti = (DynamicCast<NrUePhy>(ueDev->GetPhy (0)))->GetRnti ();
        ueCellId = (DynamicCast<NrUePhy>(ueDev->GetPhy(0)) ->GetCellId ());
        //NS_LOG_INFO ("Scheduled rnti:"<<rnti <<" ue rnti:"<< ueRnti);
        if (dci->m_rnti == ueRnti && GetCellId () == ueCellId)
          {
            NS_ABORT_MSG_IF(m_beamManager == nullptr, "Beam manager not initialized");
            m_beamManager->ChangeBeamformingVector(m_deviceMap.at (i));
            if (!m_realisticIA  && m_deviceMap.size() != 0)
            {
              CheckForOmniTX (ueDev);
            }
            if (ueDev->GetTargetEnb () != 0)
            {
              ueDev->GetPhy(0)->GetBeamManager ()->ChangeBeamformingVector (ueDev->GetTargetEnb ());
            }
            found = true;
            break;
          }
      }
    if(!found)
				{
					NS_LOG_UNCOND("ueRnti = " << ueRnti << " ueCellId = " << ueCellId << " enbCellId = " << GetCellId ());
				}
  }
  
  // in the map we stored the RBG allocated by the MAC for this symbol.
  // If the transmission last n symbol (n > 1 && n < 12) the SetSubChannels
  // doesn't need to be called again. In fact, SendDataChannels will be
  // invoked only when the symStart changes.
  NS_ASSERT (m_rbgAllocationPerSym.find(dci->m_symStart) != m_rbgAllocationPerSym.end ());
  SetSubChannels (FromRBGBitmaskToRBAssignment (m_rbgAllocationPerSym.at (dci->m_symStart)));

  std::list<Ptr<NrControlMessage> > ctrlMsgs;
  m_spectrumPhy->StartTxDataFrames (pb, ctrlMsgs, varTtiPeriod, dci->m_symStart);
}

void
NrGnbPhy::SendCtrlChannels (const Time &varTtiPeriod)
{
  NS_LOG_FUNCTION (this << "Send Ctrl");

  std::vector <int> fullBwRb (GetRbNum ());
  // The first time set the right values for the phy
  for (uint32_t i = 0; i < fullBwRb.size (); ++i)
    {
      fullBwRb[i] = static_cast<int> (i);
    }

  SetSubChannels (fullBwRb);

  m_spectrumPhy->StartTxDlControlFrames (m_ctrlMsgs, varTtiPeriod);
  m_ctrlMsgs.clear ();
}

bool
NrGnbPhy::RegisterUe (uint64_t imsi, const Ptr<NrUeNetDevice> &ueDevice)
{
  NS_LOG_FUNCTION (this << imsi);
  std::set <uint64_t>::iterator it;
  it = m_ueAttached.find (imsi);
  m_phySapUser->ForwarIASStateToSched (false);  

  if (it == m_ueAttached.end ())
    {
      if (!m_realisticIA || ueDevice->GetCellId () == GetCellId ())
      {
        m_ueAttached.insert (imsi);
        m_deviceMap.push_back (ueDevice);
      }
      
      m_ueAttachedImsiMap[imsi] = ueDevice;
      return (true);
    }
  else
    {
      NS_LOG_ERROR ("Programming error...UE already attached");
      return (false);
    }
}

bool 
NrGnbPhy::RegisterUe (uint64_t imsi)
{
  NS_LOG_FUNCTION (this << imsi);
  return RegisterUe (imsi, DynamicCast<NrUeNetDevice> (m_ueAttachedImsiMap[imsi]));
}

void
NrGnbPhy::PhyDataPacketReceived (const Ptr<Packet> &p)
{
  Simulator::ScheduleWithContext (m_netDevice->GetNode ()->GetId (),
                                  GetTbDecodeLatency (),
                                  &NrGnbPhySapUser::ReceivePhyPdu,
                                  m_phySapUser,
                                  p);
}

void
NrGnbPhy::GenerateDataCqiReport (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this << sinr);

  Values::const_iterator it;
  NrMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi;
  ulcqi.m_ulCqi.m_type = UlCqiInfo::PUSCH;
  int i = 0;
  for (it = sinr.ConstValuesBegin (); it != sinr.ConstValuesEnd (); it++)
    {
      //   double sinrdb = 10 * std::log10 ((*it));
      //       NS_LOG_INFO ("ULCQI RB " << i << " value " << sinrdb);
      // convert from double to fixed point notaltion Sxxxxxxxxxxx.xxx
      //   int16_t sinrFp = LteFfConverter::double2fpS11dot3 (sinrdb);
      ulcqi.m_ulCqi.m_sinr.push_back (*it);
      i++;
    }

  // here we use the start symbol index of the var tti in place of the var tti index because the absolute UL var tti index is
  // not known to the scheduler when m_allocationMap gets populated
  ulcqi.m_sfnSf = m_currentSlot;
  ulcqi.m_symStart = m_currSymStart;
  SpectrumValue newSinr = sinr;
  m_ulSinrTrace (0, newSinr, newSinr);
  m_phySapUser->UlCqiReport (ulcqi);
}


void
NrGnbPhy::PhyCtrlMessagesReceived (const Ptr<NrControlMessage> &msg)
{
  NS_LOG_FUNCTION (this);

  if (msg->GetMessageType () == NrControlMessage::DL_CQI)
    {
      Ptr<NrDlCqiMessage> dlcqi = DynamicCast<NrDlCqiMessage> (msg);
      DlCqiInfo dlcqiLE = dlcqi->GetDlCqi ();
      m_phyRxedCtrlMsgsTrace (m_currentSlot, GetCellId (), dlcqiLE.m_rnti, GetBwpId (), msg);

      NS_LOG_INFO ("Received DL_CQI for RNTI: " << dlcqiLE.m_rnti << " in slot " <<
                   m_currentSlot);

      m_phySapUser->ReceiveControlMessage (msg);
    }
  else if (msg->GetMessageType () == NrControlMessage::RACH_PREAMBLE)
    {
      NS_LOG_INFO ("received RACH_PREAMBLE");

      Ptr<NrRachPreambleMessage> rachPreamble = DynamicCast<NrRachPreambleMessage> (msg);
      m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), 0, GetBwpId (), msg);
      NS_LOG_INFO ("Received RACH Preamble in slot " << m_currentSlot);
      m_phySapUser->ReceiveRachPreamble (rachPreamble->GetRapId ());

    }
  else if (msg->GetMessageType () == NrControlMessage::DL_HARQ)
    {
      Ptr<NrDlHarqFeedbackMessage> dlharqMsg = DynamicCast<NrDlHarqFeedbackMessage> (msg);
      DlHarqInfo dlharq = dlharqMsg->GetDlHarqFeedback ();
      if (m_ueAttachedRnti.find (dlharq.m_rnti) != m_ueAttachedRnti.end ())
        {
          m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), dlharq.m_rnti, GetBwpId (), msg);

          NS_LOG_INFO ("Received DL_HARQ for RNTI: " << dlharq.m_rnti << " in slot " <<
                       m_currentSlot);
          m_phySapUser->ReceiveControlMessage (msg);
        }
    }
  else if (msg->GetMessageType () == NrControlMessage::CSI_REPORT)
  {
    m_csiRSTbReported.clear ();
    Ptr<NRCSIReportMessage> csiReportMsg = DynamicCast<NRCSIReportMessage> (msg);
    if (csiReportMsg->GetCellId () == GetCellId () &&
        m_ueAttachedRnti.find (csiReportMsg->GetRnti ()) != m_ueAttachedRnti.end ())
    {
      SfnSfKey csiResourceIndex;
      std::vector<double> csiReportForRRC;
      auto csiRSSnr = 0.0;
      for (auto n = 0; n < m_noOfBeamsTbReported; n++)
      {
        csiRSSnr = csiReportMsg->GetCSIRSMap (n).second.second;
        csiResourceIndex = csiReportMsg->GetCSIRSMap (n).first;
        m_csiRSTbReported.emplace_back (std::pair<SfnSfKey, std::pair<BeamId, double>> (
                          csiResourceIndex, {
                          csiReportMsg->GetCSIRSMap (n).second.first,
                          csiRSSnr
                          }
        ));
        csiReportForRRC.emplace_back (csiRSSnr);
      }

      LteEnbCphySapUser::UeRRCCSIRSReport info;
      info.cellId = GetCellId ();
      info.csiRSVector = csiReportForRRC;
      info.rnti = csiReportMsg->GetRnti ();

      m_enbCphySapUser->RelayRRCCSIRSReport (info);

      auto imsiOfCSI = m_enbCphySapUser->GetImsiFromRnti (csiReportMsg->GetRnti ());
      
      if (m_ueAttachedImsiMap.find (imsiOfCSI) != m_ueAttachedImsiMap.end ())
      {
        BeamId maxSNRBeamId = m_csiRSTbReported.at (0).second.first;
        if (maxSNRBeamId != 
          m_beamManager->GetBeamId (m_ueAttachedImsiMap.at(imsiOfCSI)))
          {
            BeamformingVector maxSNRBfv = BeamformingVector (
              CreateDirectionalBfv (m_beamManager->GetAntennaArray (),
              maxSNRBeamId.GetSector (),
              maxSNRBeamId.GetElevation ()),
              maxSNRBeamId
            );
            m_beamManager->SaveBeamformingVector (maxSNRBfv, m_ueAttachedImsiMap.at (imsiOfCSI));
            m_beamManager->ChangeBeamformingVector (m_ueAttachedImsiMap.at (imsiOfCSI));

            BeamSweepTraceParams params;
            params.m_beamSweepOrigin = BeamSweepTraceParams::GNB_BEAM_ADJUSTMENT;
            params.foundCell = GetCellId ();
            params.foundSector = maxSNRBeamId.GetSector ();
            params.foundElevation = maxSNRBeamId.GetElevation ();

            m_beamSweepTrace (params);
          }
      }

      m_sfnBeamIdMap.clear ();
    }
  }
  
  else
    {
      m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), 0, GetBwpId (), msg);
      m_phySapUser->ReceiveControlMessage (msg);
    }
}


////////////////////////////////////////////////////////////
/////////                     sap                 /////////
///////////////////////////////////////////////////////////

void
NrGnbPhy::DoSetBandwidth (uint16_t ulBandwidth, uint16_t dlBandwidth)
{
  NS_LOG_FUNCTION (this << +ulBandwidth << +dlBandwidth);
  NS_ASSERT (ulBandwidth == dlBandwidth);
  m_channelBandwidth = dlBandwidth;
  UpdateRbNum ();
}

void
NrGnbPhy::DoSetEarfcn (uint16_t ulEarfcn, uint16_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << ulEarfcn << dlEarfcn);
}


void
NrGnbPhy::DoAddUe (uint16_t rnti)
{
  NS_UNUSED (rnti);
  NS_LOG_FUNCTION (this << rnti);
  std::set <uint16_t>::iterator it;
  it = m_ueAttachedRnti.find (rnti);
  if (it == m_ueAttachedRnti.end ())
    {
      m_ueAttachedRnti.insert (rnti);
    }
}

void
NrGnbPhy::DoRemoveUe (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);

  std::set <uint16_t>::iterator it = m_ueAttachedRnti.find (rnti);
  if (it != m_ueAttachedRnti.end ())
    {
      m_ueAttachedRnti.erase (it);
      /*if (IAisPerformed)
      {
        IAisPerformed = false;
      }*/
    }
  else
    {
      NS_FATAL_ERROR ("Impossible to remove UE, not attached!");
    }
}

void
NrGnbPhy::DoSetPa (uint16_t rnti, double pa)
{
  NS_LOG_FUNCTION (this << rnti << pa);
}

void
NrGnbPhy::DoSetTransmissionMode (uint16_t  rnti, uint8_t txMode)
{
  NS_LOG_FUNCTION (this << rnti << +txMode);
  // UL supports only SISO MODE
}

void
NrGnbPhy::DoSetSrsConfigurationIndex (uint16_t  rnti, uint16_t srcCi)
{
  NS_LOG_FUNCTION (this << rnti << srcCi);
}

void
NrGnbPhy::DoSetMasterInformationBlock (LteRrcSap::MasterInformationBlock mib)
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (mib);
}

void
NrGnbPhy::DoSetSystemInformationBlockType1 (LteRrcSap::SystemInformationBlockType1 sib1)
{
  NS_LOG_FUNCTION (this);
  m_sib1 = sib1;
}

int8_t
NrGnbPhy::DoGetReferenceSignalPower () const
{
  NS_LOG_FUNCTION (this);
  return static_cast<int8_t> (m_txPower);
}

void
NrGnbPhy::SetPhySapUser (NrGnbPhySapUser* ptr)
{
  m_phySapUser = ptr;
}

void
NrGnbPhy::ReportUlHarqFeedback (const UlHarqInfo &mes)
{
  NS_LOG_FUNCTION (this);
  // forward to scheduler
  if (m_ueAttachedRnti.find (mes.m_rnti) != m_ueAttachedRnti.end ())
    {
      NS_LOG_INFO ("Received UL HARQ feedback " << mes.IsReceivedOk() <<
                   " and forwarding to the scheduler");
      m_phySapUser->UlHarqFeedback (mes);
    }
}

void
NrGnbPhy::SetPattern (const std::string &pattern)
{
  NS_LOG_FUNCTION (this);

  static std::unordered_map<std::string, LteNrTddSlotType> lookupTable =
  {
    { "DL", LteNrTddSlotType::DL },
    { "UL", LteNrTddSlotType::UL },
    { "S",  LteNrTddSlotType::S },
    { "F",  LteNrTddSlotType::F },
  };

  std::vector<LteNrTddSlotType> vector;
  std::stringstream ss (pattern);
  std::string token;
  std::vector<std::string> extracted;

   while (std::getline(ss, token, '|'))
     {
       extracted.push_back(token);
     }

   for (const auto & v : extracted)
     {
       if (lookupTable.find (v) == lookupTable.end())
         {
           NS_FATAL_ERROR ("Pattern type " << v << " not valid. Valid values are: DL UL F S");
         }
       vector.push_back (lookupTable[v]);
     }

   SetTddPattern (vector);
}

std::string
NrGnbPhy::GetPattern () const
{
  static std::unordered_map<LteNrTddSlotType, std::string, std::hash<int>> lookupTable =
  {
    { LteNrTddSlotType::DL, "DL"},
    { LteNrTddSlotType::UL, "UL"},
    { LteNrTddSlotType::S,  "S"},
    { LteNrTddSlotType::F,  "F"}
  };

  std::stringstream ss;

  for (const auto & v : m_tddPattern)
    {
      ss << lookupTable[v] << "|";
    }

  return ss.str ();
}

void
NrGnbPhy::SetPrimary ()
{
  NS_LOG_FUNCTION (this);
  m_isPrimary = true;
}

void
NrGnbPhy::ChannelAccessGranted (const Time &time)
{
  NS_LOG_FUNCTION (this);

  if (time < GetSlotPeriod ())
    {
      NS_LOG_INFO ("Channel granted for less than the slot time. Ignoring the grant.");
      m_channelStatus = NONE;
      return;
    }

  m_channelStatus = GRANTED;

  Time toNextSlot = m_lastSlotStart + GetSlotPeriod () - Simulator::Now ();
  Time grant = time - toNextSlot;
  int64_t slotGranted = grant.GetNanoSeconds () / GetSlotPeriod().GetNanoSeconds ();

  NS_LOG_INFO ("Channel access granted for " << time.GetMilliSeconds () <<
               " ms, which corresponds to " << slotGranted << " slot in which each slot is " <<
               GetSlotPeriod() << " ms. We lost " <<
               toNextSlot.GetMilliSeconds() << " ms. ");
  NS_ASSERT(! m_channelLostTimer.IsRunning ());

  if (slotGranted < 1)
    {
      slotGranted = 1;
    }
  m_channelLostTimer = Simulator::Schedule (GetSlotPeriod () * slotGranted - NanoSeconds (1),
                                            &NrGnbPhy::ChannelAccessLost, this);
}

void
NrGnbPhy::ChannelAccessLost ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("Channel access lost");
  m_channelStatus = NONE;
}
// ADDED DURING MERGING
void
NrGnbPhy::SetPhyIdealBeamformingHelper (Ptr<IdealBeamformingHelper> idealBeamformingHelper)
{
  m_phyIdealBeamformingHelper = idealBeamformingHelper;
  
  // This could be wrong, might add this to somewhere else:
  Ptr<ThreeGppSpectrumPropagationLossModel> nr3gpp = DynamicCast<ThreeGppSpectrumPropagationLossModel> (m_spectrumPropagationLossModel); 
  if (nr3gpp != nullptr)
  {
    //nr3gpp->SetSMIdealBeamformingHelper (idealBeamformingHelper);
  }
}


std::vector<BeamId>
NrGnbPhy::DoGenerateBeamVectorMap ()
{
  std::vector<BeamId> beamVectorMapGnb;
  std::pair<double, double> elevationBeginEnd;
  uint16_t txNumRows;
  
  switch (m_antennaConfig)
  {
  case AntennaConfigInets:
    elevationBeginEnd = {90.0, 150.0};
    break;
  default:
    break;
  }

  switch (m_antennaConfig)
  {
  case AntennaConfigInets:
      txNumRows = (180.0 / m_gnbHorizontalAngleStep);
      for (double txTheta = elevationBeginEnd.first; txTheta <= elevationBeginEnd.second; txTheta = txTheta + m_gnbElevationAngleStep)
      {
        for (uint16_t txSector = 0; txSector <= txNumRows; txSector++)
        {
          NS_ASSERT(txSector < UINT16_MAX);

          beamVectorMapGnb.emplace_back(BeamId(txSector, txTheta));
        }
      }
    break;
  case AntennaConfigDefault:
    txNumRows = GetAntennaArray ()->GetNumOfRowElements ();
    for (double txTheta = 60; txTheta < 121; txTheta += m_gnbElevationAngleStep)
    {
      for (uint16_t txSector = 0; txSector <= txNumRows; txSector++)
      {
        NS_ASSERT (txSector < UINT16_MAX);

        beamVectorMapGnb.emplace_back (BeamId (txSector, txTheta));
      }
    }
    break;
  default:
    NS_ABORT_MSG ("Undefined Antenna COnfiguration");
    break;
  }
  
  m_beamVectorMapGnb = beamVectorMapGnb;

  return beamVectorMapGnb;
}

bool
NrGnbPhy::GetIAState ()
{
  return m_IAcontinues;
}

void 
NrGnbPhy::DoSetOptimalGnbBeamForImsi (uint64_t imsi, SfnSf startingSfn, std::vector<uint16_t> optimalBeamIndex, bool isMaxSNRCell)
{
  uint16_t actualIndex; /*= ((startingSfn.GetSubframe() * startingSfn.GetSlotPerSubframe () * 2) 
         + (startingSfn.GetSlot () * 2) + (optimalBeamIndex - 1)) % m_beamVectorMapGnb.size ();*/
  std::vector<BeamId> beamTbRLMVector;

  for (auto beamIterator = 0; beamIterator < m_noOfBeamsTbRLM; beamIterator++)
  {
    actualIndex = ((startingSfn.GetSubframe () * startingSfn.GetSlotPerSubframe () * m_noOfSSBsPerSlot)
        + (startingSfn.GetSlot () * m_noOfSSBsPerSlot) + (optimalBeamIndex.at(beamIterator) - 1)) % m_beamVectorMapGnb.size ();
    beamTbRLMVector.emplace_back (m_beamVectorMapGnb.at (actualIndex));
  }

  if (m_imsiOptimalBeamId.find (imsi) != m_imsiOptimalBeamId.end ())
    {
      m_imsiOptimalBeamId.at(imsi) = beamTbRLMVector.at(0); 
    }
  else
    {
      m_imsiOptimalBeamId.insert ({imsi, beamTbRLMVector.at(0)});
    }  

  m_beamsTbRLM.clear ();
  m_beamsTbRLM = beamTbRLMVector;

  if (isMaxSNRCell)
  {
    BeamSweepTraceParams params;
    params.m_beamSweepOrigin = BeamSweepTraceParams::GNB_RECVD_BEAM_REPORT;
    params.foundCell = GetCellId ();
    params.foundSector = m_imsiOptimalBeamId.at (imsi).GetSector ();
    params.foundElevation = m_imsiOptimalBeamId.at (imsi).GetElevation ();

    m_beamSweepTrace (params);
  }  

  m_currBeamformingVector = BeamformingVector (
    CreateDirectionalBfv (m_beamManager->GetAntennaArray (),
          m_imsiOptimalBeamId.at(imsi).GetSector (),
          m_imsiOptimalBeamId.at(imsi).GetElevation ()), m_imsiOptimalBeamId.at(imsi));
  
  m_beamManager->SetSector (m_imsiOptimalBeamId.at(imsi).GetSector (),
                m_imsiOptimalBeamId.at(imsi).GetElevation ());
  m_beamManager->SaveBeamformingVector (m_currBeamformingVector, m_ueAttachedImsiMap.find(imsi)->second);

  SfnSf dlSfnSf = GetCurrentSfnSf ();
  auto currentFn = dlSfnSf.GetFrame ();
  auto currentSfn = dlSfnSf.GetSubframe ();
  auto currentSn = dlSfnSf.GetSlot ();
  auto frameRemainder = currentFn % (uint8_t)2;

  if (!(frameRemainder == 0 && currentSfn < 4))
  {
    auto totalNoOfSlot = ((uint8_t)1 - frameRemainder) * dlSfnSf.GetSubframesPerFrame () * dlSfnSf.GetSlotPerSubframe () + 
                      (dlSfnSf.GetSubframesPerFrame () - currentSfn - 1) * dlSfnSf.GetSlotPerSubframe () + 
                      (dlSfnSf.GetSlotPerSubframe () - currentSn - 1);
    m_phySapUser->SetMACCSIRSTimer (true);
    Simulator::Schedule (totalNoOfSlot * GetSymbolPeriod () * GetSymbolsPerSlot (),
                        &NrGnbPhy::SetCSIRSTimer, 
                        this,
                        false);
  }
}

void 
NrGnbPhy::DoAttachUeFromRRC (uint64_t imsi, const Ptr<NetDevice> &netDev)
{
  Ptr<NrUeNetDevice> ueNetDevice = DynamicCast<NrUeNetDevice> (netDev);
  Ptr<NrGnbNetDevice> gnbNetDevice = DynamicCast<NrGnbNetDevice> (m_netDevice);

  NS_ABORT_IF (gnbNetDevice == nullptr || ueNetDevice == nullptr);

  for (uint16_t i = 0; i < gnbNetDevice->GetCcMapSize (); ++i)
  {
    gnbNetDevice->GetPhy (i)->RegisterUe (imsi, ueNetDevice);
    ueNetDevice->GetPhy (i)->SetDlAmc (
            DynamicCast<NrMacSchedulerNs3> (gnbNetDevice->GetScheduler (i))->GetDlAmc ());
    ueNetDevice->GetPhy (i)->SetDlCtrlSyms (gnbNetDevice->GetMac (i)->GetDlCtrlSyms ());
    ueNetDevice->GetPhy (i)->SetUlCtrlSyms (gnbNetDevice->GetMac (i)->GetUlCtrlSyms ());
    ueNetDevice->GetPhy (i)->DoSetDlBandwidth (m_channelBandwidth);
    ueNetDevice->GetPhy (i)->SetNumRbPerRbg (gnbNetDevice->GetMac(i)->GetNumRbPerRbg());
    ueNetDevice->GetPhy (i)->SetRbOverhead (gnbNetDevice->GetPhy (i)->GetRbOverhead ());
    ueNetDevice->GetPhy (i)->SetSymbolsPerSlot (gnbNetDevice->GetPhy (i)->GetSymbolsPerSlot ());
    ueNetDevice->GetPhy (i)->SetNumerology (gnbNetDevice->GetPhy(i)->GetNumerology ());
    ueNetDevice->GetPhy (i)->SetPattern (gnbNetDevice->GetPhy (i)->GetPattern ());    
    Ptr<EpcUeNas> ueNas = ueNetDevice->GetNas ();
    ueNas->Connect (gnbNetDevice->GetBwpId (i), gnbNetDevice->GetEarfcn (i));    
  }
  m_phySapUser->ForwarIASStateToSched (false);  
}

void 
NrGnbPhy::SetPHYEpcHelper (Ptr<EpcHelper> epcHelper)
{
  m_phyEpcHelper = epcHelper;
}

void 
NrGnbPhy::CheckForOmniTX (Ptr<NrUeNetDevice> ueNetDev)
{
  if (m_beamManager->GetCurrentBeamformingVector ().size() == 0)
    {
      m_phyIdealBeamformingHelper->AddBeamformingTask (DynamicCast<NrGnbNetDevice> (m_netDevice), ueNetDev);

      BeamSweepTraceParams params;
      params.m_beamSweepOrigin = BeamSweepTraceParams::GNB_CHECK_OMNI_SWEEP;
      m_beamSweepTrace(params);
    }
}

void 
NrGnbPhy::DoDeregisterUeFromRRC (uint64_t imsi)
{
  if (m_deviceMap.size() != 0 && m_ueAttachedImsiMap.size () != 0)
    {
      if (m_ueAttachedImsiMap.find (imsi) != m_ueAttachedImsiMap.end ())
        {
          Ptr<NrUeNetDevice> netDevTBCleared = DynamicCast<NrUeNetDevice> (m_ueAttachedImsiMap[imsi]);
          uint16_t ueCounter = 0;
          for (auto counter = 0; counter < m_deviceMap.size (); counter++)
          {
            if (m_deviceMap[counter]->GetImsi () == netDevTBCleared->GetImsi () &&
                m_deviceMap[counter]->GetNode ()->GetId () == netDevTBCleared->GetNode ()->GetId ())
                {
                  ueCounter = counter;
                  break;
                }
          }
          
          /*m_deviceMap.erase (m_deviceMap.begin () + ueCounter);
          m_ueAttachedImsiMap.erase (imsi);*/
          m_deviceMap.clear ();
          m_ueAttached.clear ();

          m_phySapUser->ForwarIASStateToSched (true);
        }     
    }
}

void 
NrGnbPhy::SetGnbIAState (bool iaPerformed)
{
  IAisPerformed = iaPerformed;
  DynamicCast<ThreeGppSpectrumPropagationLossModel>(m_spectrumPropagationLossModel)->SetBeamSweepState (iaPerformed);
}

Ptr<SpectrumValue>
NrGnbPhy::CalculateRxPsdToUe (Ptr<NrUeNetDevice> ueNetDevice, bool isUpdateSinr)
{
  Ptr<NrUePhy> uePhy;
  // get tx power
  double ueTxPower = 0;
  if (ueNetDevice != 0)
  {
    uePhy = DynamicCast<NrUePhy> (ueNetDevice->GetPhy (0));
    ueTxPower = uePhy->GetTxPower ();
  }
  else
  {
    NS_FATAL_ERROR ("Unrecognized device");
  }
  NS_LOG_LOGIC ("UE Tx power = " << ueTxPower);
  double powerTxW = std::pow (10., (ueTxPower - 30) / 10);
  double txPowerDensity = 0;
  txPowerDensity = (powerTxW /(GetChannelBandwidth ()));
  NS_LOG_LOGIC ("Linear UE Tx power = " << powerTxW);
  NS_LOG_LOGIC ("System bandwidth = " << GetChannelBandwidth ());
  NS_LOG_LOGIC ("txPoswerDensity = " << txPowerDensity);
  // create tx psd
  Ptr<SpectrumValue> txPsd = GetTxPowerSpectralDensity (m_listOfSubchannels);
  // it is the eNB that dictates the conf, m_listOfSubchannels contains all the subch
  NS_LOG_LOGIC ("TxPsd " << *txPsd);

  // get this node and remote node mobility 
  Ptr<MobilityModel> enbMob = m_netDevice->GetNode ()->GetObject<MobilityModel> ();
  NS_LOG_LOGIC ("eNB mobility " << enbMob->GetPosition ());

  Ptr<MobilityModel> ueMob = ueNetDevice->GetNode ()->GetObject<MobilityModel> ();
  NS_LOG_LOGIC ("UE mobility " << ueMob->GetPosition ());

  // compute rx psd
  //testing                                                                                                                   // target not set yet
  // adjuts beamforming of antenna model wrt user
  Ptr<ThreeGppAntennaArrayModel> rxAntennaArray = ConstCast<ThreeGppAntennaArrayModel> (GetSpectrumPhy ()->GetAntennaArray ()); //enbAntenna
  Ptr<ThreeGppAntennaArrayModel> txAntennaArray = ConstCast<ThreeGppAntennaArrayModel> (uePhy->GetSpectrumPhy ()->GetAntennaArray ()); //ueAntenna        // Dl, since the Ul is not actually used (TDD device)

  if (isUpdateSinr)
  {
    if (m_omniFallback && IAisPerformed)
      {
        uePhy->GetBeamManager ()->ChangeToOmniTx ();
        m_beamManager->ChangeToOmniTx ();
      }
      else
      {
        uePhy->GetBeamManager ()->ChangeBeamformingVector (m_netDevice);
        m_beamManager->ChangeBeamformingVector (ueNetDevice);
      }
  }

  double pathLossDb = 0;
  if (txAntennaArray != 0)
  {
    Angles txAngles (enbMob->GetPosition (), ueMob->GetPosition ());
    double txAntennaGain = uePhy->GetBeamManager ()->GetAntennaArrayGainDb (txAngles);
    NS_LOG_LOGIC ("txAntennaGain = " << txAntennaGain << " dB");
    pathLossDb -= txAntennaGain;
  }
  if (rxAntennaArray != 0)
  {
    Angles rxAngles (ueMob->GetPosition (), enbMob->GetPosition ());
    double rxAntennaGain = m_beamManager->GetAntennaArrayGainDb (rxAngles);
    NS_LOG_LOGIC ("rxAntennaGain = " << rxAntennaGain << " dB");
    pathLossDb -= rxAntennaGain;
  }
  if (m_propagationLoss)
  {
    /*if (m_losTracker != 0)               // if I am using the PL propagation model with Aditya's traces
          {
            m_losTracker->UpdateLosNlosState (ueMob,enbMob);                  // update the maps to keep trak of the real PL values, before computing the PL
          }*/
    double propagationGainDb = m_propagationLoss->CalcRxPower (0, ueMob, enbMob);
    NS_LOG_LOGIC ("propagationGainDb = " << propagationGainDb << " dB");
    pathLossDb -= propagationGainDb;
  }
  double pathGainLinear = std::pow (10.0, (-pathLossDb) / 10.0);
  Ptr<SpectrumValue> rxPsd = txPsd->Copy ();
  *(rxPsd) *= pathGainLinear;

  Ptr<ThreeGppSpectrumPropagationLossModel> nr3gpp = DynamicCast<ThreeGppSpectrumPropagationLossModel> (m_spectrumPropagationLossModel);

  if (nr3gpp != 0)
  {
    if (isUpdateSinr)
    {
      NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "\t" << nr3gpp->GetCurrentUePosition()) ;
    }
    rxPsd =  nr3gpp->CalcRxPowerSpectralDensity (rxPsd, enbMob, ueMob);
    NS_LOG_LOGIC ("RxPsd " << *rxPsd);
  }

  if (isUpdateSinr)
  {
    // set back the bf vector to the main eNB
    if (ueNetDevice != 0)
    {
      // testing 
      //if ((ueNetDevice->GetTargetEnb () != m_netDevice) && (ueNetDevice->GetTargetEnb () != 0))
      if (ueNetDevice->GetTargetEnb () != 0)
      {
        uePhy->GetBeamManager()->ChangeBeamformingVector (ueNetDevice->GetTargetEnb());
      }
    }
    else
    {
      NS_FATAL_ERROR ("Unrecognized device");
    }
  }

  return rxPsd;
}

void 
NrGnbPhy::SetCSIRSTimer (bool csiRSTimer)
{
  m_csiTimer = csiRSTimer;
  m_phySapUser->SetMACCSIRSTimer (csiRSTimer);
}

void 
NrGnbPhy::DoUpdateBeamsTBRLM (uint64_t imsi, std::vector<uint16_t> optimalBeamIndexVector, SfnSf startingSfn)
{
  uint16_t actualIndex;
  auto beamIndexVectorSize = optimalBeamIndexVector.size ();

  switch (m_ssbRlmScanScenario)
  {
  case SSBRLMScanScenario::ScanAllRXDirections:
    if(beamIndexVectorSize == m_noOfBeamsTbRLM)
    {
      for (auto beamIterator = 0;beamIterator < beamIndexVectorSize; beamIterator++)
      {
        actualIndex = ((startingSfn.GetSubframe () * startingSfn.GetSlotPerSubframe () * 2)
          + (startingSfn.GetSlot () * 2) + (optimalBeamIndexVector.at(beamIterator) - 1)) % m_beamVectorMapGnb.size ();

        if (m_beamsTbRLM.size () != 0)
        {
          m_beamsTbRLM.at(beamIterator) = m_beamVectorMapGnb.at (actualIndex);
        }
        else
        {
          m_beamsTbRLM.emplace_back (m_beamVectorMapGnb.at (actualIndex));
        }
        
      }
    }
    break;
  case SSBRLMScanScenario::ScanBestCSIRLMDirections:
    if (beamIndexVectorSize == m_noOfBeamsTbRLM - m_noOfBeamsTbReported)
    {
      for (auto beamIterator = 0; beamIterator < beamIndexVectorSize; beamIterator++)
      {

      }
    }
  default:
    break;
  }
}

void 
NrGnbPhy::SetGnbHorizontalAngleStep (double gnbHorizAngleStep)
{
  m_gnbHorizontalAngleStep = gnbHorizAngleStep;
}

double 
NrGnbPhy::GetGnbHorizontalAngleStep () const
{
  return m_gnbHorizontalAngleStep;
}

void
NrGnbPhy::SetGnbVerticalAngleStep (double gnbVerticalAngleStep)
{
  m_gnbElevationAngleStep = gnbVerticalAngleStep;
}

double
NrGnbPhy::GetGnbVerticalAngleStep () const
{
  return m_gnbElevationAngleStep;
}

}
