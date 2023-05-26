#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/nr-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/log.h"
#include <ns3/buildings-module.h>
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/lte-helper.h"
#include "ns3/nr-module.h"

#include <string>

#include <sys/stat.h>

/*
**********************************************************************************************************
*   The following ns-3 code is based on the proposed work in the paper:
*   [1] A. Ichkov, O. Atasoy, P. Mähönen and L. Simić, "Full-Stack ns-3 Framework for the 
*       Evaluation of 5G-NR Beam Management in Non-Standalone Downlink Millimeter-Wave Networks," 
*       in Proc. IEEE WoWMoM 2022, Belfast UK, June 2023. (https://ieeexplore.ieee.org/document/9842765)
*
*   The code implements a full-stack 5G-NR-compliant framework with realistic scheduling, transmission, 
*   and reception of 5G-NR downlink control signals, namely synchronization signal block (SSB) and 
*   channel state information-reference signal (CSI-RS). The framework is based on a non-standalone
*   downlink 5G-NR FR2 (mm-wave) network, enabling control signal transmission via an LTE network 
*   connection for handover coordination and overcoming radio link failure (RLF). 
*   Our framework provides customizable interfaces to a threshold-based beam management operation. 
*   Furthermore, we propose three 5G-NR-compliant beam management strategies, alongside the ideal, 
*   instantaneous beamforming case. The network simulations use site-specific propagation data obtained
*   via 3D ray-tracing and realistic pedestrian mobility data.
*
*   Detailed overview on the ns-3 code structure and functionalities can be found in [1].
*
**********************************************************************************************************
*   The following parameters are the most important for setting up the network simulations:
*     - simTime:      Duration of simulation in seconds
*     - UeID:         Due to the use of realistic walking patterns, for each UE there is
*                     #walkID and #walk-path information, which is stored in /src/nr/model/XXX. 
*                     The walk information, i.e. (X,Y) coordinates and walking speed, are 
*                     loaded during simulation initialization.
*     - raytracing:   Set to true, to use the site-specific ray-tracing data for each gNB, which are 
*                     loaded during simulation initialization. If set to false, the simulations
*                     defaults to the use of the 3GPP statistical channel model, with the channel
*                     data randomly generated at each UE position throughout the walk, i.e. each 250 ms.
*
*   A variety of parameters are introduced to configure the beam management operation, 
*   representing 4 different beam management strategies as proposed in [1] and shown below in Table I. 
*
*     - Strategy 1:     Ideal beamforming/beam scanning and initial access, i.e. no exchange on any 
*                       beam management control messages or resource scheduling. Exhaustive beam search 
*                       of candidate beam pair links (BPLs) during initial access is instantaneous 
*                       (completeSSBDuration=0.0) or set to a fixed value (completeSSBDuration=20.0).
*     - Strategy 2:     Time-frequency resource resource scheduling for transmission and reception
*                       of Synchronization Signal Blocks (SSBs) for the purpose of realistic beam
*                       sweep updates for Initial Access and Radio Link Failure (RLF) recovery.
*     - Strategy 3:     In addition to SSBs, CSI-RS control signals are scheduled for monitoring
*                       of alternative BPLs from the serving gNB to enable faster beam switching
*                       via Radio Link Monitoring (RLM). The monitored CSI-RSs are sent periodically 
*                       by the serving gNB over different BPLs, which are determined after a 
*                       successful SSB beam sweep and thus can become stale over time.
*     - Strategy 4:     Implements an additional update of the monitored CSI-RSs based on the periodic
*                       SSB transmissions sent by the serving gNB. Each connected UE performs beam sweeps
*                       during the periodic SSB transmissions to keep track of long-term channel dynamics. 
*                       The SSB measurement reports are then used to update the monitored CSI-RSs for RLM.
*                       The CSI-RSs, which are transmitted more frequently than the periodic SSBs are thus
*                       used to capture short-term channel dynamics.
*
*
*                       Table I. Beam management strategies and configuration parameters
*                                        _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                       |            Beam management strategy           |
*               Associated variables    |   ideal   |     1     |     2     |     3     |  
*               ------------------------------------------------------------------------|
*               realisticIA             |   false   |   true    |   true    |   true    |   
*               rlmOn                   |   false   |   false   |   true    |   true    |   
*               ssbRlmOn                |   false   |   false   |   false   |   true    |   
*               completeSSBDuration (ms)|   0 / 20  |   -       |   -       |   -       |   
*
*
*   Our beam management operation is based on three distinct signal-to-noise ratio (SNR) thresholds,
*   which dictate the beam management events triggered by directly by the UE or by the coordinator: 
*               1. Radio Link Failure (γ_RLF = -5 dB, via the variable RLFThreshold) 
*               2. Beam Sweep Update  (γ_BU  = 10 dB, via the variable beamSweepThreshold) 
*               3. Maximum achievable data rate (γ_MR = 22.7 dB, via the variable MRThreshold), 
*   
*   The static assignemnt of CSI-RS resources is given as follows (for further details refer to [1]):
*               csiRSPeriodicity = 1
*               csiRSOffset = 3, 
*               maxCSIRSResourcesPerFrame = 2, 
*               noOfBeamsTbRLM = maxCSIRSResourcesPerFrame*2, 
*               ssbRLMTXDirections = 10; // same as number of gNBs
*
**********************************************************************************************************
*/

using namespace ns3;

class MyApp : public Application
{

public:
  MyApp ();
  virtual ~MyApp ();
  void ChangeDataRate (DataRate rate);
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets,
              DataRate dataRate);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket> m_socket;
  Address m_peer;
  uint32_t m_packetSize;
  uint32_t m_nPackets;
  DataRate m_dataRate;
  EventId m_sendEvent;
  bool m_running;
  uint32_t m_packetsSent;
};

MyApp::MyApp ()
    : m_socket (0),
      m_peer (),
      m_packetSize (0),
      m_nPackets (0),
      m_dataRate (0),
      m_sendEvent (),
      m_running (false),
      m_packetsSent (0)
{
}

MyApp::~MyApp ()
{
  m_socket = 0;
}

void
MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets,
              DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
}

void
MyApp::ChangeDataRate (DataRate rate)
{
  m_dataRate = rate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  SendPacket ();
}

void
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void
MyApp::SendPacket (void)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->Send (packet);
  // send packets infinitely, until the end of simulation
  // if (++m_packetsSent < m_nPackets)
    ScheduleTx ();
}

void
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}

// Load ENB details on location and ray-tracing channel data per ENG
void
LoadEnbLocations (Ptr<ListPositionAllocator> enbPositionAlloc)
{
  std::string input_folder = "src/nr/model/Raytracing/";
  std::string enbFile = input_folder + "enb_locations.txt";
  std::ifstream file1;
  file1.open (enbFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "File containing ENB locations not found");

  std::string line;
  std::string token;
  while (std::getline (file1, line))
    {
      doubleVector_t lineElements;
      std::istringstream stream (line);

      while (getline (stream, token, ','))
        {
          double sigma = 0.00;
          std::stringstream stream (token);
          stream >> sigma;
          lineElements.push_back (sigma);
        }
      enbPositionAlloc->Add (
          Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
      NS_LOG_UNCOND (Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
    }
}

static double totalBytesReceived = 0;
static double totalTimeElapsed = 0;
static int numPacketsReceived = 0;
static double totalDelay = 0;

static void
GetThroughput (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon,
               Ptr<OutputStreamWrapper> stream)
{
  flowMon->CheckForLostPackets ();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats ();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier ());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin ();
       stats != flowStats.end (); ++stats)
    {
      Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);

      if (fiveTuple.sourceAddress != "1.0.0.2")
        {
          continue;
        }

      *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t"
                            << (stats->second.rxBytes - totalBytesReceived) * 8.0 /
                                   (stats->second.timeLastRxPacket.GetSeconds () - totalTimeElapsed)
                            << "\t"
                            << (stats->second.delaySum.GetSeconds () - totalDelay) /
                                   (stats->second.rxPackets - numPacketsReceived)
                            << std::endl;
      totalBytesReceived = stats->second.rxBytes;
      totalDelay = stats->second.delaySum.GetSeconds ();
      numPacketsReceived = stats->second.rxPackets;
      totalTimeElapsed = stats->second.timeLastRxPacket.GetSeconds ();
    }

  Simulator::Schedule (Seconds (0.04), &GetThroughput, fmhelper, flowMon, stream);
}

static double totalUdpPacketsReceived = 0.0;
static double totalTimElapsedUdp = 0.0;

static void
GetUdpThroughput (ApplicationContainer appContainer, std::vector<Ptr<OutputStreamWrapper>> vectorOfStream)
{
  for (auto n = 0; n < appContainer.GetN (); n++)
  {
    auto tempReceivedPackets = (double) DynamicCast<UdpServer> (appContainer.Get(n))->GetReceivedBytes ();
    auto lastPacketReceived = DynamicCast<UdpServer> (appContainer.Get(n))->GetLastReceivedTime ().GetSeconds ();
    if (lastPacketReceived > totalTimElapsedUdp)
    {
      *(vectorOfStream.at(n))->GetStream () << Simulator::Now ().GetSeconds () << "\t"
                                        << (tempReceivedPackets - totalUdpPacketsReceived) * 8.0 /
                                           (lastPacketReceived - totalTimElapsedUdp)
                                        << std::endl;
    }
    totalUdpPacketsReceived = tempReceivedPackets;
    totalTimElapsedUdp = lastPacketReceived;
  }

  Simulator::Schedule (Seconds (0.04), &GetUdpThroughput, appContainer, vectorOfStream);
}

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

// IP LATENCY LOGGING
static void
GetIpLatency (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p , Ptr<Ipv4> ipv4, uint32_t interface)
{
  Ptr<Packet> packet = p->Copy ();
  Ipv4Header ipHeader;
  packet->PeekHeader (ipHeader);
  // Logging from the IP header: payloadSize, identification, TOS, TTL, protocol, flags, source IP, dst IP, checksum, headerSize
  IpTimestampTag ipTimestampTag;
  packet->RemovePacketTag (ipTimestampTag);
  // get per packet latency in us.
  uint32_t perPacketLatency = (uint32_t)((Simulator::Now().GetSeconds() - ipTimestampTag.GetIpTimestamp())*1e6); 
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " \t" << ipHeader.GetIdentification() << " \t" << perPacketLatency << std::endl;
}

// UDP LATENCY LOGGING
static void
GetUdpLatency (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy ();
  UdpTimestampTag udpTimestampTag;
  packet->RemovePacketTag (udpTimestampTag);
  // get per packet latency in us.
  uint32_t perPacketLatency = (uint32_t)((Simulator::Now().GetSeconds() - udpTimestampTag.GetUdpTimestamp())*1e6); 
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " \t" << perPacketLatency << std::endl;
}

AsciiTraceHelper asciiTraceHelper;

void SetParameters(); 

/*
*********************************       SIMULATION PARAMETER CONFIGURATION       *********************************************
*/

// UeID = 1:6, representing the 6 user walk paths available for simulations. 
size_t UeID = 3; //  UeID = 3 represents the user walk path from the reference paper [1]
double simTime; 
double walkId;
double ueX;
double ueY;

// Setting the directory for saving the simulation output results
std::string path = "./out/UE" + std::to_string(UeID) + "/"; 

// Set channel data input type:
// "raytracing = true",  use the ray-tracing channel propagation data (as in the reference paper [1]) 
// "raytracing = false", use the 3GPP channel model (generates the channel data randomly from each UE position)
bool raytracing = true;

// Set carrier frequency
// Available ray-tracing channel data includes 28 GHz and 60 GHz traces
std::string frequency = "28GHz"; 

// Set 5G-NR numerology (mu) 
uint32_t num = 3; 

// Set the system bandwidth in Hz
double bandwidth = 400e6; // 400 MHz

/* Set the beam management strategy to be used:
*   ideal = ideal beam scanning / beamforming
*           (option "_a" with no simulation delay, continuous and instantaneous beam scanning at each user positions; 
*           (option "_b" adding simulation delay of 20 ms for each SSB duration);
*   strategy1 = uses only SSB beam sweeping for initial access and RLF recovery, no CSI RLM enabled;
*   strategy2 = uses SSB beam sweeping + CSI-RS RLM (CSI-RS not updated periodically);
*   strategy3 = used SSB beam sweeping + CSI-RS RLM with periodic CSI-RS updates based on SSB transmissions; 
*/
std::string strategy = "strategy3"; 

// Threshold-based beam management operation, as explained in the reference paper [1], based on three SNR thresholds:
//  1. RLFThreshold       -> threshold for detecting radio link failure (outage)
//  2. beamSweepThreshold -> threshold for activating beam sweep update on serving link
//  3. MRThreshold        -> threshold for maximum achievable data rate based on highest supported MCS
double RLFThreshold = -5.0;
double beamSweepThreshold = 10.0;
double MRThreshold = 22.7;

// Set the transmit power (default value is 30 dBm)
// For the ray-tracing based network simulations:
// please note that the ray-tracing simulations were completed with 15 dBm,
// thus the total txPower will be (txPower + 15 dBm)!!!
double txPower = 15.0; // Total txPower = 30 dBm, thus giving you 15 dBm + 15 dBm (ray-tracing simulation)

// Enable IP/UDP per-packet latency logging
bool ipLatencyLogging = true;
bool udpLatencyLogging = true;

// !OLD pre-set settings - DON'T deal with this!
bool adaptiveBF = true; 
bool BFdelay = false;
bool omniFallback = false;
bool realisticIA = false;
bool rlmOn = false;
bool ssbRlmOn = false;
StringValue ssbRlmMode = StringValue ("AllRXDirections");
std::string antennaConfig = "AntennaConfigInets";
        //AntennaConfigDefault: gNB and UE Elevation range is from 60 to 120 degrees.
        //Angular step in Elevation can be adjusted from variables gNBVerticalBeamStep and ueVerticalBeamStep 
        // (Check line 472 & 473)
        //gNB and UE Azimuth range is from 0 to 180 degrees.
        //Angular step in Azimuth cannot be adjusted. They depend on the number of rows in antenna arrays of gNB and UE
        //AntennaConfigInets: gNB Elevation: 90, 120, 150; UE Elevation 30, 60, 90
double gNBVerticalBeamStep, ueVerticalBeamStep, gNBHorizontalBeamStep, ueHorizontalBeamStep;

// Antenna settings (number of rows and number of columns) -> default values: 8x8 (gNB), 4x4 (UE)
uint8_t ueNumRows = 4;
uint8_t ueNumColumns = 4;
uint8_t gNBNumRows = 8;
uint8_t gNBNumColumns = 8;

// SSB, CSI-RS settings (default settings tailored to 5G-NR numerology 3) 
// Please find details on the SSB / CSi-RS configuration in the documentation and the reference paper [1]
uint16_t csiRSPeriodicity = 7;
uint16_t csiRSOffset = 3;
uint16_t maxCSIRSResourcesPerFrame = 8;
uint16_t ssbRLMTXDirections = 10;
Time completeSSBDuration = MilliSeconds (0.0);

/*
***********************************************************************************************************************
*/


int
main (int argc, char *argv[])
{

  if (raytracing)
  {
    txPower = 15.0; // txPower = 30 dBm for ray-tracing channel input
                    // (total txPower setting is 15 dBm + 15 dBm (from the ray-tracing simulations)
  }
  else 
  {
    txPower = 45.0; // txPower for 3GPP channel model input
  }

  if (strategy.find("ideal_a") != std::string::npos)
  {
    realisticIA = rlmOn = ssbRlmOn = false;
    completeSSBDuration = MilliSeconds (0.0);
    adaptiveBF = false; // set this to FALSE for using the default ideal beamforming!
  }
  else if (strategy.find ("ideal_b") != std::string::npos)
  {
    realisticIA = rlmOn = ssbRlmOn = false;
    completeSSBDuration = MilliSeconds (20.0);
    adaptiveBF = false; // set this to FALSE for using the default ideal beamforming!
  }
  else if (strategy.find ("strategy1") != std::string::npos)
  {
    realisticIA = true;
    rlmOn = ssbRlmOn = false;
  }
  else if (strategy.find ("strategy2") != std::string::npos)
  {
    realisticIA = rlmOn = true;
    ssbRlmOn = false;
  }
  else if (strategy.find ("strategy3") != std::string::npos)
  {
    realisticIA = rlmOn = ssbRlmOn = true;
    ssbRlmMode = StringValue ("AllRXDirections");
  }
  else if (strategy.find ("modified") != std::string::npos) // testing purposes
  {
    realisticIA = rlmOn = ssbRlmOn = true;
    ssbRlmMode = StringValue ("BestCSIRLMDirections");
  }
  else
  {
    NS_ABORT_MSG ("Undefined strategy");
  }


  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

	switch(UeID)
	{
		case 1:
			//	UE948
			simTime = 365;
			walkId = 948;
			ueX = 401.15;
			ueY = 528.93;
			break;
		case 2:
			//	UE1121
			simTime = 440;
			walkId = 1121;
			ueX = 217.26;
			ueY = 486.52;
			break;
		case 3:
			//  UE2000
			simTime = 350;  
			walkId = 2000;
			ueX = 457.85;
			ueY = 175.9;
			break;
		case 4:
			//	UE2017
			simTime = 280; 
			walkId = 2017;
			ueX = 574.82;
			ueY = 429.64;
			break;
		case 5:
			//	UE2537
			simTime = 460;
			walkId = 2537;
			ueX = 175.08;
			ueY = 514.83;
			break;
		case 6:
			//	UE2760
			simTime = 480;
			walkId = 2760;
			ueX = 337.08;
			ueY = 324.06;
			break;
		default:
			NS_LOG_UNCOND("UE ID not recognized. ABORTING");
			return 1;
	}

	bool udp = true; // (WORK IN PROGRESS = setting to false for TCP use; TCP not working properly with some beam management strategies)
	bool harqEnabled = true; // HARQ re-transmissions of link layer
	uint32_t packetSize = 1400; // size of transmited packets
	DataRate dataRate = DataRate("1000Mb/s"); // user data rate in Mbps
    std::string option;
    std::string channelModel;

  if (antennaConfig == "AntennaConfigDefault")
  {
    gNBVerticalBeamStep = 10.0;
    ueVerticalBeamStep = 20.0;
  }
  else if (antennaConfig == "AntennaConfigInets")
  {
    gNBVerticalBeamStep = 30.0;
    ueVerticalBeamStep = 30.0;
    gNBHorizontalBeamStep = 9.0;
    ueHorizontalBeamStep = 18.0;
  }
  else
  {
    NS_ABORT_MSG ("Undefined Antenna Configuration");
  }


  if (raytracing)
  {
    channelModel = "Raytracing";
  }
  else
  {
    channelModel = "3GPP";
  }

  double centralFrequencyBand;
  if (frequency == "60GHz")
  {
    centralFrequencyBand = 60e9;
  }
  else if (frequency == "28GHz")
  {
    centralFrequencyBand = 28e9;
  }
  else
  {
    NS_ABORT_MSG ("Undefined frequency");
  }

  if (!realisticIA)
  {
    if (completeSSBDuration == MilliSeconds (0.0))
    {
      option = "a";
    }
    else
    {
      option = "b";
    }

    if (raytracing)
    {
      path = "./out/UE" + std::to_string(UeID) + "_ideal_" + option + channelModel + "_" + antennaConfig + "/";
    }
    else
    {
      path = "./out/UE" + std::to_string(UeID) + "_ideal_" + option + channelModel + "_" + antennaConfig + "/";
    }
  }
  else
  {
    if (!rlmOn)
    {
      path = "./out/UE" + std::to_string(UeID) + "_strat1_" + channelModel + "_" + antennaConfig + "/";
    }
    else
    {
      if (!ssbRlmOn)
      {
        path = "./out/UE" + std::to_string(UeID) + "_strat2_" + channelModel + "_" + antennaConfig + "/";
      }
      else
      {
        if (ssbRlmMode.Get () == "AllRXDirections")
        {
        path = "./out/UE" + std::to_string(UeID) + "_strat3_" +  channelModel + "_" + antennaConfig + "/";
        }
        else
        {
        path = "./out/UE" + std::to_string(UeID) + "_strat3_modified" + option + channelModel + "_" + antennaConfig + "/";
        }
      }
    }
  }

	Time packetInterval (Seconds (packetSize * 8 / static_cast<double> (dataRate.GetBitRate ())));
	mkdir(path.c_str(),S_IRWXU);
	SetParameters();

  //Good way to display what parameters are tried to be set
  NS_LOG_UNCOND ("Path: " << path);
  NS_LOG_UNCOND ("Simulation parameter<s:");
  NS_LOG_UNCOND ("walk Id: " << walkId);
  NS_LOG_UNCOND ("sim Time: " << simTime);
  NS_LOG_UNCOND ("dataRate: " << dataRate);
  NS_LOG_UNCOND ("numerology: " << num);
  NS_LOG_UNCOND ("raytracing: " << raytracing);
  NS_LOG_UNCOND ("bandwidth: " << bandwidth);
  NS_LOG_UNCOND ("adaptiveBF: " << adaptiveBF);
  NS_LOG_UNCOND ("BFdelay: " << BFdelay);

  // setup the Nr simulation
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();

  nrHelper->SetEpcHelper (epcHelper);
  nrHelper->SetIdealBeamformingHelper (idealBeamformingHelper);

  lteHelper->SetEpcHelper (epcHelper);

  //idealBeamformingHelper->Initialize ();
  //nrHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ThreeGppPropagationLossModel"));
  nrHelper->SetAttribute ("ChannelModel", StringValue ("ns3::ThreeGppChannelModel"));
  nrHelper->SetHarqEnabled (harqEnabled);
  nrHelper->SetSchedulerTypeId (NrMacSchedulerTdmaRR::GetTypeId ());
  //nrHelper->Initialize();
  nrHelper->LteChannelModelInitialization ();

  // Position the base stations
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  LoadEnbLocations (enbPositionAlloc);
	NodeContainer gnbNodes;
  NodeContainer ueNodes;
	NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  gnbNodes.Create (enbPositionAlloc->GetSize());
  ueNodes.Create (1);
	lteEnbNodes.Create (1);
  allEnbNodes.Add(gnbNodes);
  allEnbNodes.Add(lteEnbNodes);

  // LTE-eNB is static
  enbPositionAlloc->Add (Vector (300,300,6));

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator(enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  // Position the mobile terminals and enable the mobility
  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (ueX, ueY, 1.5)); // (x, y, z) in m

  //nummCcPerBand is intoduced to generate a bandConf variable, which will in turn be used for ccBwpCreator
  const uint8_t numCcPerBand = 1;

  BandwidthPartInfo::Scenario scenario = BandwidthPartInfo::UMi_StreetCanyon;

  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;

  CcBwpCreator::SimpleOperationBandConf bandConf (centralFrequencyBand,
                                                  bandwidth,
                                                  numCcPerBand,
                                                  scenario);

  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

  nrHelper->InitializeOperationBand(&band);
  allBwps = CcBwpCreator::GetAllBwps({band});

  // Set true to use cell scanning method, false to use the default power method.
  bool enableCellScan = true;
  if (enableCellScan)
  {
    idealBeamformingHelper->SetAttribute ("IdealBeamformingMethod", TypeIdValue (CellScanBeamforming::GetTypeId ()));
  }

  // Core latency
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

  // Antennas for all the UEs
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (ueNumRows));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (ueNumColumns));

  // Antennas for all the gNbs
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (gNBNumRows));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (gNBNumColumns));

  // install Nr net devices
  NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice (gnbNodes, allBwps);
  NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice (ueNodes, allBwps);
  NetDeviceContainer lteNetDev = nrHelper->InstallLteEnbDevice (lteEnbNodes);
  //NetDeviceContainer lteNetDev = lteHelper->InstallEnbDevice (lteEnbNodes);

  // create the internet and install the IP stack on the UEs
  // get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // connect a remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (5)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDev));
  ApplicationContainer serverApps;

  if(udp)
	{
		uint16_t dlPort = 1234;
		ApplicationContainer clientApps;

		for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
		  {
		    Ptr<Node> ueNode = ueNodes.Get (u);
		    // Set the default gateway for the UE
		    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
		    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

		    UdpServerHelper dlPacketSinkHelper (dlPort);
		    serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

		    UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
			dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
		    dlClient.SetAttribute ("Interval", TimeValue (packetInterval));
		    dlClient.SetAttribute ("MaxPackets", UintegerValue(0xFFFFFFFF));
		    clientApps.Add (dlClient.Install (remoteHost));
		  }
		  // start server and client apps
			serverApps.Start(Seconds(0.));
			clientApps.Start(Seconds(0.));
			serverApps.Stop(Seconds(simTime));
			clientApps.Stop(Seconds(simTime-0.2));
	}
	else
	{
		  uint16_t sinkPort = 20000;

		    Ptr<Node> ueNode = ueNodes.Get (0);
		    // Set the default gateway for the UE
		    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
		    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
            Address sinkAddress (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));
            PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));
            ApplicationContainer sinkApps = packetSinkHelper.Install (ueNodes.Get (0));
            Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId ());
            Ptr<MyApp> app = CreateObject<MyApp> ();
            app->Setup (ns3TcpSocket, sinkAddress, 1400, 10000000, dataRate);
            remoteHostContainer.Get (0)->AddApplication (app);
                                        
            Ptr<OutputStreamWrapper> stream5 = asciiTraceHelper.CreateFileStream (path + "tcp-data-newreno.txt");

            sinkApps.Start (Seconds (0.));
            sinkApps.Stop (Seconds (simTime - 1));
            app->SetStartTime (Seconds (0.1));
            app->SetStopTime (Seconds (simTime - 1));
	}

   for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

   for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

	//Connecting all the X2 Interfaces
	for(size_t i=0;i<gnbNodes.GetN()-1;++i){
    for(size_t j=i+1;j<gnbNodes.GetN();++j){
        epcHelper->AddX2Interface(gnbNodes.Get(i), gnbNodes.Get(j));
     }
    }

  Ptr<NetDevice> lteEnbNetDevice = lteEnbNodes.Get (0)->GetDevice (0);
  uint16_t lteCellId = lteEnbNodes.Get(0)->GetDevice (0)->GetObject <LteEnbNetDevice> ()->GetCellId();

  for(size_t i=0; i<gnbNodes.GetN(); i++)
  {
    /*
    if (gnbNodes.Get(i)->GetObject<Ipv4> () == 0)
    {
      internet.Install (gnbNodes.Get(i));
    }
    */
    epcHelper->AddX2Interface(gnbNodes.Get(i), lteEnbNodes.Get(0));
    gnbNodes.Get(i)->GetDevice (0)->GetObject <NrGnbNetDevice> ()->GetRrc ()->SetClosestLteCellId (lteCellId);

    if (gnbNodes.Get(i)->GetDevice (0)->GetObject <NrGnbNetDevice> ()->GetPhy (0)->GetBeamManager () == nullptr)
    {
        NS_LOG_UNCOND ("Beam Manager is not initialized for gnb Node with Id " << gnbNodes.Get(i)->GetId());
    }
  }

  for (size_t i=0; i<ueNodes.GetN (); i++)
  {
    if (ueNodes.Get(i)->GetDevice (0)->GetObject <NrUeNetDevice> ()->GetPhy (0)->GetBeamManager () == nullptr)
    {
      NS_LOG_UNCOND ("Beam Manager is not initialized for UE node with Id " << ueNodes.Get(i)->GetId());
    }
  }


	nrHelper->RayTraceModelLoadData (&band, walkId);
  //nrHelper->RayTraceModelLoadData (ueNetDev, &band, walkId);

  // attach UEs to the closest eNB
  nrHelper->AttachToClosestEnb (ueNetDev, enbNetDev);
  nrHelper->AttachToLteCoordinator (ueNetDev.Get(0)->GetObject <NetDevice> (), lteEnbNetDevice);

  //ueNodes.Get(0)->GetDevice (0)->GetObject <NrUeNetDevice> ()->GetPhy (0)->RegisterToRxSpectrum();
  if (realisticIA)
  {
    for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      for (uint32_t i = 0; i < DynamicCast<NrGnbNetDevice> (*it)->GetCcMapSize (); ++i)
        {
          ueNetDev.Get(0)->GetObject <NrUeNetDevice> ()->GetPhy (i)->SetNumerology (uint16_t(num));
        }
    }


  }

  // enable the traces provided by the module
  nrHelper->EnableTraces();
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  Ptr<OutputStreamWrapper> stream4 = asciiTraceHelper.CreateFileStream (path + "DlIpStats.txt");
  Simulator::Schedule (Seconds (0.3), &GetThroughput, &flowmon, monitor, stream4);

  std::vector<Ptr<OutputStreamWrapper>> streamVector;

  for (auto n = 0; n < serverApps.GetN (); n++)
  {
    Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream (path + "DlUdpStats" + std::to_string (n) + ".txt");
    streamVector.emplace_back (stream);
  }

 // IP latency logging
  for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> ipLatencyStream = asciiTraceHelper.CreateFileStream (path + "DlIpLatency" + std::to_string (imsi) + ".txt");
    
    Ptr<Ipv4> ipv4 = ueNodes.Get(imsi-1)->GetObject<Ipv4> ();
    Ptr<Ipv4L3Protocol> ipv4L3Protocol = ipv4->GetObject<Ipv4L3Protocol> ();
    ipv4L3Protocol->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&GetIpLatency, ipLatencyStream));
  }

  // UDP latency logging
  for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> udpLatencyStream = asciiTraceHelper.CreateFileStream (path + "DlUdpLatency" + std::to_string (imsi) + ".txt");
    
    Ptr<UdpServer> udpServer = DynamicCast<UdpServer> (serverApps.Get(imsi-1));
    udpServer->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&GetUdpLatency, udpLatencyStream));
  }

  Simulator::Schedule (Seconds(0.5), &GetUdpThroughput, serverApps, streamVector);

  // connect custom trace sinks for RRC connection establishment and handover notification
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkUe));

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}

void SetParameters()
{
	//This parameter controls the beamforming periodicity. If 0 then no periodic beamforming.
	int BFperiodicity;
	if(adaptiveBF)
	{
		BFperiodicity = 0;
	}
	else
	{
		BFperiodicity = 250; //if adaptive BF is turned off, trigger BF every 250 ms (on each position update)
	}
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::NrGnbPhy::UpdateSinrEstimatePeriod", DoubleValue (1600.0));
  Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingPeriodicity", TimeValue(MilliSeconds(BFperiodicity)));
  Config::SetDefault ("ns3::NrGnbPhy::IADelay", DoubleValue(5250));
  Config::SetDefault ("ns3::NrGnbPhy::BeamTrainingDelay", DoubleValue(5));
  Config::SetDefault ("ns3::NrGnbPhy::OmniNrFallback", BooleanValue(omniFallback));
  Config::SetDefault ("ns3::NrGnbMac::NumberOfBeamsTbRLM", UintegerValue (8));

  // In the following, set RaySourceType to "Inventory" if Inventory.txt is going to be used. Else, set it to "EnbTraceData"
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::RaySourceType", StringValue("Inventory"));

  //This is to calculate a new channel, not so important with raytracing, but may be crucial for 3gpp-channel
    //Config::SetDefault("ns3::MmWave3gppChannel::UpdatePeriod", TimeValue(MilliSeconds(500)));
	//Config::SetDefault("ns3::MmWave3gppChannel::AdaptiveBeamforming", BooleanValue(adaptiveBF));

  if (antennaConfig == "AntennaConfigDefault")
  {
    Config::SetDefault ("ns3::NrGnbPhy::GnbElevationAngleStep", DoubleValue (gNBVerticalBeamStep));
    Config::SetDefault ("ns3::NrGnbPhy::GnbHorizontalAngleStep", DoubleValue (180.0 / (double)gNBNumRows));

    Config::SetDefault ("ns3::NrUePhy::UeElevationAngleStep", DoubleValue (ueVerticalBeamStep));
    Config::SetDefault ("ns3::NrUePhy::UeHorizontalAngleStep", DoubleValue (180.0 / (double)ueNumRows));
  }
  else
  {
    Config::SetDefault("ns3::NrGnbPhy::GnbElevationAngleStep", DoubleValue(gNBVerticalBeamStep));
    Config::SetDefault("ns3::NrGnbPhy::GnbHorizontalAngleStep", DoubleValue(gNBHorizontalBeamStep));

    Config::SetDefault("ns3::NrUePhy::UeHorizontalAngleStep", DoubleValue (ueHorizontalBeamStep));
    Config::SetDefault("ns3::NrUePhy::UeElevationAngleStep", DoubleValue (ueVerticalBeamStep));
  }


  Config::SetDefault("ns3::NrUePhy::GnbSectionNumber", UintegerValue (63));
  Config::SetDefault("ns3::NrUePhy::CellSelectionCriterion", StringValue ("PeakSnr"));
  Config::SetDefault("ns3::NrUePhy::BeamSweepThreshold", DoubleValue (beamSweepThreshold));
  Config::SetDefault("ns3::NrUePhy::TXSSBScanDirections", UintegerValue (ssbRLMTXDirections));
  Config::SetDefault("ns3::NrPhy::SSBRLMScanScenario", ssbRlmMode);
  Config::SetDefault("ns3::NrPhy::AntennaConfiguration", StringValue(antennaConfig));

  Config::SetDefault("ns3::NrHelper::AdaptiveBeamforming", BooleanValue (adaptiveBF));
  Config::SetDefault("ns3::NrHelper::RealisticIA", BooleanValue (realisticIA));
  Config::SetDefault("ns3::ThreeGppChannelModel::RealisticBeamSweep", BooleanValue (realisticIA));
  Config::SetDefault("ns3::NrHelper::RadioLinkMonitoring", BooleanValue (rlmOn));
  Config::SetDefault("ns3::NrPhy::SSBRLMOn", BooleanValue (ssbRlmOn));


  Config::SetDefault("ns3::NrGnbPhy::ApplyBeamformingDelay", BooleanValue(BFdelay));
  Config::SetDefault ("ns3::NrGnbPhy::Numerology", UintegerValue(num));
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::ChannelTypeRaytracing", BooleanValue(raytracing));
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ChannelTypeRaytracing", BooleanValue (raytracing));

  Config::SetDefault ("ns3::NrUePhy::CompleteTxSweepDuration", TimeValue(completeSSBDuration));
  Config::SetDefault ("ns3::NrUePhy::TxPower", DoubleValue (txPower));
  Config::SetDefault ("ns3::NrUePhy::NoOfCSIRSResourcesPerSSBurst", UintegerValue(maxCSIRSResourcesPerFrame * 2));
  Config::SetDefault ("ns3::NrGnbPhy::TxPower", DoubleValue (txPower));

// Attributes that can be set for the 3GPP channel model
  //Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition", StringValue(condition));
  Config::SetDefault ("ns3::ThreeGppChannelModel::Scenario", StringValue("UMi-StreetCanyon"));
  //Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::OptionalNlos", BooleanValue(optionNlos));
  Config::SetDefault ("ns3::ThreeGppChannelModel::Frequency", DoubleValue(28e9));
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::Frequency", StringValue ("28GHz"));

// Attributes settings for LTE/gNB part
  Config::SetDefault ("ns3::NrGnbPhy::UpdateUeSinrEstimatePeriod", DoubleValue (0));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (200*1024));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue(MicroSeconds(100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity",TimeValue (MilliSeconds (1.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue(RLFThreshold));
  Config::SetDefault ("ns3::LteEnbRrc::CSIRSPeriodicity", UintegerValue (csiRSPeriodicity));
  Config::SetDefault ("ns3::LteEnbRrc::CSIRSOffset", UintegerValue (csiRSOffset));
  Config::SetDefault ("ns3::LteEnbRrc::MaxNumOfCSIRSResource", UintegerValue (maxCSIRSResourcesPerFrame));
  //Config::SetDefault ("ns3::LteEnbRrc::FirstSibTime", UintegerValue (2));

//Realisitc X2 and S1-U links
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDelay", TimeValue (MicroSeconds (5)));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDataRate", DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkMtu", UintegerValue (10000));
  Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue (MicroSeconds (5)));

// Setting TRACE output files:
  Config::SetDefault ("ns3::NrPhyRxTrace::OutputFileName",    StringValue (path + "RxPacketTrace.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::PhyRxThruFileName", StringValue (path + "PhyRxThruData.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::MacRxThruFileName", StringValue (path + "MacRxThruData.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlPdcpOutputFilename", StringValue (path + "DlPdcpStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::UlPdcpOutputFilename", StringValue (path + "UlPdcpStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlPdcpThruFilename",   StringValue (path + "DlPdcpThruStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlRlcOutputFilename",  StringValue (path + "DlRlcStats.txt"));  
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlRlcThruFilename",    StringValue (path + "DlRlcThruStats.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepFileName",               StringValue (path + "BeamSweepTrace.txt"));

  if (!realisticIA)
  {
    Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepType",
                      StringValue ("Ideal"));
  }
  else
  {
    Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepType",
                      StringValue ("Real"));
  }
}
