/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab
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
 * Authors: Nicola Baldo <nbaldo@cttc.es>
 *          Lluis Parcerisa <lparcerisa@cttc.cat>
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *          Dual Connectivity functionalities
 *
 * Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *								 Integration of Carrier Aggregation
 */

#include <ns3/fatal-error.h>
#include <ns3/log.h>
#include <ns3/nstime.h>
#include <ns3/node-list.h>
#include <ns3/node.h>
#include <ns3/simulator.h>

#include <ns3/nr-rrc-protocol-real.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/nr-ue-net-device.h>

namespace ns3{;
    
NS_LOG_COMPONENT_DEFINE ("NrRrcProtocolReal");

const Time RRC_REAL_MSG_TIME_DELAY = MicroSeconds (500);

NS_OBJECT_ENSURE_REGISTERED (NrUeRrcProtocolReal);

NrUeRrcProtocolReal::NrUeRrcProtocolReal ()
    : m_ueRrcSapProvider (0),
    m_enbRrcSapProvider (0)
{
    m_ueRrcSapUser = new MemberLteUeRrcSapUser<NrUeRrcProtocolReal> (this);
    m_completeSetupParameters.srb0SapUser = new LteRlcSpecificLteRlcSapUser<NrUeRrcProtocolReal> (this);
    m_completeSetupParameters.srb1SapUser = new LtePdcpSpecificLtePdcpSapUser<NrUeRrcProtocolReal> (this);
}

NrUeRrcProtocolReal::~NrUeRrcProtocolReal ()
{
}

void 
NrUeRrcProtocolReal::DoDispose ()
{
    NS_LOG_FUNCTION (this);
    delete m_ueRrcSapUser;
    delete m_completeSetupParameters.srb0SapUser;
    delete m_completeSetupParameters.srb1SapUser;
    m_rrc = 0;
}

TypeId
NrUeRrcProtocolReal::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrUeRrcProtocolReal")
    .SetParent<Object> ()
    .SetGroupName ("Nr")
    .AddConstructor<NrUeRrcProtocolReal> ()
  ;
  return tid;
}
}