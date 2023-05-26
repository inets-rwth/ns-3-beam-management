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

#include "nr-eesm-ir.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrEesmIr");
NS_OBJECT_ENSURE_REGISTERED (NrEesmIr);

NrEesmIr::NrEesmIr()
{
  NS_LOG_FUNCTION (this);
}

NrEesmIr::~NrEesmIr ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
NrEesmIr::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrEesmIr")
    .SetParent<NrEesmErrorModel> ()
  ;
  return tid;
}

double
NrEesmIr::ComputeSINR (const SpectrumValue &sinr, const std::vector<int> &map,
                         uint8_t mcs, uint32_t sizeBit,
                         const NrErrorModel::NrErrorModelHistory &sinrHistory) const
{
  NS_LOG_FUNCTION (this);
  // HARQ INCREMENTAL REDUNDANCY: update SINReff and ECR after retx
  // no repetition of coded bits

  std::vector<int> map_sum = map;

  // evaluate SINR_eff over "total", as per Incremental Redundancy.
  // combine at the bit level.
  SpectrumValue sinr_sum = sinr;
  double SINReff_previousTx = DynamicCast<NrEesmErrorModelOutput> (sinrHistory.back ())->m_sinrEff;
  NS_LOG_INFO ("\tHISTORY:");
  NS_LOG_INFO ("\tSINReff: " << SINReff_previousTx);

  for (uint32_t i = 0; i < map.size (); i++)
     {
        sinr_sum[map.at(i)] += SINReff_previousTx;
     }

  NS_LOG_INFO ("MAP_SUM: " << PrintMap (map_sum));
  NS_LOG_INFO ("SINR_SUM: " << sinr_sum);

  // compute equivalent effective code rate after retransmissions
  uint32_t codeBitsSum = 0;
  uint32_t infoBits = DynamicCast<NrEesmErrorModelOutput> (sinrHistory.front ())->m_infoBits;  // information bits of the first TB

  for (const Ptr<NrErrorModelOutput> & output : sinrHistory)
    {
      Ptr<NrEesmErrorModelOutput> sinrHistorytemp = DynamicCast<NrEesmErrorModelOutput> (output);
      NS_ASSERT (sinrHistorytemp != nullptr);

      NS_LOG_DEBUG (" Effective SINR " << sinrHistorytemp->m_sinrEff <<
                    " codeBits " << sinrHistorytemp->m_codeBits <<
                    " infoBits: " << sinrHistorytemp->m_infoBits);

      codeBitsSum += sinrHistorytemp->m_codeBits;
    }

  codeBitsSum += sizeBit / GetMcsEcrTable()->at (mcs);;
  const_cast<NrEesmIr*> (this)->m_Reff = infoBits / static_cast<double> (codeBitsSum);

  NS_LOG_INFO (" Reff " << m_Reff << " HARQ history (previous) " << sinrHistory.size ());

  // compute effective SINR with the sinr_sum vector and map_sum RB map
  return SinrEff (sinr_sum, map_sum, mcs);
}

double
NrEesmIr::GetMcsEq (uint8_t mcsTx) const
{
  NS_LOG_FUNCTION (this);
  // PHY abstraction for HARQ-IR retx -> get closest ECR to Reff from the
  // available ones that belong to the same modulation order

  uint8_t mcs_eq = mcsTx;

  uint8_t ModOrder = GetMcsMTable ()->at (mcsTx);

  NS_LOG_INFO (" Modulation order: " << +ModOrder );
  NS_LOG_INFO (" Reff: " << m_Reff );

  for (uint8_t mcsindex = (mcsTx-1); mcsindex != 255; mcsindex--)
    // search from MCS=mcs-1 to MCS=0. end at 255 to account for wrap around of uint
    {
      if ((GetMcsMTable ()->at (mcsindex) == ModOrder) &&
          (GetMcsEcrTable ()->at (mcsindex) > m_Reff))
        {
          mcs_eq--;
        }
    }

  return mcs_eq;
}

} // namespace ns3
