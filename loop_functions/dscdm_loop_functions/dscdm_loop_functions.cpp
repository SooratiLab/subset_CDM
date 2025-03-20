#include "dscdm_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/dist_subset/dist_subset.h>

/****************************************/
/****************************************/
CCDMLoopFunctions::CCDMLoopFunctions() :
   m_cCDMArenaSideX(-0.9f, 1.7f),
   m_cCDMArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL),
   m_pcRNG(NULL) {
}

/****************************************/
/****************************************/
void CCDMLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tCDM = GetNode(t_node, "CDM");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      
      /* Retrieve parameters from XML configuration */
      GetNodeAttribute(tCDM, "items", m_nFeatureItems);
      GetNodeAttribute(tCDM, "radius", m_fFeatureSquareRadius);
      GetNodeAttribute(tCDM, "arenasize", m_fArenaSize);
      
      /* Create a new random number generator */
      m_pcRNG = CRandom::CreateRNG("argos");
      
      /* Generate possible item positions in the environment */
      float start = -1 * (m_fArenaSize / 2) + m_fFeatureSquareRadius;
      float end = (m_fArenaSize / 2) + m_fFeatureSquareRadius;
      float step = m_fFeatureSquareRadius * 2;
      
      for (float x = start; x <= end; x += step) {
         for (float y = start; y <= end; y += step) {
               m_AllPos.push_back(CVector2(x, y));
         }
      }
      
      /* Shuffle positions and select feature items */
      m_pcRNG->Shuffle(m_AllPos);
      for (UInt32 i = 0; i < m_nFeatureItems; ++i) {
         m_cFeaturePos.push_back(m_AllPos[i]);
      }
      
      /* Retrieve output file name from XML and open the file */
      GetNodeAttribute(tCDM, "output", m_strOutput);
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::out | std::ios_base::trunc);
      
   } catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/
void CCDMLoopFunctions::Reset() {
   /* Close and reopen output file */
   m_cOutput.close();
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::out | std::ios_base::trunc);
 
   /* Redistribute feature items */
   m_cFeaturePos.clear();
   m_pcRNG->Shuffle(m_AllPos);
   for (UInt32 i = 0; i < m_nFeatureItems; ++i) {
      m_cFeaturePos.push_back(m_AllPos[i]);
   }
}

/****************************************/
/****************************************/
void CCDMLoopFunctions::Destroy() {
   m_cOutput.close();
}

/****************************************/
/****************************************/
CColor CCDMLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   for (UInt32 i = 0; i < m_cFeaturePos.size(); ++i) {
      float halfSideLength = m_fFeatureSquareRadius;
      CVector2 diff = c_position_on_plane - m_cFeaturePos[i];
      
      if (fabs(diff.GetX()) <= halfSideLength && fabs(diff.GetY()) <= halfSideLength) {
         return CColor::BLACK; // Inside feature item area
      }
   }
   return CColor::WHITE; // Default floor color
}

/****************************************/
/****************************************/
void CCDMLoopFunctions::PreStep() {
   UInt32 nWhite = 0; // Number of predictor robots with white opinion
   UInt32 nDM = 0;    // Total number of decision-making robots
   UInt32 pWhite = 0; // Percentage of predictor robots with white opinion
   UInt32 nr = 0;
   bool NoLeader = true;

   /* Retrieve all foot-bot entities */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootbotCDM& cController = dynamic_cast<CFootbotCDM&>(cFootBot.GetControllableEntity().GetController());

      /* Initialize opinions at the first step */
      if (GetSpace().GetSimulationClock() == 1) {
         if (nr % 2 == 0) { cController.SetOpinionWhite(); }
         nr++;
      }

      /* Count decision-making robots and those with a white opinion */
      if (cController.IsDMRobot()) {
         ++nDM;
         if (cController.IsOpinionWhite()) ++nWhite;
      }
   }

   /* Calculate percentage of predictor robots with white opinion */
   pWhite = (nDM != 0) ? static_cast<UInt32>(static_cast<Real>(100) * nWhite / nDM) : 0;
   
   /* Write data to output file */
   m_cOutput << pWhite << "\t" << nDM << std::endl;
}

/****************************************/
/* Register loop functions */
/****************************************/
REGISTER_LOOP_FUNCTIONS(CCDMLoopFunctions, "dscdm_loop_functions")
