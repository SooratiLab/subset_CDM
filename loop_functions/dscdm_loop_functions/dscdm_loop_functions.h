#ifndef DSCDM_LOOP_FUNCTIONS_H
#define DSCDM_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CCDMLoopFunctions : public CLoopFunctions {

public:

   CCDMLoopFunctions();
   virtual ~CCDMLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

private:

   UInt32 m_nFeatureItems;
   Real m_fFeatureSquareRadius;
   Real m_fArenaSize;
   CRange<Real> m_cCDMArenaSideX, m_cCDMArenaSideY;
   std::vector<CVector2> m_cFeaturePos;

   std::vector<CVector2> m_AllPos;
   std::vector<CVector2> m_FeaturePos;

   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;
};

#endif
