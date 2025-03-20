/* Include the controller definition */
#include "leader_subset.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

// /****************************************/
// /****************************************/

CFootbotCDM::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootbotCDM::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootbotCDM::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootbotCDM::SStateData::SStateData() :
   IDRange(1, 255),
   ProbRange(0.0f, 1.0f) {}

void CFootbotCDM::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "hop_limit", HopLimit);
      GetNodeAttribute(t_node, "mean_exploration_time", mET);
      GetNodeAttribute(t_node, "mean_dissemination_time", mDT);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void CFootbotCDM::SStateData::Reset() {
   State = STATE_CONSTRUCTION;
}

/****************************************/
/****************************************/

CFootbotCDM::CFootbotCDM() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void CFootbotCDM::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
      m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
      m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot controller for robot \"" << GetId() << "\"", ex);
   }
   /*
    * Initialize other stuff
    */
   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}

/****************************************/
/****************************************/

void CFootbotCDM::ControlStep() {
   
   if(IsLeader){m_pcLEDs->SetAllColors(CColor::YELLOW);}
   else if (!IsPredictor){m_pcLEDs->SetAllColors(CColor::BLACK);}
   else{
      if (m_sStateData.Opinion) {m_pcLEDs->SetAllColors(CColor::GREEN);}
      else {m_pcLEDs->SetAllColors(CColor::RED);}
   }

   //forcing maximum number of hopcount to 30
   if (HopCount>30){HopCount=30;}

   // processing hop information
   if(IsLeader) {
      LeaderTime++;
      HopLeaderID = m_RobotID; HopCount = 1; HopTime = 0;
      m_pcRABA->SetData(2, HopLeaderID);
      // std::cout << "Leader: " << m_RobotID << ", at lt: " << LeaderTime << ", HL: " << m_sStateData.HopLimit << std::endl;

      // uint8_t HopInfo = (HopCount << 3) | ((m_sStateData.HopLimit - 1) & 0x07);
      // m_pcRABA->SetData(3, HopInfo);
      m_pcRABA->SetData(3, HopCount);
      m_pcRABA->SetData(4, HopTime);
      m_pcRABA->SetData(5, m_sStateData.HopLimit);

      uint8_t lt1 = static_cast<uint8_t>(LeaderTime >> 8);  // Extract the higher 8 bits
      uint8_t lt2 = static_cast<uint8_t>(LeaderTime & 0xFF);  // Extract the lower 8 bits
      m_pcRABA->SetData(6, lt1);
      m_pcRABA->SetData(7, lt2);
   }
   else if (HopLeaderID!=0 && HopTime < 50) {
      // LeaderTime = LeaderTime + HopTime;
      HopTime++; LeaderTime++;
      m_pcRABA->SetData(2, HopLeaderID);
      
      // uint8_t HopInfo = ((HopCount+1) << 3) | ((m_sStateData.HopLimit - 1) & 0x07);
      // m_pcRABA->SetData(3, HopInfo);
      m_pcRABA->SetData(3, HopCount+1);
      m_pcRABA->SetData(4, HopTime);
      m_pcRABA->SetData(5, m_sStateData.HopLimit);

      uint8_t lt1 = static_cast<uint8_t>(LeaderTime >> 8);  // Extract the higher 8 bits
      uint8_t lt2 = static_cast<uint8_t>(LeaderTime & 0xFF);  // Extract the lower 8 bits
      m_pcRABA->SetData(6, lt1);
      m_pcRABA->SetData(7, lt2);
   }
   else {
      HopLeaderID = 0; HopCount = 0; HopTime = 0;
      m_pcRABA->SetData(0, 0);
      m_pcRABA->SetData(1, 0);
      m_pcRABA->SetData(2, 0);
      m_pcRABA->SetData(3, 0);
      m_pcRABA->SetData(4, 0);
      m_pcRABA->SetData(5, 0);
      m_pcRABA->SetData(6, 0);
      m_pcRABA->SetData(7, 0);
      }

   // Robots is predictor only if it is a leader or hopcount<limit
   // For alternating hopcount, add mod 2
   // if ((HopCount >= 1) && (HopCount <= m_sStateData.HopLimit) && (HopCount%2 == 0)){IsPredictor = true;} // For alternating hopcount
   //------------------------
   if (RoleTime < RoleTH){RoleTime++;}
   else{
      if ((HopCount >= 1) && (HopCount <= m_sStateData.HopLimit)){IsPredictor = true;}
      else{
         IsPredictor = false;
         //clear beacon
         m_pcRABA->SetData(0, 0);
         m_pcRABA->SetData(1, 0);
      }
      RoleTime = 0;
      RoleTH = static_cast<argos::UInt32>(m_pcRNG->Exponential(50));
   }
   


   //read ground sensor
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

   // check all four sensors, if more than two detect black, isTask true else false. If same, pick random.
   if (IsPredictor){
      bool IsTask;
      uint8_t trueCount = 0;

      for(int i = 0; i < 4; ++i) 
         trueCount += (tGroundReads[i].Value < 0.5f);

      // std::cout << "bsensor" << trueCount << std::endl;
      // Determine IsTask based on the count of true conditions
      if(trueCount > 2) {IsTask = true;} 
      else if(trueCount < 2) {IsTask = false;} 
      else {  // same count, choose randomly
         IsTask = (m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.5f);
      }
 

      if (IsTask) {mCounter++;}
      else {nCounter++;}
      // }
   }

   bool IsRobot = false;

   //read data from other robots, keep opinion and observation
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   for(size_t i = 0; i < tPackets.size(); ++i) {
      Real range = tPackets[i].Range;        // Distance to detected robot
      //add noise
      if (range < 100){ //communication range
         // Hear opinion from neighbor when in dissemination state, and neigbor sending data (ID is not 0)
         if(!m_sStateData.ExpState && IsPredictor && (tPackets[i].Data[0] != 0)){
            bool found = false;
            uint8_t j;
            // Iterate through the array to find if x is already present
            for (j = 0; j <= 1; ++j) {
               if (OpinionList[j][0] == tPackets[i].Data[0]) {
                     // If x is found, update the corresponding value in the second column
                     OpinionList[j][1] = tPackets[i].Data[1];
                     found = true;
                     break;
               }
            }
            if (!found) { //add new opinion
               OpinionList[nOpList][0] = tPackets[i].Data[0];
               OpinionList[nOpList][1] = tPackets[i].Data[1];
               nOpList++;

               if(nOpList>1){nOpList=0;}
            }

            // Update leader opinion list 
            if (IsLeader)
            {
               bool Lfound = false;
               uint8_t jl;
               // Iterate through the array to find if x is already present
               if (nLOpList > 0){
                  for (j = 0; j < nLOpList; ++j) {
                     if (LOpList[j][0] == tPackets[i].Data[0]) {
                           // If x is found, update the corresponding value in the second column
                           LOpList[j][1] = tPackets[i].Data[1];
                           Lfound = true;
                           break;
                     }
                  }
               }
               if (!Lfound) { //add new opinion
                  LOpList[nLOpList][0] = tPackets[i].Data[0];
                  LOpList[nLOpList][1] = tPackets[i].Data[1];
                  nLOpList++;
                  if (nLOpList>20){nLOpList = 20;}
               }


            }
         }

    
         //adapting hopcount
         if (tPackets[i].Data[2] != 0){ //hear other robot connected
            //catch leadertime
            uint8_t reclt1 = tPackets[i].Data[6];
            uint8_t reclt2 = tPackets[i].Data[7];
            uint16_t newlt = (static_cast<uint16_t>(reclt1) << 8) | reclt2;

            //catch hopcount and hoplimit
            uint8_t RecHopCount = tPackets[i].Data[3];
            size_t RecHopLimit = tPackets[i].Data[5];


            if (!IsLeader){ //leader doesnt need to update hopcount
               if (HopLeaderID == 0 || HopTime >= 50){
                  HopLeaderID = tPackets[i].Data[2]; 
                  HopCount = RecHopCount; m_sStateData.HopLimit = RecHopLimit;
                  HopTime = tPackets[i].Data[4]; LeaderTime = newlt;}
               else { //robot already connected, only update if hopcount is smaller
                     if (newlt > LeaderTime){
                        HopLeaderID = tPackets[i].Data[2]; 
                        HopCount = RecHopCount; m_sStateData.HopLimit = RecHopLimit;
                        HopTime = tPackets[i].Data[4]; LeaderTime = newlt;}
                     else if (newlt == LeaderTime){
                        if (HopLeaderID < tPackets[i].Data[2]){
                           HopLeaderID = tPackets[i].Data[2]; 
                           HopCount = RecHopCount; m_sStateData.HopLimit = RecHopLimit;
                           HopTime = tPackets[i].Data[4]; LeaderTime = newlt;   
                        }}
                     else if (HopCount > RecHopCount){
                        HopLeaderID = tPackets[i].Data[2]; 
                        HopCount = RecHopCount; m_sStateData.HopLimit = RecHopLimit;
                        HopTime = tPackets[i].Data[4]; LeaderTime = newlt;}
                     else if ((HopCount == RecHopCount) && (HopTime > tPackets[i].Data[4]+1)) {
                        HopLeaderID = tPackets[i].Data[2]; 
                        HopCount = RecHopCount; m_sStateData.HopLimit = RecHopLimit;
                        HopTime = tPackets[i].Data[4]; LeaderTime = newlt;}
               }
            }
         }
      }
   }


   // For leader: check the Leader opinion list, increase hopcount limit if 
   // - decision is made
   // - time is up
   if (IsLeader && !IsHopIdentified)
   {
      HopLimTime++;
      if (HopLimTime > 1500){
         // reset decision, increment hoplimit
         // Decision[0] = 9;
         m_sStateData.HopLimit++;
         HopLimTime = 0; ConsistentTime = 0;
         ResetLeaderOpinionList();
      }
      else if (nLOpList>10){//list has enough data
         // count the opinion
         uint8_t count0 = 0;
         uint8_t stepdec = 9;
         for (int i = 0; i < nLOpList; ++i) {
            if (LOpList[i][1] == 0) {count0++;}
         }
         if (m_sStateData.Opinion==0){count0++;}
         //set decision if one opinion above threshold
         Real PercentageZero = static_cast<Real>(count0) / (nLOpList+1);
         if (PercentageZero > 0.75) {stepdec = 0;}
         else if (PercentageZero < 0.25) {stepdec = 1;}

         //check if the decision is consistent
         if ((stepdec !=9) && (stepdec == prevstepdec)){ConsistentTime++;}
         else {ConsistentTime = 0;}
         prevstepdec = stepdec;

         //Record decision
         if (ConsistentTime > 100){
            if (stepdec != Decision[0]){//reset all decision
               //init decision, increment hoplimit
               Decision[0] = stepdec;
               m_sStateData.HopLimit++;
               HopLimTime = 0; ConsistentTime = 0;
            }
            else { 
               // done, exit or wait until timestep finish
               IsHopIdentified = true;
            }
            ResetLeaderOpinionList();

         }
      }
   }

   RandomWalk();
   
   if (IsPredictor){
      m_sStateData.StateCounter++;
      if (m_sStateData.ExpState){
         //clear beacon
         m_pcRABA->SetData(0, 0);
         m_pcRABA->SetData(1, 0);
         if (m_sStateData.StateCounter > m_sStateData.ExploreTime){
            //end of exploration, calculate quality and decide dissemination time
            Real Quality;
            Real b=1;
            if (mCounter==0){mCounter=1;} //assume at least 1 task and 1 robots in environment
            if (nCounter==0){nCounter=1;}
            if(m_sStateData.Opinion){ //feasible, more robots
                  Quality = static_cast<Real>(nCounter)/(nCounter+(b*mCounter));
            }
            else {Quality = static_cast<Real>((b*mCounter))/(nCounter+(b*mCounter));}
            m_sStateData.DisseminateTime = static_cast<argos::UInt32>(m_pcRNG->Exponential(m_sStateData.mDT*Quality)); //(400*Quality);//
            m_sStateData.StateCounter=0; mCounter=0; nCounter=0;
            
            //change state
            m_sStateData.ExpState = false;
         }
      }
      else{ //dissemination state
         //send ID and opinion
         m_pcRABA->SetData(0, m_RobotID);
         m_pcRABA->SetData(1, m_sStateData.Opinion);
         if (m_sStateData.StateCounter > m_sStateData.DisseminateTime){
            //end of dissemination, decide opinion
            // count opinion
            // add own opinion to OpinionList
            OpinionList[2][0] = m_RobotID; //OpinionList[nOpList][0] = m_RobotID;
            OpinionList[2][1] = m_sStateData.Opinion; //OpinionList[nOpList][1] = m_sStateData.Opinion;
            nOpList++;
            
            //DMVD
            if (OpinionList[0][0]!=0 && OpinionList[1][0]!=0) {
               uint32_t selector = (m_pcRNG->Uniform(argos::CRange<argos::UInt32>(0, 3)));
               m_sStateData.Opinion = OpinionList[selector][1];
            }
            else if (OpinionList[0][0]!=0) {
               if (OpinionList[0][1]!=OpinionList[2][1]){m_sStateData.Opinion=(m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.5f);}
            }

            //reset 
            UInt8 i;
            for (i = 0; i < 10; ++i) {
               for (std::size_t j = 0; j < 2; ++j) {
                     OpinionList[i][j] = 0;  // Initialize each element to 0
               }
            }
            nOpList = 0;
            m_sStateData.StateCounter=0;
            m_sStateData.ExploreTime = static_cast<argos::UInt32>(m_pcRNG->Exponential(m_sStateData.mET)); //200;//
            m_sStateData.ExpState = true;
         }
      }
   } 
}

/****************************************/
/****************************************/

void CFootbotCDM::Reset() {

   m_sStateData.Reset();

   m_pcRABA->ClearData();


      /* Initialize RobotID with a random number between 0 and 255 */
   m_RobotID = static_cast<argos::UInt8>(m_pcRNG->Uniform(m_sStateData.IDRange));

   m_sStateData.RWTime =(m_pcRNG->Uniform(argos::CRange<argos::UInt32>(0, 50)));

   m_sStateData.Opinion = false;//(m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.5f);

   m_sStateData.RWStateRot = true;
   m_sStateData.RWCounter = 0;

   nCounter = 0; mCounter = 0;

   UInt8 i;
   for (i = 0; i < 10; ++i) {
        for (std::size_t j = 0; j < 2; ++j) {
            OpinionList[i][j] = 0;  // Initialize each element to 0
        }
   }
   nOpList = 0;

   for (i = 0; i < 100; ++i) {
        for (std::size_t j = 0; j < 2; ++j) {
            LOpList[i][j] = 0;  // Initialize each element to 0
        }
   }
   nLOpList = 0;

   // Init exploration and dissemination
   m_sStateData.ExploreTime = static_cast<argos::UInt32>(m_pcRNG->Exponential(m_sStateData.mET)); //200;//
   m_sStateData.DisseminateTime = 0;
   m_sStateData.StateCounter = 0;
   m_sStateData.ExpState = true;

   IsLeader = false;
   LeaderID = 0;
   HopLeaderID = 0;
   HopCount = 0;
   HopTime = 0;
   IsPredictor = false;
   SenseTime = 0;
   dcCounter = 0; //(m_pcRNG->Uniform(argos::CRange<argos::UInt32>(0, 50)));

   RoleTime = 0;
   RoleTH = static_cast<argos::UInt32>(m_pcRNG->Exponential(50));

   m_sStateData.RobotOff = false;
   FaultTime = 3000;//static_cast<argos::UInt32>(m_pcRNG->Exponential(500)); //3000;//
   

   HopLimTime = 0;
   ConsistentTime = 0;
   for (i = 0; i < 2; ++i) {
        Decision[i] = 9;
   }
   prevstepdec = 9;

}

void CFootbotCDM::SetOpinionWhite(){
   m_sStateData.Opinion = true;
}

void CFootbotCDM::SetRobotOff(){
   m_sStateData.RobotOff = true;
}

void CFootbotCDM::SetLeader(){
   IsLeader = true;
}

/****************************************/
/****************************************/
void CFootbotCDM::RandomWalk(){
   //Random Walk consist of two state: rotate and go straight. 

   if (m_sStateData.RWStateRot){
      if (m_sStateData.RWCounter < m_sStateData.RWTime){
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * CVector2(-1,0)); //rotating
         m_sStateData.RWCounter++;
      }
      else {
         m_sStateData.RWStateRot = false;
         m_sStateData.RWCounter = 0;
         m_sStateData.RWTime = static_cast<argos::UInt32>(m_pcRNG->Exponential(400));
         bool bCollision;
         CVector2 cDiffusion = DiffusionVector(bCollision);
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
         // std::cout << "change to straight " << m_sStateData.RWTime << std::endl;
      }
   }
   else{
      if (m_sStateData.RWCounter < m_sStateData.RWTime){
         bool bCollision;
         CVector2 cDiffusion = DiffusionVector(bCollision);
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
         m_sStateData.RWCounter++;
      }
      else {
         m_sStateData.RWStateRot = true;
         m_sStateData.RWCounter = 0;
         m_sStateData.RWTime = (m_pcRNG->Uniform(argos::CRange<argos::UInt32>(0, 45)));
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * CVector2(-1,0)); 
         // std::cout << "change to rot " << m_sStateData.RWTime << std::endl;
      }
   }
}

void CFootbotCDM::ResetLeaderOpinionList(){
   for (uint8_t i = 0; i < 100; ++i) {
         for (std::size_t j = 0; j < 2; ++j) {
               LOpList[i][j] = 0;  // Initialize each element to 0
         }
      }
      nLOpList = 0;
}

/****************************************/
/****************************************/

CVector2 CFootbotCDM::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

/****************************************/
/****************************************/

void CFootbotCDM::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}


REGISTER_CONTROLLER(CFootbotCDM, "leader_subset_controller")
