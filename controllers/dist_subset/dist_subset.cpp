/* Include the controller definition */
#include "dist_subset.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

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

   if (m_sStateData.RobotOff){
      SetWheelSpeedsFromVector(CVector2(0.0f, 0.0f));
      IsPredictor=false;
      m_pcRABA-> ClearData();
      m_pcLEDs->SetAllColors(CColor::YELLOW);
   }
   else{
   //--------------------------------

 
   if (!IsPredictor){m_pcLEDs->SetAllColors(CColor::BLACK);} //disconnected
   else{
      if (m_sStateData.Opinion) {m_pcLEDs->SetAllColors(CColor::GREEN);}
      else {m_pcLEDs->SetAllColors(CColor::RED);}
   }

   // Robot become predictor with probability 0.1*hoplimit during roletime
   if (RoleTime < RoleTH){RoleTime++;}
   else{
      IsPredictor = (m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.1f*m_sStateData.HopLimit);
      RoleTime = 0;
      RoleTH = static_cast<argos::UInt32>(m_pcRNG->Exponential(100));
   }

   //announce predictor status
   m_pcRABA->SetData(8, IsPredictor);

   if (!IsPredictor){
      m_pcRABA->SetData(0, 0);
      m_pcRABA->SetData(1, 0);
   }

   // Update hoplimit based on convergence certainty
   float cvalueTH[] = {0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55}; // Thresholds for each HopLimit

   if (m_sStateData.ConvergenceCertainty < cvalueTH[m_sStateData.HopLimit - 1]) {
      ccTime--;
      if (ccTime <= 0) {
            m_sStateData.HopLimit++;
            ccTime = 500; // Reset ccTime after transition
      }
   } else {
      ccTime = 500; // Reset ccTime if condition is not met
   }



   // broadcast relayed info


   uint8_t newRow = 0;  // Tracks the next row to move valid entries

   // Sorting the relay memory. Maximum relay size:8
   // Relay structure:
   // RelayOpList[idx][0]: ID, RelayOpList[idx][1]: opinion, RelayOpList[idx][2]: Timer
   // Iterate through each row to filter and move rows that is no longer valid
   for (int i = 0; i < nRelay; ++i) {
      if ((RelayOpList[i][0]!=0) && (RelayOpList[i][2] <= RelayTH)) {  // If third column value is 100 or less
            // Move this row to the current newRow position
            RelayOpList[newRow][0] = RelayOpList[i][0];
            RelayOpList[newRow][1] = RelayOpList[i][1];
            RelayOpList[newRow][2] = RelayOpList[i][2];
            newRow++;  // Move to the next row for valid entries
      }
   }

   // Set remaining rows to 0
   for (int i = newRow; i < 8; ++i) {
      RelayOpList[i][0] = 0;
      RelayOpList[i][1] = 0;
      RelayOpList[i][2] = 0;
   }

   nROpList = newRow;


   // Broadcasting relay memory. Only broadcast every t_broadcast
   if (t_Broadcast<=0){
      t_Broadcast=0;
      uint8_t combinedOp = 0;
      // std::cout << "nROp: " << nROpList << std::endl;
      for (int k = 0; k < 8; ++k) {
         combinedOp |= (RelayOpList[k][1] & 1) << k;
         if (RelayOpList[k][0]!=0){RelayOpList[k][2]++;}
      }
      m_pcRABA->SetData(10, combinedOp);
      m_pcRABA->SetData(11, RelayOpList[0][0]);
      m_pcRABA->SetData(12, RelayOpList[1][0]);
      m_pcRABA->SetData(13, RelayOpList[2][0]);
      m_pcRABA->SetData(14, RelayOpList[3][0]);
      m_pcRABA->SetData(15, RelayOpList[4][0]);
      m_pcRABA->SetData(16, RelayOpList[5][0]);
      m_pcRABA->SetData(17, RelayOpList[6][0]);
      m_pcRABA->SetData(18, RelayOpList[7][0]);

      m_pcRABA->SetData(21, RelayOpList[0][2]);
      m_pcRABA->SetData(22, RelayOpList[1][2]);
      m_pcRABA->SetData(23, RelayOpList[2][2]);
      m_pcRABA->SetData(24, RelayOpList[3][2]);
      m_pcRABA->SetData(25, RelayOpList[4][2]);
      m_pcRABA->SetData(26, RelayOpList[5][2]);
      m_pcRABA->SetData(27, RelayOpList[6][2]);
      m_pcRABA->SetData(28, RelayOpList[7][2]);
   }
   else{ 
      t_Broadcast--; 
      if (!IsPredictor){m_pcRABA-> ClearData();}
   }

   //read ground sensor
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

   // For CP: check all four sensors, if more than two detect black, isTask true else false. If same, pick random.
   if (IsPredictor){
      bool IsTask;
      uint8_t trueCount = 0;

      for(int i = 0; i < 4; ++i) 
         trueCount += (tGroundReads[i].Value < 0.5f);

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
   bool IsHopUpdated = false;

   // // Method 2: non predictor robot act as relay
   uint8_t relayedID = 0;
   uint8_t relayedOp = 0;
   //-----------------------------

   //read data from other robots, keep opinion and observation
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   for(size_t i = 0; i < tPackets.size(); ++i) {
      Real range = tPackets[i].Range;        // Distance to detected robot
      if (range < 100 ){ 
         // if(range<26.7f){IsRobot=true;} //nCounter++;}
         
         // hear opinion from relay
         if(!m_sStateData.ExpState && IsPredictor && (tPackets[i].Data[8] == 0)){
            for (size_t idx = 0; idx < 8; ++idx) {
                  uint8_t rec_ID = tPackets[i].Data[11+idx];
                  if (rec_ID !=0 && rec_ID !=m_RobotID){
                     bool found = false;
                     uint8_t j;
                     // Iterate through the array to find if x is already present
                     for (j = 0; j <= 1; ++j) {
                        if (OpinionList[j][0] == rec_ID) {
                              // If x is found, update the corresponding value in the second column
                              OpinionList[j][1] = (tPackets[i].Data[10] >> idx) & 1;
                              found = true;
                              break;
                        }
                     }
                     if (!found) { //add new opinion
                        OpinionList[nOpList][0] = rec_ID;
                        OpinionList[nOpList][1] = (tPackets[i].Data[10] >> idx) & 1;
                        nOpList++;
 
                        if(nOpList>1){nOpList=0;}
                     }

                     // Update leader opinion list 
                     if (IsLeader) // in hopspace, the predictor do this
                     {
                        bool Lfound = false;
                        uint8_t jl;
                        // Iterate through the array to find if x is already present
                        if (nLOpList > 0){
                           for (j = 0; j < nLOpList; ++j) {
                              if (LOpList[j][0] == rec_ID) {
                                    // If x is found, update the corresponding value in the second column
                                    LOpList[j][1] = (tPackets[i].Data[10] >> idx) & 1;
                                    Lfound = true;
                                    break;
                              }
                           }
                        }
                        if (!Lfound) { //add new opinion
                           LOpList[nLOpList][0] = rec_ID;
                           LOpList[nLOpList][1] = (tPackets[i].Data[10] >> idx) & 1;
                           nLOpList++;
                           if (nLOpList>20){nLOpList = 20;}
                        }
                     }
               }
            }
         }

         // Hear opinion from predictor neighbor when in dissemination state, and neigbor sending data (ID is not 0)
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
            if (IsLeader) // in hopspace, the predictor do this
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

         if (!IsPredictor){
            if (tPackets[i].Data[8] == 1 && (tPackets[i].Data[0] != 0)){ //from predictor in dissemination state
               bool Rfound = false;
               uint8_t j;
               // Iterate through the array to find if x is already present
               if (nROpList > 0){
                  for (j = 0; j < nROpList; ++j) {
                     if (RelayOpList[j][0] == tPackets[i].Data[0]) {
                           // If x is found, update the corresponding value in the second column
                           RelayOpList[j][1] = tPackets[i].Data[1];
                           RelayOpList[j][2] = 0; //reset timer
                           Rfound = true;
                           break;
                     }
                  }
               }
               if (!Rfound) { //add new opinion
                  RelayOpList[nROpList][0] = tPackets[i].Data[0];
                  RelayOpList[nROpList][1] = tPackets[i].Data[1];
                  RelayOpList[nROpList][2] = 0; //reset timer
                  nROpList++;
                  if(nROpList>nRelay){nROpList=0;}
                  // std::cout << "Rec ID: " << m_RobotID << "data: " << tPackets[i].Data << std::endl;
               }
            }
            else if (tPackets[i].Data[8] == 0){ //from non DM run loop to check all relayed message
               for (size_t idx = 0; idx < 8; ++idx) {
                  uint8_t rec_ID = tPackets[i].Data[11+idx];
                  uint8_t rec_time = tPackets[i].Data[21+idx];
                  if (rec_ID !=0){
                     bool Rfound = false;
                     uint8_t j;
                     // Iterate through the array to find if x is already present
                     if (nROpList > 0){
                        for (j = 0; j < nROpList; ++j) {
                           if (RelayOpList[j][0] == rec_ID) {
                                 // If x is found, update the corresponding value in the second column
                                 if (RelayOpList[j][2] >= rec_time){
                                    RelayOpList[j][1] = (tPackets[i].Data[10] >> idx) & 1;}
                                 Rfound = true;
                                 break;
                           }
                        }
                     }
                     if (!Rfound) { //add new opinion
                        RelayOpList[nROpList][0] = rec_ID;
                        RelayOpList[nROpList][1] = (tPackets[i].Data[10] >> idx) & 1;
                        RelayOpList[nROpList][2] = rec_time;
                        nROpList++;
                        if(nROpList>nRelay){nROpList=0;}
                        // std::cout << "Rec ID: " << m_RobotID << "data: " << tPackets[i].Data << std::endl;
                     }
                  }
               }
            }
         }

      // Calculating cinvergence certainty
      // Differentiate between predictor and non predictor
      // Only listen to predictor robot: check data 8
      if (IsPredictor && (tPackets[i].Data[8] == 1) && (tPackets[i].Data[0] != 0)){
         if (m_sStateData.Opinion == tPackets[i].Data[1]){ // same opinion, increase CC
            m_sStateData.ConvergenceCertainty = (1-kCert)*m_sStateData.ConvergenceCertainty + kCert;
         }
         else{
            m_sStateData.ConvergenceCertainty = (1-kCert)*m_sStateData.ConvergenceCertainty;
         }
      }

      else if (!IsPredictor && (tPackets[i].Data[8] == 1) && (tPackets[i].Data[0] != 0)){
         if (VirtOpinion == tPackets[i].Data[1]){ // same opinion, increase CC
            m_sStateData.ConvergenceCertainty = (1-kCert)*m_sStateData.ConvergenceCertainty + kCert;
         }
         else{
            m_sStateData.ConvergenceCertainty = (1-kCert)*m_sStateData.ConvergenceCertainty;
            VirtOpinion = tPackets[i].Data[1];
         }
      }
      
      //clamp data
      if (m_sStateData.ConvergenceCertainty < 0.0) {m_sStateData.ConvergenceCertainty = 0.0;}
      else if (m_sStateData.ConvergenceCertainty > 1.0) {m_sStateData.ConvergenceCertainty = 1.0;}

      }
   }

   RandomWalk();
   
   //collective decision making routine
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
   // }
   } // for !robot off condition
}

/****************************************/
/****************************************/

void CFootbotCDM::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   m_pcRABA->ClearData();

      /* Initialize RobotID with a random number between 0 and 255 */
   m_RobotID = static_cast<argos::UInt8>(m_pcRNG->Uniform(m_sStateData.IDRange));

   m_sStateData.RWTime =(m_pcRNG->Uniform(argos::CRange<argos::UInt32>(0, 50)));

   m_sStateData.Opinion = false;//(m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.5f);
   // std::cout << "ID " << m_RobotID << "; t_e" << m_sStateData.RWCounter << "; op" << m_sStateData.Opinion << std::endl;

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

   for (i = 0; i < 10; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            RelayOpList[i][j] = 0;  // Initialize each element to 0
        }
   }
   nROpList = 0;

   // Init exploration and dissemination
   m_sStateData.ExploreTime = static_cast<argos::UInt32>(m_pcRNG->Exponential(m_sStateData.mET)); //200;//
   m_sStateData.DisseminateTime = 0;
   m_sStateData.StateCounter = 0;
   m_sStateData.ExpState = true;

   IsLeader = false; IsHopIdentified = false;
   LeaderID = 0;
   HopLeaderID = 0;
   HopCount = 0;
   HopTime = 0;
   IsPredictor = (m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.1f*m_sStateData.HopLimit);
   SenseTime = 0;
   dcCounter = 0; //(m_pcRNG->Uniform(argos::CRange<argos::UInt32>(0, 50)));
   RoleTime = 0;
   RoleTH = static_cast<argos::UInt32>(m_pcRNG->Exponential(100));

   RelayTime=0;
   RelayTH=50;//static_cast<argos::UInt32>(m_pcRNG->Exponential(100));
   nRelay = 5;
   t_Broadcast = 0;

   m_sStateData.ConvergenceCertainty = 1; 
   VirtOpinion = (m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.5f);
   kCert = 0.01;
   ccTime = 500;

   m_sStateData.RobotOff = false;
   FaultTime = 3000;//static_cast<argos::UInt32>(m_pcRNG->Exponential(500));//3000;//

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


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootbotCDM, "dist_subset_controller")
