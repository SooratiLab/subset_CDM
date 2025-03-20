#ifndef LEADER_SUBSET_H
#define LEADER_SUBSET_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
#include <fstream>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootbotCDM : public CCI_Controller {

public:

   struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };

   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The three possible states in which the controller can be */
      enum EState {
         STATE_CONSTRUCTION = 0,
         STATE_CDM,
         STATE_EVALUATION
      } State;

      CRange<Real> ProbRange;


      CRange<UInt32> IDRange;

      /* The counter for semi random walk */
      size_t RWCounter;
      /* The time limit semi random walk */
      size_t RWTime;

      /* The time limit state */
      size_t ExploreTime;
      size_t DisseminateTime;
      size_t StateCounter;
      bool ExpState;

      size_t HopLimit;
      size_t mET;
      size_t mDT;
      Real pNoise;

      Real cNoise;

      /* Robots opinion */
      bool Opinion;

      /*state rotation or straight*/
      bool RWStateRot;

      bool RobotOff;

      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
   };

public:

   /* Class constructor. */
   CFootbotCDM();
   /* Class destructor. */
   virtual ~CFootbotCDM() {}
   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

   inline bool IsOpinionWhite() const {
      return m_sStateData.Opinion;
   }

   inline bool IsDMRobot() const {
      // return m_sStateData.Opinion;
      return IsPredictor;
   }

   inline bool IsLeaderRobot() const {
      // return m_sStateData.Opinion;
      return IsLeader;
   }


   virtual void SetOpinionWhite();
   virtual void SetRobotOff();
   virtual void SetLeader();
  



private:

   void RandomWalk();
   /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
    */
   void UpdateState();

   /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
   CVector2 DiffusionVector(bool& b_collision);

   /*
    * Gets a direction vector as input and transforms it into wheel
    * actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   void ResetLeaderOpinionList();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;

   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   // RobotID member variable
   uint8_t m_RobotID;

   //counter for robot and task
   uint32_t nCounter, mCounter;

   //Array to store opinion and its pointer
   uint8_t OpinionList[10][2];
   uint8_t nOpList;

   uint8_t LeaderID;
   uint8_t HopLeaderID;
   uint8_t HopCount;
   uint8_t HopTime;
   bool IsLeader;
   bool IsPredictor;
   uint16_t LeaderTime; // count the time of the leader
   uint32_t SenseTime; 
   uint16_t dcCounter; // counter for how long robot disconnect from network
   uint16_t RoleTime;
   uint16_t RoleTH;

   bool RobotOff;

   //Array to store opinion and its pointer (for leader only)
   uint8_t LOpList[100][2];
   uint8_t nLOpList;
   uint16_t HopLimTime; // count the time of particular hop limit
   uint8_t ConsistentTime;
   uint8_t Decision[2];
   uint8_t prevstepdec;
   bool IsHopIdentified;
   std::ofstream f_Output;

   size_t FaultTime;

   /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;


};

#endif
