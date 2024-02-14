#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "sensors.h"
#include "timers.h"
#include "ledseq.h"

#define DEBUG_MODULE "SIMPLEFLIGHT"
#define TAKE_OFF_HEIGHT 0.2f
#define LED_LOCK         LED_BLUE_L
enum State {
  STATE_IDLE=0,
  STATE_READY,
  STATE_TAKE_OFF,
  STATE_TAKING_OFF,
  STATE_HOVERING,
  STATE_GO_RIGHT,
  STATE_GOING_RIGHT,
  STATE_GO_LEFT,
  STATE_GOING_LEFT,
  STATE_LAND,
  STATE_LANDING,
  STATE_END,
};

static enum State state = STATE_IDLE;
ledseqStep_t seq_lock_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_lock = {
  .sequence = seq_lock_def,
  .led = LED_LOCK,
};

static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static paramVarId_t paramIdCommanderEnHighLevel;
static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static float getX() { return logGetFloat(logIdStateEstimateX); }
static float getY() { return logGetFloat(logIdStateEstimateY); }
static float getZ() { return logGetFloat(logIdStateEstimateZ); }
static void defineLedSequence() {
  ledseqRegisterSequence(&seq_lock);
}
void appMain()
{ 
  defineLedSequence();
  while(1){
ledseqRun(&seq_lock);
}
  vTaskDelay(M2T(3000));
  // X, Y, Z ids to get position
  logIdStateEstimateX = logGetVarId("stateEstimate", "x");
  logIdStateEstimateY = logGetVarId("stateEstimate", "y");
  logIdStateEstimateZ = logGetVarId("stateEstimate", "z");

  // Commander ID
  paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
  paramSetInt(paramIdCommanderEnHighLevel, 1);
  defineLedSequence();
   vTaskDelay(M2T(3000));
  ledseqRun(&seq_lock);
  while (1)
  {
    vTaskDelay(M2T(10));
    switch (state)
    {
    case STATE_IDLE:
      state=STATE_READY;
      break;
    case STATE_READY:
      // Get position of starting point
      padX=getX();
      padY=getY();
      padZ=getZ();
      state=STATE_TAKE_OFF;
      break;
    case STATE_TAKE_OFF:
      crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, 1.0);
      state=STATE_TAKING_OFF;
      break;
    case STATE_TAKING_OFF:
      if (!crtpCommanderHighLevelIsTrajectoryFinished())
        break;
      state=STATE_GO_RIGHT;
      break;
    case STATE_GO_RIGHT:
      crtpCommanderHighLevelGoTo(1,0,0,0,2,true);
      state=STATE_GOING_RIGHT;
      break;
    case STATE_GOING_RIGHT:
      if(!crtpCommanderHighLevelIsTrajectoryFinished())
        break;
      state=STATE_GO_LEFT;
      break;
    case STATE_GO_LEFT:
      crtpCommanderHighLevelGoTo(-1,0,0,0,2,true);
      state=STATE_GOING_LEFT;
      break;
    case STATE_GOING_LEFT:
      if(!crtpCommanderHighLevelIsTrajectoryFinished())
        break;
      state=STATE_LAND;
      break;
    case STATE_LAND:
      crtpCommanderHighLevelLand(padZ, 1.0);
      state=STATE_LANDING;
      break;
    case STATE_LANDING:
      if(!crtpCommanderHighLevelIsTrajectoryFinished())
        break;
      state=STATE_END;
    case STATE_END:
      break;
    default:
      break;
    }
  }  
}
