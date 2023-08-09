/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * push.c - App layer application of the onboard push demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "commander.h"
#include "crtp_commander_high_level.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "PUSH"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define ABS(a) ((a<0)?(-a):(a))
#define POW2(a) ((a)*(a))
#define DIST(a,b) (sqrtf(POW2(a)+POW2(b)))
#define SIGN(a) ((a<0)?(-1):(1))

#define HEIGHT 0.2f
#define ACCEPTABLE_RADIUS_FROM_WAYPOINT 0.1f
#define NUM_OF_WAYPOINTS 5
#define MAX_TIME_BEFORE_OUT_OF_RANGE 1.5f
#define MAX_NUM_OF_WAY_POINTS 10

static P2PPacket p_reply;
static const uint16_t unlockLow = 100;
static const uint16_t unlockHigh = 300;
uint8_t currentWayPoint = 0;
uint8_t currentRecievedWayPoint = 0;

float timeOfLastMsg = 0.0f;
#define MAX_DISTORTED_MSGS 5
uint8_t numOfDistortedMsg = 0;


float initialPos[3] = {0,0,0};

void setInitPos(float* initialPos){
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    if(my_id == 0xE7){
      initialPos[0] = 0.0f;
      initialPos[1] = 0.0f;
      initialPos[2] = 0.0f;
    }else if(my_id == 0xE6){
      initialPos[0] = 0.0f;
      initialPos[1] = 0.4f;
      initialPos[2] = 0.0f;
    }
}

//bool lostConnection = false;
bool lostConnectionBefore = 0;
bool FIRSTMsg = false;

float wayPoints[NUM_OF_WAYPOINTS][3] = {{0,0,HEIGHT},
                                        {0.4f,0.4f,HEIGHT},
                                        {0.4f,0.0f,HEIGHT},
                                        {0.8f,0,HEIGHT},
                                        {0.8f,0.4f,HEIGHT}};//global [x,y,z]

float recievedWayPoints[MAX_NUM_OF_WAY_POINTS][3];
int NumOfRecievedWayPoints = 0;

typedef enum {
  ERROR,
  starting,
  checkIfInRange,
  sayingPos
} HighLevelMsg;




void p2pcallbackHandler(P2PPacket *p)
{
  uint8_t id = p->data[0];
  uint8_t rssi = p->rssi;
  uint8_t port = p->port;
  uint8_t data0 = p->data[1];
  float timeNow = usecTimestamp() / 1e6;
  DEBUG_PRINT("\ntime: %f, the id: %x\nthe rssi: %d\nthe port: %d\n data[0]: %x\n",(double)timeNow,id,rssi,port,data0);

  if(numOfDistortedMsg >= MAX_DISTORTED_MSGS){
    lostConnectionBefore = true;
    return;
  }

  if(data0 == (uint8_t)starting){
    DEBUG_PRINT("Other Drone Started Flying\n");
    FIRSTMsg = true;
    timeOfLastMsg = usecTimestamp() / 1e6;
    return;
  }

  if(data0 == (uint8_t)checkIfInRange){
    if(p->data[2] == (uint8_t)checkIfInRange){
      DEBUG_PRINT("other drone is still in range\n");
      FIRSTMsg = true;
      timeOfLastMsg = usecTimestamp() / 1e6;
      return;
    }
    DEBUG_PRINT("recieved incomlete range checker\n");
    numOfDistortedMsg++;
    return;
  }

  if(data0 == (uint8_t)sayingPos){
    DEBUG_PRINT("I got the %d pos!\n", NumOfRecievedWayPoints+1);
    if(lostConnectionBefore == true) return;
    numOfDistortedMsg = 0;
    timeOfLastMsg = usecTimestamp() / 1e6;
    float x;
    float y;
    float Height;
    memcpy(&x, &(p->data[2]), sizeof(float));
    memcpy(&y, &(p->data[6]), sizeof(float));
    memcpy(&Height, &(p->data[10]), sizeof(float));
    DEBUG_PRINT("X: %f, Y:%f, Z:%f\n",(double)x,(double)y,(double)Height);
    recievedWayPoints[NumOfRecievedWayPoints][0] = x;
    recievedWayPoints[NumOfRecievedWayPoints][1] = y;
    recievedWayPoints[NumOfRecievedWayPoints][2] = Height;
    NumOfRecievedWayPoints++;
    return;
  }
  numOfDistortedMsg++;
  DEBUG_PRINT("non recognized packet!\n");
  return;

}

void calculateVelToGoal(float currentX, float currentY, float goalX, float goalY, float* velX, float* velY){
  float maxSpeed = 0.3;
  float dy = goalY - currentY;
  float dx = goalX - currentX;
  
  float dist = DIST(dx,dy);
  (*velX) = maxSpeed*dx/dist;
  (*velY) = maxSpeed*dy/dist;
}

/*
float calculateYaw(float goalX, float goalY, float currX, float currY){
  float dy = goalY - currY;
  float dx = goalX - currX;
  float tanTheta = ABS(dy/dx);
  float theta = atanf(tanTheta) * 180.0f/3.14159f;
  return (SIGN(dy)*(theta + 90*(SIGN(dx)-1)))>0?(SIGN(dy)*(theta + 90*(SIGN(dx)-1))):(360-(SIGN(dy)*(theta + 90*(SIGN(dx)-1))));

}
*/
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate, bool relative)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeAbs;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = relative;
}

void sendPacket(HighLevelMsg msg){
    
    p_reply.port=0x00;
    p_reply.size=2*sizeof(uint8_t)+1*sizeof(uint8_t);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    uint8_t msg_temp = (uint8_t)msg;
    memcpy(&(p_reply.data[0]), &my_id, sizeof(uint8_t));
    memcpy(&(p_reply.data[1]), &msg_temp, sizeof(uint8_t));
    memcpy(&(p_reply.data[2]), &msg_temp, sizeof(uint8_t));
    radiolinkSendP2PPacketBroadcast(&p_reply);
}

void sendLocPacket(float x, float y, float height){
    DEBUG_PRINT("sending packet!, X:%f, Y:%f, Z:%f\n",(double)x, (double)y, (double)height);
    p_reply.port=0x00;
    p_reply.size= 3*sizeof(float)+2*sizeof(uint8_t);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    uint8_t Notempty = (uint8_t)sayingPos;// if the msg is empty or not
    memcpy(&(p_reply.data[0]), &my_id, sizeof(uint8_t));
    memcpy(&(p_reply.data[1]), &Notempty, sizeof(uint8_t));
    memcpy(&(p_reply.data[2]), &x, sizeof(float));
    memcpy(&(p_reply.data[6]), &y, sizeof(float));
    memcpy(&(p_reply.data[10]), &height, sizeof(float));
    radiolinkSendP2PPacketBroadcast(&p_reply);
}

typedef enum {
    idle,
    GetToLastKnownPos,
    lowUnlock,
    unlocked,
    moving,
    hover,
    end,
} State;

static State state = idle;

void appMain()
{
  static setpoint_t setpoint;

  p2pRegisterCB(p2pcallbackHandler);

  vTaskDelay(M2T(3000));
  setInitPos(initialPos);

  paramVarId_t idHighLevelComm = paramGetVarId("commander", "enHighLevel");
  logVarId_t idUp = logGetVarId("range", "up");
  /*logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");*/
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  /*logVarId_t idYaw = logGetVarId("stabilizer", "yaw");*/

  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");
  
  paramSetInt(idHighLevelComm, 1);
  

  DEBUG_PRINT("starting the testing!\n");
  float XEstimate = initialPos[0];
  float YEstimate = initialPos[1];

  float remTime = 0;
  float yaw = 0;

  while(1) {

    vTaskDelay(M2T(50));
    if(state == moving) sendPacket(checkIfInRange);
    vTaskDelay(M2T(50));

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);
    uint16_t my_up = logGetUint(idUp);
    float timeNow = usecTimestamp() / 1e6;
    /*float YawEstimate = logGetFloat(idYaw);*/


    if(!positioningInit){
      DEBUG_PRINT("\nFlow deck not connected\n");
      break;
    }
    if(!multirangerInit){
      DEBUG_PRINT("\nmultiranger deck not connected\n");
      break;
    }

    //state machine
    if (state == idle){
      if (my_up <= unlockLow){
        DEBUG_PRINT("unlocking...\n");
        state = lowUnlock;

      }
      if((lostConnectionBefore == true || timeNow - timeOfLastMsg >= MAX_TIME_BEFORE_OUT_OF_RANGE) && FIRSTMsg == true && NumOfRecievedWayPoints > 0){
        if(timeNow - timeOfLastMsg >= MAX_TIME_BEFORE_OUT_OF_RANGE) DEBUG_PRINT("now starting to follow the other drone because of timeout\n");
        state = GetToLastKnownPos;
        lostConnectionBefore = true;
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
      }
    }else if(state == lowUnlock){
      if(my_up >= unlockHigh){
        DEBUG_PRINT("flying!\n");
        state = unlocked;
      }
    }else if (state == unlocked){
      setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
      commanderSetSetpoint(&setpoint, 3);
      //vTaskDelay(M2T(500));
      sendPacket(starting);
      DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
      state = moving;

    }else if(state == moving){
      
      if (my_up <= unlockLow){
        DEBUG_PRINT("ending...\n");
        state = end;
        continue;
      }
      XEstimate = logGetFloat(idX) + initialPos[0];
      YEstimate = logGetFloat(idY) + initialPos[1];
      float Xvel, Yvel;
      calculateVelToGoal(XEstimate,YEstimate,wayPoints[currentWayPoint][0],wayPoints[currentWayPoint][1],&Xvel,&Yvel);
      //yaw = calculateYaw(wayPoints[currentWayPoint][0], wayPoints[currentWayPoint][1], XEstimate, YEstimate);
      float timeNow = usecTimestamp() / 1e6;
      if (remTime == 0.0f){
        remTime = timeNow;
      }
      setHoverSetpoint(&setpoint, Xvel, Yvel, HEIGHT, yaw, false);
      commanderSetSetpoint(&setpoint, 3);
      if (DIST((XEstimate-wayPoints[currentWayPoint][0]),(YEstimate-wayPoints[currentWayPoint][1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT){ continue;}
      remTime = timeNow;
      //if(DIST((XEstimate-wayPoints[currentWayPoint][0]),(YEstimate-wayPoints[currentWayPoint][1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT) {continue;}
      currentWayPoint++;
      sendLocPacket(XEstimate, YEstimate, HEIGHT);
      if(currentWayPoint >= NUM_OF_WAYPOINTS){
        state = end;
      }

    }else if(state == end){
      DEBUG_PRINT("landing\n");
      setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT/2, 0.0f, false);
      commanderSetSetpoint(&setpoint, 3);
      vTaskDelay(M2T(200));
      memset(&setpoint, 0, sizeof(setpoint_t));
      commanderSetSetpoint(&setpoint, 5);
      vTaskDelay(M2T(1000));
      state = idle;
      break;

    }else if(state == GetToLastKnownPos){
      if(currentRecievedWayPoint >= NumOfRecievedWayPoints){
        state = hover;
        continue;
      }
      if (my_up <= unlockLow){
        DEBUG_PRINT("ending...\n");
        state = end;
        continue;
      }
      XEstimate = logGetFloat(idX) + initialPos[0];
      YEstimate = logGetFloat(idY) + initialPos[1];
      float Xvel, Yvel;
      calculateVelToGoal(XEstimate,YEstimate,recievedWayPoints[currentRecievedWayPoint][0],recievedWayPoints[currentRecievedWayPoint][1],&Xvel,&Yvel);
      float timeNow = usecTimestamp() / 1e6;
      if (remTime == 0.0f){
        remTime = timeNow;
      }
      setHoverSetpoint(&setpoint, Xvel, Yvel, HEIGHT, 0.0f, false);
      commanderSetSetpoint(&setpoint, 3);
      if (DIST((XEstimate-recievedWayPoints[currentRecievedWayPoint][0]),(YEstimate-recievedWayPoints[currentRecievedWayPoint][1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT){ continue;}
      remTime = timeNow;
      //if(DIST((XEstimate-wayPoints[currentWayPoint][0]),(YEstimate-wayPoints[currentWayPoint][1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT) {continue;}
      currentRecievedWayPoint++;

    }else if(state == hover){
      DEBUG_PRINT("hovering\n");
      if (my_up <= unlockLow){
        DEBUG_PRINT("ending...\n");
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
        break;
      }
      setHoverSetpoint(&setpoint, 0, 0, HEIGHT, 0.0f, false);
      commanderSetSetpoint(&setpoint, 3);
    }

  }
  DEBUG_PRINT("ending the program\n");
}
