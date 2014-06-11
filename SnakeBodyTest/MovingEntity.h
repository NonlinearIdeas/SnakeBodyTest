/********************************************************************
 * File   : MovingEntity.h
 * Project: MissileDemo
 *
 ********************************************************************
 * Created on 10/20/13 By Nonlinear Ideas Inc.
 * Copyright (c) 2013 Nonlinear Ideas Inc. All rights reserved.
 ********************************************************************
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any 
 * damages arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any 
 * purpose, including commercial applications, and to alter it and 
 * redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must 
 *    not claim that you wrote the original software. If you use this 
 *    software in a product, an acknowledgment in the product 
 *    documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and 
 *    must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source 
 *    distribution. 
 */

#ifndef __MissileDemo__MovingEntity__
#define __MissileDemo__MovingEntity__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "PIDController.h"
#include "MathUtilities.h"
#include "Entity.h"
#include "MovingEntityIFace.h"
#include "Notifier.h"


class MovingEntity : public Entity, public MovingEntityIFace
{
private:
   typedef enum
   {
      ST_IDLE,
      ST_TURN_TOWARDS,
      ST_SEEK,
      ST_FOLLOW_PATH,
      ST_MAX
   } STATE_T;
   
   STATE_T _state;
   // Create turning acceleration
   PIDController _turnController;
   vector<Body*> _segments;
   
   void SetupTurnController()
   {
      GetBody()->SetAngularDamping(0);
      _turnController.ResetHistory();
      _turnController.SetKDerivative(5.0);
      _turnController.SetKProportional(2.0);
      _turnController.SetKIntegral(0.1);
      _turnController.SetKPlant(1.0);
   }
   
   
   void StopBody()
   {
      Vec2 vel0(0,0);
      
      GetBody()->SetLinearVelocity(vel0);
      GetBody()->SetAngularVelocity(0);
      for(int idx = 0; idx < _segments.size();idx++)
      {
         _segments[idx]->SetLinearVelocity(vel0);
         _segments[idx]->SetAngularVelocity(0);
      }
   }
   
   
   bool IsNearTarget()
   {
      Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
      
      if(toTarget.LengthSquared() < GetMinSeekDistance()*GetMinSeekDistance())
      {
         return true;
      }
      return false;
   }
   
   void ApplyTurnTorque()
   {
      Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
      
      float32 angleBodyRads = MathUtilities::AdjustAngle(GetBody()->GetAngle());
      float32 angleTargetRads = MathUtilities::AdjustAngle(atan2f(toTarget.y, toTarget.x));
      float32 angleError = MathUtilities::AdjustAngle(angleBodyRads - angleTargetRads);
      _turnController.AddSample(angleError);
      
      // Negative Feedback
      float32 angAcc = -_turnController.GetLastOutput();
      
      // This is as much turn acceleration as this
      // "motor" can generate.
      if(angAcc > GetMaxAngularAcceleration())
         angAcc = GetMaxAngularAcceleration();
      if(angAcc < -GetMaxAngularAcceleration())
         angAcc = -GetMaxAngularAcceleration();
      
      float32 torque = angAcc * GetBody()->GetInertia();
      GetBody()->ApplyTorque(torque);
   }
   
   void ApplyThrust()
   {
      // Get the distance to the target.
      Vec2 toTarget = GetTargetPos() - GetBody()->GetWorldCenter();
      toTarget.Normalize();
      Vec2 desiredVel = GetMaxSpeed()*toTarget;
      Vec2 currentVel = GetBody()->GetLinearVelocity();
      Vec2 thrust = desiredVel - currentVel;
      GetBody()->ApplyForceToCenter(GetMaxLinearAcceleration()*thrust);
   }
   
   void EnterSeek()
   {
      SetupTurnController();
   }
   
   void ExecuteSeek()
   {
      if(IsNearTarget())
      {
         StopBody();
      }
      else
      {
         ApplyTurnTorque();
         ApplyThrust();
      }
   }
   
   
   void EnterIdle()
   {
      StopBody();
   }
   
   void ExecuteIdle()
   {
   }
   
   void EnterTurnTowards()
   {
      SetupTurnController();
   }
   
   void ExecuteTurnTowards()
   {
      ApplyTurnTorque();
   }
   
   void UpdatePathTarget()
   {
      list<Vec2>& path = GetPath();
      Vec2& targetPos = GetTargetPos();
      
      if(path.size() > 0)
      {
         targetPos = *path.begin();
         while(path.size() > 0 && IsNearTarget())
         {
            targetPos = *path.begin();
            path.pop_front();
         }
      }
      else
      {
         targetPos = GetBody()->GetPosition();
      }
   }
   
   void EnterFollowPath()
   {
      // If there are any points to follow,
      // then pop the first as the target
      // and follow it.  Otherwise, go idle.
      UpdatePathTarget();
      if(GetPath().size() > 0)
      {
         SetupTurnController();
      }
      else
      {
         ChangeState(ST_IDLE);
      }
   }
   
   void ExecuteFollowPath()
   {
      UpdatePathTarget();
      if(GetPath().size() > 0)
      {
         ApplyThrust();
         ApplyTurnTorque();
      }
      else
      {
         ChangeState(ST_IDLE);
      }
   }
   
   void ExecuteState(STATE_T state)
   {
      switch(state)
      {
         case ST_IDLE:
            ExecuteIdle();
            break;
         case ST_TURN_TOWARDS:
            ExecuteTurnTowards();
            break;
         case ST_SEEK:
            ExecuteSeek();
            break;
         case ST_FOLLOW_PATH:
            ExecuteFollowPath();
            break;
         default:
            assert(false);
      }
   }
   
   void EnterState(STATE_T state)
   {
      switch(state)
      {
         case ST_IDLE:
            EnterIdle();
            break;
         case ST_TURN_TOWARDS:
            EnterTurnTowards();
            break;
         case ST_SEEK:
            EnterSeek();
            break;
         case ST_FOLLOW_PATH:
            EnterFollowPath();
            break;
         default:
            assert(false);
      }
   }
   
   void ChangeState(STATE_T state)
   {
      EnterState(state);
      _state = state;
   }
   
public:
   // Constructor
	MovingEntity(b2World& world,const Vec2& position) :
   Entity(Entity::ET_MISSILE,10),
   _state(ST_IDLE)
   {
      // Create the body.
      b2BodyDef bodyDef;
      bodyDef.position = position;
      bodyDef.type = b2_dynamicBody;
      Body* body = world.CreateBody(&bodyDef);
      assert(body != NULL);
      // Store it in the base.
      Init(body);
      
      // Now attach fixtures to the body.
      FixtureDef fixtureDef;
      b2CircleShape circleShape;
      
      const float64 BODY_RADIUS = 1.0;
      
      fixtureDef.shape = &circleShape;
      fixtureDef.density = 1.0;
      fixtureDef.friction = 1.0;
      fixtureDef.isSensor = false;
      
      // Nose
      circleShape.m_radius = BODY_RADIUS*1.25;
      body->CreateFixture(&fixtureDef);
      body->SetLinearDamping(4);
      body->SetAngularDamping(4);
      
      // NOW, create several duplicates of the "Main Body" fixture
      // but offset them from the previous one by a fixed amount and
      // overlap them a bit.
      const uint32 SNAKE_SEGMENTS = 16;
      
      b2Body* pBodyA = body;
      b2Body* pBodyB = NULL;
      b2RevoluteJointDef revJointDef;
      revJointDef.collideConnected = false;
      
      // Add some "regular segments".
      for(int idx = 0; idx < SNAKE_SEGMENTS; idx++)
      {
         // Create a body for the next segment.
         float64 bodyRadius = BODY_RADIUS;
         if(idx > SNAKE_SEGMENTS*2/3)
         {
            bodyRadius *= pow(0.9,idx-SNAKE_SEGMENTS*2/3+1);
         }
         Vec2 offset(-1.75*bodyRadius,0);
         bodyDef.position = pBodyA->GetPosition() + offset;
         pBodyB = world.CreateBody(&bodyDef);
         circleShape.m_radius = bodyRadius;
         pBodyB->CreateFixture(&fixtureDef);
         // Drag on the body as the segments get longer
         pBodyB->SetLinearDamping(0.5*(1+idx));
         pBodyB->SetAngularDamping(0.5*(1+idx));
         
         // Create a Revolute Joint at a position half way
         // between the two bodies.
         Vec2 midpoint = 0.5*(pBodyA->GetPosition() + pBodyB->GetPosition());
         revJointDef.Initialize(pBodyA, pBodyB, midpoint);
         world.CreateJoint(&revJointDef);
         // Update so the next time through the loop, we are
         // connecting the next body to the one we just
         // created.
         pBodyA = pBodyB;
      }
      
      // Setup Parameters
      SetMaxAngularAcceleration(4*M_PI);
      // As long as this is high, they forces will be strong
      // enough to get the body close to the target position
      // very quickly so the entity does not "circle" the
      // point.
      SetMaxLinearAcceleration(100);
      SetMaxSpeed(20);
      SetMinSeekDistance(1.0);
   }
   
   
   // Commands - Use thse to change the state
   // of the missile.
   void CommandFollowPath(const list<Vec2> path)
   {
      GetPath() = path;
      ChangeState(ST_FOLLOW_PATH);
   }
   
   
   void CommandTurnTowards(const Vec2& position)
   {
      GetTargetPos() = position;
      ChangeState(ST_TURN_TOWARDS);
   }
   
   void CommandSeek(const Vec2& position)
   {
      GetTargetPos() = position;
      ChangeState(ST_SEEK);
   }
   
   void SetTargetPosition(const Vec2& position)
   {
      GetTargetPos() = position;
   }
   
   void CommandIdle()
   {
      ChangeState(ST_IDLE);
   }
   
   virtual void Update()
   {
      ExecuteState(_state);
   }
   
protected:
private:
};

#endif /* defined(__MissileDemo__MovingEntity__) */
