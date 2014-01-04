/********************************************************************
 * File   : MainScene.h
 * Project: ToolsDemo
 *
 ********************************************************************
 * Created on 9/21/13 By Nonlinear Ideas Inc.
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

#ifndef __Box2DTestBed__MainScene__
#define __Box2DTestBed__MainScene__

#include "CommonProject.h"
#include "CommonSTL.h"

#include "DebugLinesLayer.h"
#include "TapDragPinchInput.h"
#include "Notifier.h"

class MovingEntityIFace;

class MainScene : public CCScene, public TapDragPinchInputTarget
{
private:
   // This class follows the "create"/"autorelease" pattern.
   // Private constructor.
   MainScene();
   

   // Box2d Physics World
   b2World* _world;
   
   
   // The moving entity
   MovingEntityIFace* _entity;
   //Missile* _entity;
   
   // Keep the last center point during a pinch.
   Vec2 _viewportCenterOrg;
   float32 _viewportScaleOrg;
   
protected:
   // This is protected so that derived classes can call it
   // in their create methods.
   bool init();
   
private:
   void CreatePhysics();
   void CreateEntity();
   void SetZoom(float zoom);
   void UpdateEntity();
   void UpdatePhysics();
   void PinchViewport(const CCPoint& p0Org,const CCPoint& p1Org,
                      const CCPoint& p0,const CCPoint& p1);
public:
   
   static MainScene* create();
   
   ~MainScene();
   
   virtual void onEnter();
   virtual void onExit();
   virtual void onEnterTransitionDidFinish();
   virtual void onExitTransitionDidStart();
   virtual void update(float dt);
      
   // Handler for Tap/Drag/Pinch Events
   typedef TapDragPinchInputTarget::TOUCH_DATA_T TOUCH_DATA_T;
   virtual void TapDragPinchInputTap(const TOUCH_DATA_T& point);
   virtual void TapDragPinchInputLongTap(const TOUCH_DATA_T& point);
   virtual void TapDragPinchInputPinchBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputPinchContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputPinchEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputDragBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputDragContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputDragEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
};


#endif /* defined(__Box2DTestBed__MainScene__) */
