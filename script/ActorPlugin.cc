/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "ActorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

#define TALKING_ANIMATION "talk_a"

#define STANDING_ANIMATION "stand"

#define ADMINISTRATOR_ACTOR "administrator"

#define VISITOR1_ACTOR "visitor1"

#define VISITOR2_ACTOR "visitor2"

#define FRONT_DESK_PROBABILITY 0.2


/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  if(this->actor->GetName().compare(ADMINISTRATOR_ACTOR) !=0)
   this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  this->frontDesk1 = this->sdf->Get<ignition::math::Vector3d>("front_desk1");
  this->frontDesk2 = this->sdf->Get<ignition::math::Vector3d>("front_desk2");

  
  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  if (this->actor->GetName().compare(ADMINISTRATOR_ACTOR) == 0)
  {
    this->target = ignition::math::Vector3d(-2, -5.5, 1.25);
    this->SetAnimation(STANDING_ANIMATION);
  }
  else
  {
      auto skelAnims = this->actor->SkeletonAnimations();
      if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
      {
        gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
      }
      else
      {
        // Create custom trajectory
        this->SetAnimation(WALKING_ANIMATION);
      }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{

  if (this->actor->GetName().compare(ADMINISTRATOR_ACTOR) == 0)
  {
      int visitorsInFrontDeskArea = 0;
    
      //Check if a visitor actor (a model that is not to be ignored) is in the front desk area
      if(FrontDeskAreaHasVisitor())
      {
        visitorsInFrontDeskArea = visitorsInFrontDeskArea + 1;                
      }

      if(visitorsInFrontDeskArea > 0)
      {
        this->SetAnimation(TALKING_ANIMATION);
      }
      else
      {
        this->SetAnimation(STANDING_ANIMATION);
      }
  }
  else
  {
      //If frontdesk1 is already the target, set frontdesk2 as target.
      //else check with probability if frontdesk is cleared if not, chose a regular target.
      //if yes, set frontdesk1 as target.
      if(this->target.Equal(this->frontDesk1))
      {
         this->target = this->frontDesk2
         this->SetAnimation(TALKING_ANIMATION);
      }
      else
      {
         double deskVerdict = ignition::math::Rand::DblUniform(0, 1);
         if(deskVerdict < FRONT_DESK_PROBABILITY && !FrontDeskAreaHasVisitor() && !this->target.Equal(this->frontDesk2))
         {
            this->target = this->frontDesk1
            this->SetAnimation(WALKING_ANIMATION);
         }
         else
         {
            ignition::math::Vector3d newTarget(this->target);
            while ((newTarget - this->target).Length() < 2.0)
            {
              newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
              newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));
          
              for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
              {
                double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
                    - newTarget).Length();
                if (dist < 2.0)
                {
                  newTarget = this->target;
                  break;
                }
              }
            }
            this->target = newTarget;           
            this->SetAnimation(WALKING_ANIMATION);
         }
      }
    
    }
}


boolean ActorPlugin::FrontDeskAreaHasVisitor(){

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  { 
      if((this->world->ModelByIndex(i)->GetPluginCount() ==1 || this->world->ModelByIndex(i)->GetType() == ACTOR) &&
        (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
              this->world->ModelByIndex(i)->GetName()) != this->ignoreModels.end()) &&
              this->world->ModelByIndex(i)->WorldPose().Pos().X() < this->frontDesk1.X() &&
              this->world->ModelByIndex(i)->WorldPose().Pos().X() > this->actor->WorldPose().Pos().X() &&
              this->world->ModelByIndex(i)->WorldPose().Pos().X() < (this->actor->WorldPose().Pos().Y() + 1.5) &&
              this->world->ModelByIndex(i)->WorldPose().Pos().X() > (this->actor->WorldPose().Pos().Y() - 1.5) )
      {
         return true;
      }
      else
      {
         return false;
      }
  }
}
////////////////////////////////////////////////////////////////
void ActorPlugin::SetAnimation(std::string &anim)
{
  this->trajectoryInfo.reset(new physics::TrajectoryInfo());
  this->trajectoryInfo->type = anim;
  this->trajectoryInfo->duration = 1.0;

  this->actor->SetCustomTrajectory(this->trajectoryInfo);

}

/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    //Since the administrator actor doesn't move, we can trigger its animation here for two second and return
    //We also trigger the animation for the visitor actor at frontDesk2
    if(this->actor->GetName().compare(ADMINISTRATOR_ACTOR) == 0 || this->target.Equal(this->frontDesk2))
    {
      this->actor->SetScriptTime(this->actor->ScriptTime() + 2);
      this->lastUpdate = _info.simTime;
      return;
    }
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

