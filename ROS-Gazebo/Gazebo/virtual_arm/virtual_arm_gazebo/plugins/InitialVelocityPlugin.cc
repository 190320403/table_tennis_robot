#include <ignition/math/Vector3.hh>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/Model.hh"
#include "InitialVelocityPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(InitialVelocityPlugin)

/////////////////////////////////////////////////
InitialVelocityPlugin::InitialVelocityPlugin()
{
}

/////////////////////////////////////////////////
InitialVelocityPlugin::~InitialVelocityPlugin()
{
}

/////////////////////////////////////////////////
void InitialVelocityPlugin::Load(physics::ModelPtr _model,
sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "_model pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;
  this->Reset();
}

/////////////////////////////////////////////////
void InitialVelocityPlugin::Reset()
{
  if (this->sdf->HasElement("linear"))
  {
    ignition::math::Vector3d linear =
    this->sdf->Get<ignition::math::Vector3d>("linear");
    this->model->SetLinearVel(linear);
  }
 if (this->sdf->HasElement("angular"))
  {
    ignition::math::Vector3d angular =
    this->sdf->Get<ignition::math::Vector3d>("angular");
    this->model->SetAngularVel(angular);
  }
}

