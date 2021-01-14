#ifndef GAZEBO_PLUGINS_INITIALVELOCITYPLUGIN_HH_
#define GAZEBO_PLUGINS_INITIALVELOCITYPLUGIN_HH_

#include <string>
#include <vector>

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo
{
class GAZEBO_VISIBLE InitialVelocityPlugin : public ModelPlugin
{
/// \brief Constructor.
public: InitialVelocityPlugin();

/// \brief Destructor
public: ~InitialVelocityPlugin();

// Documentation Inherited.
public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

// Documentation Inherited.
public: virtual void Reset();

/// \brief Parent model.
private: physics::ModelPtr model;

/// \brief SDF for this plugin;
private: sdf::ElementPtr sdf;
};
}
// ifndef _INITIAL_VELOCITY_PLUGIN_HH_
#endif
