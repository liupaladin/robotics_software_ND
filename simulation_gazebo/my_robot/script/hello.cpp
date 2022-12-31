#include <gazebo/gazebo.hh>
#include "iostream"

namespace gazebo {
    class WorldPluginMyRobot() : WorldPlugin {
        public: WorldPluginMyRobot() : WorldPlugin() {
            std::cout << "Hello World!\n";
        }

        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

        }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}