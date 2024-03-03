#ifndef _GAZEBO_FOAMBUILD_PLUGIN_HH_
#define _GAZEBO_FOAMBUILD_PLUGIN_HH_

#include <map>
#include <string>
#include <vector>
#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <std_msgs/Bool.h>


namespace gazebo {

    struct Triangle {
        uint32_t m_I0;
        uint32_t m_I1;
        uint32_t m_I2;
    };

    struct Particle{
        rendering::VisualPtr vis;
        ignition::math::Vector3d start;
        ignition::math::Vector3d ray;
        double t;
        double tlim;
    };

    class FoamBuildPlugin : public VisualPlugin {

        public: FoamBuildPlugin();

        public: virtual ~FoamBuildPlugin();

        public: virtual void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

    private:

        virtual void OnUpdate();
        virtual void OnRemove();

        void extruder_cb(const std_msgs::Bool::ConstPtr& msg);

        bool intersectSphereGround(const ignition::math::Vector3d& origin, double radius, const ignition::math::Vector3d& vel, double& t);
        bool intersectSphereFoam(const std::vector<ignition::math::Vector3d>& foam, const ignition::math::Vector3d& origin, double radius, const ignition::math::Vector3d& vel, double& t);

        Ogre::ManualObject* Spheres(std::string name, float radius = 1.0f, uint32_t subdivisions = 1, std::vector<ignition::math::Vector3f> poses = {});
        void Icosphere(std::vector<ignition::math::Vector3f>& vertices, std::vector<Triangle>& triangles, float radius = 1.0f, uint32_t subdivisions = 2);
        void Icosahedron(std::vector<ignition::math::Vector3f>& vertices, std::vector<Triangle>& triangles);

        static uint32_t MidPoint(std::vector<ignition::math::Vector3f>& vertices, std::map<uint64_t, uint32_t>& table, uint64_t iA, uint64_t iB);

    private:
        rendering::VisualPtr visual;
        sdf::ElementPtr sdf_link;

        ignition::math::Rand random;

        std::vector<Particle> particles;
        std::vector<Particle> buffer;
        std::vector<ignition::math::Vector3d> segments;

        event::ConnectionPtr updateConnection;
        event::ConnectionPtr removeConnection;

        double particleSize_ = 0.05;
        double growthFactor_ = 3.107;
        double ejectSpeed_ = 0.3;
        int bufferSize_ = 500;

        ros::Subscriber m_SubExtrusion;
        bool extruding;
  };
}
#endif
