#include "wasp_gazebo_plugin/FoamBuildPlugin.hh"

using namespace gazebo;
GZ_REGISTER_VISUAL_PLUGIN(FoamBuildPlugin)

FoamBuildPlugin::FoamBuildPlugin() : VisualPlugin(){}

/////////////////////////////////////////////////
FoamBuildPlugin::~FoamBuildPlugin(){}

void FoamBuildPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) {
    this->visual = _parent;

    _sdf = _sdf->GetFirstElement();

    do {
      std::string name = _sdf->GetName();
      if (name == "particleSize")
        _sdf->GetValue()->Get(particleSize_);
      else if (name == "divergence")
        _sdf->GetValue()->Get(divergence_);
      else if (name == "growthFactor")
        _sdf->GetValue()->Get(growthFactor_);
      else if (name == "ejectSpeed")
        _sdf->GetValue()->Get(ejectSpeed_);
      else if (name == "bufferSize")
        _sdf->GetValue()->Get(bufferSize_);
      else if (name == "useROS")
        _sdf->GetValue()->Get(useROS_);
      else if (name == "robotNamespace")
        break;
      else
        throw std::runtime_error("Invalid parameter for FoamBuildPlugin");

      _sdf = _sdf->GetNextElement();
    } while (_sdf);

    if(useROS_){
      ros::NodeHandle nh;

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      m_SubExtrusion = nh.subscribe<std_msgs::Bool>("wasp/extrusion", 10, &FoamBuildPlugin::extruder_cb, this);
    }

    extruding = !useROS_;

    th = std::thread(&FoamBuildPlugin::emitter, this);

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectRender(std::bind(&FoamBuildPlugin::OnUpdate, this));
    this->removeConnection = event::Events::ConnectPause(std::bind(&FoamBuildPlugin::OnRemove, this));
}

void FoamBuildPlugin::extruder_cb(const std_msgs::Bool::ConstPtr& msg){
    extruding = (*msg).data;
}

void FoamBuildPlugin::emitter(){
  while(true){
    if(extruding){
      emit();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void FoamBuildPlugin::emit(){

    // Calculate appropriate direction for ejection
    ignition::math::Pose3d wp = this->visual->WorldPose();
    ignition::math::Vector3d ray = (wp.Rot() * ignition::math::Vector3d(0, 0, -1)).Normalize();

    // Apply random error to ejection direction
    double phi = random.DblUniform(0, IGN_PI * divergence_ / 180), theta = random.DblUniform(-IGN_PI, IGN_PI);
    ray = (ignition::math::Quaterniond(ray, theta) * (ignition::math::Quaterniond(ray.Perpendicular(), phi) * ray)).Normalize();

    // Check for collision with ground or other segments
    double t = -1;
    bool hit = intersectSphereFoam(segments, wp.Pos(), particleSize_ / growthFactor_, ray, t);
    if(!hit) hit = intersectSphereGround(wp.Pos(), particleSize_ / growthFactor_, ray, t);

    // Only create particle if there is a collision (Assumes linear path which otherwise introduces issues)
    if(hit){
      spawnParticles.push_back({wp, ray, t});
    }

    // std::cout << "VC" << this->visual->GetScene()->VisualCount() << " B" << k << " BUF" << buffer.size() << "/" << bufferSize_ << " P" << particles.size() << "\n";
}

void FoamBuildPlugin::spawn_particles(){
    static int ID = 0;

    for(uint32_t i = 0; i < spawnParticles.size(); i++){
      std::string name = "V" + std::to_string(ID++);
      // std::cout << name << "\n";

      rendering::VisualPtr vis = std::make_shared<rendering::Visual>(name, this->visual->GetScene());
      this->visual->GetScene()->AddVisual(vis);

      vis->AttachObject(Spheres(name, (float)particleSize_, 0));

      vis->SetScale(ignition::math::Vector3d(1, 1, 1) / growthFactor_);
      vis->SetPose(spawnParticles[i].wp);

      particles.push_back({vis, spawnParticles[i].wp.Pos(), spawnParticles[i].ray, 0, spawnParticles[i].tlim});
    }

    spawnParticles.clear();
}

void FoamBuildPlugin::update_particles(){
  for(uint32_t i = 0; i < particles.size(); i++){
    ignition::math::Vector3d scale = particles[i].vis->Scale();
    scale *= scale.X() > 1.0 ? 1.0 : 1.1;
    particles[i].vis->SetScale(scale);

    particles[i].t = std::min(particles[i].tlim, particles[i].t + ejectSpeed_);
    ignition::math::Vector3d pos = particles[i].start + particles[i].ray * particles[i].t;
    particles[i].vis->SetPosition(pos);

    if(scale.X() > 1.0 && particles[i].t == particles[i].tlim){
      segments.push_back(pos);
      buffer.push_back(particles[i]);
      particles.erase(particles.begin() + i);
      i--;
      continue;
    }
  }
}

void FoamBuildPlugin::merge_particles() {
  static int BLOCK_ID = 0;

  if(buffer.size() > bufferSize_){
    std::vector<ignition::math::Vector3f> poses;
    for(const Particle& particle : buffer){
      particle.vis->Fini();// Remove old visuals
      ignition::math::Vector3d pos = particle.start + particle.ray * particle.tlim;
      poses.emplace_back((float)pos.X(), (float)pos.Y(), (float)pos.Z());
    }

    // Create new visual composed of a block of foam particles
    std::string name = "BLOCK" + std::to_string(BLOCK_ID++);
    rendering::VisualPtr vis = std::make_shared<rendering::Visual>(name, this->visual->GetScene());
    this->visual->GetScene()->AddVisual(vis);

    vis->AttachObject(Spheres(name, (float)particleSize_, 0, poses));

    buffer.clear();
  }
}

void FoamBuildPlugin::OnUpdate() {
  spawn_particles();
  update_particles();
  merge_particles();
}
void FoamBuildPlugin::OnRemove() {}

bool FoamBuildPlugin::intersectSphereGround(const ignition::math::Vector3d& origin, double radius, const ignition::math::Vector3d& vel, double& t){
  ignition::math::Vector3d normal (0, 0, 1);
  double dist = normal.Dot(origin);
  if(std::abs(dist) <= radius){
    t = 0;
    return true;
  }

  double den = normal.Dot(vel);
  if(den * dist >= 0){
    return false;
  }

  double r = dist > 0 ? radius : -radius;
  t = (r - dist) / den;
  return true;
}

bool FoamBuildPlugin::intersectSphereFoam(const std::vector<ignition::math::Vector3d>& foam, const ignition::math::Vector3d& origin, double radius, const ignition::math::Vector3d& vel, double& t){
  t = -1;
  for(const ignition::math::Vector3d& pos : foam){
      double temp = 0;

      ignition::math::Vector3d delta = origin - pos;
      double r = particleSize_ + radius;
      double c = delta.Dot(delta) - r * r;
      if(c < 0){
        t = 0;
        return true;
      }

      double a = vel.Dot(vel);
      if(a < 0.0001) continue;

      double b = vel.Dot(delta);
      if(b >= 0) continue;

      double d = b * b - a * c;
      if(d < 0) continue;

      temp = (-b - sqrt(d)) / a;
      if(t < 0 || (temp < t) && temp > 0){
        t = temp;
      }
  }

  return t >= 0;
}

Ogre::ManualObject* FoamBuildPlugin::Spheres(std::string name, float radius, uint32_t subdivisions, std::vector<ignition::math::Vector3f> poses){
  if(poses.empty()){
    poses.emplace_back(0, 0, 0);
  }

  std::vector<ignition::math::Vector3f> vertices;
  std::vector<Triangle> triangles;
  Icosphere(vertices, triangles, radius, subdivisions);

  Ogre::ManualObject* man = this->visual->GetScene()->OgreSceneManager()->createManualObject(name);
  man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  int idx = 0;
  for(const ignition::math::Vector3f& offset : poses){

      for(const ignition::math::Vector3f& vertex : vertices){
        man->position(radius * vertex.X() + offset.X(), radius * vertex.Y() + offset.Y(), radius * vertex.Z() + offset.Z());
        man->normal(vertex.X(), vertex.Y(), vertex.Z());
      }

      // Generate vertex indices
      for(const Triangle& tri : triangles){
        man->triangle(idx + tri.m_I0, idx + tri.m_I1, idx + tri.m_I2);
      }

      idx += vertices.size();
  }

    man->end();
    return man;
}

void FoamBuildPlugin::Icosphere(std::vector<ignition::math::Vector3f>& vertices, std::vector<Triangle>& triangles, float radius, uint32_t subdivisions){
    Icosahedron(vertices, triangles);

    std::map<uint64_t, uint32_t> table;
    for(uint8_t i = 0; i < subdivisions; i++){
      uint32_t triangleCount = triangles.size();

      for(uint32_t j = 0; j < triangleCount; j++){
        Triangle tri = triangles[j];

        uint32_t a = MidPoint(vertices, table, tri.m_I0, tri.m_I1);
        uint32_t b = MidPoint(vertices, table, tri.m_I1, tri.m_I2);
        uint32_t c = MidPoint(vertices, table, tri.m_I2, tri.m_I0);

        triangles.push_back({tri.m_I0, a, c});
        triangles.push_back({tri.m_I1, b, a});
        triangles.push_back({tri.m_I2, c, b});
        triangles[j] = {a, b, c};

      }
    }
}

uint32_t FoamBuildPlugin::MidPoint(std::vector<ignition::math::Vector3f>& vertices, std::map<uint64_t, uint32_t>& table, uint64_t iA, uint64_t iB){
  uint64_t key = iA > iB ? (iA << 32) + iB : (iB << 32) + iA;

  auto it = table.find(key);
  if(it == table.end()){
    vertices.push_back((vertices[iA] + vertices[iB]).Normalize());
    return table[key] = vertices.size() - 1;
  }

  return table[key];
}

void FoamBuildPlugin::Icosahedron(std::vector<ignition::math::Vector3f>& vertices, std::vector<Triangle>& triangles){
    float t = (float)(1 + sqrt(5) / 2);

    vertices.emplace_back(-1,  t, 0);
    vertices.emplace_back( 1,  t, 0);
    vertices.emplace_back(-1, -t, 0);
    vertices.emplace_back( 1, -t, 0);

    vertices.emplace_back(0, -1,  t);
    vertices.emplace_back(0,  1,  t);
    vertices.emplace_back(0, -1, -t);
    vertices.emplace_back(0,  1, -t);

    vertices.emplace_back( t, 0, -1);
    vertices.emplace_back( t, 0,  1);
    vertices.emplace_back(-t, 0, -1);
    vertices.emplace_back(-t, 0,  1);

    for(ignition::math::Vector3f& vertex : vertices){
      vertex = vertex.Normalize();
    }

    triangles.push_back({0, 11, 5});
    triangles.push_back({0, 5, 1});
    triangles.push_back({0, 1, 7});
    triangles.push_back({0, 7, 10});
    triangles.push_back({0, 10, 11});

    triangles.push_back({1, 5, 9});
    triangles.push_back({5, 11, 4});
    triangles.push_back({11, 10, 2});
    triangles.push_back({10, 7, 6});
    triangles.push_back({7, 1, 8});

    triangles.push_back({3, 9, 4});
    triangles.push_back({3, 4, 2});
    triangles.push_back({3, 2, 6});
    triangles.push_back({3, 6, 8});
    triangles.push_back({3, 8, 9});

    triangles.push_back({4, 9, 5});
    triangles.push_back({2, 4, 11});
    triangles.push_back({6, 2, 10});
    triangles.push_back({8, 6, 7});
    triangles.push_back({9, 8, 1});

}
