#include "../smplh_playback/SmplhDistanceWorker.h"

#include <cmath>
#include <stdexcept>
#include <iostream>

void testDeformingMesh()
{
  using mc_mujoco::SmplhBVH;
  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> faces;
  constexpr int side=9;
  for(int y=0;y<side;++y)
    for(int x=0;x<side;++x) vertices.emplace_back(x*.1,y*.1,0);
  for(int y=0;y<side-1;++y)
    for(int x=0;x<side-1;++x)
    {
      const int a=y*side+x;
      faces.emplace_back(a,a+1,a+side);
      faces.emplace_back(a+1,a+side+1,a+side);
    }
  auto refitted=std::make_shared<SmplhBVH>();
  refitted->beginModel(); refitted->addSubModel(vertices,faces); refitted->endModel();
  // A second triangle mesh, tested with rotations and both separated and
  // intersecting placements. Unlike a single-triangle test, this exercises
  // internal hierarchy nodes as the human surface bends and moves.
  auto robot=std::make_shared<SmplhBVH>();
  const std::vector<fcl::Vector3d> robotVertices{{-.2,-.2,0},{.2,-.2,0},{.2,.2,0},{-.2,.2,0}};
  const std::vector<fcl::Triangle> robotFaces{{0,1,2},{0,2,3}};
  robot->beginModel(); robot->addSubModel(robotVertices,robotFaces); robot->endModel();
  for(double pose : {1.,-2.,3.,0.,.5,0.})
  {
    for(int y=0;y<side;++y)
      for(int x=0;x<side;++x)
        vertices[y*side+x]=fcl::Vector3d(x*.1+pose*.03,y*.1,
                                       .25*std::sin(x*.7+pose)*std::cos(y*.4-pose));
    if(refitted->beginReplaceModel()!=fcl::BVH_OK
       || refitted->replaceSubModel(vertices)!=fcl::BVH_OK
       || refitted->endReplaceModel(true,false)!=fcl::BVH_OK)
      throw std::runtime_error("deforming top-down refit failed");
    auto fresh=std::make_shared<SmplhBVH>();
    fresh->beginModel(); fresh->addSubModel(vertices,faces); fresh->endModel();
    fcl::CollisionObjectd updatedObject(refitted), freshObject(fresh);
    for(double height : {1.,.35,0.,-.1})
    {
      auto transform=Eigen::Isometry3d::Identity();
      transform.linear()=Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
      transform.translation()=Eigen::Vector3d(.4,.4,height);
      fcl::CollisionObjectd robotObject(robot,transform);
      fcl::CollisionResultd updatedCollision,freshCollision;
      const bool updatedHit=fcl::collide(&updatedObject,&robotObject,fcl::CollisionRequestd(),updatedCollision)>0;
      const bool freshHit=fcl::collide(&freshObject,&robotObject,fcl::CollisionRequestd(),freshCollision)>0;
      if(updatedHit!=freshHit) throw std::runtime_error("refit intersection differs from fresh BVH");
      if(freshHit) continue;
      fcl::DistanceResultd updatedDistance,freshDistance;
      const double actual=fcl::distance(&updatedObject,&robotObject,fcl::DistanceRequestd(true),updatedDistance);
      const double expected=fcl::distance(&freshObject,&robotObject,fcl::DistanceRequestd(true),freshDistance);
      if(!std::isfinite(actual) || std::abs(actual-expected)>1e-9)
        throw std::runtime_error("deforming refit distance differs from fresh BVH");
      if(std::abs((updatedDistance.nearest_points[1]-updatedDistance.nearest_points[0]).norm()-actual)>1e-9)
        throw std::runtime_error("deforming refit witnesses do not match distance");
    }
  }
}

int main()
try
{
  testDeformingMesh();
  using mc_mujoco::SmplhBVH;
  std::vector<fcl::Vector3d> vertices{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<fcl::Triangle> faces{{0, 1, 2}};
  SmplhBVH model;
  model.beginModel();
  model.addSubModel(vertices, faces);
  model.endModel();

  // Move repeatedly, including a large jump and a return to the initial pose.
  for(double x : {10., -5., 0.})
  {
    for(auto & vertex : vertices) vertex.x() = x;
    if(model.beginReplaceModel() != fcl::BVH_OK
       || model.replaceSubModel(vertices) != fcl::BVH_OK
       || model.endReplaceModel(true, false) != fcl::BVH_OK)
      throw std::runtime_error("current-pose refit failed");
    if(model.prev_vertices != nullptr)
      throw std::runtime_error("refit retained previous-pose vertices");

    SmplhBVH fresh;
    fresh.beginModel();
    fresh.addSubModel(vertices, faces);
    fresh.endModel();
    // A refitted single-triangle bound must match a fresh current-pose bound.
    const auto & actual = model.getBV(0).bv;
    const auto & expected = fresh.getBV(0).bv;
    if((actual.To - expected.To).norm() > 1e-10
       || std::abs(actual.l[0] - expected.l[0]) > 1e-10
       || std::abs(actual.l[1] - expected.l[1]) > 1e-10
       || std::abs(actual.r - expected.r) > 1e-10)
      throw std::runtime_error("refit bounds differ from current-pose bounds");
  }
}
catch(const std::exception & error)
{
  std::cerr << error.what() << '\n';
  return 1;
}
