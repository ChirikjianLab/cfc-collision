#include "test/util/DistanceQueryUtils.h"
#include "geometry/SuperQuadrics.h"

void queryDistanceMink(const Shape3D* s1,
                       const Shape3D* s2,
                       std::vector<double>* timeMink,
                       std::vector<DistanceInfo3D>* distInfoMink)
{
    collision_minksum::DistanceGradientLeastSquares<SuperQuadrics, SuperQuadrics> mink(
        *s1, *s2);
    auto start = Clock::now();
    mink.query();
    auto end = Clock::now();
    chrono::duration<double> time = end - start;

    // Retrieve time and distance info
    timeMink->push_back(time.count());
    distInfoMink->emplace_back(mink.getDistanceInfo());
}

void queryDistanceImplicit(const Shape3D* s1,
                           const Shape3D* s2,
                           std::vector<double>* timeImplicit,
                           std::vector<DistanceInfo3D>* distInfoImplicit)
{
    Distance3DImplicit<SuperQuadrics, SuperQuadrics> implicit(*s1, *s2);
    auto start = Clock::now();
    implicit.query();
    auto end = Clock::now();
    chrono::duration<double> time = end - start;

    // Retrieve time and distance info
    timeImplicit->emplace_back(time.count());
    distInfoImplicit->emplace_back(implicit.getDistanceInfo());
}
