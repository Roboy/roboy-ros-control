#ifndef _GAZEBO_MESHWRAPPING_HPP_
#define _GAZEBO_MESHWRAPPING_HPP_

#include "IViaPoints.hpp"

#include <boost/numeric/odeint.hpp>

namespace roboy_simulation
{
	using namespace gazebo;

	class ITendon;

	class MeshWrapping : public IViaPoints
	{

    public:
        MeshWrapping();

        virtual void UpdateForcePoints();

        virtual void CalculateForce();



    public:
        math::Vector3 prevCoord;
        math::Vector3 nextCoord;
        math::Vector3 wrappingDir;
        std::map<int, math::Vector3> vertices;
        std::set<math::Vector3> facets;
        math::Vector3 x_axis;
        math::Vector3 y_axis;
        math::Vector3 z_axis;
        std::map<int, math::Vector3> geodesicPathPoints;
        std::map<int, math::Vector3> halfSpace;
        std::map<int, math::Vector3> convexHullVertices;
        std::map<int, math::Vector3> convexHullFacets;
        std::map<int, math::Vector3> facetNormals;
        double alphaMax;

        void CoordinateFrame();
        void HalfSpace();
        void ConvexEnvelope();
        void GeodesicPath();

	};
}

#endif
