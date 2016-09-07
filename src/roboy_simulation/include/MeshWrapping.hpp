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

        void UpdateForcePoints();



    public:
        math::Vector3 prevCoord;
        math::Vector3 nextCoord;

    /*
        struct Mesh
        {
            math::Vector3 wrappingDir;
            std::map<int, math::Vector3> vertices;
            std::set<math::Vector3> facets;
        };

        public: math::Vector3 x_axis;
        public: math::Vector3 y_axis;
        public: math::Vector3 z_axis;
        private: std::map<int, math::Vector3> convexHullVertices;
        private: std::map<int, math::Vector3> convexHullFacets;
        private: std::map<int, math::Vector3> facetNormals;
        private: double alphaMax;

        private: void CoordinateFrame();

        private: std::map<int, math::Vector3> HalfSpace();

        private: void ConvexEnvelope();

        private: std::map<int, math::Vector3> GeodesicPath();

        public: void MeshWrapping();
    */
	};
}

#endif
