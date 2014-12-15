#ifndef _RIGIDBODY_
#define _RIGIDBODY_

#include <Eigen/Dense>
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"

namespace dart{
    namespace renderer {
        class RenderInterface;
    }
}

class RigidBody {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RigidBody(dart::dynamics::Shape::ShapeType _type, Eigen::Vector3d _dim) {
        // Create a default rigid body
        mMass = 1.0;
        mPosition.setZero(); // x = (0, 0, 0)
        mOrientation.setIdentity(); // R = identity
        mColor << 0.9, 0.2, 0.2, 1.0; // Red
        if (_type == dart::dynamics::Shape::BOX) {
            mShape = new dart::dynamics::BoxShape(_dim);
            shapeID = 0;
        } else if (_type == dart::dynamics::Shape::ELLIPSOID) {
            mShape = new dart::dynamics::EllipsoidShape(_dim);
            shapeID = 1;
        }
        mLinMomentum.setZero();
        mAngMomentum.setZero();
    }
    virtual ~RigidBody() {}

    void draw(dart::renderer::RenderInterface* _ri);
    
    double mMass;
    int shapeID;
    Eigen::Vector3d mPosition;
    Eigen::Matrix3d mOrientation;
    Eigen::Vector3d mLinMomentum;
    Eigen::Vector3d mAngMomentum;
    dart::dynamics::Shape* mShape;

    Eigen::Vector4d mColor;
};

#endif
