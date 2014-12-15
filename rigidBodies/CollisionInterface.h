#ifndef _COLLISIONINTERFACE_
#define _COLLISIONINTERFACE_

#include <vector>
#include <map>
#include <Eigen/Dense>

namespace dart {
     namespace dynamics {
        class BodyNode;
        class Skeleton;
    }
}

namespace dart {
    namespace collision {
        class CollisionDetector;
    }
}

class RigidBody;

struct RigidContact {
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    RigidBody* rb1;
    RigidBody* rb2;
    dart::dynamics::Skeleton* skel1;
    dart::dynamics::Skeleton* skel2;
};

class CollisionInterface {
 public:
    CollisionInterface();
    virtual ~CollisionInterface();

    void addSkeleton(dart::dynamics::Skeleton* _skel);
    dart::dynamics::Skeleton* addRigidBody(RigidBody *_rb, const std::string& name);

    // Run the collision detector
    void checkCollision();

    int getNumContacts() {
        return mContacts.size();
    }

    // Retrieve the information from the collision detector:
    // For example, get the position and the normal of the fifth contact point
    // Vector3d  v = mWorld->getCollisionDetector()->getContact(5).point;
    // Vector3d n = mWorld->getCollisionDetector()->getContact(k).normal;
    RigidContact& getContact(int _index) {
        return mContacts[_index];
    }

    dart::dynamics::BodyNode* getCollisionSkeleton(int body_index, int collision_index);

 private:
    void updateBodyNodes();
    void postProcess();

    dart::collision::CollisionDetector* mCollisionChecker;
    std::vector<RigidContact> mContacts;
    std::vector<dart::dynamics::Skeleton*> mSkeletons;
    std::map<dart::dynamics::BodyNode*, RigidBody*> mNodeMap;
    std::map<dart::dynamics::BodyNode*, dart::dynamics::Skeleton*> mSkelMap;
};

#endif
