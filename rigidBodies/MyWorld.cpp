#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "dart/utils/urdf/DartLoader.h"
#include "dart/utils/Paths.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/BodyNode.h"
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include "dart/dynamics/MeshShape.h"

#include<cstdio>
#include<iostream>

using namespace Eigen;

MyWorld::MyWorld() {

    rotation = 0.005;
    mFrame = 0;
    mTimeStep = 0.005;
    mGravity = Vector3d(0.0, -9.8, 0.0);
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();

    // Create and intialize two default rigid bodies (You can add more rigid bodies if you want) 
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.9, 0.9, 0.9));
    mCube = mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -1.0;
    rb1->mAngMomentum = Vector3d(0.0, 0.1, 0.0);
    rb1->mMass = 2.0;
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.9, 0.9 , 0.9));
    mSphere = mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 1.0;
    rb1->mAngMomentum = Vector3d(0.1, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    rb2->mMass = 2.0;
    mRigidBodies.push_back(rb2);

    // Load a blender and a blade
    dart::utils::DartLoader dl;
    std::string blenderFileName(DART_DATA_PATH"urdf/cylinder.urdf");
    mBlender = dl.parseSkeleton(blenderFileName);
    if (mBlender) {
      mBlender->init();
    }
    mCollisionDetector->addSkeleton(mBlender); // Put blender in collision detector

    //Make the blender a WIREFRAME -- this is a hack as URDF doesn't have material properties signifying wireframes to
    //allow assimp to handle this autmatically
    dart::dynamics::MeshShape* shape = (dart::dynamics::MeshShape*)mBlender->getBodyNode(0)->getVisualizationShape(0);
    bool wireFrame = true;
    shape->getMesh()->mMaterials[0]->AddProperty(&wireFrame, 1, AI_MATKEY_ENABLE_WIREFRAME);


    std::string bladeFileName(DART_DATA_PATH"urdf/blade.urdf");
    mBlade = dl.parseSkeleton(bladeFileName);
    if (mBlade) {
      mBlade->init();
    }

    VectorXd pose = mBlade->getState();
    pose[4] = -4.0;
    mBlade->setState(pose);
    mBlade->computeForwardKinematics(true, true, false);
    mCollisionDetector->addSkeleton(mBlade); // Put blade in collision detector
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
        delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
        delete mCollisionDetector;
    if (mBlender)
        delete mBlender;
    if (mBlade)
        delete mBlade;
}

void MyWorld::increaseRotation(float f){
    rotation += f;
}
void MyWorld::decreaseRotation(float f){
    rotation -= f;
}
void MyWorld::simulate() {

    // XXX: 1) Need to assign values correctly in collisionhandling where rb2 is not null
    mFrame++;

    Eigen::Vector3d w;
    Eigen::Matrix3d star;

    Eigen::Matrix3d iBody;
    Eigen::Matrix3d iT;
    float inertia; 

    for (int i = 0; i < mRigidBodies.size(); i++) {
        mRigidBodies[i]->mPosition += mTimeStep * (mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass);
        mRigidBodies[i]->mLinMomentum += mTimeStep * (mRigidBodies[i]->mMass * mGravity);



        if(mRigidBodies[i]->mShape->getShapeType() == dart::dynamics::Shape::BOX) {
            inertia = (mRigidBodies[i]->mMass * 0.9 * 0.9)/6;
        }
        else {
            inertia = (mRigidBodies[i]->mMass * 0.4 * 0.45 * 0.45);
        }
        iBody << inertia, 0, 0,
                      0, inertia, 0,
                      0, 0, inertia;
        
        iT = mRigidBodies[i]->mOrientation * iBody * (mRigidBodies[i]->mOrientation).transpose();
        w = iT.inverse() * mRigidBodies[i]->mAngMomentum;

	    star << 0, -1 * w.z(), w.y(),
             w.z(), 0, -1 * w.x(),
             -1 * w.y(), w.x(), 0;

       std::cout << "Orientation transformation before" << std::endl;
       std::cout << mRigidBodies[i]->mOrientation << std::endl;
       mRigidBodies[i]->mOrientation += mTimeStep * star * mRigidBodies[i]->mOrientation;

       Eigen::Quaterniond q(mRigidBodies[i]->mOrientation + mTimeStep * star * mRigidBodies[i]->mOrientation);

       q.normalize();

       mRigidBodies[i]->mOrientation = q.toRotationMatrix();

       //mRigidBodies[i]->mOrientation.normalize();
       std::cout << "Orientation transformation afterwards" << std::endl;
       std::cout << mRigidBodies[i]->mOrientation << std::endl;


    }

    // Run collision detector
    mCollisionDetector->checkCollision();

    // TODO: make a better collision handler
    collisionHandling();

    // Move the blade
    VectorXd pose = mBlade->getState();
    pose[1] += rotation;
    if (pose[1] > 2 * 3.14)
    {
        pose[1] = 0.0;
    }
    mBlade->setState(pose);
    mBlade->computeForwardKinematics(true, true, false);
}

void MyWorld::collisionHandling() {

	// We'll have to calculate the changes that would happen from all the contacts

    RigidContact rc;

    RigidBody *rb1;
    RigidBody *rb2;

    Eigen::Vector3d contactPos;
    Eigen::Vector3d normal;

    Eigen::Vector3d VaMinus;
    Eigen::Vector3d VbMinus;

    Eigen::Vector3d WaMinus;
    Eigen::Vector3d WbMinus;

    Eigen::Vector3d VaPlus;
    Eigen::Vector3d VbPlus;

    Eigen::Vector3d rA;
    Eigen::Vector3d rB;

    Eigen::Vector3d PbDotMinus;
    Eigen::Vector3d PaDotMinus;

    float j;
    float jNumerator;
    float jDenominator1stTerm;
    float jDenominator2ndTerm;

    Eigen::Matrix3d iBody1;
    Eigen::Matrix3d iT1;
    float inertia1;

    Eigen::Matrix3d iBody2;
    Eigen::Matrix3d iT2;
    float inertia2;



    float vRMinus;
    Eigen::Vector3d wBlade = Vector3d(0,rotation/mTimeStep,0);

	for (int i = 0; i < mCollisionDetector->getNumContacts(); i++){
        
        rc = mCollisionDetector->getContact(i);

        rb1 = rc.rb1;
        rb2 = rc.rb2;

        if(rc.skel1 == mCube){
            if(rc.skel2 == mBlade){
                printf("Cube and Blade colliding\n");
            }
            else if(rc.skel2 == mBlender){
                printf("Cube and Blender colliding\n");
            }
            else if(rc.skel2 == mSphere){
                printf("Cube and Sphere colliding\n");
            }
        }
        else if(rc.skel1 == mSphere){
            if(rc.skel2 == mBlade){
                printf("Sphere and Blade colliding\n");
            }
            else if(rc.skel2 == mBlender){
                printf("Sphere and Blender colliding\n");
            }
        }

        contactPos = mCollisionDetector->getContact(i).point;

        printf("rigid body ID = %d, contact point = %f, %f, %f\n", rb1->shapeID, contactPos.x(), contactPos.y(), contactPos.z());
        normal = mCollisionDetector->getContact(i).normal;

        VaMinus = rb1->mLinMomentum/rb1->mMass;

        if(rb2){
            VbMinus = rb2->mLinMomentum/rb2->mMass;
        }
        else{
            VbMinus = Vector3d(0,0,0);
        }


        if (rc.skel1 == mCube){
            inertia1 = (rb1->mMass * 0.9 * 0.9)/6;
        }
        else if(rc.skel1 == mSphere){
            inertia1 = (rb1->mMass * 0.4 * 0.45 * 0.45);
        }
        iBody1 << inertia1, 0, 0,
                         0, inertia1, 0,
                         0, 0, inertia1;

        iT1 = rb1->mOrientation * iBody1 * rb1->mOrientation.transpose();

        if(rb2){
            if(rc.skel2 == mCube){
                inertia2 = (rb2->mMass * 0.9 * 0.9)/6;
            }
            else if(rc.skel2 == mSphere){
                inertia2 = rb2->mMass * 0.4 * 0.45 * 0.45;
            }
            iBody2 << inertia2, 0, 0,
                      0, inertia2, 0,
                      0, 0, inertia2;

            iT2 = rb2->mOrientation * iBody2 * rb2->mOrientation.transpose();
        }

        

        WaMinus = iT1.inverse() * rb1->mAngMomentum; // Icube = ms*s/6

        if(rb2){
            WbMinus = iT2.inverse() * rb2->mAngMomentum;
        }
        else if (rc.skel2 == mBlade){
            WbMinus = wBlade;
        }
        else{
            WbMinus = Vector3d(0,0,0);
        }

        rA = contactPos - rb1->mPosition;

        if (rb2){
            rB = contactPos - rb2->mPosition;
        }
        else if(rc.skel2 == mBlade){
            rB = Vector3d(contactPos.x(), 0, contactPos.z());
        }
        else{
            // It doesn't matter here
            rB = Vector3d(0,0,0);
        }

        PaDotMinus = VaMinus + WaMinus.cross(rA);

        PbDotMinus = VbMinus + WbMinus.cross(rB);


        vRMinus = normal.dot(PaDotMinus - PbDotMinus);

        // Computing j, assuming coefficient of restitution = 1

        jNumerator = -2 * vRMinus;
        jDenominator1stTerm = (1/rb1->mMass);

        if(rb2){
            jDenominator1stTerm += (1/rb2->mMass);
        }

        jDenominator2ndTerm = normal.dot((iT1.inverse() * rA.cross(normal)).cross(rA));
        
        if(rb2){
            jDenominator2ndTerm += normal.dot((iT2.inverse() * rB.cross(normal)).cross(rB));
        }

        j = jNumerator/(jDenominator1stTerm + jDenominator2ndTerm);

        
        rb1->mLinMomentum += j * normal;
        rb1->mAngMomentum += rA.cross(j * normal);

        if(rb2){
            rb2->mLinMomentum += -1 * j * normal;
            rb2->mAngMomentum += rB.cross(-1 * j * normal); 
        }

	}
}
