#include "MyWindow.h"
#include "dart/gui/GLFuncs.h"
#include "RigidBody.h"
#include "dart/dynamics/Skeleton.h"
#include "CollisionInterface.h"
#include <cstdio>
#include <iostream>

using namespace Eigen;

void MyWindow::displayTimer(int _val) {

  if (mPlaying) {
    mPlayFrame++;
    if (mPlayFrame >= mBakedStates.size())
      mPlayFrame = 0;
  }else if (mSimulating) {
    mWorld->simulate();
    bake(); // Store the simulated state for playback
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
  if (!mSimulating) { // Playback mode
    if (mPlayFrame < mBakedStates.size()) {
      int numRigids = mWorld->getNumRigidBodies();
      for (int i = 0; i < numRigids; i++) { 
        mWorld->getRigidBody(i)->mPosition = mBakedStates[mPlayFrame].segment(i * 12, 3);
        for (int j = 0; j < 3; j++)
          mWorld->getRigidBody(i)->mOrientation.row(j) = mBakedStates[mPlayFrame].segment(i * 12 + 3 + j * 3, 3);
      }
      if (mShowMarkers) {
        int sumDofs = mWorld->getNumRigidBodies() * 12;
        int nContact = (mBakedStates[mPlayFrame].size() - 1 - sumDofs) / 6;
        for (int i = 0; i < nContact; i++) {
          Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
          Vector3d n = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 1.0;
          glBegin(GL_LINES);
          glVertex3f(v[0], v[1], v[2]);
          glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
          glEnd();
          mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
          mRI->pushMatrix();
          mRI->translate(v);
          mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
          mRI->popMatrix();
        }
      }
      VectorXd pose = mWorld->getBlade()->getState();
      pose[1] = mBakedStates[mPlayFrame][mBakedStates[mPlayFrame].size() - 1];
      mWorld->getBlade()->setState(pose);
    }
  }else{ // Simulation mode
    if (mShowMarkers) {
      std::cout << "Frame = " << mWorld->getSimFrames() << " number of contacts = " <<  mWorld->getCollisionDetector()->getNumContacts() << std::endl;
      for (int k = 0; k < mWorld->getCollisionDetector()->getNumContacts(); k++) {
        Vector3d  v = mWorld->getCollisionDetector()->getContact(k).point;
        Vector3d n = mWorld->getCollisionDetector()->getContact(k).normal / 1.0;
        glBegin(GL_LINES);
        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
        glEnd();
        mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
        mRI->pushMatrix();
        mRI->translate(v);
        mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
        mRI->popMatrix();
      }
    }
  }

  // Draw rigid bodies
  for (int i = 0; i < mWorld->getNumRigidBodies(); i++)
    mWorld->getRigidBody(i)->draw(mRI);
    
  // Draw the blender and blade
  mWorld->getBlender()->draw(mRI);
  mWorld->getBlade()->draw(mRI);

  // Display the frame count in 2D text
  char buff[64];
  if (!mSimulating)
    sprintf(buff, "%d", mPlayFrame);
  else
    sprintf(buff, "%d", mWorld->getSimFrames());
  std::string frame(buff);
  glDisable(GL_LIGHTING);
  glColor3f(0.0,0.0,0.0);
  dart::gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}


void MyWindow::keyboard(unsigned char key, int x, int y) 
{
  switch(key) {        
  case ' ': // Use space key to play or stop the motion
    mSimulating = !mSimulating;
    if(mSimulating) {
      mPlaying = false;
      glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
    }
    break;
        
  case 'p': // playBack
    mPlaying = !mPlaying;
    if (mPlaying) {
      mSimulating = false;
      glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
    }
    break;
  case '[': // step backward
    if (!mSimulating) {
      mPlayFrame--;
      if(mPlayFrame < 0)
        mPlayFrame = 0;
      glutPostRedisplay();
    }
    break;
  case ']': // step forwardward
    if (!mSimulating) {
      mPlayFrame++;
      if(mPlayFrame >= mBakedStates.size())
        mPlayFrame = 0;
      glutPostRedisplay();
    }
    break;

  case 'v': // show or hide markers
    mShowMarkers = !mShowMarkers;
    break;
  case 'i':
    mWorld->increaseRotation(0.001);
    break;
  case 'd':
    mWorld->decreaseRotation(0.001);
    break;
  default:
    Win3D::keyboard(key,x,y);
  }
  glutPostRedisplay();
}


void MyWindow::bake()
{
  int nContact = mWorld->getCollisionDetector()->getNumContacts();
  VectorXd state(12 * mWorld->getNumRigidBodies() + 6 * nContact + 1);
  for (int i = 0; i < mWorld->getNumRigidBodies(); i++) {
    state.segment(i * 12, 3) = mWorld->getRigidBody(i)->mPosition;
    for (int j = 0; j < 3; j++)
      state.segment(i * 12 + 3 + j * 3, 3) = mWorld->getRigidBody(i)->mOrientation.row(j);
  }

  for (int i = 0; i < nContact; i++) {
    int begin = 12 * mWorld->getNumRigidBodies() + i * 6;
    state.segment(begin, 3) = mWorld->getCollisionDetector()->getContact(i).point;
    state.segment(begin + 3, 3) = mWorld->getCollisionDetector()->getContact(i).normal;
  }
  state[state.size() - 1] = mWorld->getBlade()->getState()[1];
  mBakedStates.push_back(state);
}
