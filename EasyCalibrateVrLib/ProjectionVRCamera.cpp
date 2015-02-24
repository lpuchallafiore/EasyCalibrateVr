//  Copyright Regents of the University of Minnesota and Brown University, 2010.  All rights are reserved.

#include "stdafx.h"
#include "ProjectionVRCamera.h"

namespace VRG3D {

using namespace G3D;

ProjectionVRCamera::ProjectionVRCamera(DisplayTile dispTile, CoordinateFrame initHeadFrame,
                                       double interOcularDist)
{
  tile = dispTile;
  headFrame = initHeadFrame;
  iod = interOcularDist;
}

ProjectionVRCamera::~ProjectionVRCamera()
{
}


CoordinateFrame
ProjectionVRCamera::getLeftEyeFrame()
{
  return headFrame * CoordinateFrame(Vector3(-iod/2.0, 0.0, 0.0));
}

CoordinateFrame
ProjectionVRCamera::getRightEyeFrame()
{
  return headFrame * CoordinateFrame(Vector3( iod/2.0, 0.0, 0.0));
}


void
ProjectionVRCamera::applyProjection(RenderDevice *rd, EyeProjectionType whichEye, GCamera* cam)
{
  // 1. Get the center of the camera (the eye) position from the head position
  CoordinateFrame eye2room = headFrame;

  lastProjectionEye = whichEye;

  if (whichEye == LeftEye)
  {
    eye2room = getLeftEyeFrame();
  }
  else if (whichEye == RightEye)
  {
    eye2room = getRightEyeFrame();
  }

  // 2. Setup projection matrix
  Vector3 eye = (tile.room2tile * eye2room).translation;
  double halfWidth = (tile.topRight - tile.topLeft).length() / 2.0;
  double halfHeight = (tile.topRight - tile.botRight).length() / 2.0;
  double l = (-halfWidth - eye[0]);
  double r = ( halfWidth - eye[0]);
  double b = (-halfHeight - eye[1]);
  double t = ( halfHeight - eye[1]);
  double dist = eye[2];
  double k = tile.nearClip / dist;

  // 3. Add eye position to the projection (eye is in tile coordinates)
  CoordinateFrame r2t = CoordinateFrame(-eye) * tile.room2tile;

  // 4. Apply the projection to the RenderDevice
  rd->setProjectionMatrix( Matrix4::perspectiveProjection(l*k, r*k, b*k, t*k, tile.nearClip, tile.farClip) );
  rd->setCameraToWorldMatrix(r2t.inverse());
  if (cam) { // hack to make it work (trick to disable all culling) for demos, TODO FIXME
	  cam->setCoordinateFrame(r2t.inverse());
	  cam->setFarPlaneZ(-500);
	  cam->setNearPlaneZ(-0.01);
	  cam->setFieldOfView(3.14, GCamera::FOVDirection::HORIZONTAL);
  }
}

void
ProjectionVRCamera::getCurrentMatrices(Matrix4 &view, Matrix4 &projection,
									   Matrix4 &viewLeft, Matrix4 &projectionLeft,
									   Matrix4 &viewRight, Matrix4 &projectionRight)
{
  // 1. Get the center of the camera (the eye) position from the head position
  CoordinateFrame head2Room = headFrame;
  CoordinateFrame leftEye2Room = getLeftEyeFrame();
  CoordinateFrame rightEye2Room = getRightEyeFrame();
  
  // 2. Setup projection matrix
  Vector3 head = (tile.room2tile * head2Room).translation;
  Vector3 left = (tile.room2tile * leftEye2Room).translation;
  Vector3 right = (tile.room2tile * rightEye2Room).translation;
  double halfWidth = (tile.topRight - tile.topLeft).length() / 2.0;
  double halfHeight = (tile.topRight - tile.botRight).length() / 2.0;

  double lHead = (-halfWidth - head[0]);
  double rHead = ( halfWidth - head[0]);
  double lLeft = (-halfWidth - left[0]);
  double rLeft = ( halfWidth - left[0]);
  double lRight = (-halfWidth - right[0]);
  double rRight = ( halfWidth - right[0]);

  // y and z for head and eyes are the same so don't need to calculate three times
  double b = (-halfHeight - head[1]);
  double t = ( halfHeight - head[1]);
  double dist = head[2];
  double k = tile.nearClip / dist;

  // 3. Add eye position to the projection (eye is in tile coordinates)
  CoordinateFrame r2t = CoordinateFrame(-head) * tile.room2tile;
  CoordinateFrame r2tLeft = CoordinateFrame(-left) * tile.room2tile;
  CoordinateFrame r2tRight = CoordinateFrame(-right) * tile.room2tile;

  projection = Matrix4::perspectiveProjection(lHead*k, rHead*k, b*k, t*k, tile.nearClip, tile.farClip);
  projectionLeft = Matrix4::perspectiveProjection(lLeft*k, rLeft*k, b*k, t*k, tile.nearClip, tile.farClip);
  projectionRight = Matrix4::perspectiveProjection(lRight*k, rRight*k, b*k, t*k, tile.nearClip, tile.farClip);
  view = Matrix4(r2t);//.inverse());
  viewLeft = Matrix4(r2tLeft);//.inverse());
  viewRight = Matrix4(r2tRight);//.inverse());
}


Vector3
ProjectionVRCamera::getLookVec()
{
  Vector3 filmPlaneCtr = tile.topLeft + 0.5*(tile.topRight-tile.topLeft) + 0.5*(tile.botLeft-tile.topLeft);
  return (filmPlaneCtr - headFrame.translation).unit();
}

} // end namespace

