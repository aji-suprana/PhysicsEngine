///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
  mColor = Vector4(.6f);
  mMask = (unsigned int)-1;
  mTimer = 0;
  mOnTop = false;
  mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
  mColor = color;
  return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
  mOnTop = state;
  return *this;
}

DebugShape& DebugShape::Time(float time)
{
  mTimer = time;
  return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
  mMask = 1 << bitIndex;
  return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
  mTransform = transform;
  return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
  mActiveMask = (unsigned int)-1;
  mApplication = NULL;
}

void DebugDrawer::Update(float dt)
{
  std::vector<DebugShape> newShapes;
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];
    shape.mTimer -= dt;

    // If the shape still has time left then add it to the list of shapes to keep drawing,
    // anything that has a timer that ran out will not be in the new list
    if(shape.mTimer >= 0)
      newShapes.push_back(shape);
  }

  mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
  //MODIF:AJI ( DEBUG DRAW TESTING )
  gDebugDrawer->DrawLine(LineSegment(Vector3(-0, -0, -0), Vector3(1, 1, 1)));
  gDebugDrawer->DrawAabb(Aabb(Vector3(2.04614f, 0.412525f, -0.525727f), Vector3(6.36345f, 5.07618f, 1.69215f)));
  gDebugDrawer->DrawTriangle(Triangle(Vector3(0, 0, 0), Vector3(1, 1, 0), Vector3(1, 1, 1)));
  Vector3 p0 = Vector3(-2.18192f, -1.94761f, 0.846782f);
  Vector3 p1 = Vector3(-2.18192f, 3.30789f, -2.36682f);
  Vector3 p2 = Vector3(3.33754f, 3.49997f, -2.0527f);
  Vector3 p3 = Vector3(3.33754f, -1.75554f, 1.1609f);
  gDebugDrawer->DrawQuad(p0, p1, p2, p3);
  Plane plane(Vector3(0.066559f, -0.520518f, -0.851253f), Vector3(0.577812f, 0.776177f, -0.602959f));
  gDebugDrawer->DrawPlane(plane, 3, 4);
  gDebugDrawer->DrawSphere(Sphere(Vector3(-4.36159f, -2.82456f, 3.94174f), 2.29526f));
  gDebugDrawer->DrawRay(Ray(Vector3(0, 0, 0), Vector3(0, 0, 1)), 5.0f);
  Frustum frustum;
  frustum.Set(Vector3(-2.0169f, -9.57391f, -1.3047f), Vector3(2.42922f, -9.57391f, -1.3047f), Vector3(2.42922f, -2.83268f, -1.3047f), Vector3(-2.0169f, -2.83268f, -1.3047f), Vector3(-0.94259f, -8.4996f, -2.31514f), Vector3(1.3549f, -8.4996f, -2.31514f), Vector3(1.3549f, -3.90699f, -2.31514f), Vector3(-0.94259f, -3.90699f, -2.31514f));
  gDebugDrawer->DrawFrustum(frustum);

  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];

    // If the shape doesn't have one of the active mask bits set then don't draw it
    if((shape.mMask & mActiveMask) == 0)
      continue;
    
    // If this shape always draws on top then disable depth testing
    if(shape.mOnTop)
      glDisable(GL_DEPTH_TEST);


    // Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
    float radians;
    Vector3 scale, translation, axis;
    Matrix3 rotationMat;
    shape.mTransform.Decompose(&scale, &rotationMat, &translation);
    Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
    glPushMatrix();
    // Set the transform
    glTranslatef(translation.x, translation.y, translation.z);
    glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
    glScalef(scale.x, scale.y, scale.z);

    glBegin(GL_LINES);
    glColor3fv(shape.mColor.array);

    // Draw all of the line segments of this shape
    for(size_t j = 0; j < shape.mSegments.size(); ++j)
    {
      LineSegment& segment = shape.mSegments[j];

      glVertex3fv(segment.mStart.array);
      glVertex3fv(segment.mEnd.array);
    }

    glEnd();
    glPopMatrix();

    // Make sure to re-enable depth testing
    if(shape.mOnTop)
      glEnable(GL_DEPTH_TEST);
  }
}

DebugShape& DebugDrawer::GetNewShape()
{
  mShapes.push_back(DebugShape());
  return mShapes.back();
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
  return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
  /******Student:Assignment2******/
  // Draw a simple line
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(line);
  return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
  /******Student:Assignment2******/
  // Draw a ray to a given t-length. The ray must have an arrow head for visualization
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(ray.mStart,ray.mStart + ray.mDirection * t));

  //draw arrow
  //look for 2nd and 3rd vec
  float arrowSize = 0.2f;
  Vector3 u = ray.mDirection.Cross(Vector3::cXAxis);
  Vector3 v = u.Cross(ray.mDirection);
  Vector3 p0 = ray.mStart + ray.mDirection * (t - 2 * arrowSize);
  //decide 4 point for the quad
  Vector3 p1 = p0 + u * arrowSize;
  Vector3 p2 = p0 + v * arrowSize;
  Vector3 p3 = p0 - u * arrowSize;
  Vector3 p4 = p0 - v * arrowSize;

  shape.mSegments.push_back(LineSegment(p1, p2));
  shape.mSegments.push_back(LineSegment(p2, p3));
  shape.mSegments.push_back(LineSegment(p3, p4));
  shape.mSegments.push_back(LineSegment(p1, p4));

  shape.mSegments.push_back(LineSegment(p1, ray.mStart + ray.mDirection * t));
  shape.mSegments.push_back(LineSegment(p2, ray.mStart + ray.mDirection * t));
  shape.mSegments.push_back(LineSegment(p3, ray.mStart + ray.mDirection * t));
  shape.mSegments.push_back(LineSegment(p4, ray.mStart + ray.mDirection * t));

  return shape;
}

void GenerateCirclePoints(std::vector<Vector3>& generated, int resolution,float radius,int axis)
{
  float fullRound = Math::cTwoPi;
  float itteration = fullRound /resolution;

  for (int i = 0; i < resolution; ++i)
  {
    float currentItter = itteration * i;
    switch (axis)
    {
    case 0:
      generated.push_back(Vector3(0,cos(currentItter) * radius, sin(currentItter) * radius));
      break;
    case 1:
      generated.push_back(Vector3(cos(currentItter) * radius, 0, sin(currentItter) * radius));
      break;
    case 2:
      generated.push_back(Vector3(cos(currentItter) * radius, sin(currentItter) * radius, 0));
      break;
    }
  }
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
  /******Student:Assignment2******/
  // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
  // Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
  DebugShape& shape = GetNewShape();
  std::vector<Vector3> circlePoints;

  //draw in different axis
  for (int j = 0; j < 3; j++)
  {
    circlePoints.clear();
    GenerateCirclePoints(circlePoints, 20, sphere.mRadius, j);
    for (unsigned i = 0; i < circlePoints.size(); ++i)
    {
      if (i != circlePoints.size() - 1)
        shape.mSegments.push_back(LineSegment(circlePoints[i] + sphere.mCenter, circlePoints[i + 1] + sphere.mCenter));
      else
        shape.mSegments.push_back(LineSegment(circlePoints[i] + sphere.mCenter, circlePoints[0] + sphere.mCenter));
    }
  }



  return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
  /******Student:Assignment2******/
  // Draw all edges of an aabb. Make sure to not mis-match edges!
  DebugShape& shape = GetNewShape();  
  Vector3 Points[8] = {

    Vector3(aabb.mMax.x,aabb.mMax.y,aabb.mMin.z),
    Vector3(aabb.mMin.x,aabb.mMax.y,aabb.mMin.z),
    Vector3(aabb.mMin.x,aabb.mMin.y,aabb.mMin.z),
    Vector3(aabb.mMax.x,aabb.mMin.y,aabb.mMin.z),

    Vector3(aabb.mMax.x,aabb.mMax.y,aabb.mMax.z),
    Vector3(aabb.mMin.x,aabb.mMax.y,aabb.mMax.z),
    Vector3(aabb.mMin.x,aabb.mMin.y,aabb.mMax.z),
    Vector3(aabb.mMax.x,aabb.mMin.y,aabb.mMax.z)
  };

  shape.mSegments.push_back(LineSegment(Points[0], Points[1]));
  shape.mSegments.push_back(LineSegment(Points[1], Points[2]));
  shape.mSegments.push_back(LineSegment(Points[2], Points[3]));
  shape.mSegments.push_back(LineSegment(Points[0], Points[3]));

  shape.mSegments.push_back(LineSegment(Points[4], Points[5]));
  shape.mSegments.push_back(LineSegment(Points[5], Points[6]));
  shape.mSegments.push_back(LineSegment(Points[6], Points[7]));
  shape.mSegments.push_back(LineSegment(Points[4], Points[7]));


  shape.mSegments.push_back(LineSegment(Points[0], Points[4]));
  shape.mSegments.push_back(LineSegment(Points[1], Points[5]));
  shape.mSegments.push_back(LineSegment(Points[2], Points[6]));
  shape.mSegments.push_back(LineSegment(Points[3], Points[7]));
  return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
  /******Student:Assignment2******/
  // Draw the 3 edges of a triangles
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[1]));
  shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[2]));
  shape.mSegments.push_back(LineSegment(triangle.mPoints[1], triangle.mPoints[2]));


  return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
  /******Student:Assignment2******/
  // Draw a quad with a normal at the plane's center.
  DebugShape& shape = GetNewShape();
  Vector3 p0 = plane.GetNormal() * plane.GetDistance();

  //draw normal direction
  shape.mSegments.push_back(LineSegment(p0, plane.GetNormal() * sizeX * 0.5));

  //draw quad
  //look for 2nd and 3rd vec
  Vector3 u = plane.GetNormal().Cross(Vector3::cXAxis);
  Vector3 v = u.Cross(plane.GetNormal());

  //decide 4 point for the quad
  Vector3 p1 = p0 + u;
  Vector3 p2 = p0 + v;
  Vector3 p3 = p0 - u;
  Vector3 p4 = p0 - v;

  shape.mSegments.push_back(LineSegment(p1, p2));
  shape.mSegments.push_back(LineSegment(p2, p3));
  shape.mSegments.push_back(LineSegment(p3, p4));
  shape.mSegments.push_back(LineSegment(p1, p4));
  return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
  /******Student:Assignment2******/
  // Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(p0,p1));
  shape.mSegments.push_back(LineSegment(p1,p2));
  shape.mSegments.push_back(LineSegment(p2,p3));
  shape.mSegments.push_back(LineSegment(p3,p0));

  return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
  /******Student:Assignment2******/
  // Draw the 6 faces of the frustum using the 8 frustum points.
  // See Frustum.Set for the point order. For example, Points[4] is left-bottom-front.
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[1]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[2]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[3]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[3]));

  shape.mSegments.push_back(LineSegment(frustum.mPoints[4], frustum.mPoints[5]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[5], frustum.mPoints[6]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[6], frustum.mPoints[7]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[4], frustum.mPoints[7]));


  shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[4]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[5]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[6]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[3], frustum.mPoints[7]));
  return shape;
}
