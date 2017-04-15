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
  //Frustum frustum;
  //frustum.Set(Vector3(-81.8643f, -12.6721f, 8.72603f), Vector3(14.2567f, -70.5252f, 68.9857f), Vector3(79.6266f, 38.0846f, 68.9857f), Vector3(-16.4944f, 95.9377f, 8.72603f), Vector3(27.4253f, -9.91242f, -38.8541f), Vector3(34.8995f, -14.411f, -34.1684f), Vector3(39.7053f, -6.42634f, -34.1684f), Vector3(32.2311f, -1.92775f, -38.8541f));
  //frustum.DebugDraw();
  //Sphere sphere = Sphere(Vector3(0, 0, 0), 3.44647f);
  //Sphere sphere1 = Sphere(Vector3(1, 2, 3),1);
  //Sphere sphere2 = Sphere(Vector3(1, 0, 3), 2);
  //Sphere sphere3 = Sphere(Vector3(1, 6, 3), 5);


  //sphere.DebugDraw();
  //sphere1.DebugDraw();
  //sphere2.DebugDraw();
  //sphere3.DebugDraw();

  //size_t t = 0;
  //FrustumSphere(frustum.GetPlanes(), sphere.mCenter, sphere.mRadius, t);

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

void GenerateCirclePoints(std::vector<Vector3>& generated, int resolution, float radius, Vector3 normal,Vector3 center)
{
  generated.clear();
  if (normal == Vector3::cZero)
    return;

  float fullRound = Math::cTwoPi;
  float itteration = fullRound / resolution;

  //project Random point to a plane that is perpendicular to created circle normal
  Vector3 basisX, basisY;

  Math::GenerateOrthonormalBasis(normal, &basisX, &basisY);

  for (int i = 0 ;i < resolution; ++i)
  {
    float thetha = itteration * i;
    generated.push_back(center + radius * (basisX * cos(thetha) + basisY * sin(thetha)));
  }
}

void DebugDrawer::GenerateLineSegements(std::vector<Vector3>& points, DebugShape& shape)
{
  for (unsigned i = 0; i < points.size(); ++i)
  {
    if (i != points.size() - 1)
      shape.mSegments.push_back(LineSegment(points[i] , points[i + 1] ));
    else
      shape.mSegments.push_back(LineSegment(points[i], points[0]));
  }
}


DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
  /******Student:Assignment2******/
  // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
  // Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
  DebugShape& shape = GetNewShape();
  std::vector<Vector3> circlePoints;
  Vector3 normal;

  //horizon disc
  float d = (sphere.mCenter - mApplication->mCamera.mTranslation).Length();
  float l = Math::Sqrt(Math::Sq(d) - Math::Sq(sphere.mRadius));
  float nr = (l * sphere.mRadius) / d;
  float z = Math::Sqrt(Math::Sq(sphere.mRadius) - Math::Sq(nr));
  Vector3 ncenter = sphere.mCenter - (-mApplication->mCamera.GetDirection().Normalized() * z);

  //draw circle with 3 different normal and a horizon disc
  normal = Vector3::cXAxis;
  GenerateCirclePoints(circlePoints, 20, sphere.mRadius, normal,sphere.mCenter);
  GenerateLineSegements(circlePoints, shape);

  normal = Vector3::cYAxis;
  GenerateCirclePoints(circlePoints, 20, sphere.mRadius, normal, sphere.mCenter);
  GenerateLineSegements(circlePoints, shape);

  normal = Vector3::cZAxis;
  GenerateCirclePoints(circlePoints, 20, sphere.mRadius, normal, sphere.mCenter);
  GenerateLineSegements(circlePoints, shape);

  normal = mApplication->mCamera.GetDirection();;
  GenerateCirclePoints(circlePoints, 20, nr, normal, ncenter);
  GenerateLineSegements(circlePoints, shape);



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
