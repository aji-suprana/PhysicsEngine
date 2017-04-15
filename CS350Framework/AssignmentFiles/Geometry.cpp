///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"


Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance)
{
  /******Student:Assignment1******/
  
  float d = point.Dot(normal) - planeDistance;
  return point - d * normal;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
                            float& u, float& v, float epsilon)
{
  /******Student:Assignment1******/
  float nominator;
  float denominator;

  denominator = (a - b).Dot(a - b);
  if (denominator > -epsilon && denominator < epsilon)
    return false;
  else
  {
    nominator = (point - b).Dot(a - b);
    u = nominator / denominator;
    v = 1 - u;

    if (u >= 1 + epsilon || u <= -epsilon || v >= 1 + epsilon || v <= -epsilon)
      return false;
  }
  return true;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
                            float& u, float& v, float& w, float epsilon)
{
  /******Student:Assignment1******/
 //Error Checking before start calculations
  if (a == b || b == c || c == a)
    //this is not a triangle
  {
    u = 0; v = 0; w = 0;
    return false;
  }
  
  //Initialize needed variables
  Matrix2 abcd;

  float A = 0;
  float B = 0;
  float E = 0;
  float C = 0;
  float D = 0;
  float F = 0;


  //let v0 = P - c
  //let v1 = A - C
  //let v2 = B - C
  Vector3 V[3] = { point - c , a - c , b - c };

  //E = A * u + B * v
  //F = C * u + D * v
  //look for non 0 determinant equation

  for (int i = 0; i < 3; ++i)
  {
    A = V[1].Dot(V[i]);
    B = V[2].Dot(V[i]);
    E = V[0].Dot(V[i]);
    for (int j = 0; j < 3; ++j)
    {
      if (i == j)
        continue;
      C = V[1].Dot(V[j]);
      D = V[2].Dot(V[j]);
      F = V[0].Dot(V[j]);
    }
    abcd.m00 = A;   abcd.m01 = B;
    abcd.m10 = C;   abcd.m11 = D;

    if (abcd.Determinate() != 0)
      break;
  }



  //Solve U and V

  //E = A * u + B * v
  //F = C * u + D * v
  //        ||
  //       \  /
  //        \/
  //| E | = |A B|   | u |
  //| F | = |C D| * | v |
  //        ||
  //       \  /
  //        \/
  //             -1
  //| u | = |A B|    | E |
  //| v | = |C D|  * | F |
  //Inversed = abcd.Inverse();
  //Vector2 ef = Vector2(E, F);
  //Vector2 result = Inversed.Transform(ef);

  //simplified with cramer's rule
  //        |A B|
  // Det =  |C D|
  float Determinant = A * D - B * C;

  //         |E B|
  // DetX =  |F D|
  float DeterminantX = E * D - B * F;

  //         |A E|
  // DetY =  |C F|
  float DeterminantY =  A * F - E * C;

  u = DeterminantX/Determinant;
  v = DeterminantY/Determinant;
  w = 1 - u - v;

  if (u < -epsilon || u > 1 + epsilon ||
    v < -epsilon || v > 1 + epsilon ||
    w < -epsilon || w > 1 + epsilon
    )
    return false;

  return true;
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
  /******Student:Assignment1******/
  //Warn("Assignment1: Required function un-implemented");
  float d = point.Dot(Vector3(plane.x,plane.y,plane.z)) - plane.w;

  if (d < -epsilon)      return IntersectionType::Outside;
  else if (d > epsilon) return IntersectionType::Inside;
  else            return IntersectionType::Coplanar;

  //Just to remove a warning
  return IntersectionType::NotImplemented;
}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
  /******Student:Assignment1******/
  float distanceSquare = (point - sphereCenter).LengthSq();

  if (distanceSquare > sphereRadius * sphereRadius)
    return false;

  return true;
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
  /******Student:Assignment1******/
  bool onX = point.x <= aabbMax.x && point.x >= aabbMin.x;
  bool onY = point.y <= aabbMax.y && point.y >= aabbMin.y;
  bool onZ = point.z <= aabbMax.z && point.z >= aabbMin.z;

  return onX && onY && onZ;
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
              const Vector4& plane, float& t, float epsilon)
{
  ++Application::mStatistics.mRayPlaneTests;
  /******Student:Assignment1******/

  Vector3 planeNormal = Vector3(plane.x, plane.y, plane.z);
  float planeRayDot = planeNormal.Dot(rayDir);

  //The plane and the ray are parrarel ( plane normal and ray are perpendicular -> n.d = 0)
  if (planeRayDot > -epsilon && planeRayDot < epsilon)
    return false;

  t = (plane.w - planeNormal.Dot(rayStart)) / planeNormal.Dot(rayDir);

  if (t > 0)
    return true;

  return false;
}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                 float& t, float triExpansionEpsilon)
{
  ++Application::mStatistics.mRayTriangleTests;
  /******Student:Assignment1******/
  Plane curPlane(triP0, triP1, triP2);
  if (RayPlane(rayStart, rayDir, curPlane.mData, t, triExpansionEpsilon))
  {
    float u;
    float v;
    float w;
    return BarycentricCoordinates(rayStart + t * rayDir, triP0, triP1, triP2, u, v, w);
  }

  return false;
}

bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
               const Vector3& sphereCenter, float sphereRadius,
               float& t)
{
  ++Application::mStatistics.mRaySphereTests;
  /******Student:Assignment1******/

  float distanceCenterStart = (sphereCenter - rayStart).Length();
  if (distanceCenterStart <= sphereRadius)
  {
    t = 0.f;
    return true;
  }

  //let n be centerSpehere - raystart
  Vector3 n = sphereCenter - rayStart;

  //let a be rayDir*2
  //let b be 2 * n * raydir
  //let c be n^2 - r^2

  float a = rayDir.Dot(rayDir);
  float b = -2.0f * n.Dot(rayDir);
  float c = n.Dot(n) - (sphereRadius * sphereRadius);

  float delta = (b * b) - (4.f * a * c);

  if (delta < 0)
    return false;
  else if (delta == 0)
  {
    // one intersection
    t = (-b + Math::Sqrt(delta)) / (2.0f * a);

    if (t >= 0.f)
      return true;
    else
      return false;
  }

  else
  {
    float t1 = (-b - Math::Sqrt(delta)) / (2 * a);
    float t2 = (-b + Math::Sqrt(delta)) / (2 * a);

    t1 < t2 ? t = t1 : t = t2;

    return t > 0;
  }

}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
  const Vector3& aabbMin, const Vector3& aabbMax, float& t)
{
  ++Application::mStatistics.mRayAabbTests;
  /******Student:Assignment1******/
  // rayStart inside AABB cube
  if ((aabbMin.x <= rayStart.x && rayStart.x <= aabbMax.x) &&
    (aabbMin.y <= rayStart.y && rayStart.y <= aabbMax.y) &&
    (aabbMin.z <= rayStart.z && rayStart.z <= aabbMax.z))
  {
    t = 0.0f;
    return true;
  }

  Vector3 Min;
  Vector3 Max;
  for (int i = 0; i < 3; ++i)
  {
    Min[i] = (aabbMin[i] - rayStart[i]) / rayDir[i];
    Max[i] = (aabbMax[i] - rayStart[i]) / rayDir[i];
    if (rayDir[i] < 0.0f)
    {
      float tmp;
      tmp = Min[i];
      Min[i] = Max[i];
      Max[i] = tmp;
    }
    else if (rayDir[i] == 0.0f)
    {
      if (aabbMin[i] > rayStart[i]) return false;
      if (rayStart[i] > aabbMax[i]) return false;
    }
  }

  float tMin = Math::Max(Math::Max(Min[0], Min[1]), Min[2]);
  float tMax = Math::Min(Math::Min(Max[0], Max[1]), Max[2]);

  // no collision
  if (tMin > tMax)
    return false;
  else if (tMin <= tMax)
  {
    t = tMin;
    if (tMin >= 0.0f)
      return true;
  }

  return false;
}

IntersectionType::Type PlaneTriangle(const Vector4& plane,
  const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
  float epsilon)
{
  ++Application::mStatistics.mPlaneTriangleTests;
  /******Student:Assignment1******/
  using namespace IntersectionType;
  if (triP0 == triP1 || triP1 == triP2 || triP0 == triP2)
    return IntersectionType::NotImplemented;

  std::map<IntersectionType::Type, int> existingType;

  existingType[PointPlane(triP0, plane, epsilon)]++;
  existingType[PointPlane(triP1, plane, epsilon)]++;
  existingType[PointPlane(triP2, plane, epsilon)]++;

  //inside case
  if ((existingType[Inside] == 2 && existingType[Coplanar] == 1) ||
    (existingType[Inside] == 1 && existingType[Coplanar] == 2) ||
    (existingType[Inside] == 3))
  {
    return Inside;
  }

  else if ((existingType[Outside] == 2 && existingType[Coplanar] == 1) ||
    (existingType[Outside] == 1 && existingType[Coplanar] == 2) ||
    (existingType[Outside] == 3))
  {
    return Outside;
  }
  else if (existingType[Coplanar] == 3)
    return Coplanar;
  else
    return Overlaps;

  return IntersectionType::NotImplemented;
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter, float sphereRadius)
{
  using namespace IntersectionType;
  ++Application::mStatistics.mPlaneSphereTests;
  /******Student:Assignment1******/
  Vector3 projectedPoint = ProjectPointOnPlane(sphereCenter, Vector3(plane.x, plane.y, plane.z), plane.w);
  Vector3 distVec = (sphereCenter - projectedPoint);

  //if it's right in the middle
  if (distVec.LengthSq() <= sphereRadius * sphereRadius)
    return Overlaps;

  else
  {
    Vector3 distVecNorm = distVec.Normalized();
    Vector3 planeNorm = Vector3(plane.x, plane.y, plane.z);

    if ((distVecNorm - planeNorm).LengthSq() < -0.005 || ( distVecNorm - planeNorm).LengthSq() > 0.005) return Outside;
    else return Inside;
  }

  return IntersectionType::NotImplemented;
}

IntersectionType::Type PlaneAabb(const Vector4& plane,
                                 const Vector3& aabbMin, const Vector3& aabbMax)
{
  ++Application::mStatistics.mPlaneAabbTests;
  /******Student:Assignment1******/
  //generate all 8 points

  using namespace IntersectionType;

  //xYz XYz xyz Xyz
  Vector3 Points[8] = {
    Vector3(aabbMin.x,aabbMax.y,aabbMin.z),
    Vector3(aabbMax.x,aabbMax.y,aabbMin.z),
    Vector3(aabbMin.x,aabbMin.y,aabbMin.z),
    Vector3(aabbMax.x,aabbMin.y,aabbMin.z),

    Vector3(aabbMin.x,aabbMax.y,aabbMax.z),
    Vector3(aabbMax.x,aabbMax.y,aabbMax.z),
    Vector3(aabbMin.x,aabbMin.y,aabbMax.z),
    Vector3(aabbMax.x,aabbMin.y,aabbMax.z)
  };

  IntersectionType::Type prevType = NotImplemented;
  IntersectionType::Type curType = NotImplemented;

  //store all point status
  for (int i = 0; i < 8; ++i)
  {
    IntersectionType::Type temp = PointPlane(Points[i], plane, 0.0005f);
    if (temp == Coplanar )
      continue;

    prevType = curType;
    curType = temp;

    if (curType != prevType && i != 0 && prevType != NotImplemented)
      return Overlaps;


  }

  return curType;
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                       float epsilon)
{
  ++Application::mStatistics.mFrustumTriangleTests;
  using namespace IntersectionType;

  /******Student:Assignment1******/
  IntersectionType::Type prevType = NotImplemented;
  IntersectionType::Type curType = NotImplemented;

  bool OverlapsMet = false;
  for (int i = 0; i < 6; ++i)
  {
    IntersectionType::Type temp = PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon);
    if (temp == Outside)
      return Outside;
    if (temp == Overlaps)
      OverlapsMet = true;
  }

  return OverlapsMet ? Overlaps : Inside;
}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumSphereTests;
  /******Student:Assignment1******/
  using namespace IntersectionType;

  /******Student:Assignment1******/
  IntersectionType::Type prevType = NotImplemented;
  IntersectionType::Type curType = NotImplemented;

  bool OverlapsMet = false;
  for (int i = 0; i < 6; ++i)
  {
    IntersectionType::Type temp = PlaneSphere(planes[i], sphereCenter, sphereRadius);

    if (temp == Outside)
      return Outside;
    if (temp == Overlaps)
      OverlapsMet = true;
  }

  return OverlapsMet ? Overlaps : Inside;
}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumAabbTests;
  /******Student:Assignment1******/
  using namespace IntersectionType;

  /******Student:Assignment1******/
  IntersectionType::Type prevType = NotImplemented;
  IntersectionType::Type curType = NotImplemented;
  
  size_t t = -1;
  if (lastAxis != t)
  {
    IntersectionType::Type type = PlaneAabb(planes[lastAxis], aabbMin, aabbMax);
    if (type == Outside)
    {
      return Outside;
    }
    else
    {
      lastAxis = -1;
    }
  }


  bool OverlapsMet = false;
  for (int i = 0; i < 6; ++i)
  {
    IntersectionType::Type temp = PlaneAabb(planes[i], aabbMin, aabbMax);
    if (temp == Outside)
    {
      lastAxis = i;
      return Outside;
    }
    if (temp == Overlaps)
    {
      OverlapsMet = true;
    }
  }

  return OverlapsMet ? Overlaps : Inside;
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
                  const Vector3& sphereCenter1, float sphereRadius1)
{
  ++Application::mStatistics.mSphereSphereTests;
  /******Student:Assignment1******/
  float centerDistSq = (sphereCenter0 - sphereCenter1).LengthSq();
  float radiusSum = sphereRadius0 + sphereRadius1;
  if(radiusSum * radiusSum < centerDistSq)
    return false;
  return true;
}

bool AabbAabb(const Vector3& aMin, const Vector3& aMax,
              const Vector3& bMin, const Vector3& bMax)
{
  ++Application::mStatistics.mAabbAabbTests;
  /******Student:Assignment1******/
  //if(bmin > a max || amin > bmax)
  bool noIntersection = false;
  for (int i = 0; i < 3; i++)
  {
    if (bMin[i] > aMax[i] || aMin[i] > bMax[i])
      noIntersection |= true;
  }

  return !noIntersection;
}
