///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include <iostream>
#include <iomanip>
//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
  mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
  mStart = start;
  mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
  return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
  mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
  mStart = start;
  mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
  Ray transformedRay;
  transformedRay.mStart = Math::TransformPoint(transform, mStart);
  transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
  return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
  return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
  return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
  /******Student:Assignment2******/
  Vector3 u;
  for (auto it : points)
  {
    u += it;
  }

  u /= (float)points.size();

  Matrix3 covariance;
  for (auto it : points)
  {
    Matrix3 curMat;
    Vector3 cur = it - u;

    curMat[0].x = cur[0] * cur[0];      curMat[0].y = cur[0] * cur[1];      curMat[0].z = cur[0] * cur[2];
    curMat[1].x = cur[0] * cur[1];      curMat[1].y = cur[1] * cur[1];      curMat[1].z = cur[1] * cur[2];
    curMat[2].x = cur[0] * cur[2];      curMat[2].y = cur[1] * cur[2];      curMat[2].z = cur[2] * cur[2];

    covariance += curMat;
  }

  covariance /= (float)points.size();
  return covariance;
}

Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
  /******Student:Assignment2******/
  // Compute the jacobi rotation matrix that will turn the largest (magnitude) off-diagonal element of the input
  // matrix into zero. Note: the input matrix should always be (near) symmetric.
  Matrix3 jac = Matrix3::cIdentity;
  
  int selectedRow = 0;
  int selectedColumn = 0;
  float selectedValue = 0;

  //Get the index
  for (int row = 0; row < 2; ++row)
  {
    for (int column = 1; column < 3; ++column)
    {
      //skip column 1 row 1
      if (row == column)
        continue;
      float currentValue = matrix[row][column];
      //Get Absolute value
      currentValue < 0 ? currentValue *= -1 : currentValue *= 1;
      if (selectedValue < currentValue)
      {
        selectedRow = row;
        selectedColumn = column;
        selectedValue = currentValue;
      }
    }
  }
  float theta;
  float cosValue;
  float sinValue;
  if (selectedRow == 0)
  {
    switch (selectedColumn)
    {
    case 1: // 01
      theta = atan((2.0f * matrix.m01) / (matrix.m00 - matrix.m11)) / 2.0f;
      cosValue = cos(theta);
      sinValue = sin(theta);
      jac.m00 = cosValue;
      jac.m01 = -sinValue;
      jac.m10 = sinValue;
      jac.m11 = cosValue;
      break;
    case 2: // 02
      theta = atan((2.0f * matrix.m02) / (matrix.m22 - matrix.m00)) / 2.0f;
      cosValue = cos(theta);
      sinValue = sin(theta);
      jac.m00 = cosValue;
      jac.m02 = sinValue;
      jac.m20 = -sinValue;
      jac.m22 = cosValue;
      break;
    }
  }
  else // 01
  {
    theta = atan((2.0f * matrix.m12) / (matrix.m11 - matrix.m22)) / 2.0f;
    cosValue = cos(theta);
    sinValue = sin(theta);

    jac.m11 = cosValue;
    jac.m12 = -sinValue;
    jac.m21 = sinValue;
    jac.m22 = cosValue;
  }

  return jac;
}

void PrintMatrix(Matrix3 print)
{
  for (int i = 0; i < 3; ++i)
  {
    std::cout <<"|";
    std::cout.width(5);

    std::cout  << std::setprecision(2) << print[i].x << "|";
    std::cout.width(5);
    std::cout << std::setprecision(2) << print[i].y << "|";
    std::cout.width(5);
    std::cout << std::setprecision(2) << print[i].z ;
    std::cout  << std::endl;
  }
  std::cout << std::endl;
}

void ComputeEigenValuesAndVectors(const Matrix3& covariance, Vector3& eigenValues, Matrix3& eigenVectors, int maxIterations)
{
  /******Student:Assignment2******/
  // Iteratively rotate off the largest off-diagonal elements until the resultant matrix is diagonal or maxIterations.
  int i = 0;
  Matrix3 eigenMatrix = Matrix3::cIdentity;
  Matrix3 eigenValue = covariance;
  Matrix3 currentMat;
  while (i < maxIterations)
  {
    float eps = 0.0001f;
    if ((eigenValue.m01  < eps  && eigenValue.m01 > -eps)
      && (eigenValue.m02 < eps && eigenValue.m02 > -eps)
      && (eigenValue.m12 < eps && eigenValue.m12 > -eps))
      break;

    Matrix3 jac = ComputeJacobiRotation(eigenValue);
    Matrix3 MatrixInverted = jac.Inverted();
    currentMat = MatrixInverted * eigenValue * jac;
    eigenMatrix = eigenMatrix * jac;

    eigenValue = currentMat;
    ++i;
  }

  for (int i = 0; i < 3; ++i)
    eigenValues[i] = eigenValue[i][i];

  eigenVectors = eigenMatrix;

}


//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
  mCenter = Vector3::cZero;
  mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
  mCenter = center;
  mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
  /******Student:Assignment2******/
  // The centroid method is roughly describe as: find the centroid (not mean) of all
  // points and then find the furthest away point from the centroid.
  //mCenter = Vector3(0,0,0);
  //for (auto pt : points)
  //{
  //  mCenter += pt;
  //}

  //mCenter /= (float)points.size();

  Aabb ab;

  for (auto it : points)
  {
    ab.Expand(it);
  }

  mCenter = ab.GetCenter();
  mRadius = Math::Length(points[0] - mCenter);

  for (auto it : points)
  {
    float distance = Math::Length(it - mCenter);
    if(distance > mRadius)
      mRadius = distance;
  }
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
  /******Student:Assignment2******/
  // The ritter method:
  // Find the largest spread on each axis.
  Vector3 min[3];
  Vector3 max[3];

  for (int i = 0; i < 3; ++i)
  {
    min[i] = points[0];
    max[i] = points[0];
  }

  for (auto &it : points)
  {
    for (int id = 0; id < 3; id++)
    {
      if (it[id] < min[id][id])
        min[id] = it;

      if (it[id] > max[id][id])
        max[id] = it;
    }
  }

  // Find which axis' pair of points are the furthest (euclidean distance) apart.
  float longestDist = -1;
  int selected;
  for (int i = 0; i < 3; ++i)
  {
    if (longestDist < Math::Length(min[i] - max[i]))
    {
      longestDist = Math::Length(min[i] - max[i]);
      selected = i;
    }
  }
  // Choose the center of this line as the sphere center. Now incrementally expand the sphere.
  mCenter = (max[selected] + min[selected]) / 2;
  mRadius = longestDist / 2;

  int i = 0;
  for (auto p : points)
  {
    if (ContainsPoint(p))
      continue;

    Vector3 b;
    Vector3 direction = (mCenter - p).Normalized() * mRadius;
    b = mCenter + direction;
    mCenter = (b + p) / 2;
    mRadius = Math::Length(b - p) / 2;
    ++i;
  }
}

void Sphere::ComputePCA(const std::vector<Vector3>& points)
{
  // The PCA method:
  // Compute the eigen values and vectors. Take the largest eigen vector as the axis of largest spread.
  // Compute the sphere center as the center of this axis then expand by all points.
  /******Student:Assignment2******/
  Vector3 eigenValues;
  Matrix3 eigenVectors;
  Matrix3 covariance = ComputeCovarianceMatrix(points);

  ComputeEigenValuesAndVectors(covariance, eigenValues, eigenVectors, 50);

  Vector3 largestEigenVector;
  float largestEigenValue = Math::PositiveMin();

  for (int i = 0; i < 3; ++i)
  {
    if (largestEigenValue < eigenValues[i])
    {
      largestEigenValue = eigenValues[i];
      largestEigenVector = eigenVectors.Basis(i);
    }
  }

  Vector3 min, max;
  min = max = points[0];
  float dot;
  float maxdist = Dot(max, largestEigenVector);
  float mindist = maxdist;
  for (auto p : points)
  {
    dot = Math::Dot(p, largestEigenVector);

    if (dot > maxdist)
    {
      max = p;
      maxdist = dot;
    }

    else if (dot < mindist)
    {
      min = p;
      mindist = dot;
    }
  }

  mCenter = (min + max) / 2.0f;
  mRadius = Length(min - max) / 2.0f;

  int i = 0;
  for (auto p : points)
  {
    if (ContainsPoint(p))
      continue;

    Vector3 b;
    Vector3 direction = (mCenter - p).Normalized() * mRadius;
    b = mCenter + direction;
    mCenter = (b + p) / 2;
    mRadius = Math::Length(b - p)/2;
    ++i;
  }
}


bool Sphere::ContainsPoint(const Vector3& point)
{
  return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
  return mCenter;
}

float Sphere::GetRadius() const
{
  return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
  float posDiff = Math::Length(mCenter - rhs.mCenter);
  float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

  return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
  return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
  //set the aabb to an initial bad value (where the min is smaller than the max)
  mMin.Splat(Math::PositiveMax());
  mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
  mMin = min;
  mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
  return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
  /******Student:Assignment2******/
  // Return the aabb's volume
  Vector3 diagonal = (mMax - mMin);
  float volume = 1;
  for (int i = 0; i < 3; ++i)
  {
    volume *= diagonal[i];
  }

  return volume;
}

float Aabb::GetSurfaceArea() const
{
  /******Student:Assignment2******/
  // Return the aabb's surface area
  Vector3 diagonal = mMax - mMin;
  float surface = 0;
  for (int i = 0; i < 3; ++i)
  {
    if (i == 2)
    {
      surface += diagonal[2] * diagonal[0];
    }
    else
    {
      surface += diagonal[i] * diagonal[i+1];
    }
  }

  return surface *= 2;
}

bool Aabb::Contains(const Aabb& aabb) const
{
  /******Student:Assignment2******/
  bool contained = aabb.mMax.x <= mMax.x && aabb.mMax.y <= mMax.y && aabb.mMin.x >= mMin.x && aabb.mMin.y >= aabb.mMin.y;
  return contained;
}

void Aabb::Expand(const Vector3& point)
{
  for(size_t i = 0; i < 3; ++i)
  {
    mMin[i] = Math::Min(mMin[i], point[i]);
    mMax[i] = Math::Max(mMax[i], point[i]);
  }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
  Aabb result;
  for(size_t i = 0; i < 3; ++i)
  {
    result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
    result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
  }
  return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
  float pos1Diff = Math::Length(mMin - rhs.mMin);
  float pos2Diff = Math::Length(mMax - rhs.mMax);

  return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation)
{
  /******Student:Assignment2******/
  // Compute aabb of the this aabb after it is transformed.
  //Vector3 r[4];

  //r[0] = (mMax - mMin) / 2;
  //r[1] = Vector3(r[0].x,  r[0].y, r[0].z);
  //r[2] = Vector3(-r[0].x, r[0].y, r[0].z);
  //r[3] = Vector3(-r[0].x, r[0].y, -r[0].z);

  //Vector3 center = GetCenter();

  //r[0] *= scale;
  //r[0] = Math::Transform(rotation, r[0]);
  //r[1] *= scale;
  //r[1] = Math::Transform(rotation, r[1]);
  //r[2] *= scale;
  //r[2] = Math::Transform(rotation, r[2]);
  //r[3] *= scale;
  //r[3] = Math::Transform(rotation, r[3]);


  //center *= scale;
  //center = Math::Transform(rotation, center);
  //center += translation;

  //mMax = r[0];

  //for (int i = 1; i < 4; i++)
  //{
  //  if (mMax.x < r[i].x)
  //    mMax.x = r[i].x;
  //  if (mMax.y < r[i].y)
  //    mMax.y = r[i].y;
  //  if (mMax.z < r[i].z)
  //    mMax.z = r[i].z;
  //}

  //mMin = -mMax + center;
  //mMax += center;

  Matrix3 rot = rotation;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
      if (rot[i][j] < 0)
        rot[i][j] *= -1;
  }

  Vector3 r = (mMax - mMin)/2;
  r *= scale;
  r = Math::Transform(rot, r);

  Vector3 center = GetCenter();
  center *= scale;
  center = Math::Transform(rotation, center);
  center += translation;

  mMax = center + r;
  mMin = center - r;
}

Vector3 Aabb::GetMin() const
{
  return mMin;
}

Vector3 Aabb::GetMax() const
{
  return mMax;
}

Vector3 Aabb::GetCenter() const
{
  return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
  return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
  return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
  mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  mPoints[0] = p0;
  mPoints[1] = p1;
  mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
  return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
  mData = Vector4::cZero;
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
  Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  /******Student:Assignment1******/
  // Set mData from the 3 points. Note: You should most likely normalize the plane normal.
  Vector3 normal(0, 0, 0);
  Vector3 v[2] = { p1 - p0,p2 - p0 };

  normal = v[0].Cross(v[1]);

  normal.Normalize();
  float d = normal.Dot(p0);

  mData = Vector4(normal.x,normal.y,normal.z,d);
  //Warn("Assignment1: Required function un-implemented");
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
  /******Student:Assignment1******/
  // Set mData from the normal and point. Note: You should most likely normalize the plane normal.
  Vector3 normalizedNorm = normal.Normalized();
  mData.x = normalizedNorm.x;
  mData.y = normalizedNorm.y;
  mData.z = normalizedNorm.z;
  mData.w = normalizedNorm.Dot(point);
}

Vector3 Plane::GetNormal() const
{
  return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
  return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
  return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
  return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
                  const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
  mPoints[0] = lbn;
  mPoints[1] = rbn;
  mPoints[2] = rtn;
  mPoints[3] = ltn;
  mPoints[4] = lbf;
  mPoints[5] = rbf;
  mPoints[6] = rtf;
  mPoints[7] = ltf;

  //left
  mPlanes[0].Set(lbf, ltf, lbn);
  //right
  mPlanes[1].Set(rbn, rtf, rbf);
  //top
  mPlanes[2].Set(ltn, ltf, rtn);
  //bot
  mPlanes[3].Set(rbn, lbf, lbn);
  //near
  mPlanes[4].Set(lbn, ltn, rbn);
  //far
  mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
  return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
  return gDebugDrawer->DrawFrustum(*this);
}
