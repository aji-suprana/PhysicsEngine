///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  // Doing this lazily (and bad, but it's n-squared...).
  // Just store as the key what the client data is so we can look it up later.
  key.mVoidKey = data.mClientData;
  mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  // Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  // Find the key data and remove it
  for(size_t i = 0; i < mData.size(); ++i)
  {
    if(mData[i] == key.mVoidKey)
    {
      mData[i] = mData.back();
      mData.pop_back();
      break;
    }
  }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    for(size_t j = i + 1; j < mData.size(); ++j)
    {
      results.AddResult(QueryResult(mData[i], mData[j]));
    }
  }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
  data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for(size_t i = 0; i < mData.size(); ++i)
  {
    SpatialPartitionQueryData data;
    data.mClientData = mData[i];
    results.push_back(data);
  }
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  //check if there were anything been removed
  if (removed.empty())
  {
    key.mUIntKey = mData.size();
    mData.push_back(&data);
  }
  //else insert to the idx that exist in removed
  else
  {
    mData[removed.front()] = &data;
    key.mUIntKey = removed.front();
    removed.pop_front();
  }
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  //I don't need to update the data, because the data that is inserted is a pointer to the original data
  //assuming there will be a garbage manager for all boundingAABB and boundingSphere
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  //add current key
  mData[key.mUIntKey] = nullptr;
  removed.push_back(key.mUIntKey);
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  for (auto it: mData)
  {
    if (it == nullptr)
      continue;

    it->mBoundingSphere.DebugDraw().SetMaskBit(bitMask);
    it->mBoundingSphere.DebugDraw().SetTransform(transform);
    it->mBoundingSphere.DebugDraw().Color(color);
    gDebugDrawer->DrawSphere(it->mBoundingSphere);
  }
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  for (auto it : mData)
  {
    if (it == nullptr)
      break;;
    float t = 0;
    if (RaySphere(ray.mStart, ray.mDirection, it->mBoundingSphere.mCenter, it->mBoundingSphere.mRadius, t))
    {
      CastResult curResult;
      curResult.mTime = t;
      curResult.mClientData = it->mClientData;
      results.AddResult(curResult);
    }
  }
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  for (auto it : mData)
  {
    if (it == nullptr)
      break;;
    size_t axis;

    IntersectionType::Type intersectType = FrustumSphere(frustum.GetPlanes(), it->mBoundingSphere.mCenter, it->mBoundingSphere.mRadius, axis);
    if(intersectType == IntersectionType::Inside || intersectType == IntersectionType::Overlaps)
    {
      CastResult curResult;
      curResult.mTime = 0;
      curResult.mClientData = it->mClientData;
      results.AddResult(curResult);
    }
  }
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
  for (unsigned i = 0; i < mData.size(); ++i)
  {
    SpatialPartitionData it1 = *mData[i];
    for (unsigned j= i; j < mData.size(); ++j)
    {
      if (i == j)
        continue;
      SpatialPartitionData it2 = *mData[j];

      const Sphere& sphere0 = it1.mBoundingSphere;
      const Sphere& sphere1 = it2.mBoundingSphere;

      if (SphereSphere(sphere0.mCenter, sphere0.mRadius, sphere1.mCenter, sphere1.mRadius))
      {
        results.AddResult(QueryResult(it1.mClientData, it2.mClientData));
      }
    }
  }
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for (auto it : mData)
  {
    if (it == nullptr)
      continue;

    SpatialPartitionQueryData spData;
    spData.mClientData = it->mClientData;
    spData.mBoundingSphere = it->mBoundingSphere;
    results.push_back(spData);
  }
}
