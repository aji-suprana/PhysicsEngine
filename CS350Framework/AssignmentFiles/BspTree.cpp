///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
BspTreeQueryData::BspTreeQueryData()
{
  mDepth = 0;
}

void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri, TriangleList& coplanarFront, TriangleList& coplanarBack, TriangleList& front, TriangleList& back, float epsilon)
{
  PointStruct p;
  IntersectionType::Type type = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);

  if (type == IntersectionType::Inside)
    ComputeFront(plane, tri, p);
  else if (type == IntersectionType::Outside)
    ComputeBack(plane, tri, p);
  else if (type == IntersectionType::Coplanar)
    ComputeCoplanar(plane, tri, p);
  else
  {
    for (unsigned i = 0; i < 3; ++i)
    {
      int i1 = i;
      int i2 = (i + 1) % 3;
      ComputeEdge(plane, tri.mPoints[i1], tri.mPoints[i2], p, epsilon);
    }
  }

  CreateTriangle(coplanarFront, coplanarBack, front, back, p);
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
  /******Student:Assignment4******/
  Triangle t = triangles[testIndex];
  if (t.mPoints[0] == t.mPoints[1] || t.mPoints[1] == t.mPoints[2] || t.mPoints[2] == t.mPoints[0])
    return Math::PositiveMax();

  Plane plane(triangles[testIndex].mPoints[0], triangles[testIndex].mPoints[1], triangles[testIndex].mPoints[2]);
  int stradeling = 0, front = 0, back = 0;

  for (unsigned i = 0; i < triangles.size(); ++i)
  {
    if (i == testIndex)
      continue;

    IntersectionType::Type type = PlaneTriangle(plane.mData, triangles[i].mPoints[0], triangles[i].mPoints[1], triangles[i].mPoints[2], epsilon);

    if (type == IntersectionType::Outside)
      ++back;
    else if (type == IntersectionType::Inside)
      ++front;
    else if (type == IntersectionType::Overlaps)
      ++stradeling;
  }

  return k * stradeling + (1.0f - k) * std::abs(front - back);
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
  /******Student:Assignment4******/
  //Warn("Assignment4: Required function un-implemented");
  if (triangles.size() < 2)
    return 0;

  unsigned smallest = 0;
  float min;

  min = CalculateScore(triangles, 0, k, epsilon);

  for (unsigned i = 0; i < triangles.size(); ++i)
  {
    float score = CalculateScore(triangles, i, k, epsilon);

    // Check if we have a new saller in and fill the new data
    if (score < min)
    {
      min = score;
      smallest = i;
    }
  }

  return smallest;
}

BspTree::Node* BspTree::CreateNode()
{
  Node* node = new Node();
  node->front = node->back = nullptr;
  return node;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
  /******Student:Assignment4******/
  //Warn("Assignment4: Required function un-implemented");
  if (NodeRoot)
    NodeRoot = nullptr;

  NodeRoot = CreateNode();
  //rConstruct(NodeRoot, triangles, k, epsilon);
  NodeRoot = rConstruct(triangles, k, epsilon);
}

bool BspTree::RayCast(const Ray& ray, float& t, float rayPlaneEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
  ///******Student:Assignment4******/
  //Warn("Assignment4: Required function un-implemented");

  if (RayCast(NodeRoot, ray, t, rayPlaneEpsilon, triExpansionEpsilon))
    return true;
  else
  {
    t = Math::PositiveMax();
    return false;
  }
}

bool BspTree::RayCast(Node* node, const Ray& ray, float& t, float epsilon, float triExpansionEpsilion)
{
  if (node == nullptr)
    return false;

  if (node->n_tris.size() == 0)
    return false;

  Triangle tri = node->n_tris[0];
  Plane plane;
  plane.Set(tri.mPoints[0], tri.mPoints[1], tri.mPoints[2]);

  IntersectionType::Type type;
  type = PointPlane(ray.mStart, plane.mData, epsilon);

  if (type == IntersectionType::Inside)
  {
    if (RayCast(node->front, ray, t, epsilon, triExpansionEpsilion))
      return true;
  }
  else
    if (RayCast(node->back, ray, t, epsilon, triExpansionEpsilion))
      return true;

  for (auto tri : node->n_tris)
    if (RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], t, triExpansionEpsilion))
      return true;

  if (abs(ray.mDirection.Dot(Vector3(plane.mData[0], plane.mData[1], plane.mData[2]))) >= 0.f)
  {
    if (type == IntersectionType::Inside)
    {
      if (RayCast(node->back, ray, t, epsilon, triExpansionEpsilion))
        return true;
    }
    else
    {
      if (RayCast(node->front, ray, t, epsilon, triExpansionEpsilion))
        return true;
    }
  }
  return false;
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
  Traverse(NodeRoot, triangles);
}

void BspTree::Traverse(Node* node, TriangleList& tl) const
{
  if (node == nullptr)
    return;

  for (unsigned i = 0; i < node->n_tris.size(); ++i)
  {
    tl.push_back(node->n_tris[i]);
  }

  Traverse(node->back, tl);
  Traverse(node->front, tl);
}

void BspTree::Invert()
{
  Invert(NodeRoot);
}

void BspTree::Invert(Node* node)
{
  if (!node)
    return;

  Vector3 temp;
  Node* node_tmp;

  for (unsigned i = 0; i < node->n_tris.size(); ++i)
  {
    temp = node->n_tris[i].mPoints[0];
    node->n_tris[i].mPoints[0] = node->n_tris[i].mPoints[1];
    node->n_tris[i].mPoints[1] = temp;
  }

  node->n_splitPlane.mData.ScaleByVector((Vector4(-1.0f, -1.0f, -1.0f, -1.0f)));
  node_tmp = node->front;
  node->front = node->back;
  node->back = node_tmp;

  Invert(node->front);
  Invert(node->back);
}

void BspTree::Node::ClipTo(TriangleList& n_tris, float epsilon)
{
  TriangleList copfront, copback, front, back;

  for (unsigned i = 0; i < n_tris.size(); ++i)
    SplitTriangle(n_splitPlane, n_tris[i], front, back, front, back, epsilon);

  n_tris.clear();

  if (this->front)
    this->front->ClipTo(front, epsilon);

  if (this->back)
  {
    this->back->ClipTo(back, epsilon);
    for (unsigned i = 0; i < back.size(); ++i)
      n_tris.push_back(back[i]);
  }

  for (unsigned i = 0; i < front.size(); ++i)
    n_tris.push_back(front[i]);
}

void BspTree::Node::ClipTo(Node* node, float epsilon)
{
  node->ClipTo(n_tris, epsilon);

  if (front)
    front->ClipTo(node, epsilon);
  if (back)
    back->ClipTo(node, epsilon);
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
  ///******Student:Assignment4******/
  //Warn("Assignment4: Required function un-implemented");
  NodeRoot->ClipTo(tree->NodeRoot, epsilon);
  //ClipTo(tree->NodeRoot, epsilon);
  //Clip(NodeRoot , tree, epsilon);
}

//
//void BspTree::Clip(Node* node, BspTree *tree, float epsilon)
//{
//  if (node == nullptr)
//    return;
//
//  tree->ClipT(tree->NodeRoot, node->n_tris, false, epsilon);
//  Clip(node->back, tree, epsilon);
//  Clip(node->front, tree, epsilon);
//}
//

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
  ClipTo(tree, epsilon);
  tree->ClipTo(this, epsilon);
  tree->Invert();
  tree->ClipTo(this, epsilon);
  tree->Invert();

  TriangleList allTriangles;
  AllTriangles(allTriangles);
  tree->AllTriangles(allTriangles);
  Construct(allTriangles, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
  Invert();
  tree->Invert();
  Union(tree, k, epsilon);
  Invert();
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
  tree->Invert();
  this->Intersection(tree, k, epsilon);
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results, Node* node, int depth) const
{
  if (node == nullptr)
    return;

  BspTreeQueryData treeData;
  treeData.mTriangles = node->n_tris;
  treeData.mDepth = depth;
  results.push_back(treeData);

  FilloutData(results, node->front, depth + 1);
  FilloutData(results, node->back, depth + 1);
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
  FilloutData(results, NodeRoot, 0);
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
  //Warn("Assignment4: Required function un-implemented");
}

void BspTree::ComputeFront(const Plane& p, const Triangle& t, PointStruct& pts)
{
  if (!FindDuplicate(t))
    return;

  for (unsigned i = 0; i < 3; ++i)
    pts.f.push_back(t.mPoints[i]);
}

void BspTree::ComputeBack(const Plane& p, const Triangle& t, PointStruct& pts)
{
  if (!FindDuplicate(t))
    return;

  for (unsigned i = 0; i < 3; ++i)
    pts.b.push_back(t.mPoints[i]);
}

void BspTree::ComputeCoplanar(const Plane& p, const Triangle& t, PointStruct& pts)
{
  if (!FindDuplicate(t))
    return;

  Plane tplane;
  tplane.Set(t.mPoints[0], t.mPoints[1], t.mPoints[2]);
  float ndir = tplane.GetNormal().Dot(p.GetNormal());

  if (ndir >= 0)
    for (unsigned i = 0; i < 3; ++i)
      pts.cf.push_back(t.mPoints[i]);
  else
    for (unsigned i = 0; i < 3; ++i)
      pts.cb.push_back(t.mPoints[i]);
}

bool BspTree::FindDuplicate(const Triangle& t)
{
  if (t.mPoints[0] == t.mPoints[1] || t.mPoints[1] == t.mPoints[2] || t.mPoints[2] == t.mPoints[0])
    return false;
  else
    return true;
}

void BspTree::ComputeEdge(const Plane& p, const Vector3& p1, const Vector3& p2, PointStruct& pts, float epsilon)
{
  IntersectionType::Type r1 = PointPlane(p1, p.mData, epsilon);
  IntersectionType::Type r2 = PointPlane(p2, p.mData, epsilon);

  // handle the 9 cases
  if (r1 == IntersectionType::Inside)
  {
    if (r2 == IntersectionType::Inside)
      pts.f.push_back(p2);
    else if (r2 == IntersectionType::Outside)
    {
      float intersectionTime;
      Vector3 rayStart, rayDirection;
      rayStart = p1;
      rayDirection = (p2 - p1);
      RayPlane(rayStart, rayDirection, p.mData, intersectionTime);

      if (intersectionTime >= 0.f - epsilon&& intersectionTime <= 1.0f + epsilon)
      {
        Vector3 intersectiontPoint;
        intersectiontPoint = rayStart + rayDirection * intersectionTime;
        pts.f.push_back(intersectiontPoint);
        pts.b.push_back(intersectiontPoint);
      }

      pts.b.push_back(p2);
    }
    else if (r2 == IntersectionType::Coplanar)
      pts.f.push_back(p2);
  }
  else if (r1 == IntersectionType::Outside)
  {
    if (r2 == IntersectionType::Inside)
    {

      float intersectionTime;
      Vector3 rayStart, rayDirection;
      rayStart = p1;
      rayDirection = (p2 - p1);
      RayPlane(rayStart, rayDirection, p.mData, intersectionTime);

      if (intersectionTime >= 0.f - epsilon && intersectionTime <= 1.0f + epsilon)
      {
        Vector3 intersectiontPoint;
        intersectiontPoint = rayStart + rayDirection * intersectionTime;
        pts.f.push_back(intersectiontPoint);
        pts.b.push_back(intersectiontPoint);
      }
      pts.f.push_back(p2);
    }
    else if (r2 == IntersectionType::Outside)
      pts.b.push_back(p2);
    else if (r2 == IntersectionType::Coplanar)
    {
      pts.f.push_back(p2);
      pts.b.push_back(p2);
    }
  }
  else if (r1 == IntersectionType::Coplanar)
  {
    if (r2 == IntersectionType::Inside)
      pts.f.push_back(p2);
    else if (r2 == IntersectionType::Outside)
    {
      pts.b.push_back(p1);
      pts.b.push_back(p2);
    }
    else if (r2 == IntersectionType::Coplanar)
      pts.f.push_back(p2);
  }
}

void BspTree::CreateTriangle(TriangleList& coplanarFront,
  TriangleList& coplanarBack,
  TriangleList& front,
  TriangleList& back,
  PointStruct& pts)
{
  CreateTriangle(coplanarFront, pts.cf);
  CreateTriangle(coplanarBack, pts.cb);
  CreateTriangle(front, pts.f);
  CreateTriangle(back, pts.b);
}

void BspTree::CreateTriangle(TriangleList& tl, std::vector<Vector3>& pts)
{
  if (pts.size() == 3)
  {
    Triangle t(pts[0], pts[1], pts[2]);
    tl.push_back(t);
  }
  else if (pts.size() > 3)
  {
    for (unsigned i = 0; i < pts.size() - 2; ++i)
    {
      unsigned i1 = i + 1;
      unsigned i2 = i + 2;

      if (i1 >= pts.size() || i2 >= pts.size())
        break;

      Triangle t(pts[0], pts[i1], pts[i2]);
      tl.push_back(t);
    }
  }
}

BspTree::Node* BspTree::rConstruct(const TriangleList& triangles, float k, float epsilon)
{
  if (NodeRoot == nullptr)
    return nullptr;

  if (triangles.size() < 1)
    return nullptr;
  else if (triangles.size() == 1)
  {
    Node * node = new Node();
    node->back = node->front = nullptr;
    node->n_tris.push_back(triangles[0]);
  }

  int idx = PickSplitPlane(triangles, k, epsilon);
  Plane SplitPlane;
  SplitPlane.Set(triangles[idx].mPoints[0], triangles[idx].mPoints[1], triangles[idx].mPoints[2]);

  TriangleList coplanarFront, coplanarBack, Front, Back;

  for (unsigned i = 0; i < triangles.size(); ++i)
  {
    if (i == idx)
      continue;

    SplitTriangle(SplitPlane, triangles[i], coplanarFront, coplanarBack, Front, Back, epsilon);
  }

  Node* node = new Node();

  for (unsigned i = 0; i < coplanarFront.size(); ++i)
    node->n_tris.push_back(coplanarFront[i]);

  for (unsigned i = 0; i < coplanarBack.size(); ++i)
    node->n_tris.push_back(coplanarBack[i]);

  // set the data
  node->n_splitPlane = SplitPlane;
  node->n_tris.push_back(triangles[idx]);

  node->front = rConstruct(Front, k, epsilon);
  node->back = rConstruct(Back, k, epsilon);
  return node;
}