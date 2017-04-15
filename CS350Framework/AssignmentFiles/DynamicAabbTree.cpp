///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;
int DynamicAabbTree::Node::count = 0;

DynamicAabbTree::DynamicAabbTree()
{
  mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  // First case inserting into an empty tree node becomes roots
  if (m_root == nullptr)
    m_root = CreateNode(key, data);
  else
  {
    Node* pNode = CreateNode(key, data);
    Node* insertingPos = GetInsertingPos(pNode);

    if (insertingPos->m_Left == nullptr && insertingPos->m_right == nullptr)
    {
      Node* node = CreateNode();
      Node* parent;
      if (insertingPos == m_root)
      {
        node->m_Left = m_root;
        node->m_right = pNode;
        m_root = node;
        node->m_Left->m_parent = node;
        node->m_right->m_parent = node;
        node->mClientData = node->m_Left->mClientData;
        node->m_parent = nullptr;
      }
      else
      {
        parent = insertingPos->m_parent;
        node->m_parent = parent;
        node->m_Left = insertingPos;
        node->m_Left->m_parent = node;
        node->m_right = pNode;
        node->m_right->m_parent = node;
        node->mClientData = nullptr;

        if (node->m_Left == parent->m_Left)
          parent->m_Left = node;
        else
          parent->m_right = node;

      }
      CalculateHeight(m_root);
      CalculateBalance(pNode);
      BalanceAABB(pNode);
    }
  }
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Node* updatingNode = (Node*)key.mVoidKey;

  if (!updatingNode->mAabb.Contains(data.mAabb))
  {
    RemoveData(key);
    InsertData(key, data);
  }
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
  Node* pNode = (Node*)key.mVoidKey;

  if (pNode == nullptr)
    return;

  Node* parent = pNode->m_parent;
  Node* parentsParent = nullptr;

  if (parent)
    parentsParent = parent->m_parent;

  Node* replaceNode;

  if (parent == nullptr)
  {
    m_root = nullptr;
    delete pNode;
  }
  else if (parent && parentsParent == nullptr)
  {
    if (pNode == parent->m_Left)
      replaceNode = parent->m_right;
    else if (pNode == parent->m_right)
      replaceNode = parent->m_Left;

    replaceNode->m_parent = nullptr;
    m_root = replaceNode;

    delete parent;
    delete pNode;
  }
  else if (parent && parentsParent)
  {
    if (pNode == parent->m_Left)
    {
      replaceNode = parent->m_right;
      replaceNode->m_parent = parentsParent;
    }
    else if (pNode == parent->m_right)
    {
      replaceNode = parent->m_Left;
      replaceNode->m_parent = parentsParent;
    }
    if (parent == parentsParent->m_Left)
      parentsParent->m_Left = replaceNode;
    else
      parentsParent->m_right = replaceNode;

    delete parent;
    delete pNode;

    CalculateHeight(m_root);
    CalculateBalance(replaceNode);
    BalanceAABB(replaceNode);
  }
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
  RecursiveRay(m_root, ray, results);
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
  if (m_root == nullptr)
    return;

  RecursiveFrustum(m_root, frustum, results);
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
  SelfQuery(m_root, results);
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  Filldata(m_root, results);
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
    RecursiveDebugDraw(level,0,m_root, transform, color);
}
void DynamicAabbTree::RecursiveDebugDraw(int level,int curLevel,Node* node, const Math::Matrix4& transform, const Vector4& color)
{
  if (node == nullptr)
    return;
 
  if(curLevel >= level || level == -1)
    node->mAabb.DebugDraw();

  RecursiveDebugDraw(level,curLevel + 1,node->m_Left, transform, color);
  RecursiveDebugDraw(level, curLevel + 1, node->m_right, transform, color);

}

DynamicAabbTree::Node* DynamicAabbTree::FindLowestLeaf(Node* node)
{
  if (!node->m_Left && !node->m_right)
    return node;

  if (node->m_Left->m_TreeHi >= node->m_right->m_TreeHi)
    return FindLowestLeaf(node->m_Left);
  else
    return FindLowestLeaf(node->m_right);
}


DynamicAabbTree::Node* DynamicAabbTree::FindNode(Node* insertingNode, Node* node0, Node* node1)
{
  Aabb left, right;

  left = Aabb::Combine(node0->mAabb, insertingNode->mAabb);
  right = Aabb::Combine(node1->mAabb, insertingNode->mAabb);
  float dif0 = left.GetSurfaceArea() - node0->mAabb.GetSurfaceArea();
  float dif1 = right.GetSurfaceArea() - node1->mAabb.GetSurfaceArea();

  if (dif0 < dif1)
    return node0;

  return node1;
}

DynamicAabbTree::Node* DynamicAabbTree::GetInsertingPos(Node* node)
{
  Node* insertingPos = m_root;

  while (insertingPos->m_TreeHi != 0)
  {
    insertingPos = FindNode(node, insertingPos->m_Left,insertingPos->m_right);
  }

  return insertingPos;
}


void DynamicAabbTree::CalculateBalance(Node* node)
{
  if (node == m_root)
    return;

  Node* parent = node->m_parent;

  if (abs((int)(parent->m_Left->m_TreeHi - parent->m_right->m_TreeHi)) >= 2)
    Balance(parent);

  CalculateBalance(parent);
  return;
}

void DynamicAabbTree::BalanceAABB(Node* node)
{
  if (node == m_root)
    return;

  Node* parent = node->m_parent;
  parent->mAabb = Aabb::Combine(parent->m_Left->mAabb, parent->m_right->mAabb);
  BalanceAABB(parent);

  return;
}

void DynamicAabbTree::Balance(Node* node)
{
  Node* notBalance = nullptr;
  Node* small = nullptr;
  Node* larger = nullptr;
  Node* parentsParent = nullptr;

  if (node->m_Left->m_TreeHi >= node->m_right->m_TreeHi)
    notBalance = node->m_Left;
  else
    notBalance = node->m_right;

  if (node->m_parent == nullptr)
  {
    notBalance->m_parent = nullptr;
    m_root = notBalance;
  }

  else
  {
    parentsParent = node->m_parent;
    if (node == parentsParent->m_Left)
      parentsParent->m_Left = notBalance;
    else
      parentsParent->m_right = notBalance;

    notBalance->m_parent = parentsParent;
  }

  if (notBalance->m_Left->m_TreeHi <= notBalance->m_right->m_TreeHi)
  {
    small = notBalance->m_Left;
    larger = notBalance->m_right;
    notBalance->m_Left = node;
  }
  else
  {
    small = notBalance->m_right;
    larger = notBalance->m_Left;
    notBalance->m_right = node;
  }
  node->m_parent = notBalance;

  if (notBalance == node->m_Left)
    node->m_Left = small;
  else
    node->m_right = small;

  small->m_parent = node;

  Node* pNode;
  if (node->m_Left->m_TreeHi >= node->m_right->m_TreeHi)
    pNode = FindLowestLeaf(node->m_Left);
  else
    pNode = FindLowestLeaf(node->m_right);


  CalculateHeight(m_root);
  BalanceAABB(pNode);
}

void DynamicAabbTree::RecursiveRay(Node* node, const Ray& ray, CastResults& r)
{
  if (node == nullptr)
    return;

  Aabb aabb = node->mAabb;
  float time;

  if (!RayAabb(ray.mStart, ray.mDirection, aabb.GetMin(), aabb.GetMax(), time))
    return;
  else  if (node->m_Left == nullptr && node->m_right == nullptr)
  {
    CastResult result;
    result.mTime = time;
    result.mClientData = node->mClientData;
    r.AddResult(result);
  }

  // walk nodes tree
  RecursiveRay(node->m_Left, ray, r);
  RecursiveRay(node->m_right, ray, r);
}



void DynamicAabbTree::CalculateHeight(Node* node)
{
  if (node->m_Left == nullptr && node->m_right == nullptr)
  {
    node->m_TreeHi = 0;
    return;
  }

  CalculateHeight(node->m_Left);
  CalculateHeight(node->m_right);

  if (node->m_Left->m_TreeHi >= node->m_right->m_TreeHi)
    node->m_TreeHi = (node->m_Left->m_TreeHi + 1);
  else
    node->m_TreeHi = (node->m_right->m_TreeHi + 1);

}

void DynamicAabbTree::RecursiveFrustum(Node* node, const Frustum& frustum, CastResults& results)
{
  if (node == nullptr)
    return;
  Aabb aabb = node->mAabb;

  Vector4 plane[6];
  for (int i = 0; i < 6; ++i)
    plane[i] = frustum.mPlanes[i].mData;

  IntersectionType::Type type = FrustumAabb(plane, node->mAabb.GetMin(), node->mAabb.GetMax(), node->last_axis);

  if (type == IntersectionType::Outside)
    return;
  else if (type == IntersectionType::Inside)
  {
    GetAllChild(node, results);
    return;
  }

  // if the cast shape overlaps a nodes aabb recurse
  if (node->m_Left == nullptr && node->m_right == nullptr)
  {
    if (type == IntersectionType::Overlaps)
    {
      CastResult result;
      result.mTime = 0;
      result.mClientData = node->mClientData;
      results.AddResult(result);
    }
  }
  // otherwise recurse
  RecursiveFrustum(node->m_Left, frustum, results);
  RecursiveFrustum(node->m_right, frustum, results);
}

void DynamicAabbTree::Filldata(Node* root, std::vector<SpatialPartitionQueryData>& data) const
{
  if (!root)
    return;

  SpatialPartitionQueryData insertingData;
  insertingData.mAabb = root->mAabb;
  insertingData.mClientData = root->mClientData;
  insertingData.mDepth = GetDepth(m_root, root->node_id);
  data.push_back(insertingData);
  Filldata(root->m_Left, data);
  Filldata(root->m_right, data);
}

int DynamicAabbTree::GetDepth(Node*node, int id, int level) const
{
  if (node == nullptr)
    return 0;

  if (node->node_id == id)
    return level;

  int downlevel = GetDepth(node->m_Left, id, level + 1);
  if (downlevel != 0)
    return downlevel;

  downlevel = GetDepth(node->m_right, id, level + 1);
  return downlevel;
}

int DynamicAabbTree::GetDepth(Node* node, int id) const
{
  return GetDepth(node, id, 0);
}

void DynamicAabbTree::GetAllChild(Node* node, CastResults& results)
{
  if (!node)
    return;

  if (!node->m_Left && !node->m_right)
  {
    CastResult result;
    result.mTime = 0;
    result.mClientData = node->mClientData;
    results.AddResult(result);
  }

  GetAllChild(node->m_Left, results);
  GetAllChild(node->m_right, results);
}

void DynamicAabbTree::SelfQuery(Node* node, QueryResults& results)
{
  if (node == nullptr)
    return;

  if (node->m_Left == nullptr && node->m_right == nullptr)
    return;

  SelfQuery(node->m_Left, node->m_right, results);
  SelfQuery(node->m_Left, results);
  SelfQuery(node->m_right, results);
}

void DynamicAabbTree::SelfQuery(Node* nodeA, Node* nodeB, QueryResults& results)
{
  if (!nodeA || !nodeB)
    return;

  if (AabbAabb(nodeA->mAabb.mMin, nodeA->mAabb.mMax, nodeB->mAabb.mMin, nodeB->mAabb.mMax))
  {
    if (nodeA->m_Left == nullptr && nodeA->m_right == nullptr && nodeB->m_Left == nullptr && nodeB->m_right == nullptr)
    {
      QueryResult r(nodeA->mClientData, nodeB->mClientData);
      results.AddResult(r);
    }
  }
  else
  {
    return;
  }

  if (nodeA->m_Left == nullptr && nodeA->m_right == nullptr && !(nodeB->m_Left == nullptr && nodeB->m_right == nullptr))
  {
    SelfQuery(nodeA, nodeB->m_Left, results);
    SelfQuery(nodeA, nodeB->m_right, results);
  }

  else if (!(nodeA->m_Left == nullptr && nodeA->m_right == nullptr) && !(nodeB->m_Left == nullptr && nodeB->m_right == nullptr))
  {
    if (nodeA->mAabb.GetVolume() <= nodeB->mAabb.GetVolume())
    {
      SelfQuery(nodeA, nodeB->m_Left, results);
      SelfQuery(nodeA, nodeB->m_right, results);
    }
    else
    {
      SelfQuery(nodeB, nodeA->m_Left, results);
      SelfQuery(nodeB, nodeA->m_right, results);
    }
  }
  else
  {
    SelfQuery(nodeB, nodeA->m_Left, results);
    SelfQuery(nodeB, nodeA->m_right, results);
  }
}

DynamicAabbTree::Node* DynamicAabbTree::CreateNode(void)
{
  Node* newNode = new Node();
  // creating an empty node
  newNode->m_parent = nullptr;
  newNode->m_Left = nullptr;
  newNode->m_right = nullptr;
  newNode->node_id = ++DynamicAabbTree::Node::count;

  return newNode;
}
DynamicAabbTree::Node* DynamicAabbTree::CreateNode(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Node* node = new Node(data);
  node->mAabb = Aabb::BuildFromCenterAndHalfExtents(data.mAabb.GetCenter(), data.mAabb.GetHalfSize() * mFatteningFactor);
  key.mVoidKey = node;
  node->node_id = ++DynamicAabbTree::Node::count;
  return node;
}