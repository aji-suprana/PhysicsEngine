///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"
#include <list>

/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:
  DynamicAabbTree();
  ~DynamicAabbTree();

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  static const float mFatteningFactor;

  // Add your implementation here
  struct Node
  {
    Node()
    {
      m_Left = nullptr;
      m_right = nullptr;
      m_parent = nullptr;
      m_TreeHi = 0;
    }

    Node(SpatialPartitionData& data)
    {
      m_Left = nullptr;
      m_right = nullptr;
      m_parent = nullptr;
      m_TreeHi = 0;
      mClientData = data.mClientData;
    }

    Aabb mAabb;
    void *mClientData;
    Node* m_Left;
    Node* m_right;
    Node* m_parent;
    size_t m_TreeHi;
    size_t last_axis = -1;
    int node_id;
    static int count;
  };

  Node* m_root = nullptr;
  Node* CreateNode(SpatialPartitionKey& key, SpatialPartitionData& data);
  Node* CreateNode(void);
  void CalculateBalance(Node* node);
  void BalanceAABB(Node* node);
  void GetAllChild(Node* node, CastResults& results);
  void CalculateHeight(Node* node);
  Node* FindNode(Node* insertingNode, Node* node0, Node* node1);
  Node* GetInsertingPos(Node* node);
  Node* FindLowestLeaf(Node* node);
  void Balance(Node* node);
  void Filldata(Node* filloutData, std::vector<SpatialPartitionQueryData>& data)const;
  int GetDepth(Node *node, int id, int level) const;
  int GetDepth(Node* node, int id) const;
  void RecursiveRay(Node* node, const Ray& ray, CastResults& r);
  void RecursiveFrustum(Node* node, const Frustum& frustum, CastResults& results);
  void RecursiveDebugDraw(int level,int curLevel,Node* node, const Math::Matrix4& transform, const Vector4& color = Vector4(1));

  void SelfQuery(Node* node, QueryResults& results);
  void SelfQuery(Node* nodeA, Node* nodeB, QueryResults& results);
};
