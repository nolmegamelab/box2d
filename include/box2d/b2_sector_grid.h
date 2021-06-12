#ifndef B2_SECTOR_GRID_H
#define B2_SECTOR_GRID_H

#include "b2_sector.h"
#include <queue>
#include <unordered_map>

struct b2SectorSettings
{
  b2AABB bounds;          // ��ü �� ����
  float sectorSize;       // x, y size are same
};

/// ���ο� b2Sector���� ���� ���͵��� �׸��� ���� ������ �浹 ó��
/**
 */
class B2_API b2SectorGrid
{
public: 
  b2SectorGrid(const b2SectorSettings& settings); 

  ~b2SectorGrid();

  // shape, filter, tf, userdata�� b2SectorObject ���� 
  /** 
   * @param shape: shape�� heap �Ҵ��ϰ�, �������� b2SectorGrid�� �Ѿ��. 
   * @param userData: userData�� ������ ���̵�� ���� ���� ����ϴ� ���� ���� (������ ���� �� ��) 
   */
  b2ObjectId Spawn(b2Shape* shape, const b2Filter& filter, b2Transform& tf, void* userData);

  // oid�� b2SectorObject�� ����.  
  void Despawn(b2ObjectId oid);

  // oid�� b2SectorObject�� ��ġ�� �����ϰ� ȸ����Ŵ. 
  /**
   * TODO: ȿ�� ���
   */
  void Update(b2ObjectId oid, const b2Vec2& position, const b2Vec2& rotation);

  static bool IsValid(b2ObjectId oid)
  {
    return oid > 0;
  }

private: 
  using ObjectMap = std::unordered_map<b2ObjectId, b2SectorObject*>;
  using SectorMap = std::unordered_map<int, b2Sector*>;   // y index * m_sectorCountX + x index

  int GetSectorIndexFrom(float x, float y);

  int AcquireObjectId();

  void ReleaseObjectId(int id);

private: 
  rx::lockable m_lock;            // recursive shared mutex
  b2SectorSettings m_settings;

  int m_sectorCountX;             // X�� ���� ����  
  int m_sectorCountY;             // Y�� ���� ����
  b2AABB m_sectorBoundsExtended;  // ��ü �ʹ��� �ٱ��� �ϳ��� ���͸� �� ��. ��� ó��

  ObjectMap m_objects;
  SectorMap m_sectors;

  std::queue<b2ObjectId> m_idQueue;
  int m_objectIdSequence;
};

#endif //B2_SECTOR_GRID_H
