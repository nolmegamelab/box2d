#ifndef B2_SECTOR_GRID_H
#define B2_SECTOR_GRID_H

#include "b2_sector.h"
#include <queue>
#include <unordered_map>

struct b2SectorSettings
{
  b2AABB bounds;          // 전체 맵 범위
  float sectorSize;       // x, y size are same
};

/// 내부에 b2Sector들을 갖는 섹터들의 그리드 월드 내에서 충돌 처리
/**
 */
class B2_API b2SectorGrid
{
public: 
  b2SectorGrid(const b2SectorSettings& settings); 

  ~b2SectorGrid();

  // shape, filter, tf, userdata로 b2SectorObject 생성 
  /** 
   * @param shape: shape는 heap 할당하고, 소유권이 b2SectorGrid로 넘어옴. 
   * @param userData: userData는 안전한 아이디와 같은 값을 사용하는 것이 좋음 (소유권 이전 안 함) 
   */
  b2ObjectId Spawn(b2Shape* shape, const b2Filter& filter, b2Transform& tf, void* userData);

  // oid의 b2SectorObject를 제거.  
  void Despawn(b2ObjectId oid);

  // oid의 b2SectorObject의 위치를 변경하고 회전시킴. 
  /**
   * TODO: 효율 명시
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

  int m_sectorCountX;             // X축 섹터 개수  
  int m_sectorCountY;             // Y축 섹터 개수
  b2AABB m_sectorBoundsExtended;  // 전체 맵범위 바깥에 하나의 섹터를 더 둠. 경계 처리

  ObjectMap m_objects;
  SectorMap m_sectors;

  std::queue<b2ObjectId> m_idQueue;
  int m_objectIdSequence;
};

#endif //B2_SECTOR_GRID_H
