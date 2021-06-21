#ifndef B2_SECTOR_GRID_H
#define B2_SECTOR_GRID_H

#include "b2_sector.h"
#include "b2_sector_collider.h"
#include <functional>
#include <set>
#include <queue>
#include <unordered_map>

struct b2Result
{
  enum Code
  {
    Success,
    Fail_Too_Many_Shape_Child_Count,
    Fail_Invalid_Object_Position
  };

  template <typename T>
  static bool Succeeded(std::pair<T, Code> code)
  {
    return code.second == Code::Success;
  }
};

struct b2SectorSettings
{
  b2AABB bounds;          // 전체 맵 범위
  float sectorSize;       // x, y size are same
};

/// 내부에 b2Sector들을 갖는 섹터들의 그리드 월드 내에서 충돌 처리
/**
 * b2SectorSettings로 월드 경계와 섹터 크기를 지정
 */
class B2_API b2SectorGrid
{
public:
  // b2SectorSettings로 월드 경계와 섹터 크기를 지정
  /**
   * 내부적으로 월드를 확장하여 안전하게 섹터들에 접근할 수 있게 준비
   */
  b2SectorGrid(const b2SectorSettings& settings);

  ~b2SectorGrid();

  // shape, filter, tf, userdata로 b2SectorObject 생성 
  /**
   * @param shape: shape는 heap 할당하고, 소유권이 b2SectorGrid로 넘어옴.
   * @param userData: userData는 안전한 아이디와 같은 값을 사용하는 것이 좋음 (소유권 이전 안 함)
   * @return 추가된 오브젝트의 b2ObjectId를 리턴한다. 실패하면 음수를 리턴한다.
   */
  std::pair<b2ObjectId, b2Result::Code>
    Spawn(b2Shape* shape, const b2Filter& filter, b2Transform& tf, void* userData);

  // oid의 b2SectorObject를 제거.  
  void Despawn(b2ObjectId oid);

  // oid의 b2SectorObject의 위치를 변경하고 회전시킴. 
  /**
   * @param oid - the object to move
   * @param position - the new position of the object
   * @param rotation - the rotation angle of the object
   */
  void Move(b2ObjectId oid, const b2Vec2& position, const b2Rot& rotation);

  // shape와 겹치는 오브젝트들을 추출한다. 
  /**
   * @param obj - b2SectorObject. Shape, Filter, Transform needs to be valid
   * @param objects - 충돌한 오브젝트의 userData를 T 타잎으로 변환한 목록
   */
  template <typename T>
  int Query(const b2SectorCollider& collider, std::vector<T>& objects);

  // 섹터 범위를 넘지 않는 AABB로 겹치는 오브젝트 검색
  template <typename T>
  int Query(const b2AABB& aabb, std::vector<T>& objects);

  // 원 범위로 검색
  /** 
   * 내부적으로 b2SectorCollider를 사용하는 Query 함수를 호출
   * @param pos - 원의 중심 위치 
   * @param radius - 원의 반경 
   * @param filter - 충돌 필터링 정보 
   * @param objects - 충돌한 오브젝트의 userData를 T 타잎으로 변환한 목록
   */
  template <typename T>
  int QueryCircle(const b2Vec2& pos, float radius, const b2Filter& filter, std::vector<T>& objects);

  // OBB 범위로 검색
  /** 
   * @param hx - half width. OBB의 가로 길이의 절반 
   * @param hy - half height. OBB의 세로 길이의 절반
   * @param pos - OBB의 중심점 위치 
   * @param angline - OBB의 회전값
   */
  template <typename T>
  int QueryOBB(float hx, float hy, const b2Vec2& pos, float angle, const b2Filter& filter, std::vector<T>& objects);

  const b2AABB& GetWorldBounds() const
  {
    return m_settings.bounds;
  }

  const b2AABB& GetWorldBoundsExtended() const
  {
    return m_sectorBoundsExtended;
  }

  float GetSectorSize() const
  {
    return m_settings.sectorSize;
  }

  int GetSectorCountX() const
  {
    return m_sectorCountX;
  }

  int GetSectorCountY() const
  {
    return m_sectorCountY;
  }

  static bool IsValid(b2ObjectId oid)
  {
    return oid > 0;
  }

private:
  using ObjectMap = std::unordered_map<b2ObjectId, b2SectorObject*>;
  using SectorMap = std::unordered_map<int, b2Sector*>;   // y index * m_sectorCountX + x index

  // x 인덱스가 ix이고, y 인덱스가 iy인 섹터가 없으면 만듦
  void EnsureSector(int ix, int iy);

  // 섹터에 대해 함수 f를 호출
  bool Apply(b2Sector* sector, std::function<bool(b2Sector*)> f)
  {
    if (sector)
    {
      return f(sector);
    }

    return false;
  }

  // 인접한 9개 섹터에 대해 함수 f를 호출
  int ApplyNeighbors(int ix, int iy, std::function<bool(b2Sector*)> f)
  {
    int trueCount = 0;

    if (Apply(GetSector(ix - 1, iy - 1), f)) { trueCount++; }
    if (Apply(GetSector(ix - 0, iy - 1), f)) { trueCount++; }
    if (Apply(GetSector(ix + 1, iy - 1), f)) { trueCount++; }

    if (Apply(GetSector(ix - 1, iy - 0), f)) { trueCount++; }
    if (Apply(GetSector(ix - 0, iy - 0), f)) { trueCount++; }
    if (Apply(GetSector(ix + 1, iy - 0), f)) { trueCount++; }

    if (Apply(GetSector(ix - 1, iy + 1), f)) { trueCount++; }
    if (Apply(GetSector(ix - 0, iy + 1), f)) { trueCount++; }
    if (Apply(GetSector(ix + 1, iy + 1), f)) { trueCount++; }

    return trueCount;
  }

  // ix, iy의 섹터를 얻음
  b2Sector* GetSector(int ix, int iy)
  {
    int index = GetSectorIndexFrom(ix, iy);

    {
      rx::slock slock(m_lock);

      auto iter = m_sectors.find(index);
      if (iter != m_sectors.end())
      {
        return iter->second;
      }
    }

    return nullptr;
  }

  int GetSectorIndexX(float x)
  {
    float sx = x - m_sectorBoundsExtended.lowerBound.x;

    if (sx < 0) // object cannot be outside bounds
    {
      return -1;
    }

    int ix = static_cast<int>(sx / m_settings.sectorSize);
    int addX = (sx - m_settings.sectorSize * ix) > 0 ? 1 : 0;
    ix += addX;

    if (ix >= (m_sectorCountX - 1))
    {
      return -2;
    }

    return ix;
  }

  int GetSectorIndexY(float y)
  {
    float sy = y - m_sectorBoundsExtended.lowerBound.y;

    if (sy < 0) // object cannot be outside bounds
    {
      return -1;
    }

    int iy = static_cast<int>(sy / m_settings.sectorSize);
    int addY = (sy - m_settings.sectorSize * iy) > 0 ? 1 : 0;
    iy += addY;

    if (iy >= (m_sectorCountY - 1))
    {
      return -2;
    }

    return iy;
  }

  int GetSectorIndexFrom(float x, float y)
  {
    return GetSectorIndexY(y) * m_sectorCountX + GetSectorIndexX(x);
  }

  int GetSectorIndexFrom(const b2AABB& aabb)
  {
    return GetSectorIndexFrom(aabb.GetCenter().x, aabb.GetCenter().y);
  }

  int GetSectorIndexFrom(int ix, int iy)
  {
    return iy * m_sectorCountX + ix;
  }

  bool CheckIndexBounds(int ix, int iy)
  {
    return
      ix >= 1 && ix < m_sectorCountX - 1 &&
      iy >= 1 && iy < m_sectorCountY - 1;
  }

  // 그룹이 같거나 같은 카테고리에 대해 마스크 비트가 설정되면 충돌 체크
  bool ShouldCollide(const b2Filter& filterA, const b2Filter& filterB);

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

template <typename T>
int b2SectorGrid::Query(const b2SectorCollider& collider, std::vector<T>& objects)
{
  b2AABB aabb;
  collider.GetShape()->ComputeAABB(&aabb, collider.GetTransform(), 0);

  int ix = GetSectorIndexX(aabb.GetCenter().x);
  int iy = GetSectorIndexX(aabb.GetCenter().y);

  b2Assert(CheckIndexBounds(ix, iy));
  if (!CheckIndexBounds(ix, iy))
  {
    return 0;
  }
  
  std::set<b2ObjectId> sobjs;

  // slock
  {
    rx::slock slock(m_lock);

    // 겹치는 attach 안 된 인접한 섹터들에 추가.
    int cnt = ApplyNeighbors(ix, iy, [this, &aabb, &objects, &sobjs](b2Sector* sector) {
      if (sector->IsOverlapping(aabb))
      {
        std::vector<int32> lst;
        sector->Query(aabb, lst);

        for (auto& proxy : lst)
        {
          auto o = sector->GetObject(proxy);
          sobjs.insert(o->GetObjectId());
        }

        return true;
      }

      return false;
      });

    // detailed collision filtering

    for (auto& oid : sobjs)
    {
      auto iter = m_objects.find(oid);
      if (iter != m_objects.end())
      {
        auto obj2 = iter->second;

        if (ShouldCollide(collider.GetFilter(), obj2->GetFilter()))
        {
          auto col = b2TestOverlap(
            collider.GetShape(), 0, obj2->GetShape(), 0, collider.GetTransform(), obj2->GetTransform()
          );

          if (col)
          {
            const T value = obj2->GetUserData<T>();
            objects.push_back(value);
          }
        }
      }
    }
  }

  return static_cast<int>(objects.size());
}

template <typename T>
int b2SectorGrid::Query(const b2AABB& aabb, std::vector<T>& objects)
{
  int ix = GetSectorIndexX(aabb.GetCenter().x);
  int iy = GetSectorIndexX(aabb.GetCenter().y);

  b2Assert(ix >= 1 && iy >= 1);

  int overlapCount = 0;

  // slock
  {
    rx::slock slock(m_lock);

    // 겹치는 attach 안 된 인접한 섹터들에 추가.
    int cnt = ApplyNeighbors(ix, iy, [this, &aabb, &objects, &overlapCount](b2Sector* sector) {
      if (sector->IsOverlapping(aabb))
      {
        std::vector<int32> lst;
        sector->Query(aabb, lst);

        for (auto& proxy : lst)
        {
          auto obj = sector->GetObject(proxy);
          objects.push_back(reinterpret_cast<T>(*obj->GetUserData()));
          ++overlapCount;
        }

        return true;
      }

      return false;
      });
  }

  return overlapCount;
}

template <typename T>
int b2SectorGrid::QueryCircle(const b2Vec2& pos, float radius, const b2Filter& filter, std::vector<T>& objects)
{
  b2CircleShape circle; 
  circle.m_radius = radius;
  circle.m_p = pos;

  b2SectorCollider collider(&circle, filter, b2Transform(b2Vec2(0, 0)));
  return Query(collider, objects);
}

template <typename T>
int b2SectorGrid::QueryOBB(float hx, float hy, const b2Vec2& pos, float angle, const b2Filter& filter, std::vector<T>& objects)
{
  b2PolygonShape obb; 
  obb.SetAsBox(hx, hy, pos, angle);

  b2SectorCollider collider(&circle, filter, b2Transform(b2Vec2(0, 0)));
  return Query(cobj, objects);
}

inline bool b2SectorGrid::ShouldCollide(const b2Filter& filterA, const b2Filter& filterB)
{
  if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
  {
    return filterA.groupIndex > 0;
  }

  bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
  return collide;
}

#endif //B2_SECTOR_GRID_H
