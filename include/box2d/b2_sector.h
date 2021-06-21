#ifndef B2_SECTOR_H
#define B2_SECTOR_H

#include "b2_api.h"
#include "b2_fixture.h"
#include "b2_math.h"
#include "b2_settings.h"
#include "b2_collision.h"
#include "b2_dynamic_tree.h"
#include <rx/lock/lock_guards.hpp>
#include <cstddef>

using b2ObjectId = int;

class b2Sector;

// b2SectorGrid 내부에서 관리하고 b2DynamicTree의 Proxy와 연결하여 
// 관리할 수 있게 하는 오브젝트
class b2SectorObject
{
public: 

  struct Proxy
  {
    b2Sector* sector; 
    int proxyId;
    bool attached;
  };

public: 
  // 생성자
  /** 
   * @param oid - objectId로 b2SectorGrid에서 할당
   * @param shape - b2Shape 모양으로 소유권을 넘겨 받아 소멸자에서 해제
   * @param filter - 충돌 필터 
   * @param tf - 변환 
   * @param userData - 애플리케이션에서 전달한 데이터. 소유권을 받지 않음
   */
  b2SectorObject(
    b2ObjectId oid, b2Shape* shape, const b2Filter& filter,
    const b2Transform& tf, void* userData)
    : m_objectId(oid)
    , m_shape(shape)
    , m_filter(filter)
    , m_transform(tf)
    , m_userData(userData)
    , m_proxyCount(0)
    , m_proxies()
  {
    b2Assert(m_shape);
    b2Assert(m_userData);
  }

  ~b2SectorObject()
  {
    delete m_shape;
  }

  b2ObjectId GetObjectId() const 
  {
    return m_objectId;
  }

  const b2Shape* GetShape() const
  {
    return m_shape;
  }

  const b2Filter& GetFilter() const
  {
    return m_filter;
  }

  const b2Transform& GetTransform() const
  {
    return m_transform;
  }

  const void* GetUserData() const
  {
    return m_userData;
  }

  template <typename T>
  const T GetUserData() const
  {
    T* v = reinterpret_cast<T*>(m_userData);
    return *v;
  }

  void UpdateTransfom(const b2Transform& tf)
  {
    m_transform = tf;
  }

  // b2DynamicTree의 proxy와 연결
  bool AttachProxy(b2Sector* sector, int proxyId)
  {
    b2Assert(sector);
    b2Assert(0 <= m_proxyCount && m_proxyCount <= 4);

    for ( auto& proxy : m_proxies )
    { 
      if (!proxy.attached)
      {
        proxy = Proxy{ sector, proxyId, true };
        m_proxyCount++;
        return true;
      }
    }
    return false;
  }

  // aabb와 겹치지 않는 섹터들에서 나옴
  void DetachProxy(const b2AABB& aabb);

  // proxyId에 해당하는 섹터에서 나옴
  void DetachProxy(int proxyId);

  // attach 된 모든 섹터에서 나옴
  void DetachProxyAll();

  int GetProxyIdBy(b2Sector* sector)
  {
    for (auto& proxy : m_proxies)
    {
      if (proxy.attached && proxy.sector == sector)
      {
        return proxy.proxyId;
      }
    }
    return -1;
  }

  bool IsAttached(b2Sector* sector)
  {
    for (auto& proxy : m_proxies)
    {
      if (proxy.attached && proxy.sector == sector)
      {
        return true;
      }
    }
    return false;
  }

  int GetProxyCount() const
  {
    return m_proxyCount;
  }


private: 
  b2ObjectId m_objectId;
  b2Shape* m_shape; 
  b2Filter m_filter; 
  b2Transform m_transform; 
  void* m_userData;

  int m_proxyCount; 
  std::array<Proxy, 4> m_proxies;
};

// 섹터 내부의 b2SectorObject들을 관리하고, b2DynamicTree의 프록시와 연결
/**
 * b2DynamicTree와 연결해 주는 중간 인터페이스. 락 처리가 주 임무.
 */
class B2_API b2Sector
{
public: 
  using ObjectId = std::size_t;

public: 
  // index와 bounds로 생성
  b2Sector(int index, const b2AABB& bounds);

  ~b2Sector();

  // aabb를 범위로 갖고 userData를 보관하는 b2DynamicTree의 Proxy 생성. userData는 b2SectorObject.
  int32 CreateProxy(const b2AABB& aabb, void* userData);

  // proxyId의 프록시를 b2DynamicTree에서 제거
  void DestroyProxy(int32 proxyId);

  // proxyId의 프록시를 b2DynamicTree에서 이동. 
  bool MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);

  // aabb 범위 내에 들어오는 proxyId 목록을 얻음
  int Query(const b2AABB& aabb, std::vector<int32>& lst) const
  {
    rx::slock slock(m_lock);
    return m_tree->Query(aabb, lst);
  }

  // input 레이와 충돌하는 proxyId 목록을 얻음
  int RayCast(const b2RayCastInput& input, std::vector<int32>& lst) const
  {
    rx::slock slock(m_lock);
    return m_tree->RayCast(input, lst);
  }

  // proxyId에 해당하는 b2SectorObject를 얻음
  b2SectorObject* GetObject(int32 proxyId)
  {
    rx::slock slock(m_lock);
    return reinterpret_cast<b2SectorObject*>(m_tree->GetUserData(proxyId));
  }

  // 섹터 범위와 겹치는 지 확인
  bool IsOverlapping(const b2AABB& aabb)
  {
    return b2TestOverlap(m_bounds, aabb);
  }

private: 
  mutable rx::lockable m_lock;            // recursive shared mutex

  int m_index;
  b2AABB m_bounds;
  b2DynamicTree* m_tree;
};

#endif //B2_SECTOR_H
