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
  b2SectorObject(
    b2ObjectId oid, b2Shape* shape, const b2Filter& filter,
    const b2Transform& tf, void* userData, bool collider = false)
    : m_objectId(oid)
    , m_shape(shape)
    , m_filter(filter)
    , m_transform(tf)
    , m_userData(userData)
    , m_proxyCount(0)
    , m_proxies()
    , m_collider(collider)
  {
    b2Assert(m_shape);
    b2Assert(m_userData);
  }

  ~b2SectorObject()
  {
    if (!m_collider)
    {
      delete m_shape;
    }
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

  void DetachProxy(const b2AABB& aabb);

  void DetachProxy(int proxyId);

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
  bool m_collider;
};

/// 섹터 내부의 b2SectorObject들을 관리한다. 
/**
 */
class B2_API b2Sector
{
public: 
  using ObjectId = std::size_t;

public: 
  b2Sector(int index, const b2AABB& bounds);

  ~b2Sector();

  int32 CreateProxy(const b2AABB& aabb, void* userData);

  void DestroyProxy(int32 proxyId);

  bool MoveProxy(int32 proxyId, const b2AABB& aabb1, const b2Vec2& displacement);

  int Query(const b2AABB& aabb, std::vector<int32>& lst) const
  {
    rx::slock slock(m_lock);
    return m_tree->Query(aabb, lst);
  }

  int RayCast(const b2RayCastInput& input, std::vector<int32>& lst) const
  {
    rx::slock slock(m_lock);
    return m_tree->RayCast(input, lst);
  }

  b2SectorObject* GetObject(int32 proxyId)
  {
    rx::slock slock(m_lock);
    return reinterpret_cast<b2SectorObject*>(m_tree->GetUserData(proxyId));
  }

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
