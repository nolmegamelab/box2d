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

class b2SectorObject
{
public: 

  struct Proxy
  {
    int sectorIndex; 
    int proxyId;
  };

public: 
  b2SectorObject(
    b2ObjectId oid, b2Shape* shape, const b2Filter& filter, 
    const b2Transform& tf, void* userData)
    : m_objectId(oid)
    , m_shape(shape)
    , m_filter(filter)
    , m_transform(tf)
    , m_userData(userData)
    , m_proxyCount(0)
  {
    b2Assert(m_shape);
    b2Assert(m_userData);
  }

  b2ObjectId GetObjectId() const 
  {
    return m_objectId;
  }

  b2Shape* GetShape()
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

  bool AttachProxy(int sectorId, int proxyId)
  {
    b2Assert(m_proxyCount < 3);
    m_proxies[m_proxyCount++] = Proxy{ sectorId, proxyId };
    return m_proxyCount < 4;
  }

  int GetProxyCount() const
  {
    return m_proxyCount;
  }

  const Proxy& GetProxy(int index) const
  {
    b2Assert(index < 4);
    return m_proxies[index];
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

/// 섹터 내부의 b2SectorObject들을 관리한다. 
/**
 */
class B2_API b2Sector
{
public: 
  using ObjectId = std::size_t;

public: 
  b2Sector(const b2AABB& bounds);

  ~b2Sector();

  int32 CreateProxy(const b2AABB& aabb, void* userData);

  void DestroyProxy(int32 proxyId);

  bool MoveProxy(int32 proxyId, const b2AABB& aabb1, const b2Vec2& displacement);

private: 
  rx::lockable m_lock;            // recursive shared mutex

  b2AABB m_bounds;
  b2DynamicTree* m_tree;
};

#endif //B2_SECTOR_H
