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

// b2SectorGrid ���ο��� �����ϰ� b2DynamicTree�� Proxy�� �����Ͽ� 
// ������ �� �ְ� �ϴ� ������Ʈ
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
  // ������
  /** 
   * @param oid - objectId�� b2SectorGrid���� �Ҵ�
   * @param shape - b2Shape ������� �������� �Ѱ� �޾� �Ҹ��ڿ��� ����
   * @param filter - �浹 ���� 
   * @param tf - ��ȯ 
   * @param userData - ���ø����̼ǿ��� ������ ������. �������� ���� ����
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

  // b2DynamicTree�� proxy�� ����
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

  // aabb�� ��ġ�� �ʴ� ���͵鿡�� ����
  void DetachProxy(const b2AABB& aabb);

  // proxyId�� �ش��ϴ� ���Ϳ��� ����
  void DetachProxy(int proxyId);

  // attach �� ��� ���Ϳ��� ����
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

// ���� ������ b2SectorObject���� �����ϰ�, b2DynamicTree�� ���Ͻÿ� ����
/**
 * b2DynamicTree�� ������ �ִ� �߰� �������̽�. �� ó���� �� �ӹ�.
 */
class B2_API b2Sector
{
public: 
  using ObjectId = std::size_t;

public: 
  // index�� bounds�� ����
  b2Sector(int index, const b2AABB& bounds);

  ~b2Sector();

  // aabb�� ������ ���� userData�� �����ϴ� b2DynamicTree�� Proxy ����. userData�� b2SectorObject.
  int32 CreateProxy(const b2AABB& aabb, void* userData);

  // proxyId�� ���Ͻø� b2DynamicTree���� ����
  void DestroyProxy(int32 proxyId);

  // proxyId�� ���Ͻø� b2DynamicTree���� �̵�. 
  bool MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);

  // aabb ���� ���� ������ proxyId ����� ����
  int Query(const b2AABB& aabb, std::vector<int32>& lst) const
  {
    rx::slock slock(m_lock);
    return m_tree->Query(aabb, lst);
  }

  // input ���̿� �浹�ϴ� proxyId ����� ����
  int RayCast(const b2RayCastInput& input, std::vector<int32>& lst) const
  {
    rx::slock slock(m_lock);
    return m_tree->RayCast(input, lst);
  }

  // proxyId�� �ش��ϴ� b2SectorObject�� ����
  b2SectorObject* GetObject(int32 proxyId)
  {
    rx::slock slock(m_lock);
    return reinterpret_cast<b2SectorObject*>(m_tree->GetUserData(proxyId));
  }

  // ���� ������ ��ġ�� �� Ȯ��
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
