#include "box2d/b2_sector.h"
#include <rx/lock/lockable.hpp>

void b2SectorObject::DetachProxy(const b2AABB& aabb)
{
  for (auto& proxy : m_proxies)
  {
    if (proxy.attached && !proxy.sector->IsOverlapping(aabb))
    {
      proxy.attached = false;
      m_proxyCount--;
    }
  }
}

void b2SectorObject::DetachProxy(int proxyId)
{
  b2Assert(m_proxyCount >= 0);
  b2Assert(m_proxyCount < 3);

  for (auto& proxy : m_proxies)
  {
    if (proxy.proxyId == proxyId)
    {
      b2Assert(proxy.attached);
      proxy.sector->DestroyProxy(proxyId);
      proxy.attached = false;
      m_proxyCount--;
      return;
    }
  }
}

void b2SectorObject::DetachProxyAll()
{
  for (auto& proxy : m_proxies)
  {
    if (proxy.attached)
    {
      proxy.sector->DestroyProxy(proxy.proxyId);
      proxy.attached = false;
      m_proxyCount--;
    }
  }
}

b2Sector::b2Sector(int index, const b2AABB& bounds)
  : m_index(index)
  , m_bounds(bounds)
{
  m_tree = new b2DynamicTree();
}

b2Sector::~b2Sector()
{
  delete m_tree;
}

int32 b2Sector::CreateProxy(const b2AABB& aabb, void* userData)
{
  rx::xlock xlock(m_lock);
  return m_tree->CreateProxy(aabb, userData);
}

void b2Sector::DestroyProxy(int32 proxyId)
{
  rx::xlock xlock(m_lock);
  return m_tree->DestroyProxy(proxyId);
}

bool b2Sector::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
  rx::xlock xlock(m_lock);
  return m_tree->MoveProxy(proxyId, aabb, displacement);
}