#include "box2d/b2_sector_grid.h"

constexpr b2ObjectId InvalidObjectId = -1;

b2SectorGrid::b2SectorGrid(const b2SectorSettings& settings)
  : m_settings(settings)
  , m_objectIdSequence(1)     // 0 is invalid id
{
  b2Assert(m_settings.bounds.upperBound.x > m_settings.bounds.lowerBound.x);
  b2Assert(m_settings.bounds.upperBound.y > m_settings.bounds.lowerBound.y);
  b2Assert(m_settings.sectorSize >= 1);
  b2Assert(m_settings.bounds.GetExtents().x >= m_settings.sectorSize / 2);
  b2Assert(m_settings.bounds.GetExtents().y >= m_settings.sectorSize / 2);

  auto extend = b2Vec2(m_settings.sectorSize, m_settings.sectorSize);
  extend *= 3; // have 3 or 4 more sectors around the original map bounds

  m_sectorBoundsExtended.lowerBound = m_settings.bounds.lowerBound - extend;
  m_sectorBoundsExtended.upperBound = m_settings.bounds.upperBound + extend;

  auto area = m_sectorBoundsExtended.GetExtents();
  area *= 2;  

  m_sectorCountX = static_cast<int>(area.x / m_settings.sectorSize);
  m_sectorCountY = static_cast<int>(area.y / m_settings.sectorSize);

  // 섹터 크기의 2배로 범위를 확장해서 개수를 계산했으므로 
  // 적어도 하나의 섹터가 여분으로 확장되어 있다. 

  b2Assert(m_sectorCountX >= 2);
  b2Assert(m_sectorCountY >= 2);
}

b2SectorGrid::~b2SectorGrid()
{
  for (auto& kv : m_sectors)
  {
    delete kv.second;
  }

  for (auto& kv : m_objects)
  {
    delete kv.second;
  }

  m_sectors.clear();
  m_objects.clear();
}

std::pair<b2ObjectId, b2Result::Code> b2SectorGrid::Spawn(
  b2Shape* shape, const b2Filter& filter, b2Transform& tf, void* userData)
{
  b2Assert(shape);
  b2Assert(shape->GetChildCount() == 1); // 자식이 하나인 모양만 지원

  if (shape->GetChildCount() > 1)
  {
    return std::pair(InvalidObjectId, b2Result::Fail_Too_Many_Shape_Child_Count);
  }

  b2AABB aabb;
  shape->ComputeAABB(&aabb, tf, 0);

  auto obj = new b2SectorObject(AcquireObjectId(), shape, filter, tf, userData);

  // Get overlapping sectors 

  int ix = GetSectorIndexX(aabb.GetCenter().x);
  int iy = GetSectorIndexX(aabb.GetCenter().y);

  if (!CheckIndexBounds(ix, iy))
  {
    return std::pair(InvalidObjectId, b2Result::Fail_Invalid_Object_Position);
  }

  // Ensure neighboring sectors 
  EnsureSector(ix-1, iy-1); EnsureSector(ix, iy-1); EnsureSector(ix+1, iy-1);
  EnsureSector(ix-1, iy);   EnsureSector(ix, iy);   EnsureSector(ix+1, iy);
  EnsureSector(ix-1, iy+1); EnsureSector(ix, iy+1); EnsureSector(ix+1, iy+1); 

  // 섹터는 만들어지면 소멸하지 않으므로 아래는 안전하다. 
  int cnt = ApplyNeighbors(ix, iy, [this, &aabb, obj](b2Sector* sector) {
    if (sector->IsOverlapping(aabb))
    {
      auto proxyId = sector->CreateProxy(aabb, (void*)obj);
      obj->AttachProxy(sector, proxyId);
      return true;
    }

    return false;
    });

  b2Assert(cnt == obj->GetProxyCount());

  // keep object
  {
    rx::xlock xlock(m_lock);
    m_objects.insert(ObjectMap::value_type(obj->GetObjectId(), obj));
  }

  return std::pair(obj->GetObjectId(), b2Result::Success);
}

void b2SectorGrid::Despawn(b2ObjectId oid)
{
  // Remove from Sectors

  // slock
  {
    rx::slock slock(m_lock);

    auto iter = m_objects.find(oid);
    if (iter != m_objects.end())
    {
      auto obj = iter->second;
      obj->DetachProxyAll();

      // remove
      {
        rx::xlock xlock(m_lock);
        m_objects.erase(oid);
      }

      delete obj;
    }
  }
}

void b2SectorGrid::Move(b2ObjectId oid, const b2Vec2& position, const b2Rot& rotation)
{
  // 한 오브젝트에 대해 한 쓰레드에서 한번한 호출한다고 가정 

  // b2SectorObject를 찾아서 position, rotation으로 tf를 만든 후에 
  // AABB를 계산하여 기존 섹터들과 겹치는 지를 보고 추가/삭제한다. 

  b2SectorObject* obj = nullptr;

  // slock
  {
    rx::slock slock(m_lock);

    auto iter = m_objects.find(oid);
    if (iter != m_objects.end())
    {
      obj = iter->second;
    }
  }

  if (obj == nullptr)
  {
    return;
  }

  b2Transform tf(position, rotation);
  obj->UpdateTransfom(tf);

  b2AABB aabb;
  obj->GetShape()->ComputeAABB(&aabb, tf, 0);

  int ix = GetSectorIndexX(aabb.GetCenter().x);
  int iy = GetSectorIndexX(aabb.GetCenter().y);

  b2Assert(CheckIndexBounds(ix, iy));

  if (!CheckIndexBounds(ix, iy))
  {
    return;
  }

  // Ensure neighboring sectors 
  EnsureSector(ix - 1, iy - 1); EnsureSector(ix, iy - 1); EnsureSector(ix + 1, iy - 1);
  EnsureSector(ix - 1, iy);     EnsureSector(ix, iy);     EnsureSector(ix + 1, iy);
  EnsureSector(ix - 1, iy + 1); EnsureSector(ix, iy + 1); EnsureSector(ix + 1, iy + 1);

  // slock
  {
    rx::slock slock(m_lock);
    obj->DetachProxy(aabb);
  }

  // 겹치는 attach 안 된 인접한 섹터들에 추가.
  int cnt = ApplyNeighbors(ix, iy, [this, &aabb, obj](b2Sector* sector) {
    if (sector->IsOverlapping(aabb))
    {
      if (obj->IsAttached(sector))
      {
        auto proxyId = obj->GetProxyIdBy(sector);
        b2Assert(proxyId >= 0);
        sector->MoveProxy(proxyId, aabb, b2Vec2());
        return true;
      }
      // else
      auto proxyId = sector->CreateProxy(aabb, (void*)obj);
      obj->AttachProxy(sector, proxyId);
      return true;
    }

    return false;
    });
}

void b2SectorGrid::EnsureSector(int ix, int iy)
{
  b2Assert(CheckIndexBounds(ix, iy));

  rx::slock slock(m_lock);

  auto sector = GetSector(ix, iy);
  if (sector == nullptr)
  {
    b2AABB bounds; 
    bounds.lowerBound.x = m_sectorBoundsExtended.lowerBound.x + ix * m_settings.sectorSize;
    bounds.lowerBound.y = m_sectorBoundsExtended.lowerBound.y + iy * m_settings.sectorSize;
    bounds.upperBound.x = m_sectorBoundsExtended.lowerBound.x + (ix+1) * m_settings.sectorSize;
    bounds.upperBound.y = m_sectorBoundsExtended.lowerBound.y + (iy+1) * m_settings.sectorSize;

    auto sectorIndex = GetSectorIndexFrom(ix, iy);
    sector = new b2Sector(sectorIndex, bounds);

    rx::xlock xlock(m_lock);
    m_sectors.insert(SectorMap::value_type(sectorIndex, sector));
  }
}

int b2SectorGrid::AcquireObjectId()
{
  // xlock
  {
    rx::xlock xlock(m_lock);

    if (!m_idQueue.empty())
    {
      int id = m_idQueue.front();
      m_idQueue.pop();
      return id;
    }
  }

  if (m_objectIdSequence < INT32_MAX)
  {
    return ++m_objectIdSequence;
  }
  else
  {
    throw std::exception("used all of the object ids");
  }
}

void b2SectorGrid::ReleaseObjectId(int id)
{
  rx::xlock xlock(m_lock);
  m_idQueue.push(id);
}
