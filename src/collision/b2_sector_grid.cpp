#include "box2d/b2_sector_grid.h"

b2SectorGrid::b2SectorGrid(const b2SectorSettings& settings)
  : m_settings(settings)
  , m_objectIdSequence(1)     // 0 is invalid id
{
  b2Assert(m_settings.sectorSize >= 1);
  b2Assert(m_settings.bounds.GetExtents().x >= m_settings.sectorSize / 2);
  b2Assert(m_settings.bounds.GetExtents().y >= m_settings.sectorSize / 2);

  auto extend = b2Vec2(m_settings.sectorSize, m_settings.sectorSize);
  extend *= 2; // have 2 or 3 more sectors around the original map bounds

  m_sectorBoundsExtended.lowerBound = m_settings.bounds.lowerBound - extend;
  m_sectorBoundsExtended.upperBound = m_settings.bounds.upperBound + extend;

  auto area = m_sectorBoundsExtended.GetExtents();
  area *= 2;  

  m_sectorCountX = area.x / m_settings.sectorSize;
  m_sectorCountY = area.y / m_settings.sectorSize;

  // ���� ũ���� 2��� ������ Ȯ���ؼ� ������ ��������Ƿ� 
  // ��� �ϳ��� ���Ͱ� �������� Ȯ��Ǿ� �ִ�. 
}

b2SectorGrid::~b2SectorGrid()
{
  // TODO: clear sectors and sector objects
}

b2ObjectId b2SectorGrid::Spawn(
  b2Shape* shape, const b2Filter& filter, b2Transform& tf, void* userData)
{
  b2Assert(shape);
  b2Assert(shape->GetChildCount() == 1); // �ڽ��� �ϳ��� ��縸 ����

  b2AABB aabb;
  shape->ComputeAABB(&aabb, tf, 0);

  auto obj = new b2SectorObject(AcquireObjectId(), shape, filter, tf, userData);

  // get overlapping sectors 
  // CreateProxy()
  // attach proxy


  // keep object
  {
    rx::xlock xlock(m_lock);
    m_objects.insert(ObjectMap::value_type(obj->GetObjectId(), obj));
  }

  return obj->GetObjectId();
}

void b2SectorGrid::Despawn(b2ObjectId oid)
{

}

void b2SectorGrid::Update(b2ObjectId oid, const b2Vec2& position, const b2Vec2& rotation)
{
  // 
}

int b2SectorGrid::GetSectorIndexFrom(float x, float y)
{
  // lowerBound�� �������� ����Ѵ�. 
  float sx = x - m_sectorBoundsExtended.lowerBound.x;
  float sy = y - m_sectorBoundsExtended.lowerBound.y;

  int ix = sx / m_settings.sectorSize;
  int addX = (sx - m_settings.sectorSize * ix) > 0 ? 1 : 0;

  ix += addX;

  int iy = sy / m_settings.sectorSize;
  int addY = (sy - m_settings.sectorSize * iy) > 0 ? 1 : 0;

  iy += addY;

  // �ٱ� �� ��迡 �� ���� ��� ���� ���� �ε����� ������ 
  // AABB ����� �� ���� ���Ϳ͵� overlap �ǹǷ� �����Ѵ�.

  return iy * m_sectorCountX + ix;
}

int b2SectorGrid::AcquireObjectId()
{
  rx::xlock xlock(m_lock);

  if (!m_idQueue.empty())
  {
    int id = m_idQueue.front();
    m_idQueue.pop();
    return id;
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
