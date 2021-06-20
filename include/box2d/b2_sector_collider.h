#ifndef B2_SECTOR_COLLIDER_H
#define B2_SECTOR_COLLIDER_H

#include "b2_api.h"
#include "b2_fixture.h"
#include "b2_math.h"
#include "b2_settings.h"
#include "b2_collision.h"

class b2SectorCollider
{
public:
  b2SectorCollider(b2Shape* shape, const b2Filter& filter, const b2Transform& tf)
    : m_shape(shape)
    , m_filter(filter)
    , m_transform(tf)
  {
    b2Assert(m_shape);
  }

  ~b2SectorCollider()
  {
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

private:
  b2Shape* m_shape;
  b2Filter m_filter;
  b2Transform m_transform;
};

#endif // B2_SECTOR_COLLIDER_H