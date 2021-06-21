#ifndef B2_SECTOR_COLLIDER_H
#define B2_SECTOR_COLLIDER_H

#include "b2_api.h"
#include "b2_fixture.h"
#include "b2_math.h"
#include "b2_settings.h"
#include "b2_collision.h"

// b2Shape와 필터, 변환을 갖는 충돌 체크 오브젝트
/** 
 * b2SectorGrid의 Query 함수 호출을 직접할 때 필요한 오브젝트. 
 */
class b2SectorCollider
{
public:
  // 충돌 체크 정보 전달 
  /** 
   * NOTE: shape에 대한 소유권이 이전 되지 않으므로 호출 쪽에서 해제
   */
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