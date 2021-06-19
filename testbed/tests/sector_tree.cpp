#include "test.h"
#include "settings.h"
#include <box2d/b2_sector_grid.h>

class Projectile
{
public:
	Projectile(b2SectorGrid* grid, int id, b2Filter filter)
		: m_grid(grid)
		, m_id(id)
		, m_filter(filter)
		, m_objectId(0)
	{
		m_rotation = b2Rot(RandomFloat(-3.14f, 3.14f));

		float w = RandomFloat(30, 100);
		float h = RandomFloat(30, 100);
		m_obb = new b2PolygonShape();
		m_obb->SetAsBox(w, h);

		auto bounds = grid->GetWorldBounds();

		m_spos = b2Vec2(
			RandomFloat(bounds.lowerBound.x, bounds.upperBound.x),
			RandomFloat(bounds.lowerBound.y, bounds.upperBound.y)
		);

		m_position = m_spos;
	}

	void Spawn()
	{
		b2Transform xf(m_position, m_rotation);
		auto res = m_grid->Spawn(m_obb, m_filter, xf, reinterpret_cast<void*>(&m_id));

		if (b2Result::Succeeded(res))
		{
			m_objectId = res.first;
		}
	}

	void Move()
	{
		m_dir = b2Vec2(m_rotation.c, m_rotation.s);
		m_dir *= 30;
		m_position += m_dir;

		if ( !IsInWorldBounds() )
		{
			m_position = m_spos;
		}

		m_grid->Move(m_objectId, m_position, m_rotation);
	}

	void Collide(std::function<void(int)> cb)
	{
		b2Transform xf(m_position, m_rotation);
		b2SectorObject collider(m_objectId, m_obb, m_filter, xf, &m_id, true);

		std::vector<int> lst;

		if (m_grid->Query<int>(collider, lst) > 0)
		{
			for (auto id : lst)
			{
				cb(id);
			}
		}
	}

	const b2Shape* GetShape() const
	{
		return m_obb;
	}

	const b2Vec2& GetPosition() const
	{
		return m_position;
	}

	const b2Rot& GetRotation() const
	{
		return m_rotation;
	}

	bool IsInWorldBounds()
	{
		if (m_position.x < m_grid->GetWorldBounds().lowerBound.x)
		{
			return false;
		}

		if (m_position.y < m_grid->GetWorldBounds().lowerBound.y)
		{
			return false;
		}

		if (m_position.x > m_grid->GetWorldBounds().upperBound.x)
		{
			return false;
		}

		if (m_position.y > m_grid->GetWorldBounds().upperBound.y)
		{
			return false;
		}
		return true;
	}

private: 
	b2Filter m_filter;
	b2SectorGrid* m_grid; 
	int m_id;
	b2ObjectId m_objectId;
	b2Vec2 m_spos;
	b2Vec2 m_position;
	b2Rot m_rotation;
	b2Vec2 m_dir;
	b2PolygonShape* m_obb;
};

class Actor
{
public: 
	Actor(
		b2SectorGrid* grid, b2Shape* shape, int id, 
		const b2Vec2& pos, const b2Rot& rotation, b2Filter filter
	)
		: m_grid(grid)
		, m_shape(shape)
		, m_id(id)
		, m_position(pos)
		, m_rotation(rotation)
		, m_filter(filter)
		, m_objectId(0)
		, m_overlap(false)
	{
	}

	void Spawn()
	{
		b2Transform xf(m_position, m_rotation);
		auto res = m_grid->Spawn(m_shape, m_filter, xf, reinterpret_cast<void*>(&m_id));

		if (b2Result::Succeeded(res))
		{
			m_objectId = res.first;
		}
	}

	void MoveRandom()
	{
		b2Vec2 dir; 

		dir.x = RandomFloat(-1.0f, 1.0f);
		dir.y = RandomFloat(-1.0f, 1.0f);
		dir.Normalize();
		dir *= 30;

		m_position += dir;
		m_rotation = b2Rot(RandomFloat(-0.785f, 0.785f));

		ClampToWorldBounds();
		m_grid->Move(m_objectId, m_position, m_rotation);
	}

	void ClampToWorldBounds()
	{
		if (m_position.x < m_grid->GetWorldBounds().lowerBound.x)
		{
			m_position.x = m_grid->GetWorldBounds().lowerBound.x + RandomFloat(100, 200);
		}

		if (m_position.y < m_grid->GetWorldBounds().lowerBound.y)
		{
			m_position.y = m_grid->GetWorldBounds().lowerBound.y + RandomFloat(100, 200);
		}

		if (m_position.x > m_grid->GetWorldBounds().upperBound.x)
		{
			m_position.x = m_grid->GetWorldBounds().upperBound.x - RandomFloat(100, 200);
		}

		if (m_position.y > m_grid->GetWorldBounds().upperBound.y)
		{
			m_position.y = m_grid->GetWorldBounds().lowerBound.y - RandomFloat(100, 200);
		}
	}

	void SetOverlap(bool overlap)
	{
		m_overlap = overlap;
	}

	bool IsOverlapped() const
	{
		return m_overlap;
	}

	b2Shape* GetShape() const
	{
		return m_shape;
	}

	const b2Vec2& GetPosition() const
	{
		return m_position;
	}

	const b2Rot& GetRotation() const
	{
		return m_rotation;
	}

private:
	b2SectorGrid* m_grid;
	b2Shape* m_shape;
	int m_id; 
	b2Filter m_filter;
	b2AABB m_aabb;
	b2ObjectId m_objectId;
	bool m_overlap;
	b2Vec2 m_position;
	b2Rot m_rotation;
};

// SectrGrid
class SectorGrid : public Test
{
private: 

public:

	enum
	{
		e_actorCount = 1000, 
		e_projectileCount = 1000 
	};

	SectorGrid()
		: m_stepCount(0)
	{
		b2SectorSettings settings; 

		settings.bounds.lowerBound.x = -20000;
		settings.bounds.lowerBound.y = -20000;
		settings.bounds.upperBound.x = 20000;
		settings.bounds.upperBound.y = 20000;
		settings.sectorSize = 1000;

		m_grid = new b2SectorGrid(settings);

		srand(888);

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			float w = RandomFloat(30, 100);
			float h = RandomFloat(30, 100);

			b2PolygonShape* obb = new b2PolygonShape();
			obb->SetAsBox(w, h);

			auto pos = b2Vec2( 
					RandomFloat(settings.bounds.lowerBound.x, settings.bounds.upperBound.x),
					RandomFloat(settings.bounds.lowerBound.y, settings.bounds.upperBound.y)
				);
			auto rot = b2Rot(RandomFloat(-3.14f, 3.14f));

			Actor* actor = new Actor(m_grid, obb, i+1, pos, rot, b2Filter());
			m_actors[i] = actor;
			actor->Spawn();
		}

		for (int32 i = 0; i < e_projectileCount; ++i)
		{
			Projectile* projectile = new Projectile(m_grid, i + 1 + e_actorCount, b2Filter());
			m_projectiles[i] = projectile;
			projectile->Spawn();
		}
	}

	~SectorGrid()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			delete m_actors[i];
		}

		for (int32 i = 0; i < e_projectileCount; ++i)
		{
			delete m_projectiles[i];
		}

		delete m_grid;
	}

	static Test* Create()
	{
		return new SectorGrid;
	}

	void Step(Settings& settings) override
	{
		static int stepCount = 0;

		++stepCount;

		B2_NOT_USED(settings);

		if (!settings.m_pause)
		{
			for (int32 i = 0; i < e_actorCount; ++i)
			{
				m_actors[i]->SetOverlap(false);
			}

			for (int32 i = 0; i < e_actorCount; ++i)
			{
				m_actors[i]->MoveRandom();
			}

			for (int32 i = 0; i < e_actorCount; ++i)
			{
				m_projectiles[i]->Move();
			}

			for (int32 i = 0; i < e_actorCount; ++i)
			{
				m_projectiles[i]->Collide([this](int id) {
					if (1 <= id && id <= e_actorCount)
					{
						int index = id - 1;
						m_actors[index]->SetOverlap(true);
					}
					});
			}
		}

		DrawGrids();
		DrawActors();
		DrawProjectiles();

		g_debugDraw.Flush();
	}

	void DrawGrids()
	{
		b2Color c(0.7f, 0.7f, 0.7f);

		for (int ix = 0; ix <= m_grid->GetSectorCountX(); ++ix)
		{
			float px = m_grid->GetWorldBoundsExtended().lowerBound.x + ix * m_grid->GetSectorSize();
			float ly = m_grid->GetWorldBoundsExtended().lowerBound.y;
			float uy = m_grid->GetWorldBoundsExtended().upperBound.y;

			b2Vec2 p1(px, ly);
			b2Vec2 p2(px, uy);

			g_debugDraw.DrawSegment(p1, p2, c);
		}

		for (int iy = 0; iy <= m_grid->GetSectorCountY(); ++iy)
		{
			float py = m_grid->GetWorldBoundsExtended().lowerBound.y + iy * m_grid->GetSectorSize();
			float lx = m_grid->GetWorldBoundsExtended().lowerBound.x;
			float rx = m_grid->GetWorldBoundsExtended().upperBound.x;

			b2Vec2 p1(lx, py);
			b2Vec2 p2(rx, py);

			g_debugDraw.DrawSegment(p1, p2, c);
		}
	}

	void DrawActors()
	{
		for (int i = 0; i < e_actorCount; ++i)
		{
			auto actor = m_actors[i];
			auto obb = reinterpret_cast<b2PolygonShape*>(actor->GetShape());
			auto rot = actor->GetRotation();
			auto pos = actor->GetPosition();

			b2Color c1(0.0f, 0.9f, 0.0f);
			b2Color c2(0.9f, 0.0f, 0.0f);

			b2Vec2 vs[4];

			for (int iv = 0; iv < 4; ++iv)
			{
				auto v = obb->m_vertices[iv];
				auto vx = v.x * rot.c - v.y * rot.s;
				auto vy = v.x * rot.s + v.y * rot.c;
				vs[iv].x = vx + pos.x;
				vs[iv].y = vy + pos.y;
			}
			
			b2Color c = actor->IsOverlapped() ? c2 : c1;
			
			g_debugDraw.DrawSegment(vs[0], vs[1], c);
			g_debugDraw.DrawSegment(vs[1], vs[2], c);
			g_debugDraw.DrawSegment(vs[2], vs[3], c);
			g_debugDraw.DrawSegment(vs[3], vs[0], c);

			char buf[24];
			sprintf_s(buf, "%d", i + 1);
			g_debugDraw.DrawString(pos, buf);
		}
	}

	void DrawProjectiles()
	{
		for (int i = 0; i < e_projectileCount; ++i)
		{
			auto projectile = m_projectiles[i];
			auto obb = reinterpret_cast<const b2PolygonShape*>(projectile->GetShape());
			auto rot = projectile->GetRotation();
			auto pos = projectile->GetPosition();

			b2Color c(0.0f, 0.0f, 1.0f);

			b2Vec2 vs[4];

			for (int iv = 0; iv < 4; ++iv)
			{
				auto v = obb->m_vertices[iv];
				auto vx = v.x * rot.c - v.y * rot.s;
				auto vy = v.x * rot.s + v.y * rot.c;
				vs[iv].x = vx + pos.x;
				vs[iv].y = vy + pos.y;
			}

			g_debugDraw.DrawSegment(vs[0], vs[1], c);
			g_debugDraw.DrawSegment(vs[1], vs[2], c);
			g_debugDraw.DrawSegment(vs[2], vs[3], c);
			g_debugDraw.DrawSegment(vs[3], vs[0], c);
		}
	}

private:
	b2SectorGrid* m_grid;
	std::array<Actor*, e_actorCount> m_actors;
	std::array<Projectile*, e_projectileCount> m_projectiles;
	int32 m_stepCount;
};

static int testIndex = RegisterTest("Collision", "Sector Grid", SectorGrid::Create);
