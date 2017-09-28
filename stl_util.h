#ifndef STL_UTIL_H
#define STL_UTIL_H

#include <string>
#include <vector>
#include <cmath>
#include <cassert>

namespace stl
{
	struct point_t
	{
		float x;
		float y;
		float z;

		point_t() : x(0), y(0), z(0) {}
		point_t(float xp, float yp, float zp) : x(xp), y(yp), z(zp) {}
		point_t operator-(const point_t& point) const 
		{ 
			return point_t(x - point.x, x - point.y, x - point.z);
		}
		float distance_to(const point_t& point) const 
		{
			point_t d = *this - point; 
			return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
		}
		bool operator==(const point_t &point) const 
		{
			return x == point.x && y == point.y && z == point.z;
		}
		bool operator!=(const point_t &point) const 
		{
			return !operator==(point);
		}
	};
	
	struct direction_t 
	{
		point_t start;
		point_t target;
	};
	struct triangle_t
	{
		point_t normal;
		point_t v1;
		point_t v2;
		point_t v3;
        triangle_t() {}
		triangle_t(point_t &normalp, point_t &v1p, point_t &v2p, point_t &v3p):
					normal(normalp), v1(v1p), v2(v2p), v3(v3p) {}
		float distance_to(const point_t& point) const
		{
			return std::min
			(
				point.distance_to(v1),
				std::min(point.distance_to(v2), point.distance_to(v1))
			);
		}
		bool operator==(const triangle_t &trian) const
		{
			return
					v1 == trian.v1
				&& 	v2 == trian.v2
				&&	v3 == trian.v3
				&&	normal == trian.normal;

		}
		bool operator!=(const triangle_t &trian) const
		{
			return  !operator==(trian);
		}
	};
    std::ostream& operator<<(std::ostream& out, const point_t p);
    std::istream& operator>>(std::istream& in, point_t &p);
    std::ostream& operator<<(std::ostream& out, const triangle_t& t);

	typedef std::vector<triangle_t> trians_t;
	typedef std::vector<point_t> path_t;

	class fstl_t
	{
		point_t gpoint_(std::ifstream& s);

		static std::string m_header;
		static trians_t m_triangles;
	public:
		void fread(const std::string& fstl_path);
		const trians_t& gtriangles() const { return m_triangles; }
		const std::string& gheader() const { return m_header; }

	};

	class find_path_t
	{
	private:
		void gnearest_vertex_to_point_(	const point_t &point, 
										point_t &rvertex);

		
		void gnext_vertex_();		
		bool gpath_();
		//here lines are presented in vector view (line = r + t * s, t - parameter)
		float gdistance_berween_lines_(	const point_t &r1, const point_t &s1,
											const point_t &r2, const point_t &s2) const;
		
			
		const trians_t &m_triangles;		
		const direction_t &m_direction;
			
		path_t	m_path;
		
		point_t m_start_vertex;
		point_t m_target_vertex;
		point_t m_svertex;	

		point_t m_rvertex;
		
	public:
		find_path_t(const trians_t &trians, const direction_t &dir):
		m_triangles(trians), m_direction(dir)
		{
			assert(trians.begin() != trians.end() && dir.start != dir.target);
				
			gnearest_vertex_to_point_(m_direction.start, m_start_vertex);
			gnearest_vertex_to_point_(m_direction.target, m_target_vertex);
			m_svertex = m_target_vertex - m_start_vertex;
		}
		void operator()(path_t *path = NULL);
        const path_t& gpath() { return m_path; }
        point_t gstart_vertex() { return m_start_vertex; }
        point_t gtarget_vertex() { return m_target_vertex; }
	};
}

#endif