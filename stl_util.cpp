#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <cfloat>
#include <map>
#include "stl_util.h"

namespace stl {

	std::string fstl_t::m_header;
	trians_t fstl_t::m_triangles;

	std::istream& operator>>(std::istream& in, point_t &p)
    {
		in >> p.x >> p.y >> p.z;
		return in;
	}
	std::ostream& operator<<(std::ostream& out, const point_t p)
    {
		out << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
		return out;
	}

	std::ostream& operator<<(std::ostream& out, const triangle_t& t)
    {
		out << "----TRIANGLE----" << std::endl;
		out << t.normal << std::endl;
		out << t.v1 << std::endl;
		out << t.v2 << std::endl;
		out << t.v3 << std::endl;             
		return out;
	}

	point_t fstl_t::gpoint_(std::ifstream& s)
    {
		float x;
        s.read((char*)&x, sizeof(x));
        float y;
        s.read((char*)&y, sizeof(y));
        float z;
        s.read((char*)&z, sizeof(z));
		return point_t(x, y, z);
	}

	void fstl_t::fread(const std::string& stl_path)
    {
		std::ifstream fstl(stl_path.c_str(), std::ios::in | std::ios::binary);
		if (!fstl) {
			std::cout << "ERROR: COULD NOT READ FILE" << std::endl;
			assert(false);
		}
		m_triangles.clear();
		char header_info[80] = "";
		char n_triangles[4];
		fstl.read(header_info, 80);
		fstl.read(n_triangles, 4);
		m_header = header_info;
		unsigned int* r = (unsigned int*) n_triangles;
		unsigned int num_triangles = *r;
		for (unsigned int i = 0; i < num_triangles; i++) {
			point_t normal = gpoint_(fstl);
			point_t v1 = gpoint_(fstl);
			point_t v2 = gpoint_(fstl);
			point_t v3 = gpoint_(fstl);
			m_triangles.push_back(triangle_t(normal, v1, v2, v3));
			char dummy[2];
			fstl.read(dummy, 2);
		}
	}
//-----------------------------------------------------------------------
	void find_path_t::gnearest_vertex_to_point_(	const point_t &point,
													point_t &rvertex)													
	
	{
		triangle_t triangle;
		trians_t::const_iterator iter = m_triangles.begin();
		triangle=(*iter);

		float dist = triangle.distance_to(point); 
		while(iter != m_triangles.end()) 
		{
			float d = iter->distance_to(point); 
			if(d < dist) 
			{
				dist = d;
				triangle = *iter;
			}
			++iter;
		}
		float d1 = triangle.v1.distance_to(point);
		float d2 = triangle.v2.distance_to(point);
		if(dist == d1) 
			rvertex = triangle.v1;
		else if(dist == d2) 
			rvertex = triangle.v2;
		else 
			rvertex = triangle.v3;
	}

	void find_path_t::gnext_vertex_()
	{
		float dist;
		point_t s;
		std::map<float, point_t> distance_and_next_vertex;
		trians_t::const_iterator iter = m_triangles.begin();
		
		path_t::iterator ibeg_f = m_path.begin();
		path_t::iterator ibeg_s = m_path.begin();
		path_t::iterator iend = m_path.end();
		
		while(iter != m_triangles.end()) 
		{
			
			if( m_rvertex == iter->v1 && 
				std::find(ibeg_f, iend, iter->v2) == iend && 
				std::find(ibeg_s, iend, iter->v3) == iend
			) 
			{
				s = iter->v2 - m_rvertex;
				dist = gdistance_berween_lines_( m_start_vertex, m_svertex, m_rvertex, s );
				distance_and_next_vertex.insert( std::pair<float, point_t>(dist, iter->v2) );
				s = iter->v3 - m_rvertex;
				dist = gdistance_berween_lines_( m_start_vertex, m_svertex, m_rvertex, s );
				distance_and_next_vertex.insert( std::pair<float, point_t>(dist, iter->v3) );
			}
			
			ibeg_f = m_path.begin();
			ibeg_s = m_path.begin();			
			if( m_rvertex == iter->v2  && 
				std::find(ibeg_f, iend, iter->v1) == iend && 
				std::find(ibeg_s, iend, iter->v3) == iend
			) 
			{
				s = iter->v3 - m_rvertex;
				dist = gdistance_berween_lines_( m_start_vertex, m_svertex, m_rvertex, s );
				distance_and_next_vertex.insert(std::pair<float, point_t>(dist, iter->v3));
				s = iter->v1 - m_rvertex;
				dist = gdistance_berween_lines_( m_start_vertex, m_svertex, m_rvertex, s );
				distance_and_next_vertex.insert( std::pair<float, point_t>(dist, iter->v1) );
			}
			
			ibeg_f = m_path.begin();
			ibeg_s = m_path.begin();				
			if( m_rvertex == iter->v3  && 
				std::find(ibeg_f, iend, iter->v2) == iend && 
				std::find(ibeg_s, iend, iter->v1) == iend
			) 
			{
				s = iter->v2 - m_rvertex;
				dist = gdistance_berween_lines_( m_start_vertex, m_svertex, m_rvertex, s );
				distance_and_next_vertex.insert(std::pair<float, point_t>(dist, iter->v2));
				s = iter->v1 - m_rvertex;
				dist = gdistance_berween_lines_( m_start_vertex, m_svertex, m_rvertex, s );
				distance_and_next_vertex.insert( std::pair<float, point_t>(dist, iter->v1) );
			}
			++iter;
		}
        std::map<float, point_t>::iterator it = distance_and_next_vertex.begin();
		m_rvertex = it->second;
	}
	bool find_path_t::gpath_() 
	{
		gnext_vertex_();
		m_path.push_back(m_rvertex);
		if(m_rvertex == m_target_vertex) return true;
		if(m_path.size() > m_triangles.size() / 2 ) return false;
		gpath_();
        return false;
	}
	
	float find_path_t::gdistance_berween_lines_(    const point_t &r1, const point_t &s1,
													const point_t &r2, const point_t &s2) const
	{
		float dist;
        float syz = s1.y * s2.z - s1.z * s2.y;
        float szx = s1.z * s2.x - s1.x * s2.z;
        float sxy = s1.x * s2.y - s1.y * s2.x;

        point_t zero;
		point_t s1_s2 =	point_t( syz, szx, sxy);

		dist = s1_s2.distance_to(zero);
		if(dist <= FLT_EPSILON) // lines are parallel
		{
            dist = s1.distance_to(zero);
			syz = (r2.y - r1.y) * s1.z - (r2.z - r1.z) * s1.y;
			szx = (r2.z - r1.z) * s1.x - (r2.x - r1.x) * s1.z;
			sxy = (r2.x - r1.x) * s1.y - (r2.y - r1.y) * s1.x;
			s1_s2 =	point_t( syz, szx, sxy);
			dist = dist < FLT_EPSILON? FLT_MAX: s1_s2.distance_to(zero) / dist;
		}
		else
		{
			float r2_r1_s1s2 =  (r2.x - r1.x)*syz +
								(r2.y - r1.y)*szx +
								(r2.z - r1.z)*sxy;
			dist = dist < FLT_EPSILON? FLT_MAX: std::abs(r2_r1_s1s2) / dist;
		}
        return dist;
		
	}
	
	void find_path_t::operator()(path_t *path)
	{
		m_path.clear();
		m_rvertex = m_start_vertex;
		m_path.push_back(m_start_vertex);
		bool b = gpath_();
		if(b && path)
		{
			path_t::const_iterator iter = m_path.begin();
			while(iter != m_path.end())
			{
				path->push_back(*iter);
				++iter;
			}
		}      
	}
}
