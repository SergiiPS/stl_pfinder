//---------------------------------------------------------------------------
#include <cassert>
#include <iostream>
#include "stl_util.h"
//---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    using namespace stl;
	std::string stl_fname = "./cube.stl";

	if (argc == 2) {
		stl_fname = argv[1];
	} else if (argc > 2) {
		std::cout << "ERROR: Too many command line arguments" << std::endl;
	}
	
	direction_t d;

	std::cout << "Enter start point (x y z):" << std::endl;
	std::cin >> d.start;
    std::cout << d.start;
	std::cout << "Enter target point (x y z):" << std::endl;
	std::cin >> d.target;
    std::cout << d.target;


	fstl_t fstl;
//Reading of a file *.stl
	fstl.fread(stl_fname);
//Creating object for finding a optimal path on the mesh surfice	
	find_path_t fpath(fstl.gtriangles(), d);
//Searching of optimal path on the mesh surfice along direction between entered points 
    fpath();

	std::cout << "stl header = " << fstl.gheader() << std::endl;
	std::cout << "# triangles = " << fstl.gtriangles().size() << std::endl;
    trians_t::const_iterator it = fstl.gtriangles().begin();
	int n = 1;
	while (it != fstl.gtriangles().end())
	{
		std::cout << n << ". ";
		std::cout << *it  << std::endl;
		++it;
		n++;
	}
    std::cout << "Start vertex:  " << fpath.gstart_vertex();
    std::cout << "Target vertex: " << fpath.gtarget_vertex();
    std::cout << "Path:" << std::endl;

    path_t::const_iterator iter = fpath.gpath().begin();

	n = 1;
	while (iter != fpath.gpath().end())
	{
		std::cout << n << ". ";
		std::cout << *iter  << std::endl;
		++iter;
		n++;
	}
/*
	for (auto t : fstl.get_triangles()) {
		std::cout << t << std::endl;
	}
*/
	return 0;
}
//---------------------------------------------------------------------------
 