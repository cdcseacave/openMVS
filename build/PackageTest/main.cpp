// Loads a scene through the installed OpenMVS libraries, see CMakeLists.txt
#include <MVS.h>
#include <SFM.h>
#include <iostream>

int main(int argc, char** argv)
{
	MVS::Scene scene;
	if (argc > 1 && !scene.Load(argv[1]))
		return EXIT_FAILURE;
	SFM::Scene sceneSfM;
	std::cout << scene.images.size() << " images, " << scene.pointcloud.points.size() << " points; "
		<< sceneSfM.images.size() << " SfM images" << std::endl;
	return EXIT_SUCCESS;
}
