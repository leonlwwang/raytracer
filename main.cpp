#include "raytracer.h"
#include "lodepng.h"

#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char* argv[]) {
    std::ifstream file = openFile(argc, argv);
    std::pair<std::pair<std::vector<unsigned char>, string>, std::pair<int, int>> scene = generateImage(file);
    std::string save_directory = "out/" + scene.first.second;
    unsigned int error = lodepng_encode32_file(save_directory.c_str(), scene.first.first.data(), scene.second.first, scene.second.second);
    if (error) {
        std::cout << "Error " << error << ": " << lodepng_error_text(error) << std::endl;
    }
    file.close();
    return 0;
}
