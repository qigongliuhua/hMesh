#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
bool createDirectoryIfNotExists(const std::string& path) {
    try {
        if (fs::exists(path)) {
            if (fs::is_directory(path)) {
                return true;
            } else {
                return false;
            }
        }
        if (fs::create_directory(path)) {
            return true;
        } else {
            return false;
        }
    } catch (const fs::filesystem_error& e) {
        return false;
    }
}

extern void TestSurfaceMesh();
extern void TestVolumeMesh();

int main() {
    if (!createDirectoryIfNotExists(ROOT_PATH "/result")) {
        std::cerr << "无法创建目录：" << ROOT_PATH "/result" << std::endl;
        return EXIT_FAILURE;
    } 
    TestSurfaceMesh();
    TestVolumeMesh();
    return EXIT_SUCCESS;
}
