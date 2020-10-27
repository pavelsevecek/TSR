/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "App/TSR.h"
#include "Platform/ResourceManagement/MemoryManager.h"
#include "Platform/Utilities/HelperFunctions.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"
#include "Platform/Utilities/ParametersManager.h"

#include "KdTree.hpp"

#include <fstream>
#include <sstream>

using namespace Platform;

void loadPly(const std::string& filename,
             int stride,
             std::vector<Math::Vector3>& points,
             std::vector<Math::Vector3>& normals,
             std::vector<Math::Vector3>& sensors) {
    std::size_t current = points.size();
    std::ifstream f(filename);
    if (!f.good()) {
        throw std::runtime_error("Can't open " + filename);
    }

    f.exceptions(std::ios::badbit | std::ios::failbit);

    // read header
    std::string line;
    int nvert = -1;
    do {
        if (getline(f, line).eof()) {
            break;
        }
        sscanf(line.c_str(), "element vertex %d", &nvert);
    } while (line != "end_header");

    if (nvert < 0) {
        throw std::runtime_error(filename + ": unknown PLY format.");
    }

    // load points
    for (int i = 0; i < nvert; i++) {
        double x, y, z, nx, ny, nz, sx, sy, sz;
        std::string line;
        std::getline(f, line);
        std::stringstream ss(line);
        ss >> x >> y >> z >> nx >> ny >> nz >> sx >> sy >> sz;
        if (i % stride == 0) {
        points.emplace_back(x, y, z);
        normals.emplace_back(nx, ny, nz);
        sensors.emplace_back(sx, sy, sz);
        }
    }
    std::cout << "Loaded point cloud with " << points.size() - current << " points"
              << std::endl;
}

int main(int argc, const char** arcv) {
    std::vector<Math::Vector3> points, normals, sensors;
    loadPly("street.ply", 4,points, normals, sensors);
    loadPly("aerial.ply", 1,points, normals, sensors);

    Utilities::ParametersManager mgr("/home/pavel/projects/melown/TSR/SurfaceReconstruction/WorkingDirectory/Data/App.cfg");

    SurfaceReconstruction::Scene scene({});

    SurfaceReconstruction::Samples& samples = scene.getSamples();
    samples.updateMaxCamerasPerSample(1);


    SurfaceReconstruction::Cameras& cameras = scene.getCameras();

    Math::Vector3 aabb[2];
    aabb[0] = Math::Vector3(1.e6, 1.e6, 1.e6);
    aabb[1] = Math::Vector3(-1.e6, -1.e6, -1.e6);
    for (std::size_t i = 0; i < points.size(); ++i) {
        aabb[0] = aabb[0].minimum(points[i]);
        aabb[1] = aabb[1].maximum(points[i]);
    }
    Real distort[2] = { 0, 0 };
    for (std::uint32_t i = 0; i < points.size(); ++i) {
        Math::Vector3 dir = points[i] - sensors[i];
        Math::Vector3 zup(0, 0, 1);
        Math::Vector3 up = dir.crossProduct(zup).crossProduct(dir);
        up.normalize();
        float phi = atan2(dir.y, dir.x);
        Math::Quaternion q(up, phi);
        cameras.addCamera(i,
                          q,
                          sensors[i],
                          0.036,
                          Math::Vector2(0.5, 0.5),
                          1.f,
                          distort);
    }

    std::cout << "Building a tree" << std::endl;
    std::vector<float> scales(points.size());
    std::vector<Pvl::Vec3f> pvl;
    for (std::uint32_t i = 0; i < points.size(); ++i) {
        pvl.emplace_back(points[i].x, points[i].y, points[i].z);
    }
    Pvl::KdTree<Pvl::Vec3f> kdtree;
    kdtree.build(pvl);

    std::vector<int> neighs;
    for (std::uint32_t i = 0; i < points.size(); ++i) {
        float radius = 0.05;
        do {
            neighs.clear();
            kdtree.rangeQuery(pvl[i], radius, std::back_inserter(neighs));
            radius *= 1.2;
        } while (neighs.size() < 5);
        scales[i] = radius;
    }



    samples.addSamples(normals, points, scales);

    std::cout << "Valid link count = " << samples.getValidParentLinkCount()
              << std::endl;


    auto tree = new SurfaceReconstruction::Tree();
    tree->getNodes().checkSamplesOrder(tree->getRootScope());
    scene.mTree = tree;

    auto occupancy = new SurfaceReconstruction::Occupancy(tree);
    scene.mOccupancy = occupancy;
    /*const SurfaceReconstruction::FlexibleMesh &crustSurface = occupancy->extractCrust().getSurface();
    crustSurface.saveToFile("crust", true, false);*/
    const SurfaceReconstruction::DualMarchingCells *crust = occupancy->getCrust();
    crust = &occupancy->extractCrust();

    // check crust
    if (0 == crust->getSurface().getTriangleCount())
    {
        std::cout << "Empty crust." << std::endl;
        return -1;
    }
    occupancy->outputEdgeConflicts();

    // take over

    const SurfaceReconstruction::Scene::ReconstructionType type = SurfaceReconstruction::Scene::RECONSTRUCTION_VIA_OCCUPANCIES;
    SurfaceReconstruction::FlexibleMesh *mesh = new SurfaceReconstruction::FlexibleMesh(crust->getSurface());
    mesh->saveToFile("crust", true, false);

    scene.takeOverReconstruction(mesh, type);

    scene.createFSSFRefiner();
    scene.refine(SurfaceReconstruction::Scene::RECONSTRUCTION_VIA_SAMPLES);

    const SurfaceReconstruction::FlexibleMesh* refined
            = scene.getReconstruction(SurfaceReconstruction::Scene::RECONSTRUCTION_VIA_SAMPLES);
    refined->saveToFile("refined", true, false);

    return 0;
}
