/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "CollisionDetection/CollisionDetection.h"
#include "Math/MathCore.h"
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/Storage/File.h"
#include "Platform/Utilities/Array.h"
#include "Platform/Utilities/ParametersManager.h"
#include "Platform/Utilities/PlyFile.h"
#include "Platform/Utilities/RandomManager.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const char *Samples::MAX_RELATIVE_SAMPLING_DISTANCE = "Samples::maxRelativeSamplingDistance";

const uint32 Samples::FILE_VERSION = 0;
const uint32 Samples::INVALID_INDEX = (uint32) -1;

Samples::Samples(const uint32 camsPerSample, const uint32 sampleCount, const Vector3 *AABBWS) :
	mAABBWS{Vector3(REAL_MAX, REAL_MAX, REAL_MAX), Vector3(-REAL_MAX, -REAL_MAX, -REAL_MAX)},
	mMaxRelativeSamplingDistance(1.0f),
	mMaxCamsPerSample(camsPerSample), mValidParentLinkCount(0)
{
	if (sampleCount > 0)
		resize(sampleCount);

	if (AABBWS)
	{
		mAABBWS[0] = AABBWS[0];
		mAABBWS[1] = AABBWS[1];
	}

	// load parameters
	const ParametersManager &manager = ParametersManager::getSingleton();
	bool samplingDistanceLoaded = manager.get(mMaxRelativeSamplingDistance, MAX_RELATIVE_SAMPLING_DISTANCE);
	if (samplingDistanceLoaded)
		return;

	// error handling
	string message = "Scene: Could not load parameters:\n";
	if (!samplingDistanceLoaded)
	{
		message += "Samples::maxRelativeSamplingDistance";
		message += ", choosing 1.0\n";
	}

	cerr << message << endl;
}

Samples::Samples(const Path &fileName) :
	Samples((uint32) -1, 0, NULL)
{
	loadFromFile(fileName);
}

Samples::~Samples()
{
	clear();
	shrinkToFit();
}

void Samples::clear()
{
	// invalidate members
	mAABBWS[0] = Vector3(REAL_MAX, REAL_MAX, REAL_MAX);
	mAABBWS[1] = Vector3(-REAL_MAX, -REAL_MAX, -REAL_MAX);
	mValidParentLinkCount = 0;
	mMaxCamsPerSample = 0;
	
	// clear vectors
	mColors.clear();
	mNormals.clear();
	mPositions.clear();
	mConfidences.clear();
	mScales.clear();

	mParentCameras.clear();
}

void Samples::shrinkToFit()
{
	mColors.shrink_to_fit();
	mNormals.shrink_to_fit();
	mPositions.shrink_to_fit();
	mConfidences.shrink_to_fit();
	mScales.shrink_to_fit();

	mParentCameras.shrink_to_fit();
}

void Samples::addToAABB(Vector3 AABB[2], const uint32 sampleIdx) const
{
	// sample AABB
	Vector3 sampleMin;
	Vector3 sampleMax;
	getAABBWS(sampleMin, sampleMax, sampleIdx);

	AABB[0] = AABB[0].minimum(sampleMin);
	AABB[1] = AABB[1].maximum(sampleMax);
}

void Samples::addSamplesViaClouds(const vector<Path> &plyCloudFileNames, const vector<uint32> &viewToCameraIndices,
	const Matrix3x3 &inputOrientation, const Vector3 &inputOrigin)
{
	// load each point cloud
	const uint32 fileCount = (uint32) plyCloudFileNames.size();
	for (uint32 fileIdx = 0; fileIdx < fileCount; ++fileIdx)
		addSamplesViaCloud(plyCloudFileNames[fileIdx]);

	// transform samples
	Matrix3x3 inverseRotation(inputOrientation);
	inverseRotation.transpose();
	const Vector3 translation = -inputOrigin * inverseRotation;
	const uint32 sampleCount = getCount();

	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
		transform((uint32) sampleIdx, inverseRotation, translation);

	// update and count sample to parent camera links
	transformViewToParentCameraLinks(viewToCameraIndices);
	check();
	computeValidParentCameraCount();
	computeAABB();
}

void Samples::addSamplesViaCloud(const Path &plyCloudFileName)
{
	// open the file
	#ifdef _DEBUG
		cout << "\nStarting loading of ply sample cloud: " << plyCloudFileName << endl;
	#endif // _DEBUG

	PlyFile file(plyCloudFileName, File::OPEN_READING, true);

	// process ply header & body
	VerticesDescription verticesFormat;
	file.loadHeader(verticesFormat);

	// process ply body
	cout << "Loading samples from " << plyCloudFileName << "." << endl;
	const uint32 loadedSamplesCount = addSamplesViaCloud(file, plyCloudFileName, verticesFormat);

	#ifdef _DEBUG
		cout << "\nFinished loading ply sample cloud, loaded " << loadedSamplesCount << " samples from cloud file." << endl;
	#endif // _DEBUG
}

uint32 Samples::addSamplesViaCloud(PlyFile &file, const Path &fileName, const VerticesDescription &verticesFormat)
{
	// update parent links if necessary
	updateMaxCamerasPerSample(verticesFormat);

	// access vertex structure
	const ElementsSyntax &types = verticesFormat.getTypeStructure();
	const ElementsSemantics &semantics = verticesFormat.getSemantics();
	const uint32 propertyCount = verticesFormat.getPropertyCount();
	const uint32 fileSampleCount = verticesFormat.getElementCount();

	// reserve memory for the samples to be loaded
	const uint32 oldSampleCount = getCount();
	const uint32 maxSampleCount = ((uint32) -1) - 1;
	const uint32 maxNewTotalCount = min(fileSampleCount + oldSampleCount, maxSampleCount);
	reserve(maxNewTotalCount);

	// read each sample / vertex
	uint32 totalCount = oldSampleCount;
	for (uint32 fileSampleIdx = 0; fileSampleIdx < fileSampleCount && totalCount < maxNewTotalCount; ++fileSampleIdx)
	{
		if (!file.hasLeftData())
			throw FileCorruptionException("Could not read all vertices which were defined by the ply header.", fileName);

		// create sample
		addSample();

		// load data for current sample
		for (uint32 propertyIdx = 0; propertyIdx < propertyCount; ++propertyIdx)
			readSampleProperty(file, totalCount, types[propertyIdx], (VerticesDescription::SEMANTICS) semantics[propertyIdx]);
		
		// ignore zero confidence samples or samples with invalid scale
		if (Math::EPSILON >= mConfidences.back() || Math::EPSILON >= mScales.back())
			popBackSample();
		else
			++totalCount;
	}

	// return loaded sample count
	return (totalCount - oldSampleCount);
}

void Samples::addSamples(const std::vector<Math::Vector3>& normals,
                         const std::vector<Math::Vector3>& positions,
                         const std::vector<float>& scales)
{
    updateMaxCamerasPerSample(1);

    for (uint32 i = 0; i < positions.size(); ++i)
    {
        addSample(Math::Vector3(0,0,0), normals[i], positions[i], 1.e6f, scales[i], &i, 1);
    }

    check();
    computeValidParentCameraCount();
    computeAABB();
}

void Samples::readSampleProperty(PlyFile &file, const uint32 sampleIdx,
	const ElementsDescription::TYPES type, const VerticesDescription::SEMANTICS semantic)
{
	// get sample data destinations
	Vector3 *color = mColors.data() + sampleIdx;
	Vector3 *normal = mNormals.data() + sampleIdx;
	Vector3 &position = mPositions[sampleIdx];
	Real *confidence = mConfidences.data() + sampleIdx;
	Real *scale = mScales.data() + sampleIdx;
	uint32 *camIDs = mParentCameras.data() + mMaxCamsPerSample * sampleIdx;

	file.readVertexProperty(color, normal, position, NULL, confidence, scale, camIDs, type, semantic);
}

void Samples::addSamplesViaMeshes(const vector<FlexibleMesh *> &meshes,
	const vector<vector<uint32> *> &cameraIndices, const vector<uint32> &camerasPerSamples)
{
	// only reference cameras (one camera for each sample) or larger camera set via cameraIndices?
	assert(meshes.size() == cameraIndices.size() && meshes.size() == camerasPerSamples.size());
	const uint32 meshCount = (uint32) meshes.size();
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
		updateMaxCamerasPerSample(camerasPerSamples[meshIdx]);

	// compute number of new samples
	uint64 additionalSampleCount = 0;
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
		if (meshes[meshIdx])
			additionalSampleCount += meshes[meshIdx]->getVertexCount();

	// check sample & link count & reserve memory
	const uint64 newSampleCount = additionalSampleCount + getCount();
	checkSampleCount(newSampleCount);
	checkLinkCount(newSampleCount, mMaxCamsPerSample);
	reserve(newSampleCount);

	// create and add samples for each mesh
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
	{
		const vector<uint32> *const meshCameraIndices = cameraIndices[meshIdx];
		if (!meshCameraIndices || !meshes[meshIdx])
		{
			cerr << "Could not add samples for a mesh, index: " << meshIdx << "\n." << flush;
			continue;
		}

		addSamplesViaMesh(*meshes[meshIdx], *meshCameraIndices, camerasPerSamples[meshIdx]);
	}
	
	check();
	computeValidParentCameraCount();
	computeAABB();
}

void Samples::addSamplesViaMesh(const FlexibleMesh &mesh, const std::vector<uint32> &cameraIndices, const uint32 &camerasPerSample)
{
	const uint32 *sampleParents = cameraIndices.data();
	const Real confidence = 1.0f; // todo: how to get reasonable confidence values?
	
	// add a sample for each mesh vertex
	const uint32 vertexCount = mesh.getVertexCount();
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx, sampleParents += camerasPerSample)
		addSample(mesh.getColor(vertexIdx), mesh.getNormal(vertexIdx), mesh.getPosition(vertexIdx), 
				  confidence, mesh.getScale(vertexIdx), sampleParents, camerasPerSample);
}

void Samples::check() const
{
	const int64 sampleCount = getCount();

	// check sample scale / samples' 3D footprint sizes
	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		const Real &scale = getScale((uint32) sampleIdx);
		assert(scale > 0.0f);
		if (scale <= 0.0f)
			throw Exception("Invalid (non-positive) sample scale detected.");
	}
}

void Samples::computeAABB()
{
	mAABBWS[0].set(REAL_MAX, REAL_MAX, REAL_MAX);
	mAABBWS[1].set(-REAL_MAX, -REAL_MAX, -REAL_MAX);

	const uint32 sampleCount = (uint32) mNormals.size();
	for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
		addToAABB(mAABBWS, sampleIdx);
}

void Samples::compact(const uint32 *sampleOffsets)
{
	// counts for filling
	const uint32 oldSampleCount = getCount();
	const uint32 doomedSampleCount = sampleOffsets[oldSampleCount];
	const uint32 newSampleCount = oldSampleCount - doomedSampleCount;
	
	cout << "Deleting " << doomedSampleCount << " of " << oldSampleCount << " samples, remaining: " << newSampleCount << " samples." << endl;
	
	Array<Vector3>::compaction(mColors, sampleOffsets);
	Array<Vector3>::compaction(mNormals, sampleOffsets);
	Array<Vector3>::compaction(mPositions, sampleOffsets);
	Array<Real>::compaction(mConfidences, sampleOffsets);
	Array<Real>::compaction(mScales, sampleOffsets);
	Array<uint32>::compaction(mParentCameras, sampleOffsets, mMaxCamsPerSample);
	
	computeValidParentCameraCount();
	computeAABB();
	cout << "Finished deletion of samples. " << endl;
}

bool Samples::computeMeans(Vector3 &meanColor, Vector3 &meanNormal, Vector3 &meanPosition, Real &meanScale,
	const vector<uint32> &sampleSet, bool weightedByConfidences) const
{
	// weighted mean data of sample set
	const Real MIN_LENGTH_NORMALS	= 0.5f; // todo magic number

	// initial mean values
	meanColor.set(0.0f, 0.0f, 0.0f);
	meanPosition.set(0.0f, 0.0f, 0.0f);
	meanNormal.set(0.0f, 0.0f, 0.0f);
	meanScale = 0.0f;

	const uint32 count = (uint32) sampleSet.size();
	Real sum = 0.0f;	

	for (uint32 i = 0; i < count; ++i)
	{
		const uint32	index		= sampleSet[i];
		const Vector3	&color		= mColors[index];
		const Vector3	&normal		= mNormals[index];
		const Vector3	&position	= mPositions[index];
		const Real		confidence	= mConfidences[index];
		const Real		scale		= mScales[index];
		const Real		weight		= (weightedByConfidences ? mConfidences[index] : 1.0f);

		meanColor		+= color * weight;
		meanNormal		+= normal * weight;
		meanPosition	+= position * weight;
		meanScale		+= scale * weight;

		sum += weight;
	}

	// normalized results
	const Real normFactor = 1.0f / sum;
	
	meanColor	 *= normFactor;
	meanNormal	 *= normFactor;
	meanPosition *= normFactor;
	meanScale	 *= normFactor;

	const Real lengthSq = meanNormal.getLengthSquared();
	meanNormal /= sqrtr(lengthSq);

	// small distribution of sample normals?
	// it does not make sense to compute a model if sample oriantations are too chaotic
	return (lengthSq >= MIN_LENGTH_NORMALS * MIN_LENGTH_NORMALS); // todo magic number
}

bool Samples::computeViewAngles(Real &azimuthAngle, Real &polarAngle, const uint32 parentCameraIdx, const uint32 sampleIdx) const
{
	// bad default values if something goes wrong
	azimuthAngle = -REAL_MAX;
	polarAngle	 = -REAL_MAX;

	Vector3 viewDirection;
	if (!computeViewDirection(viewDirection, parentCameraIdx, sampleIdx))
		return false;

	Math::transformCartesianToSpherical(azimuthAngle, polarAngle, viewDirection);
	return true;
}

bool Samples::computeViewDirection(Vector3 &viewDirection, const uint32 parentCameraIdx, const uint32 sampleIdx) const
{
	// bad default values if something goes wrong
	viewDirection.set(-REAL_MAX, -REAL_MAX, REAL_MAX);

	// check parent view index
	assert(parentCameraIdx < mMaxCamsPerSample);
	if (parentCameraIdx >= mMaxCamsPerSample)
		throw Exception("Invalid parentCameraIdx for a sample.");

	// valid parent view?
	const Scene &scene = Scene::getSingleton();
	const Cameras &cameras = scene.getCameras();
	const uint32 cameraIdx = getCameraIdx(parentCameraIdx, sampleIdx);
	if (!cameras.isValid(cameraIdx))
		return false;

	// compute view direction
	const Vector3 &camWS = cameras.getPositionWS(cameraIdx);
	viewDirection = mPositions[sampleIdx] - Vector3(camWS.x, camWS.y, camWS.z);
	viewDirection.normalize();
	return true;
}

void Samples::erase(const vector<uint32> theDoomed, const uint32 doomedCount)
{
	// todo improve this - in place compaction does not make sense as resize is called later anyway
	// replace this with compact?
	const uint32 parentCameraBytes = sizeof(uint32) * mMaxCamsPerSample;
	const uint32 temp = (uint32) theDoomed.size();
	uint32 numRemovedSamples = 0;

	for (uint32 i = 0; i < temp - 1; i += 2)
	{
		const uint32 targetIdx = theDoomed[i] - numRemovedSamples;
		const uint32 doomedCount = theDoomed[i + 1];
		const uint32 sourceIdx = theDoomed[i] + doomedCount;
		const uint32 next = theDoomed[i + 2];
		const uint32 moveCount = next - sourceIdx;
		if (0 == moveCount && next == getCount())
			continue;

		memcpy(&mColors[targetIdx], &mColors[sourceIdx], sizeof(Vector3) * moveCount);
		memcpy(&mNormals[targetIdx], &mNormals[sourceIdx], sizeof(Vector3) * moveCount);
		memcpy(&mPositions[targetIdx], &mPositions[sourceIdx], sizeof(Vector3) * moveCount);
		memcpy(&mConfidences[targetIdx], &mConfidences[sourceIdx], sizeof(Real) * moveCount);
		memcpy(&mScales[targetIdx], &mScales[sourceIdx], sizeof(Real) * moveCount);

		memcpy(&mParentCameras[targetIdx * mMaxCamsPerSample], &mParentCameras[sourceIdx * mMaxCamsPerSample], parentCameraBytes * moveCount);

		numRemovedSamples += doomedCount;
	}

	resize(getCount() - doomedCount);
	computeValidParentCameraCount();
	computeAABB();
}

void Samples::getAABBWS(Vector3 &minWS, Vector3 &maxWS, const uint32 sampleIdx) const
{
	const Vector3 &p = mPositions[sampleIdx];
	const Real r = getMaxSamplingDistance(sampleIdx) + EPSILON;

	minWS.set(p.x - r, p.y - r, p.z - r);
	maxWS.set(p.x + r, p.y + r, p.z + r);
}

Real Samples::getDistanceToPlane(const Vector3 &pWS, const uint32 sampleIdx) const
{
	CollisionDetection::Plane plane(mPositions[sampleIdx], mNormals[sampleIdx], true);
	return plane.getDistanceToPlane(pWS);
}

Real Samples::getFSSRWeight(const Math::Vector3 &pWS, const uint32 sampleIdx) const
{
	const Vector3 vSS = toSampleSpace(pWS, sampleIdx);
	const Real s = mScales[sampleIdx];

	// FSSR weighting function: weightX along x axis * weightY along y axis
	const Real range	= 3.0f * s;
	const Real scaleSq	= s * s;
	const Real scaleCu	= scaleSq * s;
	const Real xSq		= vSS.x * vSS.x;
		  Real weight	= 0.0f;

	// weight along normal
	if (vSS.x > -range && vSS.x < 0.0f)
		weight = xSq / (9.0f * scaleSq) + 2.0f * vSS.x / range + 1.0f;
	else if (vSS.x >= 0.0f && vSS.x < range)
		weight = 2.0f * xSq * vSS.x / (27.0f * scaleCu) - xSq / (3.0f * scaleSq) + 1.0f;
	else
		return 0.0f;

	// weight along tangent
	const Real tSq	= vSS.y * vSS.y + vSS.z * vSS.z;
	const Real t	= sqrtr(tSq);
	if (t < range)
		weight *= 2.0f * tSq * t / (27.0f * scaleCu) - tSq / (3.0f * scaleSq) + 1.0f;
	else
		return 0.0f;

	return weight;
}

Real Samples::getMeasureDistanceSquared(const uint32 sampleIdx, const uint32 parentCameraIdx) const
{
	// get parent camera position
	const uint32 globalCameraIdx = getCameraIdx(parentCameraIdx, sampleIdx);
	const Cameras &cameras = Scene::getSingleton().getCameras();
	const Vector3 &camPosWS = cameras.getPositionWS(globalCameraIdx);
	
	// measurement distance / sample depth
	const Vector3 camToSample = getPositionWS(sampleIdx) - camPosWS;
	const Real distanceSq = camToSample.getLengthSquared();

	return distanceSq;
}

uint32 Samples::getCameraIdx(const uint32 parentCameraIdx, const uint32 sampleIdx) const
{
	// sanity checks
	const uint32 sampleCount = getCount();
	assert(parentCameraIdx < mMaxCamsPerSample);
	assert(sampleIdx < sampleCount);

	if (parentCameraIdx >= mMaxCamsPerSample || sampleIdx >= sampleCount)
		return Cameras::INVALID_ID;
	else
		return mParentCameras[sampleIdx * mMaxCamsPerSample + parentCameraIdx];
}

void Samples::setSample(const uint32 targetIdx, const Vector3 &color, const Vector3 &normal, const Vector3 &positionWS,
	const Real &confidence, const Real &scale, const uint32 *parentCameraIDs, const uint32 &parentCameraCount)
{
	// update simple sample attributes
	mColors[targetIdx] = color;
	mNormals[targetIdx] = normal;
	mPositions[targetIdx] = positionWS;
	mConfidences[targetIdx] = confidence;
	mScales[targetIdx] = scale;

	// update parent cameras and valid parent camera count
	uint32 *parentCameras = mParentCameras.data() + targetIdx * mMaxCamsPerSample;

	// copy cameras
	for (uint32 i = 0; i < mMaxCamsPerSample && i < parentCameraCount; ++i)
	{
		// is a parent camera added or erased?
		if (Cameras::INVALID_ID == parentCameras[i] && Cameras::INVALID_ID != parentCameraIDs[i])
			++mValidParentLinkCount;
		else if (Cameras::INVALID_ID != parentCameras[i] && Cameras::INVALID_ID == parentCameraIDs[i])
			--mValidParentLinkCount;

		parentCameras[i] = parentCameraIDs[i];
	}

	// fill remaining links with invalid indices
	for (uint32 i = parentCameraCount; i < mMaxCamsPerSample; ++i)
		parentCameras[i] = INVALID_INDEX;
}

void Samples::makeNoisy(normal_distribution<Real> noise[3], const uint32 sampleIdx)
{
	// get necessary variables
	RandomManager	&random		= RandomManager::getSingleton();
	Vector3			&normal		= mNormals[sampleIdx];
	Vector3			&position	= mPositions[sampleIdx];
	Real			&scale		= mScales[sampleIdx];
	
	// get view direction
	Vector3 viewDirection;
	if (!computeViewDirection(viewDirection, 0, sampleIdx))
	{
		// todo log this
		viewDirection.set(random.getNormal(noise[2]), random.getNormal(noise[2]), random.getNormal(noise[2]));
		viewDirection.normalize();
	}

	// add noise to position
	const Real positionNoise = scale * random.getNormal(noise[0]);
	position += viewDirection * positionNoise;

	// add noise to scale
	const Real f = 1.0f + random.getNormal(noise[1]);
	scale = scale * Math::clamp<Real>(f, 1.75f, 0.25f);

	// add noise to normal
	Real azimuth;
	Real polar;

	Math::transformCartesianToSpherical(azimuth, polar, normal, 1.0f);
	azimuth += random.getNormal(noise[2]);
	polar	+= random.getNormal(noise[2]);
	Math::transformSphericalToCartesian(normal, azimuth, polar, 1.0f);
}

uint32 Samples::addSample()
{
	// compute & check new sample count
	const uint32 sampleIdx = (uint32) mNormals.size();
	const uint64 count = sampleIdx + 1;
	checkSampleCount(count);
	checkLinkCount(count, mMaxCamsPerSample);

	// resize containers
	mColors.resize(count);
	mNormals.resize(count);
	mPositions.resize(count);
	mConfidences.resize(count, 1.0f);
	mScales.resize(count);
	mParentCameras.resize(count * mMaxCamsPerSample, INVALID_INDEX);

	return sampleIdx;
}

void Samples::invalidateParentCameras(const uint32 sampleIdx)
{
	const uint32 offset = sampleIdx * mMaxCamsPerSample;
	for (uint32 i = 0; i < mMaxCamsPerSample; ++i)
		mParentCameras[offset + i] = Cameras::INVALID_ID;
}

void Samples::deleteSample(const uint32 sampleIdx)
{
	// swap last sample with replacedIdx and erase the last (replacedIdx)
	const uint32 lastSampleIdx = (uint32) mNormals.size() - 1;
	swap(sampleIdx, lastSampleIdx);
	popBackSample();
}

void Samples::popBackSample()
{
	// erase last sample from all containers
	mColors.pop_back();
	mNormals.pop_back();
	mPositions.pop_back();
	mConfidences.pop_back();
	mScales.pop_back();

	// erase parent cameras & reduce the number of parent cameras accordingly
	const uint32 vectorSize = (uint32) mParentCameras.size();
	uint32 erasedParentViewsCount = 0;
	for (uint32 parentCameraIdx = vectorSize - mMaxCamsPerSample; parentCameraIdx < vectorSize; ++parentCameraIdx)
		if (Cameras::INVALID_ID != mParentCameras[parentCameraIdx])
			++erasedParentViewsCount;

	mParentCameras.resize(mParentCameras.size() - mMaxCamsPerSample);
	mValidParentLinkCount -= erasedParentViewsCount;
}

void Samples::reserve(const uint64 sampleCount)
{
	// check new counts
	checkSampleCount(sampleCount);
	checkLinkCount(sampleCount, mMaxCamsPerSample);

	// reserve memory
	mColors.reserve(sampleCount);
	mNormals.reserve(sampleCount);
	mPositions.reserve(sampleCount);
	mConfidences.reserve(sampleCount);
	mScales.reserve(sampleCount);
	mParentCameras.reserve(sampleCount * mMaxCamsPerSample);
}

void Samples::swap(const uint32 i, const uint32 j)
{
	// same sample?
	if (i == j)
		return;

	// swap everything belonging to the two samples i and j

	// swap color, normal, position & scale
	Utilities::swap(mColors[i], mColors[j]);
	Utilities::swap(mNormals[i], mNormals[j]);
	Utilities::swap(mPositions[i], mPositions[j]);
	Utilities::swap(mConfidences[i], mConfidences[j]);
	Utilities::swap(mScales[i], mScales[j]);

	// update camera indices
	const uint32 offsetI = i * mMaxCamsPerSample;
	const uint32 offsetJ = j * mMaxCamsPerSample;

	uint32 *cameraI = mParentCameras.data() + (i * mMaxCamsPerSample);
	uint32 *cameraJ = mParentCameras.data() + (j * mMaxCamsPerSample);
	for (uint32 cameraIdx = 0; cameraIdx < mMaxCamsPerSample; ++cameraIdx, ++cameraI, ++cameraJ)
			Utilities::swap(*cameraI, *cameraJ);
}

Vector3 Samples::toSampleSpace(const Vector3 &pWS, const uint32 sampleIdx) const
{
	// transform to sample space "SS" which depends on sample position and orientation
	Matrix3x3 offsetWSToSS = Matrix3x3::createBasisFromVector(mNormals[sampleIdx]);
	offsetWSToSS.transpose();

	// convert pWS into coordinate system(mPosition, mNormal, tangent0, tangent1)
	const Vector3 offsetWS = pWS - mPositions[sampleIdx];
	const Vector3 pSS = offsetWS * offsetWSToSS;

	return pSS;
}

void Samples::transform(const uint32 sampleIdx, const Matrix3x3 &transformation, const Vector3 &translation)
{
	mNormals[sampleIdx] = mNormals[sampleIdx] * transformation;
	mPositions[sampleIdx] = (mPositions[sampleIdx] * transformation + translation);
}

void Samples::loadFromFile(const Path &fileName)
{
	clear();
	cout << "Loading samples from file " << fileName << "." << endl;
	
	// open file & check version
	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// get #samples & #cameras per sample & request memory
	uint32 sampleCount;
	file.read(&sampleCount, sizeof(uint32), sizeof(uint32), 1);
	file.read(&mMaxCamsPerSample, sizeof(uint32), sizeof(uint32), 1);
	file.read(mAABBWS, sizeof(Vector3) * 2, sizeof(Vector3), 2);

	if (sampleCount != mNormals.capacity())
	{
		shrinkToFit();
		resize(sampleCount);
	}

	// load samples
	uint32 parentCount = sampleCount * mMaxCamsPerSample;

	file.read(mColors.data(), sizeof(Vector3) * sampleCount, sizeof(Vector3), sampleCount);
	file.read(mNormals.data(), sizeof(Vector3) * sampleCount, sizeof(Vector3), sampleCount);
	file.read(mPositions.data(), sizeof(Vector3) * sampleCount, sizeof(Vector3), sampleCount);
	file.read(mConfidences.data(), sizeof(Real) * sampleCount, sizeof(Real), sampleCount);
	file.read(mScales.data(), sizeof(Real) * sampleCount, sizeof(Real), sampleCount);
	file.read(mParentCameras.data(), sizeof(uint32) * parentCount, sizeof(uint32), parentCount);
	
	computeValidParentCameraCount();

	cout << "Loaded " << getCount() << " samples." << endl;
}

void Samples::saveToFile(const Path &beginning, const bool saveAsPly, const bool saveAsSamples) const
{
	// is there anything to save?
	if (mNormals.empty())
		return;

	// as Stanford ply mesh?
	if (saveAsPly)
	{
		const Path fileName = Path::extendLeafName(beginning, FileNaming::ENDING_PLY);
		PlyFile file(fileName, File::CREATE_WRITING, true);
		file.saveTriangleMesh(ENCODING_BINARY_LITTLE_ENDIAN, true,
			getCount(), 0, mColors.data(), mNormals.data(), mPositions.data(), 
			mConfidences.data(), mScales.data(), mParentCameras.data(), mMaxCamsPerSample, NULL);
	}
	
	// internal mesh format?
	if (saveAsSamples)
	{
		// create file & write version
		const Path fileName = Path::extendLeafName(beginning, FileNaming::ENDING_SAMPLES);
		File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);

		// save sample count, cameras per sample & AABB
		const uint32 sampleCount = (uint32) mNormals.size();
		file.write(&sampleCount, sizeof(uint32), 1);
		file.write(&mMaxCamsPerSample, sizeof(uint32), 1);
		file.write(mAABBWS, sizeof(Vector3), 2);

		// save all samples
		file.write(mColors.data(), sizeof(Vector3), sampleCount);
		file.write(mNormals.data(), sizeof(Vector3), sampleCount);
		file.write(mPositions.data(), sizeof(Vector3), sampleCount);
		file.write(mConfidences.data(), sizeof(Real), sampleCount);
		file.write(mScales.data(), sizeof(Real), sampleCount);
		file.write(mParentCameras.data(), sizeof(uint32), sampleCount * mMaxCamsPerSample);
	}
}

void Samples::resize(const uint32 sampleCount)
{
	// still valid indices?
	checkSampleCount(sampleCount);
	checkLinkCount(sampleCount, mMaxCamsPerSample);

	// resize memory
	const uint32 oldCount = (uint32) mColors.size();
	mColors.resize(sampleCount);
	mNormals.resize(sampleCount);
	mPositions.resize(sampleCount);
	mConfidences.resize(sampleCount, 1.0f);
	mScales.resize(sampleCount, 0.0f);

	mParentCameras.resize(sampleCount * mMaxCamsPerSample);

	// invalid cameras for all new samples
	if (sampleCount <= oldCount)
		return;

	// set invalid camera indices
	const uint32 newSamples = sampleCount - oldCount;
	const size_t byteCount = newSamples * mMaxCamsPerSample * sizeof(uint32);
	memset(mParentCameras.data() + oldCount * mMaxCamsPerSample, Cameras::INVALID_ID, byteCount);
}

void Samples::transformViewToParentCameraLinks(const vector<uint32> &viewToCameraIndices)
{
	// update each link
	const int64 linkCount = getCount() * mMaxCamsPerSample;
	#pragma omp parallel for
	for (int64 linkIdx = 0; linkIdx < linkCount; ++linkIdx)
	{
		const uint32 viewID = mParentCameras[linkIdx];
		if (Cameras::INVALID_ID != viewID)
			mParentCameras[linkIdx] = viewToCameraIndices[viewID];
	}
}

void Samples::computeValidParentCameraCount()
{
	// get & check parent link count
	uint64 linkCount = mMaxCamsPerSample * getCount();
	if (linkCount >= (uint32) -1)
		throw Exception("Number of sample to parent camera links exceeds supported maximum = 2^32 - 2.");
	const uint32 parentCount = (uint32) linkCount;

	// count number of valid parent links
	uint32 invalidCount = 0;
	for (uint32 parentIdx = 0; parentIdx < parentCount; ++parentIdx)
		if (Cameras::INVALID_ID == mParentCameras[parentIdx])
			++invalidCount;
	mValidParentLinkCount = parentCount - invalidCount;

	cout << "Computed number of valid parent cameras for all samples.\n";
	cout << "Parent camera count: " << mValidParentLinkCount << "; invalid view link count: " << invalidCount << "; parent view link count: " << getCount() * mMaxCamsPerSample << endl;
}

void Samples::reorder(const uint32 *targetIndices)
{
	const uint32 sampleCount = getCount();

	Array<Vector3>::reorder(mColors, targetIndices);
	Array<Vector3>::reorder(mNormals, targetIndices);
	Array<Vector3>::reorder(mPositions, targetIndices);
	Array<Real>::reorder(mConfidences, targetIndices);
	Array<Real>::reorder(mScales, targetIndices);
	Array<uint32>::reorder(mParentCameras, targetIndices, mMaxCamsPerSample);
}

void Samples::updateMaxCamerasPerSample(const VerticesDescription &verticesFormat)
{
	const ElementsSemantics &semantics = verticesFormat.getSemantics();
	const uint32 propertyCount = verticesFormat.getPropertyCount();

	uint32 camerasPerSample = 0;
	for (uint32 propertyIdx = 0; propertyIdx < propertyCount; ++propertyIdx)
		if (semantics[propertyIdx] >= VerticesDescription::SEMANTIC_VIEWID0)
			++camerasPerSample;

	updateMaxCamerasPerSample(camerasPerSample);
}

bool Samples::updateMaxCamerasPerSample(const uint32 camerasPerSample)
{
	// no increase?
	if (camerasPerSample <= mMaxCamsPerSample)
		return false;

	// more links
	const uint32 oldMaxCamsPerSample = mMaxCamsPerSample;
	checkLinkCount(getCount(), camerasPerSample);
	mMaxCamsPerSample = camerasPerSample;

	// no update of links buffer necesary?
	if (empty())
		return true;

	// new links buffer
	const uint32 sampleCount = getCount();
	const int64 newParentsCount = sampleCount * mMaxCamsPerSample;
	vector<uint32> newParentCameras(newParentsCount);
	
	// copy old parent links and fill new gaps with invalid IDs
	const uint32 oldBytesPerParentsBlock = sizeof(uint32) * oldMaxCamsPerSample;
	const uint32 gapBytesForNewLinks = sizeof(uint32) * (mMaxCamsPerSample - oldMaxCamsPerSample);

	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		uint32 *targetStart = newParentCameras.data() + sampleIdx * mMaxCamsPerSample;
		const uint32 *sourceStart = mParentCameras.data() + sampleIdx * oldMaxCamsPerSample;

		// copy old links & fill gaps for new links with INVALID_INDEX
		memcpy(targetStart, sourceStart, oldBytesPerParentsBlock);
		memset(targetStart + oldMaxCamsPerSample, INVALID_INDEX, gapBytesForNewLinks);
	}

	mParentCameras.swap(newParentCameras);
	return true;
}

void Samples::checkLinkCount(const uint64 &newSampleCount, const uint32 &newMaxCamerasPerSample) const
{
	const uint64 newLinkCount = (uint64) newSampleCount * (uint64) newMaxCamerasPerSample;
	if (newLinkCount < INVALID_INDEX)
		return;

	assert(false);
	throw Exception("Link count (sample to camera links) is larger than 2^32 - 1 which is not supported.");
}

void Samples::checkSampleCount(const uint64 &newSampleCount) const
{
	if (newSampleCount < INVALID_INDEX)
		return;

	assert(false);
	throw Exception("Sample count is larger than 2^32 - 1 which is not supported.");
}
