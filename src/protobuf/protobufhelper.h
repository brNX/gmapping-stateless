/*
 * ProtobufHelper.h
 *
 *  Created on: Mar 2, 2013
 *      Author: bgouveia
 */

#ifndef PROTOBUFHELPER_H_
#define PROTOBUFHELPER_H_

#include "particle_reading.pb.h"
#include <boost/shared_ptr.hpp>
#include "../gmapping/utils/point.h"

namespace GMapping{
struct PointAccumulator;
template <class Cell> class HierarchicalArray2D;
template <class Cell> class Array2D;
template <class Cell,class Storage, const bool isClass> class Map;
struct Particle;
class RangeReading;
class ScanMatcher;
typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator>,true > ScanMatcherMap;
}

namespace ProtoBuf{

class ProtobufHelper {

public:
	ProtobufHelper();
	virtual ~ProtobufHelper();

    void serializePointAccumulator(const GMapping::PointAccumulator* pointAccumulator,gmapping_structs::PointAccumulator * pbufPointAccumulator);
    void serializeHArray2D(const GMapping::HierarchicalArray2D<GMapping::PointAccumulator>* hArray2D,gmapping_structs::HierarchicalArray2D * pbufHArray2D);
    void serializeArray2D(const GMapping::Array2D<GMapping::PointAccumulator> & array2D, gmapping_structs::Array2D * pbufArray2d);
    void serializeScanMatcherMap(const GMapping::ScanMatcherMap * smap , gmapping_structs::ScanMatcherMap * pbufSmap);
    void serializeParticle(const GMapping::Particle & particle, gmapping_structs::Particle * pbufParticle,unsigned int index);
    void serializeParticleUsedAreas(const GMapping::Particle & particle, gmapping_structs::Particle * pbufParticle,unsigned int index,const std::pair<int,int>& minr ,const std::pair<int,int>& maxr, const std::pair<int,int>& minf ,const std::pair<int,int>& maxf);
    void serializeScanMatcherMapUsedAreas(const GMapping::ScanMatcherMap * smap , gmapping_structs::ScanMatcherMap * pbufSmap,const std::pair<int,int>& minr ,const std::pair<int,int>& maxr, const std::pair<int,int>& minf ,const std::pair<int,int>& maxf);
    void serializeHArray2DUsedAreas(const GMapping::HierarchicalArray2D<GMapping::PointAccumulator>* hArray2D,gmapping_structs::HierarchicalArray2D * pbufHArray2D,const std::pair<int,int>& minr ,const std::pair<int,int>& maxr, const std::pair<int,int>& minf ,const std::pair<int,int>& maxf);

    //void addParticleToWorkPackage(const GMapping::Particle & particle,unsigned int id,gmapping_structs::Workpackage & workpackage);
    //void clearParticlesFromWorkPackage();
    void initWorkPackage(const double minimumScore , const GMapping::ScanMatcher & scanMatcher);
    void setWorkPackageLaserReading(double * reading , unsigned int size);

    void deserializePointAccumulator(gmapping_structs::PointAccumulator * pbufPointAccumulator ,GMapping::PointAccumulator* pointAccumulator);
    void deserializeHArray2D(const gmapping_structs::HierarchicalArray2D & pbufHArray2D,GMapping::HierarchicalArray2D<GMapping::PointAccumulator> * harray2d);
    GMapping::Array2D<GMapping::PointAccumulator> * deserializeArray2D(const gmapping_structs::Array2D & pbufArray2d);
    void deserializeArray2DZeroCopy(boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> > & array2D, const gmapping_structs::Array2D & pbufArray2d);

    void deserializeScanMatcherMap(const gmapping_structs::ScanMatcherMap & pbufSmap , GMapping::ScanMatcherMap * smap);
    int deserializeParticle(const gmapping_structs::Particle & pbufParticle , GMapping::Particle & particle);
    void deserializeScanMatcherFromWorkPackage(const gmapping_structs::Workpackage & workpackage, GMapping::ScanMatcher &);
    void deserializeReply(const gmapping_structs::WorkResponse & pbufReply , GMapping::Particle & particle);

    gmapping_structs::Workpackage m_workpackage;

};

}


#endif /* PROTOBUFHELPER_H_ */
