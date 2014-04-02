/*
 * ProtobufHelper.cpp
 *
 *  Created on: Mar 2, 2013
 *      Author: bgouveia
 */

#include "protobufhelper.h"
#include <scanmatcher/smmap.h>
#include <grid/harray2d.h>
#include <grid/array2d.h>
#include <gridfastslam/gridslamprocessor.h>
#include <sensor/sensor_range/rangereading.h>
#include <utils/point.h>

#include <boost/shared_ptr.hpp>


#include <vector>

using namespace std;

namespace ProtoBuf{

ProtobufHelper::ProtobufHelper() {
    // TODO Auto-generated constructor stub

}

ProtobufHelper::~ProtobufHelper() {
    // TODO Auto-generated destructor stub
}

inline void ProtobufHelper::serializePointAccumulator(const GMapping::PointAccumulator* pointAccumulator,gmapping_structs::PointAccumulator * pbufPointAccumulator){
    pbufPointAccumulator->set_x(pointAccumulator->acc.x);
    pbufPointAccumulator->set_y(pointAccumulator->acc.y);
    pbufPointAccumulator->set_n(pointAccumulator->n);
    pbufPointAccumulator->set_visits(pointAccumulator->visits);
}

void ProtobufHelper::serializeHArray2D(const GMapping::HierarchicalArray2D<GMapping::PointAccumulator>* hArray2D,gmapping_structs::HierarchicalArray2D * pbufHArray2D){
    pbufHArray2D->set_m_patchmagnitude(hArray2D->m_patchMagnitude);
    pbufHArray2D->set_m_patchsize(hArray2D->m_patchSize);
    pbufHArray2D->set_m_xsize(hArray2D->m_xsize);
    pbufHArray2D->set_m_ysize(hArray2D->m_ysize);

    //activearea
    for (std::set<GMapping::point<int> >::iterator it = hArray2D->m_activeArea.begin(); it != hArray2D->m_activeArea.end(); ++it)
    {
        gmapping_structs::IntPoint * point = pbufHArray2D->add_m_activearea();
        point->set_x(it->x);
        point->set_y(it->y);
    }

    boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> > ** m_cells = hArray2D->m_cells;

    //array2d autoptr
    for (int x=0 ; x<hArray2D->m_xsize; x++){
        gmapping_structs::HierarchicalArray2D_innerType * m_cells_x = pbufHArray2D->add_m_cells_x();

        for (int y =0 ;y<hArray2D->m_ysize;y++){

            gmapping_structs::Autoptr * m_cells_y = m_cells_x->add_m_cells_y();

            if (m_cells[x][y]) {//if initialized
                //gmapping_structs::Array2D data;
                serializeArray2D(*m_cells[x][y], m_cells_y->mutable_data());
                //m_cells_y->mutable_data()->CopyFrom(data);
            }
        }
    }
}

void ProtobufHelper::serializeHArray2DUsedAreas(const GMapping::HierarchicalArray2D<GMapping::PointAccumulator>* hArray2D,gmapping_structs::HierarchicalArray2D * pbufHArray2D,const std::pair<int,int>& minr,const std::pair<int,int>& maxr, const std::pair<int,int>& minf ,const std::pair<int,int>& maxf){
    pbufHArray2D->set_m_patchmagnitude(hArray2D->m_patchMagnitude);
    pbufHArray2D->set_m_patchsize(hArray2D->m_patchSize);
    pbufHArray2D->set_m_xsize(hArray2D->m_xsize);
    pbufHArray2D->set_m_ysize(hArray2D->m_ysize);

    //activearea
    for (std::set<GMapping::point<int> >::iterator it = hArray2D->m_activeArea.begin(); it != hArray2D->m_activeArea.end(); ++it)
    {
        gmapping_structs::IntPoint * point = pbufHArray2D->add_m_activearea();
        point->set_x(it->x);
        point->set_y(it->y);
    }

    boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> > ** m_cells = hArray2D->m_cells;


    //std::set<std::pair<int,int> > values;

    //array2d autoptr
    for (int x=0 ; x<hArray2D->m_xsize; x++){
        gmapping_structs::HierarchicalArray2D_innerType * m_cells_x = pbufHArray2D->add_m_cells_x();

        for (int y =0 ;y<hArray2D->m_ysize;y++){

            /*gmapping_structs::Autoptr * m_cells_y = */m_cells_x->add_m_cells_y();

        }
    }

    for (int x = minr.first; x <= maxr.first; x++){
        for (int y = minr.second; y <= maxr.second; y++){

            //values.insert(pair<int,int>(x,y));

            if (m_cells[x][y]) {//if initialized
                //gmapping_structs::Array2D data;
                serializeArray2D(*m_cells[x][y], pbufHArray2D->mutable_m_cells_x(x)->mutable_m_cells_y(y)->mutable_data());
                //m_cells_y->mutable_data()->CopyFrom(data);
            }
        }

    }

    for (int x = minf.first; x <= maxf.first; x++){
        for (int y = minf.second; y <=  maxf.second; y++){
            if (m_cells[x][y]) {//if initialized

                //gmapping_structs::Array2D data;

                /*if (values.find(pair<int,int>(x,y))!=values.end()){
                    std::cerr << x <<" "<< y << "already in protobuf" << "ranges: minf "<<  (minf.first)<< " " ;
                    std::cerr << minf.second <<  " maxf " << (maxf.first)<< " " ;
                    std::cerr <<(maxf.second) << " minr " << (minr.first) << " ";
                    std::cerr <<(minr.second) << " maxr " << (maxr.first) << " ";
                    std::cerr << (maxr.second) << std::endl;
                }*/

                serializeArray2D(*m_cells[x][y], pbufHArray2D->mutable_m_cells_x(x)->mutable_m_cells_y(y)->mutable_data());
                //m_cells_y->mutable_data()->CopyFrom(data);
            }
        }

    }


}


inline void ProtobufHelper::serializeArray2D(const GMapping::Array2D<GMapping::PointAccumulator> & array2D, gmapping_structs::Array2D * pbufArray2d){
    pbufArray2d->set_m_xsize(array2D.m_xsize);
    pbufArray2d->set_m_ysize(array2D.m_ysize);

    GMapping::PointAccumulator ** m_cells = array2D.m_cells;

    for (int x=0 ; x<array2D.m_xsize; x++){

        gmapping_structs::Array2D_innerType * inner_x = pbufArray2d->add_x();

        for (int y =0 ;y<array2D.m_ysize;y++){
            gmapping_structs::PointAccumulator * pointAcc = inner_x->add_y();
            pointAcc->set_n(m_cells[x][y].n);
            pointAcc->set_visits(m_cells[x][y].visits);
            pointAcc->set_x(m_cells[x][y].acc.x);
            pointAcc->set_y(m_cells[x][y].acc.y);
        }
    }
}


void ProtobufHelper::serializeScanMatcherMap(const GMapping::ScanMatcherMap * smap , gmapping_structs::ScanMatcherMap * pbufSmap){

    pbufSmap->set_m_center_x(smap->m_center.x);
    pbufSmap->set_m_center_y(smap->m_center.y);
    pbufSmap->set_m_worldsizex(smap->m_worldSizeX);
    pbufSmap->set_m_worldsizey(smap->m_worldSizeY);
    pbufSmap->set_m_delta(smap->m_delta);
    pbufSmap->set_m_mapsizex(smap->m_mapSizeX);
    pbufSmap->set_m_mapsizey(smap->m_mapSizeY);
    pbufSmap->set_m_sizex2(smap->m_sizeX2);
    pbufSmap->set_m_sizey2(smap->m_sizeY2);

    //gmapping_structs::HierarchicalArray2D local_m_storage;

    serializeHArray2D(&(smap->m_storage),pbufSmap->mutable_m_storage());

    //pbufSmap->mutable_m_storage()->CopyFrom(local_m_storage);

}

void ProtobufHelper::serializeScanMatcherMapUsedAreas(const GMapping::ScanMatcherMap * smap , gmapping_structs::ScanMatcherMap * pbufSmap,
                                                      const std::pair<int,int>& minr ,const std::pair<int,int>& maxr, const std::pair<int,int>& minf ,const std::pair<int,int>& maxf)
{

    pbufSmap->set_m_center_x(smap->m_center.x);
    pbufSmap->set_m_center_y(smap->m_center.y);
    pbufSmap->set_m_worldsizex(smap->m_worldSizeX);
    pbufSmap->set_m_worldsizey(smap->m_worldSizeY);
    pbufSmap->set_m_delta(smap->m_delta);
    pbufSmap->set_m_mapsizex(smap->m_mapSizeX);
    pbufSmap->set_m_mapsizey(smap->m_mapSizeY);
    pbufSmap->set_m_sizex2(smap->m_sizeX2);
    pbufSmap->set_m_sizey2(smap->m_sizeY2);

    serializeHArray2DUsedAreas(&(smap->m_storage),pbufSmap->mutable_m_storage(),minr,maxr,minf,maxf);
}


void ProtobufHelper::serializeParticle(const GMapping::Particle & particle, gmapping_structs::Particle * pbufParticle,unsigned int index){
    pbufParticle->Clear();

    pbufParticle->set_id(index);
    pbufParticle->set_weight(particle.weight);
    pbufParticle->set_weightsum(particle.weightSum);

    //orientedpoint pose
    //gmapping_structs::OrientedPoint localpose;
    pbufParticle->mutable_pose()->set_x(particle.pose.x);
    pbufParticle->mutable_pose()->set_y(particle.pose.y);
    pbufParticle->mutable_pose()->set_theta(particle.pose.theta);
    //pbufParticle->mutable_pose()->CopyFrom(localpose);

    //scan matcher map
    //gmapping_structs::ScanMatcherMap localsmap;
    serializeScanMatcherMap(&(particle.map),pbufParticle->mutable_smap());
    //bufParticle->mutable_smap()->CopyFrom(localsmap);

}

void ProtobufHelper::serializeParticleUsedAreas(const GMapping::Particle & particle, gmapping_structs::Particle * pbufParticle,unsigned int index,
                                                const std::pair<int,int>& minr ,const std::pair<int,int>& maxr, const std::pair<int,int>& minf ,const std::pair<int,int>& maxf)
{

    pbufParticle->Clear();

    pbufParticle->set_id(index);
    pbufParticle->set_weight(particle.weight);
    pbufParticle->set_weightsum(particle.weightSum);

    //orientedpoint pose
    pbufParticle->mutable_pose()->set_x(particle.pose.x);
    pbufParticle->mutable_pose()->set_y(particle.pose.y);
    pbufParticle->mutable_pose()->set_theta(particle.pose.theta);

    //scan matcher map
    serializeScanMatcherMapUsedAreas(&(particle.map),pbufParticle->mutable_smap(),minr,maxr,minf,maxf);


}



void ProtobufHelper::initWorkPackage(const double minimumScore , const GMapping::ScanMatcher & scanMatcher){

    m_workpackage.set_minimumscore(minimumScore);

    m_workpackage.set_activeareacomputed(scanMatcher.m_activeAreaComputed);
    m_workpackage.set_angularodometryreliability(scanMatcher.m_angularOdometryReliability);
    m_workpackage.set_enlargestep(scanMatcher.m_enlargeStep);
    m_workpackage.set_freecellratio(scanMatcher.m_freeCellRatio);
    m_workpackage.set_fullnessthreshold(scanMatcher.m_fullnessThreshold);
    m_workpackage.set_gaussiansigma(scanMatcher.m_gaussianSigma);
    m_workpackage.set_generatemap(scanMatcher.m_generateMap);
    m_workpackage.set_initialbeamskip(scanMatcher.m_initialBeamsSkip);
    m_workpackage.set_kernelsize(scanMatcher.m_kernelSize);
    m_workpackage.set_lasamplerange(scanMatcher.m_lasamplerange);
    m_workpackage.set_lasamplestep(scanMatcher.m_lasamplestep);

    for(unsigned int i = 0; i<scanMatcher.m_laserBeams;i++){
        m_workpackage.add_laserangles(scanMatcher.m_laserAngles[i]);
    }

    m_workpackage.set_laserbeams(scanMatcher.m_laserBeams);
    m_workpackage.set_lasermaxrange(scanMatcher.m_laserMaxRange);

    //gmapping_structs::OrientedPoint laserpose;
    m_workpackage.mutable_laserpose()->set_x(scanMatcher.m_laserPose.x);
    m_workpackage.mutable_laserpose()->set_y(scanMatcher.m_laserPose.y);
    m_workpackage.mutable_laserpose()->set_theta(scanMatcher.m_laserPose.theta);
    //m_workpackage.mutable_laserpose()->CopyFrom(laserpose);

    m_workpackage.set_likelihoodsigma(scanMatcher.m_likelihoodSigma);
    m_workpackage.set_likelihoodskip(scanMatcher.m_likelihoodSkip);
    m_workpackage.set_linearodometryreliability(scanMatcher.m_linearOdometryReliability);
    m_workpackage.set_llsamplerange(scanMatcher.m_llsamplerange);
    m_workpackage.set_llsamplestep(scanMatcher.m_llsamplestep);
    m_workpackage.set_optangulardelta(scanMatcher.m_optAngularDelta);
    m_workpackage.set_optlineardelta(scanMatcher.m_optLinearDelta);
    m_workpackage.set_optrecursiveiterations(scanMatcher.m_optRecursiveIterations);
    m_workpackage.set_usablerange(scanMatcher.m_usableRange);
}

void ProtobufHelper::setWorkPackageLaserReading(double * reading, unsigned int size){

    m_workpackage.clear_plainreading();

    //laser reading
    for (unsigned int i = 0; i < size; i++) {
        m_workpackage.add_plainreading(reading[i]);
    }

}

void ProtobufHelper::deserializePointAccumulator(gmapping_structs::PointAccumulator * pbufPointAccumulator ,GMapping::PointAccumulator* pointAccumulator){
    pointAccumulator->acc.x = pbufPointAccumulator->x();
    pointAccumulator->acc.y=pbufPointAccumulator->y();
    pointAccumulator->n= pbufPointAccumulator->n();
    pointAccumulator->visits = pbufPointAccumulator->visits();
}

void ProtobufHelper::deserializeHArray2D(const gmapping_structs::HierarchicalArray2D & pbufHArray2D,GMapping::HierarchicalArray2D<GMapping::PointAccumulator> * harray2d){

    int m_patchMagnitude = pbufHArray2D.m_patchmagnitude();
    int m_patchSize=pbufHArray2D.m_patchsize();
    int m_xsize=pbufHArray2D.m_xsize();
    int m_ysize=pbufHArray2D.m_ysize();



    //GMapping::HierarchicalArray2D<GMapping::PointAccumulator> * harray2d = new GMapping::HierarchicalArray2D<GMapping::PointAccumulator>(m_xsize,m_ysize,m_patchMagnitude);
    harray2d->m_patchSize = m_patchSize;
    harray2d->m_patchMagnitude = m_patchMagnitude;

    if ((harray2d->m_xsize != m_xsize) || (harray2d->m_ysize != m_ysize)){

        harray2d->m_xsize = m_xsize;
        harray2d->m_ysize = m_ysize;

        delete [] harray2d->m_cells;
        harray2d->m_cells=new boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> > *[m_xsize];
        for (int x=0; x<m_xsize; x++){
            harray2d->m_cells[x]=new boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> >[m_ysize];
            for (int y=0; y<m_ysize; y++){
                harray2d->m_cells[x][y]=boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> >();
            }
        }
    }

    //activearea
    harray2d->m_activeArea.clear();
    for (int i = 0 ; i<pbufHArray2D.m_activearea_size();i++){
        int x = pbufHArray2D.m_activearea(i).x();
        int y = pbufHArray2D.m_activearea(i).y();
        harray2d->m_activeArea.insert(GMapping::point<int>(x,y));
    }


    //m_cells  autoptr<array2d<pointacc> >
    for (int x=0 ; x<m_xsize; x++){
        for (int y =0 ;y<m_ysize;y++){

            if (pbufHArray2D.m_cells_x(x).m_cells_y(y).has_data()) {//if initialized

                if (harray2d->m_cells[x][y]){
                    deserializeArray2DZeroCopy(harray2d->m_cells[x][y],pbufHArray2D.m_cells_x(x).m_cells_y(y).data());
                }else{
                    GMapping::Array2D<GMapping::PointAccumulator> * array2d = deserializeArray2D (pbufHArray2D.m_cells_x(x).m_cells_y(y).data());
                    harray2d->m_cells[x][y].reset(array2d);
                }

            }
        }
    }

}

inline GMapping::Array2D<GMapping::PointAccumulator> * ProtobufHelper::deserializeArray2D(const gmapping_structs::Array2D & pbufArray2d){

    int m_xsize = pbufArray2d.m_xsize();
    int m_ysize = pbufArray2d.m_ysize();
    GMapping::Array2D<GMapping::PointAccumulator> * array2D = new GMapping::Array2D<GMapping::PointAccumulator>(m_xsize,m_ysize);


    for (int x=0 ; x<m_xsize; x++){
        for (int y =0 ;y<m_ysize;y++){
            array2D->m_cells[x][y].acc.x= pbufArray2d.x(x).y(y).x();
            array2D->m_cells[x][y].acc.y= pbufArray2d.x(x).y(y).y();
            array2D->m_cells[x][y].n= pbufArray2d.x(x).y(y).n();
            array2D->m_cells[x][y].visits= pbufArray2d.x(x).y(y).visits();
        }
    }
    return array2D;

}

void ProtobufHelper::deserializeArray2DZeroCopy(boost::shared_ptr<GMapping::Array2D<GMapping::PointAccumulator> > & array2D, const gmapping_structs::Array2D & pbufArray2d){
    int m_xsize = pbufArray2d.m_xsize();
    int m_ysize = pbufArray2d.m_ysize();

    if ((array2D->m_xsize != m_xsize) || (array2D->m_ysize != m_ysize)){
        array2D.reset(new GMapping::Array2D<GMapping::PointAccumulator>(m_xsize,m_ysize));
    }

    for (int x=0 ; x<m_xsize; x++){
        for (int y =0 ;y<m_ysize;y++){
            array2D->m_cells[x][y].acc.x= pbufArray2d.x(x).y(y).x();
            array2D->m_cells[x][y].acc.y= pbufArray2d.x(x).y(y).y();
            array2D->m_cells[x][y].n= pbufArray2d.x(x).y(y).n();
            array2D->m_cells[x][y].visits= pbufArray2d.x(x).y(y).visits();
        }
    }

}


void ProtobufHelper::deserializeScanMatcherMap(const gmapping_structs::ScanMatcherMap & pbufSmap , GMapping::ScanMatcherMap * smap){

    smap->m_center.x = pbufSmap.m_center_x();
    smap->m_center.y = pbufSmap.m_center_y();

    smap->m_worldSizeX = pbufSmap.m_worldsizey();
    smap->m_worldSizeY = pbufSmap.m_worldsizey();
    smap->m_delta = pbufSmap.m_delta();
    smap->m_mapSizeX = pbufSmap.m_mapsizex();
    smap->m_mapSizeY = pbufSmap.m_mapsizey();
    smap->m_sizeX2 = pbufSmap.m_sizex2();
    smap->m_sizeY2 = pbufSmap.m_sizey2();
    deserializeHArray2D(pbufSmap.m_storage(),&smap->m_storage);

}

int ProtobufHelper::deserializeParticle(const gmapping_structs::Particle & pbufParticle ,GMapping::Particle & particle){

    particle.weight = pbufParticle.weight();
    particle.weightSum = pbufParticle.weightsum();

    //orientedpoint pose
    particle.pose.x=pbufParticle.pose().x();
    particle.pose.y=pbufParticle.pose().y();
    particle.pose.theta=pbufParticle.pose().theta();

    //smap
    deserializeScanMatcherMap(pbufParticle.smap(),&particle.map);

    return pbufParticle.id();
}


void ProtobufHelper::deserializeScanMatcherFromWorkPackage(const gmapping_structs::Workpackage & workpackage, GMapping::ScanMatcher & scanmatcher){

    scanmatcher.m_activeAreaComputed = workpackage.activeareacomputed();
    scanmatcher.m_angularOdometryReliability=workpackage.angularodometryreliability();
    scanmatcher.m_enlargeStep=workpackage.enlargestep();
    scanmatcher.m_freeCellRatio=workpackage.freecellratio();
    scanmatcher.m_fullnessThreshold= workpackage.fullnessthreshold();
    scanmatcher.m_gaussianSigma=workpackage.gaussiansigma();
    scanmatcher.m_generateMap=workpackage.generatemap();
    scanmatcher.m_initialBeamsSkip = workpackage.initialbeamskip();
    scanmatcher.m_kernelSize = workpackage.kernelsize();
    scanmatcher.m_lasamplerange= workpackage.lasamplerange();
    scanmatcher.m_lasamplestep= workpackage.lasamplestep();

    for (unsigned int i = 0 ; i<workpackage.laserbeams(); i ++ ){
        scanmatcher.m_laserAngles[i]=workpackage.laserangles(i);
    }
    scanmatcher.m_laserBeams =   workpackage.laserbeams();
    scanmatcher.m_laserMaxRange = workpackage.lasermaxrange();

    scanmatcher.m_laserPose.x = workpackage.laserpose().x();
    scanmatcher.m_laserPose.y = workpackage.laserpose().y();
    scanmatcher.m_laserPose.theta = workpackage.laserpose().theta();
    scanmatcher.m_likelihoodSigma= workpackage.likelihoodsigma();
    scanmatcher.m_likelihoodSkip  = workpackage.likelihoodskip();
    scanmatcher.m_linearOdometryReliability= workpackage.linearodometryreliability();
    scanmatcher.m_llsamplerange= workpackage.llsamplerange();
    scanmatcher.m_llsamplestep= workpackage.llsamplestep();
    scanmatcher.m_optAngularDelta= workpackage.optangulardelta();
    scanmatcher.m_optLinearDelta = workpackage.optlineardelta();
    scanmatcher.m_optRecursiveIterations= workpackage.optrecursiveiterations();
    scanmatcher.m_usableRange = workpackage.usablerange();
}

////laser reading
//reading->clear();
////vector<double> * reading = new vector<double>();
//for(int i = 0 ; i < pbufParticle->plainreading_size();i++){
//    reading->push_back(pbufParticle->plainreading(i));
//}

void ProtobufHelper::deserializeReply(const gmapping_structs::WorkResponse & pbufReply , GMapping::Particle & particle){

    if (pbufReply.has_minx()){
        particle.map.resize(pbufReply.minx(),pbufReply.miny(),pbufReply.maxx(),pbufReply.maxy());
    }

    particle.pose.x=pbufReply.pose().x();
    particle.pose.y=pbufReply.pose().y();
    particle.pose.theta=pbufReply.pose().theta();

    particle.weight = pbufReply.weight();
    particle.weightSum =  pbufReply.weightsum();

    //activearea
    particle.map.m_storage.m_activeArea.clear();
    for (int i = 0 ; i<pbufReply.m_activearea_size();i++){
        int x = pbufReply.m_activearea(i).x();
        int y = pbufReply.m_activearea(i).y();
        particle.map.m_storage.m_activeArea.insert(GMapping::point<int>(x,y));
    }

}


}


