#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include <utils/stat.h>
#include "gridslamprocessor.h"
#include <boost/detail/endian.hpp>
#include "../lz4/lz4.h"

#ifdef BOOST_BIG_ENDIAN
#define LITTLE_ENDIAN32(i) { i = swap32(i); }
#else
#define LITTLE_ENDIAN32(i) {}
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

//#define MAP_CONSISTENCY_CHECK
//#define GENERATE_TRAJECTORIES

namespace GMapping {

const double m_distanceThresholdCheck = 20;

using namespace std;

GridSlamProcessor::GridSlamProcessor() :
    m_infoStream(cout) {

    period_ = 5.0;
    m_obsSigmaGain = 1;
    m_resampleThreshold = 0.5;
    m_minimumScore = 0.;

    context = new  zmq::context_t(1);

    threadcounter = 0;
    std::string line;

    ifstream ifile(FILENAME);
    if (!ifile){
        cout << "workers.txt not found" << endl;
        exit(-1);
    }

    cout << "GridSlamProcessor MAINTHREAD: start" << endl;

    //workqueue
    workqueue = new  zmq::socket_t(*context, ZMQ_REP);
    workqueue->bind("inproc://workqueue");

    //syncqueue
    syncqueue = new zmq::socket_t(*context, ZMQ_PULL);
    syncqueue->bind("inproc://syncqueue");


    int localcounter=0;
    int remotecounter=0;


    while(getline(ifile,line)){

        boost::algorithm::trim(line);
        if (line.size()>0){
            threadcounter++;
            if (line.compare("local")==0){
                threads.create_thread(boost::bind(&GridSlamProcessor::localWorker,this,localcounter));
                localcounter++;
            }else{
                threads.create_thread(boost::bind(&GridSlamProcessor::requester,this,remotecounter,line));
                remotecounter++;

            }
        }
    }
}

GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) :
    last_update_time_(0.0), m_particles(gsp.m_particles), m_infoStream(cout) {

    period_ = 5.0;

    //context = gsp.context;

    m_obsSigmaGain = gsp.m_obsSigmaGain;
    m_resampleThreshold = gsp.m_resampleThreshold;
    m_minimumScore = gsp.m_minimumScore;

    m_beams = gsp.m_beams;
    m_indexes = gsp.m_indexes;
    m_motionModel = gsp.m_motionModel;
    m_resampleThreshold = gsp.m_resampleThreshold;
    m_matcher = gsp.m_matcher;

    m_count = gsp.m_count;
    m_readingCount = gsp.m_readingCount;
    m_lastPartPose = gsp.m_lastPartPose;
    m_pose = gsp.m_pose;
    m_odoPose = gsp.m_odoPose;
    m_linearDistance = gsp.m_linearDistance;
    m_angularDistance = gsp.m_angularDistance;
    m_neff = gsp.m_neff;

    cerr << "FILTER COPY CONSTRUCTOR" << endl;
    cerr << "m_odoPose=" << m_odoPose.x << " " << m_odoPose.y << " "
         << m_odoPose.theta << endl;
    cerr << "m_lastPartPose=" << m_lastPartPose.x << " " << m_lastPartPose.y
         << " " << m_lastPartPose.theta << endl;
    cerr << "m_linearDistance=" << m_linearDistance << endl;
    cerr << "m_angularDistance=" << m_linearDistance << endl;

    m_xmin = gsp.m_xmin;
    m_ymin = gsp.m_ymin;
    m_xmax = gsp.m_xmax;
    m_ymax = gsp.m_ymax;
    m_delta = gsp.m_delta;

    m_regScore = gsp.m_regScore;
    m_critScore = gsp.m_critScore;
    m_maxMove = gsp.m_maxMove;

    m_linearThresholdDistance = gsp.m_linearThresholdDistance;
    m_angularThresholdDistance = gsp.m_angularThresholdDistance;
    m_obsSigmaGain = gsp.m_obsSigmaGain;

#ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": trajectories copy.... ";
#endif
    TNodeVector v = gsp.getTrajectories();
    for (unsigned int i = 0; i < v.size(); i++) {
        m_particles[i].node = v[i];
    }
#ifdef MAP_CONSISTENCY_CHECK
    cerr << "end" << endl;
#endif

    cerr
            << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ...";
    updateTreeWeights(false);
    cerr << ".done!" << endl;
}

GridSlamProcessor::GridSlamProcessor(std::ostream& infoS) :
    m_infoStream(infoS) {
    cout << "GridSlamProcessor ostream" << endl;
    period_ = 5.0;
    m_obsSigmaGain = 1;
    m_resampleThreshold = 0.5;
    m_minimumScore = 0.;
    context = new  zmq::context_t(1);

    threadcounter = 0;
    std::string line;

    ifstream ifile(FILENAME);
    if (!ifile){
        cout << "workers.txt not found" << endl;
        exit(-1);
    }

    cout << "GridSlamProcessor MAINTHREAD: start" << endl;

    //workqueue
    workqueue = new  zmq::socket_t(*context, ZMQ_REP);
    workqueue->bind("inproc://workqueue");

    //syncqueue
    syncqueue = new zmq::socket_t(*context, ZMQ_PULL);
    syncqueue->bind("inproc://syncqueue");

    int localcounter=0;
    int remotecounter=0;


    while(getline(ifile,line)){

        boost::algorithm::trim(line);
        if (line.size()>0){
            threadcounter++;
            if (line.compare("local")==0){
                threads.create_thread(boost::bind(&GridSlamProcessor::localWorker,this,localcounter));
                localcounter++;
            }else{
                threads.create_thread(boost::bind(&GridSlamProcessor::requester,this,remotecounter,line));
                remotecounter++;

            }
        }
    }

}

GridSlamProcessor* GridSlamProcessor::clone() const {
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing preclone_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
        const ScanMatcherMap& m1(it->map);
        const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
        for (int x=0; x<h1.getXSize(); x++) {
            for (int y=0; y<h1.getYSize(); y++) {
                const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
                if (a1.m_reference) {
                    PointerMap::iterator f=pmap.find(a1.m_reference);
                    if (f==pmap.end())
                        pmap.insert(make_pair(a1.m_reference, 1));
                    else
                        f->second++;
                }
            }
        }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
        assert(it->first->shares==(unsigned int)it->second);

    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
    GridSlamProcessor* cloned = new GridSlamProcessor(*this);

# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": trajectories end" << endl;
    cerr << __PRETTY_FUNCTION__ << ": performing afterclone_fit_test" << endl;
    ParticleVector::const_iterator jt=cloned->m_particles.begin();
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
        const ScanMatcherMap& m1(it->map);
        const ScanMatcherMap& m2(jt->map);
        const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
        const HierarchicalArray2D<PointAccumulator>& h2(m2.storage());
        jt++;
        for (int x=0; x<h1.getXSize(); x++) {
            for (int y=0; y<h1.getYSize(); y++) {
                const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
                const autoptr< Array2D<PointAccumulator> >& a2(h2.m_cells[x][y]);
                assert(a1.m_reference==a2.m_reference);
                assert((!a1.m_reference) || !(a1.m_reference->shares%2));
            }
        }
    }
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
    return cloned;
}

GridSlamProcessor::~GridSlamProcessor() {

    cout << "MAINTHREAD: sending stop "<< endl;
    for (int i=0;i<threadcounter;i++){
        zmq::message_t gimme;
        workqueue->recv(&gimme);
        workqueue->send(0,0);
    }
    threads.join_all();
    cout << "MAINTHREAD: done" << endl;
    delete workqueue;
    delete syncqueue;
    delete context;
    cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
    cerr << __PRETTY_FUNCTION__ << ": Deleting tree" << endl;
    for (std::vector<Particle>::iterator it = m_particles.begin();
         it != m_particles.end(); it++) {
#ifdef TREE_CONSISTENCY_CHECK		
        TNode* node=it->node;
        while(node)
            node=node->parent;
        cerr << "@" << endl;
#endif
        if (it->node)
            delete it->node;
        //cout << "l=" << it->weight<< endl;
    }

# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing predestruction_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
        const ScanMatcherMap& m1(it->map);
        const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
        for (int x=0; x<h1.getXSize(); x++) {
            for (int y=0; y<h1.getYSize(); y++) {
                const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
                if (a1.m_reference) {
                    PointerMap::iterator f=pmap.find(a1.m_reference);
                    if (f==pmap.end())
                        pmap.insert(make_pair(a1.m_reference, 1));
                    else
                        f->second++;
                }
            }
        }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
        assert(it->first->shares>=(unsigned int)it->second);
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
}

void GridSlamProcessor::setMatchingParameters(double urange, double range,
                                              double sigma, int kernsize, double lopt, double aopt, int iterations,
                                              double likelihoodSigma, double likelihoodGain,
                                              unsigned int likelihoodSkip) {
    m_obsSigmaGain = likelihoodGain;
    m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt,
                                    iterations, likelihoodSigma, likelihoodSkip);
    if (m_infoStream)
        m_infoStream << " -maxUrange " << urange << " -maxUrange " << range
                     << " -sigma     " << sigma << " -kernelSize " << kernsize
                     << " -lstep " << lopt << " -lobsGain " << m_obsSigmaGain
                     << " -astep " << aopt << endl;

}

void GridSlamProcessor::setMotionModelParameters(double srr, double srt,
                                                 double str, double stt) {
    m_motionModel.srr = srr;
    m_motionModel.srt = srt;
    m_motionModel.str = str;
    m_motionModel.stt = stt;

    if (m_infoStream)
        m_infoStream << " -srr " << srr << " -srt " << srt << " -str " << str
                     << " -stt " << stt << endl;

}

void GridSlamProcessor::setUpdateDistances(double linear, double angular,
                                           double resampleThreshold) {
    m_linearThresholdDistance = linear;
    m_angularThresholdDistance = angular;
    m_resampleThreshold = resampleThreshold;
    if (m_infoStream)
        m_infoStream << " -linearUpdate " << linear << " -angularUpdate "
                     << angular << " -resampleThreshold " << m_resampleThreshold
                     << endl;
}

//HERE STARTS THE BEEF

/*Particle::Particle(const ScanMatcherMap& m) :
    map(m), pose(0, 0, 0), weight(0), weightSum(0), gweight(0), previousIndex(
                                                                    0) {
    node = 0;
}*/

void GridSlamProcessor::setSensorMap(const SensorMap& smap) {

    /*
     Construct the angle table for the sensor

     FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot
     */

    SensorMap::const_iterator laser_it = smap.find(std::string("FLASER"));
    if (laser_it == smap.end()) {
        cerr << "Attempting to load the new carmen log format" << endl;
        laser_it = smap.find(std::string("ROBOTLASER1"));
        assert(laser_it!=smap.end());
    }
    const RangeSensor* rangeSensor =
            dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());

    m_beams = static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles = new double[rangeSensor->beams().size()];
    for (unsigned int i = 0; i < m_beams; i++) {
        angles[i] = rangeSensor->beams()[i].pose.theta;
    }
    m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
    delete[] angles;
}

void GridSlamProcessor::init(unsigned int size, double xmin, double ymin,
                             double xmax, double ymax, double delta, OrientedPoint initialPose) {
    m_xmin = xmin;
    m_ymin = ymin;
    m_xmax = xmax;
    m_ymax = ymax;
    m_delta = delta;
    if (m_infoStream)
        m_infoStream << " -xmin " << m_xmin << " -xmax " << m_xmax << " -ymin "
                     << m_ymin << " -ymax " << m_ymax << " -delta " << m_delta
                     << " -particles " << size << endl;

    m_particles.clear();
    TNode* node = new TNode(initialPose, 0, 0, 0);
    ScanMatcherMap lmap(Point(xmin + xmax, ymin + ymax) * .5, xmax - xmin,
                        ymax - ymin, delta);
    for (unsigned int i = 0; i < size; i++) {
        m_particles.push_back(Particle(lmap));
        m_particles.back().pose = initialPose;
        m_particles.back().previousPose = initialPose;
        m_particles.back().setWeight(0);
        m_particles.back().previousIndex = 0;

        // this is not needed
        //		m_particles.back().node=new TNode(initialPose, 0, node, 0);

        // we use the root directly
        m_particles.back().node = node;
    }
    m_neff = (double) size;
    m_count = 0;
    m_readingCount = 0;
    m_linearDistance = m_angularDistance = 0;
}

void GridSlamProcessor::processTruePos(const OdometryReading& o) {
    const OdometrySensor* os =
            dynamic_cast<const OdometrySensor*>(o.getSensor());
    if (os && os->isIdeal() && m_outputStream) {
        m_outputStream << setiosflags(ios::fixed) << setprecision(3);
        m_outputStream << "SIMULATOR_POS " << o.getPose().x << " "
                       << o.getPose().y << " ";
        m_outputStream << setiosflags(ios::fixed) << setprecision(6)
                       << o.getPose().theta << " " << o.getTime() << endl;
    }
}

bool GridSlamProcessor::processScan(const RangeReading & reading,
                                    int adaptParticles) {

    /**retireve the position from the reading, and compute the odometry*/
    OrientedPoint relPose = reading.getPose();
    if (!m_count) {
        m_lastPartPose = m_odoPose = relPose;
    }

    //write the state of the reading and update all the particles using the motion model
    //TODO: use NEON/SSE
    //#pragma omp parallel for
    for (unsigned int i = 0; i < m_particles.size(); i++) {

        /*OrientedPoint& pose(m_particles[i].pose);
        pose = m_motionModel.drawFromMotion(m_particles[i].pose, relPose,m_odoPose);*/

        m_particles[i].pose =  m_motionModel.drawFromMotion(m_particles[i].pose, relPose,m_odoPose);
    }


    // update the output file
    /*if (m_outputStream.is_open()){
     m_outputStream << setiosflags(ios::fixed) << setprecision(6);
     m_outputStream << "ODOM ";
     m_outputStream << setiosflags(ios::fixed) << setprecision(3) << m_odoPose.x << " " << m_odoPose.y << " ";
     m_outputStream << setiosflags(ios::fixed) << setprecision(6) << m_odoPose.theta << " ";
     m_outputStream << reading.getTime();
     m_outputStream << endl;
     }
     if (m_outputStream.is_open()){
     m_outputStream << setiosflags(ios::fixed) << setprecision(6);
     m_outputStream << "ODO_UPDATE "<< m_particles.size() << " ";
     for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
     OrientedPoint& pose(it->pose);
     m_outputStream << setiosflags(ios::fixed) << setprecision(3) << pose.x << " " << pose.y << " ";
     m_outputStream << setiosflags(ios::fixed) << setprecision(6) << pose.theta << " " << it-> weight << " ";
     }
     m_outputStream << reading.getTime();
     m_outputStream << endl;
     }*/

    //invoke the callback
    onOdometryUpdate();

    // accumulate the robot translation and rotation
    OrientedPoint move = relPose - m_odoPose;
    move.theta = atan2(sin(move.theta), cos(move.theta));
    m_linearDistance += sqrt(move * move);
    m_angularDistance += fabs(move.theta);

    // if the robot jumps throw a warning
    if (m_linearDistance > m_distanceThresholdCheck) {
        cerr
                << "***********************************************************************"
                << endl;
        cerr
                << "********** Error: m_distanceThresholdCheck overridden!!!! *************"
                << endl;
        cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
        cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y
             << " " << m_odoPose.theta << endl;
        cerr << "New Odometry Pose (reported from observation)= " << relPose.x
             << " " << relPose.y << " " << relPose.theta << endl;
        cerr
                << "***********************************************************************"
                << endl;
        cerr
                << "** The Odometry has a big jump here. This is probably a bug in the   **"
                << endl;
        cerr
                << "** odometry/laser input. We continue now, but the result is probably **"
                << endl;
        cerr
                << "** crap or can lead to a core dump since the map doesn't fit.... C&G **"
                << endl;
        cerr
                << "***********************************************************************"
                << endl;
    }

    m_odoPose = relPose;

    bool processed = false;

    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
    if (!m_count || m_linearDistance >= m_linearThresholdDistance
            || m_angularDistance >= m_angularThresholdDistance
            || (period_ >= 0.0
                && (reading.getTime() - last_update_time_) > period_)) {
        last_update_time_ = reading.getTime();

        /*if (m_outputStream.is_open()) {
            m_outputStream << setiosflags(ios::fixed) << setprecision(6);
            m_outputStream << "FRAME " << m_readingCount;
            m_outputStream << " " << m_linearDistance;
            m_outputStream << " " << m_angularDistance << endl;
        }*/

        if (m_infoStream)
            m_infoStream << "update frame " << m_readingCount << endl
                         << "update ld=" << m_linearDistance << " ad="
                         << m_angularDistance << endl;

        cerr << "Laser Pose= " << reading.getPose().x << " "
             << reading.getPose().y << " " << reading.getPose().theta
             << endl;

        //this is for converting the reading in a scan-matcher feedable form
        assert(reading.size()==m_beams);
        double * plainReading = new double[m_beams];
        for (unsigned int i = 0; i < m_beams; i++) {
            plainReading[i] = reading[i];
        }
        m_infoStream << "m_count " << m_count << endl;

        RangeReading* reading_copy = new RangeReading(reading.size(),
                                                      &(reading[0]),
                static_cast<const RangeSensor*>(reading.getSensor()),
                reading.getTime());


        //other scans
        if (m_count > 0) {
            //scanMatch(reading);
            //scanMatchOld(plainReading);
            scanMatch(plainReading);
            /*if (m_outputStream.is_open()) {
                m_outputStream << "LASER_READING " << reading.size() << " ";
                m_outputStream << setiosflags(ios::fixed) << setprecision(2);
                for (RangeReading::const_iterator b = reading.begin();
                        b != reading.end(); b++) {
                    m_outputStream << *b << " ";
                }
                OrientedPoint p = reading.getPose();
                m_outputStream << setiosflags(ios::fixed) << setprecision(6);
                m_outputStream << p.x << " " << p.y << " " << p.theta << " "
                        << reading.getTime() << endl;
                m_outputStream << "SM_UPDATE " << m_particles.size() << " ";

                for (ParticleVector::const_iterator it = m_particles.begin();
                        it != m_particles.end(); it++) {
                    const OrientedPoint& pose = it->pose;
                    m_outputStream << setiosflags(ios::fixed) << setprecision(3)
                            << pose.x << " " << pose.y << " ";
                    m_outputStream << setiosflags(ios::fixed) << setprecision(6)
                            << pose.theta << " " << it->weight << " ";
                }
                m_outputStream << endl;
            }*/
            onScanmatchUpdate();

            updateTreeWeights(false);

            if (m_infoStream) {
                m_infoStream << "neff= " << m_neff << endl;
            }
            /*if (m_outputStream.is_open()) {
                m_outputStream << setiosflags(ios::fixed) << setprecision(6);
                m_outputStream << "NEFF " << m_neff << endl;
            }*/

            resample(plainReading, adaptParticles, reading_copy);

        } else { //first scan
            m_infoStream << "Registering First Scan" << endl;


            //#pragma omp parallel for
            for (unsigned int i = 0; i < m_particles.size(); i++) {
                m_matcher.invalidateActiveArea();
                m_matcher.computeActiveArea(m_particles[i].map, m_particles[i].pose, plainReading);
                m_matcher.registerScan(m_particles[i].map, m_particles[i].pose, plainReading);

                // cyr: not needed anymore, particles refer to the root in the beginning!
                TNode* node = new TNode(m_particles[i].pose, 0., m_particles[i].node, 0);
                //node->reading=0;
                node->reading = reading_copy;
                m_particles[i].node = node;
            }
        }


        //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
        updateTreeWeights(false);
        //		cerr  << ".done!" <<endl;

        delete[] plainReading;
        m_lastPartPose = m_odoPose; //update the past pose for the next iteration
        m_linearDistance = 0;
        m_angularDistance = 0;
        m_count++;
        processed = true;


        //keep ready for the next step
        //#pragma omp parallel for
        for (unsigned int i = 0; i < m_particles.size(); i++) {
            m_particles[i].previousPose = m_particles[i].pose;
        }

    }


    if (m_outputStream.is_open())
        m_outputStream << flush;
    m_readingCount++;
    return processed;
}

std::ofstream& GridSlamProcessor::outputStream() {
    return m_outputStream;
}

std::ostream& GridSlamProcessor::infoStream() {
    return m_infoStream;
}

int GridSlamProcessor::getBestParticleIndex() const {
    unsigned int bi = 0;
    double bw = -std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < m_particles.size(); i++)
        if (bw < m_particles[i].weightSum) {
            bw = m_particles[i].weightSum;
            bi = i;
        }
    return (int) bi;
}

void GridSlamProcessor::onScanmatchUpdate() {
}
void GridSlamProcessor::onResampleUpdate() {
}
void GridSlamProcessor::onOdometryUpdate() {
}


void GridSlamProcessor::findUsedAreas(const int id, const int offset , std::pair<int, int> &minr, std::pair<int, int> &minf, std::pair<int, int> &maxr, std::pair<int, int> &maxf)
{
    OrientedPoint p = m_particles[id].pose;
    OrientedPoint lp=p;
    lp.x+=cos(p.theta)*m_matcher.getlaserPose().x-sin(p.theta)*m_matcher.getlaserPose().y;
    lp.y+=sin(p.theta)*m_matcher.getlaserPose().x+cos(p.theta)*m_matcher.getlaserPose().y;
    lp.theta+=m_matcher.getlaserPose().theta;

    double freeDelta=m_particles[id].map.getDelta()*m_matcher.getfreeCellRatio();

    maxr.first = INT_MIN;
    maxr.second = INT_MIN;
    minr.first = INT_MAX;
    minr.second = INT_MAX;

    maxf.first = INT_MIN;
    maxf.second = INT_MIN;
    minf.first = INT_MAX;
    minf.second = INT_MAX;


    const double * angle=m_matcher.m_laserAngles+m_matcher.getinitialBeamsSkip();
    for (unsigned int i = 0 + m_matcher.getinitialBeamsSkip(); i<m_matcher.laserBeams(); i++, angle++){

        double r = pbufhelper.m_workpackage.plainreading().Get(i);
        if (r>m_matcher.getusableRange()) continue;

        Point minphit=lp;
        Point maxphit=lp;
        Point phit=lp;

        phit.x+=r*cos(lp.theta+*angle);
        phit.y+=r*sin(lp.theta+*angle);

        minphit.x+=r*cos(lp.theta+*angle);
        minphit.y+=r*sin(lp.theta+*angle);
        maxphit.x+=r*cos(lp.theta+*angle);
        maxphit.y+=r*sin(lp.theta+*angle);
        minphit.x-= m_matcher.getoptLinearDelta();
        minphit.y-= m_matcher.getoptLinearDelta();
        maxphit.x+= m_matcher.getoptLinearDelta();
        maxphit.y+= m_matcher.getoptLinearDelta();

        IntPoint miniphit=m_particles[id].map.world2map(minphit);
        IntPoint maxiphit=m_particles[id].map.world2map(maxphit);

        Point pfree=lp;
        pfree.x+=(r-freeDelta)*cos(lp.theta+*angle);
        pfree.y+=(r-freeDelta)*sin(lp.theta+*angle);
        pfree=pfree-phit;
        IntPoint ipfree=m_particles[id].map.world2map(pfree);

        miniphit.x -= m_matcher.getkernelSize();
        miniphit.y -= m_matcher.getkernelSize();
        maxiphit.x += m_matcher.getkernelSize();
        maxiphit.y += m_matcher.getkernelSize();

        if (miniphit.x < minr.first ){
            minr.first = miniphit.x;
            minf.first= miniphit.x + ipfree.x;
        }
        if (miniphit.y  < minr.second ){
            minr.second = miniphit.y;
            minf.second = miniphit.y + ipfree.y;
        }
        if (maxiphit.x > maxr.first ){
            maxr.first = maxiphit.x;
            maxf.first = maxiphit.x + ipfree.x;
        }
        if (maxiphit.y > maxr.second ){
            maxr.second = maxiphit.y;
            maxf.second = maxiphit.y + ipfree.y;
        }

    }

    maxr.first += offset;
    maxr.second += offset;
    minr.first -= offset;
    minr.second -= offset;

    maxf.first += offset;
    maxf.second += offset;
    minf.first-= offset;
    minf.second -= offset;

    //truncate

    if (maxr.first > (m_particles[id].map.getMapSizeX()-1)){
        maxr.first=m_particles[id].map.getMapSizeX()-1;
    }

    if (maxr.second > (m_particles[id].map.getMapSizeY()-1)){
        maxr.second=m_particles[id].map.getMapSizeY()-1;
    }

    if (minr.first < 0){
        minr.first=0;
    }

    if (minr.second < 0){
        minr.second=0;
    }


    if (maxf.first > (m_particles[id].map.getMapSizeX()-1)){
        maxf.first=m_particles[id].map.getMapSizeX()-1;
    }

    if (maxf.second > (m_particles[id].map.getMapSizeY()-1)){
        maxf.second=m_particles[id].map.getMapSizeY()-1;
    }

    if (minf.first< 0){
        minf.first=0;
    }

    if (minf.second < 0){
        minf.second=0;
    }

    if (minf.first> (m_particles[id].map.getMapSizeX()-1)){
        minf.first=m_particles[id].map.getMapSizeX()-1;
    }

    if (minf.second > (m_particles[id].map.getMapSizeY()-1)){
        minf.second=m_particles[id].map.getMapSizeY()-1;
    }

    int patchmagnitude = m_particles[id].map.storage().getPatchMagnitude();
    minf.first>>=patchmagnitude;
    minf.second>>=patchmagnitude;
    maxf.first>>=patchmagnitude;
    maxf.second>>=patchmagnitude;
    minr.first>>=patchmagnitude;
    minr.second>>=patchmagnitude;
    maxr.first>>=patchmagnitude;
    maxr.second>>=patchmagnitude;

    /*if (minf.first<=maxr.first){
        minf.first= maxr.first+1;
    }

    if (minf.second<=maxr.second){
        minf.second = maxr.second+1;
    }*/


    //printf("!!!\n");
   // printf("particle %d robot laser min %d %d max %d %d free min %d %d max %d %d\n",id,minr.first,minr.second,maxr.first,maxr.second,minf.first,minf.second,maxf.first,maxf.second);
}

void GridSlamProcessor::requester(const int id, const string & address){

    printf("THREAD %d started\n",id);
    printf("THREAD %d trying to connect to %s\n",id,address.c_str());

    zmq::socket_t requester(*context, ZMQ_REQ);
    requester.connect(address.c_str());

    zmq::socket_t queuesubscriber(*context, ZMQ_REQ);
    queuesubscriber.connect("inproc://workqueue");

    zmq::socket_t syncqueuesubscriber(*context, ZMQ_PUSH);
    syncqueuesubscriber.connect("inproc://syncqueue");

    bool first = true;
    gmapping_structs::Workpackage workpackage;
    gmapping_structs::WorkResponse reply;


    //int scratchsize = PARTICLESCRATCHSPACE;
    //char * scratcharray = new char[PARTICLESCRATCHSPACE];

    google::protobuf::io::GzipOutputStream::Options options;
    options.format = google::protobuf::io::GzipOutputStream::ZLIB;
    options.compression_level = 1;

    //int compressedscratchsize = COMPRESSEDESCRATCHSPACE;
    //char * compressedscratcharray = new char[COMPRESSEDESCRATCHSPACE];

    std::string workstring;


    for(;;){
        queuesubscriber.send(0,0);
        zmq::message_t work;
        queuesubscriber.recv(&work);
        //printf("THREAD %d pulled workpackage\n",id);

        if (unlikely(work.size()==0)){
            printf("THREAD %d ending\n",id);
            break;
        }

        if (first){
            workpackage.CopyFrom(pbufhelper.m_workpackage);
        }


        unsigned int index = *(static_cast<unsigned int*>(work.data()));


        std::pair<int,int> minr ,minf,maxr, maxf;

        findUsedAreas(index,10,minr,minf,maxr,maxf);


        workpackage.clear_plainreading();
        workpackage.clear_particle();
        workpackage.mutable_plainreading()->CopyFrom(pbufhelper.m_workpackage.plainreading());
        pbufhelper.serializeParticleUsedAreas(m_particles[index],workpackage.mutable_particle(),index,minr,maxr,minf,maxf);
        //pbufhelper.serializeParticle(m_particles[index],workpackage.mutable_particle(),index);

        //int workpackagesize = workpackage.ByteSize();

        workstring.clear();
        {
            google::protobuf::io::StringOutputStream compressedStream(&workstring);
            google::protobuf::io::GzipOutputStream compressingStream(&compressedStream, options);
            workpackage.SerializeToZeroCopyStream(&compressingStream);
        }

        //resize array if needed
        /*while(unlikely(workpackagesize > scratchsize)){
            scratchsize = scratchsize + CHUNKSIZE;
            delete [] scratcharray;
            scratcharray = new char[scratchsize];
        }*/


        /*workpackage.SerializeToArray(scratcharray,workpackagesize);
        int maxcompressedsize = LZ4_compressBound(workpackagesize)+4;

        //resize array if needed
        while(unlikely(maxcompressedsize > compressedscratchsize)){
            compressedscratchsize = compressedscratchsize + COMPRESSEDCHUNKSIZE;
            delete [] compressedscratcharray;
            compressedscratcharray = new char[compressedscratchsize];
        }

        //compress to lz4
        int32_t compressedsize = LZ4_compress(scratcharray,compressedscratcharray+4,workpackagesize);
        LITTLE_ENDIAN32(workpackagesize);
        * (uint32_t*) compressedscratcharray = workpackagesize;*/


        //send it
        //zmq::message_t workmessage(scratcharray,compressedsize,NULL,NULL); //zero copy
        //requester.send(workmessage);

        zmq::message_t workmessage((void*)workstring.c_str(),workstring.length(),NULL,NULL); //zero copy
        requester.send(workmessage);

        zmq::message_t reply_message;
        requester.recv(&reply_message);

        /*int32_t decompressedsize = * (uint32_t*)  reply_message.data();
        LITTLE_ENDIAN32(decompressedsize);

        //resize array if needed
        while(unlikely(decompressedsize > scratchsize)){
            scratchsize = scratchsize + CHUNKSIZE;
            delete [] scratcharray;
            scratcharray = new char[scratchsize];
        }

        LZ4_uncompress(static_cast<char*>(reply_message.data())+4,scratcharray,decompressedsize);
        */


        if (unlikely(!reply.ParseFromArray(reply_message.data(),reply_message.size()))) {
            std::cerr << "Failed to parse workresponse." << std::endl;
            exit(-1);
        }

        int id = reply.id();


        pbufhelper.deserializeReply(reply,m_particles[id]);


        /*pbufhelper.deserializeParticle(reply.particle(),m_particles[id]);*/

        syncqueuesubscriber.send(0,0);
    }

    //delete [] scratcharray;
    //delete [] compressedscratcharray;
}

void GridSlamProcessor::localWorker(const int id){
    printf("THREAD localWorker %d started\n",id);

    zmq::socket_t queuesubscriber(*context, ZMQ_REQ);
    queuesubscriber.connect("inproc://workqueue");

    zmq::socket_t syncqueuesubscriber(*context, ZMQ_PUSH);
    syncqueuesubscriber.connect("inproc://syncqueue");

    unsigned int workcounter = 0;

    for(;;){
        queuesubscriber.send(0,0);
        zmq::message_t work;
        queuesubscriber.recv(&work);


        if (work.size()==0){
            printf("THREAD localWorker %d ending\n",id);
            break;
        }


        unsigned int index = *(static_cast<unsigned int*>(work.data()));

        double reading[pbufhelper.m_workpackage.plainreading_size()];
        for(int i = 0 ; i< pbufhelper.m_workpackage.plainreading_size();i++){
            reading[i]=pbufhelper.m_workpackage.plainreading(i);

        }

        scanMatchLocal(reading,index);
        workcounter++;
        printf("THREAD localWorker %d: workpackages processed %u\n",id,workcounter);
        syncqueuesubscriber.send(0,0);



    }

}



}
// end namespace

