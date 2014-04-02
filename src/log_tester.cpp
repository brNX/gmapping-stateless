#include <utils/commandline.h>
#include <utils/stat.h>
#include <configfile/configfile.h>
#include <unistd.h>
#include <pthread.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <log/carmenconfiguration.h>
#include <log/sensorstream.h>
#include <gridfastslam/gridslamprocessor.h>

using namespace std;


string filename;
//This are the processor parameters
std::string outfilename;

double xmin;
double ymin;
double xmax;
double ymax;
bool autosize;
double delta;
double resampleThreshold;

//scan matching parameters
double sigma;
double maxrange;
double maxUrange;
double regscore;
double lstep;
double astep;
int kernelSize;
int iterations;
double critscore;
double maxMove;
unsigned int lskip;

//likelihood
double lsigma;
double ogain;
double llsamplerange, lasamplerange;
double llsamplestep, lasamplestep;
double linearOdometryReliability;
double angularOdometryReliability;


//motion model parameters
double srr, srt, str, stt;
//particle parameters
int particles;
bool skipMatching;

//gfs parameters
double angularUpdate;
double linearUpdate;

//robot config
GMapping::SensorMap sensorMap;
//input stream
GMapping::InputSensorStream* input;
std::ifstream plainStream;
bool readFromStdin;
bool onLine;
bool generateMap;
bool considerOdometryCovariance;
unsigned int randseed;
unsigned int mapUpdateTime;

//dirty carmen interface
const char* const * m_argv;
unsigned int m_argc;
double m_minimumScore;

GMapping::GridSlamProcessor * gsp_;


int loadFiles(const char * fn){
	ifstream is;
	if (fn)
		is.open(fn);
	else
		is.open(filename.c_str());
	if (! is){
		cout << "no file found" << endl;
		return -1;
	}

	GMapping::CarmenConfiguration conf;
	conf.load(is);
	is.close();

	sensorMap=conf.computeSensorMap();

	if (input)
		delete input;

	if (! readFromStdin){
		plainStream.open(filename.c_str());
		input=new GMapping::InputSensorStream(sensorMap, plainStream);
		cout << "Plain Stream opened="<< (bool) plainStream << endl;
	} else {
		input=new GMapping::InputSensorStream(sensorMap, cin);
		cout << "Plain Stream opened on stdin" << endl;
	}
	return 0;
}

GMapping::OrientedPoint boundingBox(GMapping::SensorLog* log, double& xmin, double& ymin, double& xmax, double& ymax){
	GMapping::OrientedPoint initialPose(0,0,0);
	initialPose=log->boundingBox(xmin, ymin, xmax, ymax);
	xmin-=3*maxrange;
	ymin-=3*maxrange;
	xmax+=3*maxrange;
	ymax+=3*maxrange;
	return initialPose;
}

int init(int argc, const char * const * argv){
	m_argc=argc;
	m_argv=argv;
	std::string configfilename;
	std::string ebuf="not_set";

	CMD_PARSE_BEGIN_SILENT(1,argc);
	parseStringSilent("-cfg",configfilename);
	CMD_PARSE_END_SILENT;

	if (configfilename.length()>0){
		GMapping::ConfigFile cfg(configfilename);

		filename = (std::string) cfg.value("gfs","filename",filename);
		outfilename = (std::string) cfg.value("gfs","outfilename",outfilename);
		xmin = cfg.value("gfs","xmin", xmin);
		xmax = cfg.value("gfs","xmax",xmax);
		ymin = cfg.value("gfs","ymin",ymin);
		ymax = cfg.value("gfs","ymax",ymax);
		delta =  cfg.value("gfs","delta",delta);
		maxrange = cfg.value("gfs","maxrange",maxrange);
		maxUrange = cfg.value("gfs","maxUrange",maxUrange);
		regscore = cfg.value("gfs","regscore",regscore);
		critscore = cfg.value("gfs","critscore",critscore);
		kernelSize = cfg.value("gfs","kernelSize",kernelSize);
		sigma = cfg.value("gfs","sigma",sigma);
		iterations = cfg.value("gfs","iterations",iterations);
		lstep = cfg.value("gfs","lstep",lstep);
		astep = cfg.value("gfs","astep",astep);
		maxMove = cfg.value("gfs","maxMove",maxMove);
		srr = cfg.value("gfs","srr", srr);
		srt = cfg.value("gfs","srt", srt);
		str = cfg.value("gfs","str", str);
		stt = cfg.value("gfs","stt", stt);
		particles = cfg.value("gfs","particles",particles);
		angularUpdate = cfg.value("gfs","angularUpdate", angularUpdate);
		linearUpdate = cfg.value("gfs","linearUpdate", linearUpdate);
		lsigma = cfg.value("gfs","lsigma", lsigma);
		ogain = cfg.value("gfs","lobsGain", ogain);
		lskip = (int)cfg.value("gfs","lskip", lskip);
		mapUpdateTime = cfg.value("gfs","mapUpdate", mapUpdateTime);
		randseed = cfg.value("gfs","randseed", randseed);
		autosize = cfg.value("gfs","autosize", autosize);
		readFromStdin = cfg.value("gfs","stdin", readFromStdin);
		resampleThreshold = cfg.value("gfs","resampleThreshold", resampleThreshold);
		skipMatching = cfg.value("gfs","skipMatching", skipMatching);
		onLine = cfg.value("gfs","onLine", onLine);
		generateMap = cfg.value("gfs","generateMap", generateMap);
		m_minimumScore = cfg.value("gfs","minimumScore", m_minimumScore);
		llsamplerange = cfg.value("gfs","llsamplerange", llsamplerange);
		lasamplerange = cfg.value("gfs","lasamplerange",lasamplerange );
		llsamplestep = cfg.value("gfs","llsamplestep", llsamplestep);
		lasamplestep = cfg.value("gfs","lasamplestep", lasamplestep);
		linearOdometryReliability = cfg.value("gfs","linearOdometryReliability",linearOdometryReliability);
		angularOdometryReliability = cfg.value("gfs","angularOdometryReliability",angularOdometryReliability);
		ebuf = (std::string) cfg.value("gfs","estrategy", ebuf);
		considerOdometryCovariance = cfg.value("gfs","considerOdometryCovariance",considerOdometryCovariance);

	}


	CMD_PARSE_BEGIN(1,argc);
	parseString("-cfg",configfilename);     /* to avoid the warning*/
	parseString("-filename",filename);
	parseString("-outfilename",outfilename);
	parseDouble("-xmin",xmin);
	parseDouble("-xmax",xmax);
	parseDouble("-ymin",ymin);
	parseDouble("-ymax",ymax);
	parseDouble("-delta",delta);
	parseDouble("-maxrange",maxrange);
	parseDouble("-maxUrange",maxUrange);
	parseDouble("-regscore",regscore);
	parseDouble("-critscore",critscore);
	parseInt("-kernelSize",kernelSize);
	parseDouble("-sigma",sigma);
	parseInt("-iterations",iterations);
	parseDouble("-lstep",lstep);
	parseDouble("-astep",astep);
	parseDouble("-maxMove",maxMove);
	parseDouble("-srr", srr);
	parseDouble("-srt", srt);
	parseDouble("-str", str);
	parseDouble("-stt", stt);
	parseInt("-particles",particles);
	parseDouble("-angularUpdate", angularUpdate);
	parseDouble("-linearUpdate", linearUpdate);
	parseDouble("-lsigma", lsigma);
	parseDouble("-lobsGain", ogain);
	parseInt("-lskip", lskip);
	parseInt("-mapUpdate", mapUpdateTime);
	parseInt("-randseed", randseed);
	parseFlag("-autosize", autosize);
	parseFlag("-stdin", readFromStdin);
	parseDouble("-resampleThreshold", resampleThreshold);
	parseFlag("-skipMatching", skipMatching);
	parseFlag("-onLine", onLine);
	parseFlag("-generateMap", generateMap);
	parseDouble("-minimumScore", m_minimumScore);
	parseDouble("-llsamplerange", llsamplerange);
	parseDouble("-lasamplerange", lasamplerange);
	parseDouble("-llsamplestep", llsamplestep);
	parseDouble("-lasamplestep", lasamplestep);
	parseDouble("-linearOdometryReliability",linearOdometryReliability);
	parseDouble("-angularOdometryReliability",angularOdometryReliability);
	parseString("-estrategy", ebuf);

	parseFlag("-considerOdometryCovariance",considerOdometryCovariance);
	CMD_PARSE_END;

	if (filename.length() <=0){
		cout << "no filename specified" << endl;
		return -1;
	}



	return 0;
}

int main(int argc, char ** argv){



	gsp_ = new GMapping::GridSlamProcessor();

	xmin=-100.;
	ymin=-100.;
	xmax=100.;
	ymax=100.;
	delta=0.05;

	//scan matching parameters
	sigma=0.05;
	maxrange=80.;
	maxUrange=80.;
	regscore=1e4;
	lstep=.05;
	astep=.05;
	kernelSize=1;
	iterations=5;
	critscore=0.;
	maxMove=1.;
	lsigma=.075;
	ogain=3;
	lskip=0;
	autosize=false;
	skipMatching=false;

	//motion model parameters
	srr=0.1, srt=0.1, str=0.1, stt=0.1;
	//particle parameters
	particles=30;
	randseed=0;

	//gfs parameters
	angularUpdate=0.5;
	linearUpdate=1;
	resampleThreshold=0.5;

	input=0;

	mapUpdateTime=5;
	readFromStdin=false;
	onLine=false;
	generateMap=false;

	// This  are the dafault settings for a grid map of 5 cm
	llsamplerange=0.01;
	llsamplestep=0.01;
	lasamplerange=0.005;
	lasamplestep=0.005;
	linearOdometryReliability=0.;
	angularOdometryReliability=0.;

	considerOdometryCovariance=false;

	/// @todo Expose setting an initial pose
	/*GMapping::OrientedPoint initialPose;
	  if(!getOdomPose(initialPose, scan.header.stamp))
	    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
	 */


	double temporalUpdate= -1.0;

	init(argc,argv);

	cout <<"GSP INITIALIZED"<< endl;
		if (loadFiles(NULL)){
			cout <<"GSP READFILE ERROR"<< endl;
			return -2;
		}

	cout <<"FILES LOADED"<< endl;

	gsp_->setSensorMap(sensorMap);
	gsp_->setMatchingParameters(maxUrange, maxrange, sigma,kernelSize, lstep, astep, iterations,lsigma, ogain, lskip);


	GMapping::OrientedPoint initialPose(0,0,0);

	if (autosize){
		GMapping::SensorLog * log=new GMapping::SensorLog(sensorMap);
		ifstream is(filename.c_str());
		log->load(is);
		is.close();
		initialPose=boundingBox(log, xmin, ymin, xmax, ymax);
		delete log;
	}

	gsp_->setMotionModelParameters(srr, srt, str, stt);
	gsp_->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	gsp_->setUpdatePeriod(temporalUpdate);
	gsp_->setgenerateMap(false);
	gsp_->init(particles, xmin, ymin, xmax, ymax, delta, initialPose);
	gsp_->setllsamplerange(llsamplerange);
	gsp_->setllsamplestep(llsamplestep);
	gsp_->setlasamplerange(llsamplerange);
	gsp_->setlasamplestep(llsamplestep);

	GMapping::sampleGaussian(1,time(NULL));


	ofstream rawpath("rawpath.dat");
		while(*(input)){
			const GMapping::SensorReading* r;
			(*(input)) >> r;
			if (! r)
				continue;
			const GMapping::RangeReading* rr=dynamic_cast<const GMapping::RangeReading*>(r);
			if (rr){
				const GMapping::RangeSensor* rs=dynamic_cast<const GMapping::RangeSensor*>(rr->getSensor());
				assert (rs && rs->beams().size()==rr->size());

				bool processed=gsp_->processScan(*rr);
				rawpath << rr->getPose().x << " " << rr->getPose().y << " " << rr->getPose().theta << endl;
				if (0 && processed){
				cerr << "Retrieving state .. ";
				GMapping::TNodeVector trajetories=gsp_->getTrajectories();
					cerr << "Done" <<  endl;
					cerr << "Deleting Tree state .. ";
					for (GMapping::TNodeVector::iterator it=trajetories.begin(); it!=trajetories.end(); it++)
						delete *it;
					cerr << "Done" << endl;
				}
// 				if (0 && processed){
// 					cerr << "generating copy" << endl;;
// 					GridSlamProcessor* m_gsp=gpt->clone();
// 					Map<double, DoubleArray2D, false>*  pmap=m_gsp->getParticles()[0].map.toDoubleMap() ;
// 					cerr << "deleting" << endl;
// 					delete m_gsp;
// 					delete pmap;
// 				}
			}
			/*const GMapping::OdometryReading* o=dynamic_cast<const GMapping::OdometryReading*>(r);
			if (o && gpt->running){
				gpt->processTruePos(*o);
				GMapping::TruePosEvent* truepos=new TruePosEvent;
				truepos->pose=o->getPose();
			}*/
		}

	rawpath.close();

	GMapping::TNodeVector trajetories=gsp_->getTrajectories();
	cerr << "WRITING WEIGHTS" << endl;
	int pnumber=0;
	for (GMapping::TNodeVector::iterator it=trajetories.begin(); it!=trajetories.end(); it++){
		char buf[10];
		sprintf(buf, "w-%03d.dat",pnumber);
		ofstream weightsStream(buf);
		GMapping::TNode* n=*it;
		double oldWeight=0, oldgWeight=0;
		while (n!=0){
			double w=n->weight-oldWeight;
			double gw=n->gweight-oldgWeight;
			oldWeight=n->weight;
			oldgWeight=n->gweight;
			weightsStream << w << " " << gw << endl;
			n=n->parent;
		}
		weightsStream.close();
		pnumber++;
		cerr << buf << endl;
	}


	delete gsp_;

}
