#include <boost/program_options.hpp>
#include <string>
#include <gridfastslam/gridslamprocessor.h>
#include <scanmatcher/scanmatcher.h>
#include "protobuf/protobufhelper.h"
#include <zmq.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/gzip_stream.h>
#include <boost/detail/endian.hpp>
#include "lz4/lz4.h"


#define PARTICLESCRATCHSPACE (1024*1024*2) //2MB
#define CHUNKSIZE (1024*512) //512k
#define COMPRESSEDESCRATCHSPACE (1024*512) //512k
#define COMPRESSEDCHUNKSIZE (1024*64) //64k


#ifdef BOOST_BIG_ENDIAN
#define LITTLE_ENDIAN32(i) { i = swap32(i); }
#else
#define LITTLE_ENDIAN32(i) {}
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)


namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

} // namespace


std::string parsecmdline(int argc, char *argv[]){

    /** Define and parse the program options
         */
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
            ("help", "Print help messages")
            ("address,a",po::value<std::string>()->required() ,"Address used to listen to requests");

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc),
                  vm); // can throw

        /** --help option*/
        if ( vm.count("help")  )
        {
            std::cout << "SLAM GMapping Worker App" << std::endl
                      << desc << std::endl;
            exit(SUCCESS);
        }

        po::notify(vm); // throws on error, so do after help in case
        // there are any problems
    }
    catch(po::error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        exit(ERROR_IN_COMMAND_LINE);
    }


    return vm["address"].as<std::string>();
}

GMapping::ScanMatcher scanmatcher;

inline void scanMatch(const double* plainReading , GMapping::Particle & particle,const double minimumScore,gmapping_structs::WorkResponse & reply_) {
    // sample a new pose from each scan in the reference

    GMapping::OrientedPoint corrected;
    double score, l, s;
    score = scanmatcher.optimize(corrected, particle.map, particle.pose, plainReading);
    //    it->pose=corrected;
    if (score > minimumScore) {
        particle.pose = corrected;
    }

    gmapping_structs::OrientedPoint * pbufpose = reply_.mutable_pose();
    pbufpose->set_x(particle.pose.x);
    pbufpose->set_y(particle.pose.y);
    pbufpose->set_theta(particle.pose.theta);


    scanmatcher.likelihoodAndScore(s, l, particle.map, particle.pose, plainReading);

    reply_.set_weight(particle.weight + l);
    reply_.set_weightsum(particle.weightSum + l);

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    scanmatcher.invalidateActiveArea();
    GMapping::IntPoint min,max;

    if (scanmatcher.computeActiveAreaRemote(particle.map, particle.pose, plainReading,min,max)){
        reply_.set_minx(min.x);
        reply_.set_miny(min.y);
        reply_.set_maxx(max.x);
        reply_.set_maxy(max.y);
    }

   const std::set< GMapping::point<int>, GMapping::pointcomparator<int> > & activearea = particle.map.storage().getActiveArea();

    //activearea
    for (std::set<GMapping::point<int> >::iterator it = activearea.begin(); it != activearea.end(); ++it)
    {
        gmapping_structs::IntPoint * point = reply_.add_m_activearea();
        point->set_x(it->x);
        point->set_y(it->y);
    }


}

int main (int argc, char *argv[])
{

    try{

        unsigned int workcounter = 0;

        std::string address = parsecmdline(argc,argv);

        zmq::context_t context(1);
        zmq::socket_t responder(context, ZMQ_REP);


        std::cout << "listening to: " << address << std::endl;
        responder.bind(address.c_str());

        ProtoBuf::ProtobufHelper pbufhelper;

        google::protobuf::io::GzipOutputStream::Options options;
        options.format = google::protobuf::io::GzipOutputStream::ZLIB;
        options.compression_level = 1;

        gmapping_structs::Workpackage workpackage;
        gmapping_structs::WorkResponse reply;


        GMapping::ScanMatcherMap map(1,1,5);
        //GMapping::Particle particle(map);

        //int scratchsize = PARTICLESCRATCHSPACE;
        //char * scratcharray = new char[PARTICLESCRATCHSPACE];

        //int compressedscratchsize = COMPRESSEDESCRATCHSPACE;
        //char * compressedscratcharray = new char[COMPRESSEDESCRATCHSPACE];


        while(1)
        {
            //Wait for next request from client
            zmq::message_t workMessage;
            responder.recv(&workMessage);

            //std::string work(static_cast<char*>(workMessage.data()), workMessage.size());




            /*int32_t decompressedsize = * (uint32_t*)  workMessage.data();
            LITTLE_ENDIAN32(decompressedsize);*/

            printf("%lu\n",workMessage.size());


            //resize array if needed
           /* while(unlikely(decompressedsize > scratchsize)){
                scratchsize = scratchsize + CHUNKSIZE;
                delete [] scratcharray;
                scratcharray = new char[scratchsize];
            }*/

            //LZ4_uncompress(static_cast<char*>(workMessage.data())+4,scratcharray,decompressedsize);


            google::protobuf::io::ArrayInputStream input(workMessage.data(), workMessage.size());
            google::protobuf::io::GzipInputStream decompressingStream(&input);

            if (unlikely(!workpackage.ParseFromZeroCopyStream(&decompressingStream))){
                std::cerr << "Failed to parse workpackage." << std::endl;
                return -1;
            }

            pbufhelper.deserializeScanMatcherFromWorkPackage(workpackage,scanmatcher);


            double reading[workpackage.plainreading_size()];
            for(int i = 0 ; i< workpackage.plainreading_size();i++){
                reading[i]=workpackage.plainreading(i);

            }

            reply.Clear();

            GMapping::Particle particle(map);
            int id = pbufhelper.deserializeParticle(workpackage.particle(),particle);
            scanMatch(reading,particle,workpackage.minimumscore(),reply);
            reply.set_id(id);

            //pbufhelper.serializeParticle(particle,reply.mutable_particle(),id);

            int replysize = reply.ByteSize();
            char sendarray[replysize];

            //resize array if needed
            /*while(unlikely(replysize > scratchsize)){
                scratchsize = scratchsize + CHUNKSIZE;
                delete [] scratcharray;
                scratcharray = new char[scratchsize];
            }*/

            reply.SerializeToArray(&sendarray,replysize);
            //printf("replysize %d\n",replysize);
            //int maxcompressedsize = LZ4_compressBound(replysize)+4;

            //resize array if needed
            /*while(unlikely(maxcompressedsize > compressedscratchsize)){
                compressedscratchsize = compressedscratchsize + COMPRESSEDCHUNKSIZE;
                delete [] compressedscratcharray;
                compressedscratcharray = new char[compressedscratchsize];
            }*/

            //compress to lz4
            /*int32_t compressedsize = LZ4_compress(scratcharray,compressedscratcharray+4,replysize);
            LITTLE_ENDIAN32(replysize);
            * (uint32_t*) compressedscratcharray = replysize;
            */

            //send it
            zmq::message_t reply_message(&sendarray,replysize,NULL,NULL); //zero copy
            responder.send(reply_message);
            workcounter++;
            //printf("workpackages processed %u \n",workcounter);

        }

    }
    catch(std::exception& e)
    {


        std::cerr << "Unhandled Exception reached the top of main: "
                  << e.what() << ", application will now exit" << std::endl;
        return ERROR_UNHANDLED_EXCEPTION;

    }

    return SUCCESS;

}

