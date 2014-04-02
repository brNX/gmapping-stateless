
//#include "zhelpers.hpp"
#include <zmq.hpp>
#include <iostream>
#include <fstream>
#include "protobuf/protobufhelper.h"
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/gzip_stream.h>
#include <gridfastslam/gridslamprocessor.h>
#include <scanmatcher/smmap.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include "zhelpers.hpp"

using namespace std;


zmq::context_t context(1);

void requester(const int id, const string & address){


    printf("THREAD %d started\n",id);

    printf("THREAD %d trying to connect to %s\n",id,address.c_str());

    zmq::socket_t requester(context, ZMQ_REQ);
    requester.connect(address.c_str());

    zmq::socket_t queuesubscriber(context, ZMQ_REQ);
    queuesubscriber.connect("inproc://workqueue");

    //syncqueue
    zmq::socket_t syncqueue(context, ZMQ_PUSH);
    syncqueue.connect("inproc://syncqueue");

    int synccount =0;

    for(;;){
        //zmq::message_t empty;
        queuesubscriber.send(0,0);
        //s_send(queuesubscriber,"gimme work");
        zmq::message_t work;
        queuesubscriber.recv(&work);
        //std::string work = s_recv(queuesubscriber);
        printf("THREAD %d pulled workpackage\n",id);

        if (work.size()==0){
            printf("THREAD %d ending\n",id);
            break;
        }
        printf("sending...\n");
        //sleep(1);

        cout << "THREAD "<<id << ": work size " << work.size() << endl;

        requester.send(work);


        zmq::message_t reply_message;
        gmapping_structs::WorkResponse reply;

        requester.recv(&reply_message);
        std::string reply_string(static_cast<char*>(reply_message.data()), reply_message.size());

        if (!reply.ParseFromString(reply_string)) {
             std::cerr << "Failed to parse workresponse." << std::endl;
             exit(-1);
        }

        std::cout << "THREAD "<<id << ": Received reply with " << reply.particles_size() << " particles"<<endl;
        cout << "THREAD "<<id << "send sync " << synccount << endl;
        ostringstream oss;
        oss << synccount;

        s_send(syncqueue,oss.str());
        synccount++;
        //syncqueue.send(0,0);

        //TODO : modify particle vector
    }
}

#define FILENAME "workers.txt"

int main (int argc, char *argv[])
{

    int threadcounter = 0;
    string line;

    ifstream ifile(FILENAME);

    if (!ifile){
        cout << "workers.txt not found" << endl;
        return -1;
    }

    printf("MAINTHREAD: start\n");

    //workqueue
    zmq::socket_t workqueue(context, ZMQ_REP);
    workqueue.bind("inproc://workqueue");

    //syncqueue
    zmq::socket_t syncqueue(context, ZMQ_PULL);
    syncqueue.bind("inproc://syncqueue");

    boost::thread_group threads;

    int i=0;

    while(getline(ifile,line)){

        boost::algorithm::trim(line);
        if (line.size()>0){
            threads.create_thread(boost::bind(requester,i,line));
            threadcounter++;
            i++;
        }
    }


    sleep(2);



    using namespace google::protobuf::io;
    // Write the new address book back to disk.
    std::fstream input("workpackage", std::ios::in | std::ios::binary);

    //google::protobuf::io::IstreamInputStream file_stream((std::istream *) &input);
    //GzipOutputStream::Options options;
    //options.format = GzipOutputStream::GZIP;
    //options.compression_level = 2;
    //google::protobuf::io::GzipInputStream gzip_stream(&file_stream);

    ProtoBuf::ProtobufHelper pbufhelper;

    gmapping_structs::Workpackage workpackage;

    if (!workpackage.ParseFromIstream(&input)) {
         std::cerr << "Failed to parse workpackage." << std::endl;
         return -1;
    }

    std::string work;
    workpackage.SerializeToString(&work);

    /*GMapping::ScanMatcherMap map(4000,4000,5);
    GMapping::Particle particle(map);

    double reading[workpackage.plainreading_size()];

    pbufhelper.deserializeParticle(workpackage.particles(0),particle);
    for(int i = 0 ; i< workpackage.plainreading_size();i++){
        reading[i]=workpackage.plainreading(i);
    }

    GMapping::ScanMatcher scanmatcher;
    pbufhelper.deserializeScanMatcherFromWorkPackage(workpackage,scanmatcher);*/



    for( int request = 0 ; request < 30 ; request++) {
        /*ostringstream oss("Hello ");
        oss<< request;
        cout << "MAINTHREAD: sending hello " <<  request << " to workqueue\n";*/
        zmq::message_t gimme;
        workqueue.recv(&gimme);
        std::cout << work.length() << std::endl;

        zmq::message_t workmessage(work.size());
        memcpy(workmessage.data(), work.c_str(), work.length());
        workqueue.send(workmessage);

        //s_send (workqueue, work.c_str());
    }

    for (int i=0; i<30;i++){
        /*zmq::message_t sync;
        syncqueue.recv(&sync);*/
        string answer =  s_recv(syncqueue);
        printf("got sync count %d : %s\n",i,answer.c_str());
    }

    printf("MAINTHREAD: sending stop\n");
    for (int i=0;i<threadcounter;i++){
        zmq::message_t gimme;
        workqueue.recv(&gimme);
        workqueue.send(0,0);
    }


    threads.join_all();
    printf("MAINTHREAD: done\n");

    return 0;
}
