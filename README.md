#gmapping-stateless

This is a modified version of the GMapping SLAM package ([http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping)) to run in a distributed environment.

##Installation

This package was developed and tested using ROS Fuerte. 

#### Dependencies: 

 * ZeroMQ ([http://zeromq.org/](http://zeromq.org/)) 
 * Google Protocol Buffers ([https://developers.google.com/protocol-buffers/](https://developers.google.com/protocol-buffers/))
 * All the dependencies from the fuerte branch of GMapping ([http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping))

To install you can follow the BuildingPackages instructions on [http://wiki.ros.org/ROS/Tutorials/BuildingPackages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) (rosbuild)
  
##Setup

#### Main Computer:
Create a *workers.txt* text file in the same folder where you are going to launch the GMapping node with the following Syntax:

`tcp://RemoteWorkerIP:Port`   for each remote worker  
`local` for each local Worker/Thread 

e.g.: for 2 remote workers with IP 192.168.1.1 and 192.168.1.2 with the RemoteWorker program  running on port 8010 and 8011 respectively  and 1 local worker the contents of the file would be:
    
    local
    tcp://192.168.1.1:8010     
    tcp://192.168.1.2:8011

For the GMapping parameters see [http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping)

#### Remote Workers:

The remote workers need to run the **zmq\_reqrep\_worker** from the comand line using the following syntax : 

   `$ ./zmq_reqrep_worker  -a tcp://NetworkInterfaceIP:Port`

e.g.:    `$ ./zmq_reqrep_worker  -a  tcp://192.168.1.1:8010` 

or    `$ ./zmq_reqrep_worker  -a  tcp://*:8010`  if you don't want to specify a network interface
