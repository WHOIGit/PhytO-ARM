---
title: PhytO-ARM
---
PhytO-ARM is a toolkit for integration of an Imaging FlowCybobot ([IFCB][mclane]) with 
other oceanographic sensors and platforms. It has been developed by the [Brosnahan Lab at 
the Woods Hole Oceanographic Institution][blab]. At this site, you'll find part 
inventories and assembly instructions for a variety of different sensor configurations 
that enable different sampling behaviors and automated response on detection of harmful 
species. 
 
PhytO-ARM systems are supported by robot operating system (ROS) software that is available 
through GitHub [here][repo]. The supported ROS release is [Noetic Ninjemys][noetic]. 
Systems built around Nvidia IFCBs can use a containerized version of IFCBacquire that is 
available [here][contifcbacq]. Containerization of IFCBacqure allows updates to the Nvidia 
host computer so that it can support real-time image classification using [Triton 
Inference Server][triton].

Additional code for export of ROS data to CF-compliant NetCDF data formats is available 
[here][PAnet].

  [mclane]: https://mclanelabs.com/imaging-flowcytobot/
  [blab]: https://www2.whoi.edu/site/brosnahanlab/
  [repo]: https://github.com/WHOIGit/PhytO-ARM
  [noetic]: http://wiki.ros.org/noetic
  [contifcbacq]: https://github.com/WHOIGit/Dockerized-IFCBacquire
  [triton]: https://github.com/triton-inference-server/server
  [PAnet]: https://github.com/WHOIGit/rosbag-netcdf
