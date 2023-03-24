---
title: PhytO-ARM
weight: 1
---
PhytO-ARM is a toolkit for integration of the Imaging FlowCybobot ([IFCB][mclane]) 
phytoplankton sensor with other oceanographic sensors and platforms. It has been 
developed by the [Brosnahan Lab][blab] at the Woods Hole Oceanographic Institution. 

At this site, you'll find bills of materials and assembly instructions for a variety of 
sensor configurations. These enable different sampling behaviors and automated responses 
upon detection of HABs or other species of interest. 
 
PhytO-ARM systems use robot operating system (ROS) software that is available 
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
