---
title: "Base System""
weight: 1
---

### Applications:  
The base sensor configuration described here is used in most PhytO-ARM deployments 
including shipboard, vehicle, and floating platform installs. Location data from a GPS 
antenna are associated with IFCB samples and written to .hdr files at their time of 
creation. Likewise, CTD data (if available) is written to .hdr files so that measurements
can be distributed alongside IFCB images through the [IFCB dashboard][ifcbdb] data portal.

### Physical layout:  
A host IFCB is connected to a GPS modem router. If available, it is also connected to a 
CTD.

### Sampling behavior:  
PhytO-ARM directs continuous sample collection by the IFCB and associates location and 
other contextual observations with IFCB sample metadata in real time.

  [ifcbdb]: https://github.com/WHOIGit/ifcbdb.git