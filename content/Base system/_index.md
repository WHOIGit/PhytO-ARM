---
title: "Base System"
weight: 10
---

### Applications:  
The base configuration serves most PhytO-ARM deployments including shipboard, vehicle, 
and floating platform installs. Location data from a GPS antenna are associated with IFCB 
samples and written to .hdr files at their time of creation. Likewise, CTD data (if 
available) is written to .hdr files so that measurements can be distributed alongside IFCB 
images through the [IFCB dashboard][ifcbdb] data portal.

### Physical layout:  
A host IFCB is connected to a GPS modem router. If available, the IFCB sensor is 
also connected to a CTD using a non-standard bulkhead connector through the IFCB vent port.

### Sampling behavior:  
PhytO-ARM directs continuous sample collection by the IFCB and associates location and 
other contextual observations with IFCB sample metadata in real time.

  [ifcbdb]: https://github.com/WHOIGit/ifcbdb.git
