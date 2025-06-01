---
title: "IFCB profiler"
weight: 4
---

### Applications:  
The profiler sensor configuration is deployed from barge and float platforms in protected 
waters. It's used to characterize the vertical distribution of phytoplankton communities 
and their interaction with physical and chemical gradients 
through the water column.

### Physical layout:  
A host IFCB is connected to a CTD and suspended from a winch built from a deep-sea 
electric fishing reel. Its power supply and a GPS modem router that connects it to the 
winch are both housed in a surface-mounted deck box.

### Sampling behavior:  
PhytO-ARM directs collection of a vertical hydrographic profile, interprets its data, and 
identifies a feature of interest for IFCB sampling. It then repositions the IFCB at a 
target sampling depth and initiates IFCB sample collection. CTD observations of salinity, 
temperature, fluorescence, and other water properties measured at time of IFCB sample 
collection are written to IFCB sample metadata for distribution through [IFCB dashboards][ifcbdb]. 
Once IFCB has collected its sample, a new CTD profile is triggered, initiating 
the system's next sampling cycle. This behavior will repeat continuously, pausing only for 
standard cleaning and bead sample collection at an operator defined interval until stopped.

  [ifcbdb]: https://github.com/WHOIGit/ifcbdb.git