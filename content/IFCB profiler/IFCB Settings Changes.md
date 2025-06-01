---
title: "IFCB Settings Alterations"
output: md_document
weight: 1
---

##Settings Alterations

###PhytoArmDataSource

To ensure data streaming between attached CTD IFCB and Phyto-Arm software it is important to alter the settings file on your IFCB to include a local url to host data. The settings file is stored under `IFCBacquire/Host/Settings.txt`

In the settings.txt file, there is shoud be an entry called "PhytoArmDataSource"

If this entry is `"PhytoArmDataSource:0"` then this feature will be disabled. If you plan on using your IFCB to profile with a CTD and the Phyto-Arm software you want to change this.

If the entry is `"PhytoArmDataSource:1:XXXXXXXXXX"` Then this feature will be enabled, and the XXXXXX should be populated with a valid URL that when queried with a WebRequest, will respond with valid JSON data, in a totally flat structure. To be a valid URL, the field must start with <http://> or <https://> e.g. `"http://localhost/phytoarmdata"` It can include an explicit port definition i.e. `"<http://localhost:8818/phytoarmdata>"` This is the url that points back to the ros Web node that will interpret this data.

This change allows Phyto-Arm to interpret and write data received froim CTD cast into the header file. This is done by the API call "general:getphytoarmdata" that will cause the IFCB to query this URL if the feature is enabled. If the IFCB is not currently in a sample, the response from the Phytoarm will be cached. When the SaveFile step is performed, this cached value will be parsed, and all Key/Value pairs in the cached value will be stored in the header. It is important to note that there is no protection against duplicate entries. This does not overwrite any IFCB key value pairs, it only inserts the key value pairs in the PhytoArm Json response. As such it would be prudent to add a suffix/prefix to the PhytoArm keys, such that there is no risk of having duplicate Keys in the header file.


###GPSData

It is also important update the GPSData source in the same settings file. When using the Phyto-Arm software this wants to be set to `GPSData:0`. This allows the PhytoArm software to write GPS data and turns off the internal IFCB GPS data stream. 