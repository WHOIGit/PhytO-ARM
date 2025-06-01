---
title: "Winch Assembly & Installation"
---

This guide walks through the necessary components for winch assembly on a PhytO-ARM system, along with instructions for setup and maintenance. In most cases, the necessary hardware can be determined by referring to the user manuals for individual components or referencing the drawings for any fabricated parts. You may choose to alter the order of steps, but this is the easiest way for most users.

#### Hardware

The original installations use custom-fabricated stands for mounting the winches. Plans for those stands can be found here:

The original design took several factors into account: 

- The variable height off the deck needed for different instruments to be safely deployed
- The length of the boom and resulting distance between the hull and the instruments
- The location of safely railings, cables, or other possible obstructions
- The stand base shape needed to support and secure the winch while also allowing it to be as close to the rub rail as possible

The winch is powered by a MAC400 motor manufactured by JVL DK. A gearbox reducer from Cone Drive Inc. is used to both maximize the available torque and to hold the instruments in a stationary position.

The MAC400 motor is rated to IP66 and theoretically can withstand being sprayed directly with water. However, given the relative cost of replacement and the conditions it will experience, we chose to fabricate an aluminum housing for the motor. Plans for the housing can be found here: LINK

The motor will also generate substantial heat while operating. If left sitting in direct sun even when powered down, the motor will likely be hot to the touch, so some amount of sun protection or shading is advised. Though we have not implemented this yet, we also recommend you consider a line of caulk or a rubber gasket when using a housing to prevent condensation and corrosion.

Attached to the gearbox are deep-sea fishing parts supplied by Hiliner Inc, specifically a boom fitting for the top of the gearbox, a fiberglass boom with stackable pieces, and a large heavy-duty spool.

Other necessary items are appropriate fasteners, a rigging pulley, some anti-slip decking tape, Molykote, Blue Grease HT, Aquashield, and a sturdy, lightweight line.



#### Installing the Winches

This step-by-step guide reflects our current setup, and may need to be adapted depending on the exact equipment configuration you use. 

To begin, secure the lower section of the winch stand to the mounting block or deck with lag screws or use bolts with lock nuts if the underside of the decking is easily accessible. Since the load will be centered past the front of the stand, be sure the back leg is secure and will not tip.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Winch1.jpeg)

Once the stand is installed, attach the stand insert to the base of the gearbox using the threaded bores. Fit the bolts with washers and coat the threads in Aquashield to prevent corrosion.

Seat the gearbox and post in the stand base, and adjust the height as needed. To secure the stand at the appropriate height, run a bolt or pin through the paired holes in the stand base and post. Tighten with a lock nut.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Winch2.jpeg)

Next, fit the motor in the aluminum case. 
Set two #10 bolts in the bottom two holes on the motor face with washers between the bolt head and the motor casing. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Winch3.jpeg)

With these in place, carefully tip the motor forward and guide the motor shaft at an angle through the face of the case. Press the back of the motor down until it is in place at the bottom of the case, and adjust so the motor face is seated in the opening in the case. Tip forward so the bolts slide through the bores in the case. 

Insert a hex key through the slots on either side of the case and find the bolt head. Holding the bolt in place from the inside, finger-tighten a lock nut on the end of each bolt. Set the remaining two bolts into the bores on the top side of the motor. This will ensure that as the motor is moved around, the bolts don't slip out or fall into the case.

Some users may prefer to connect the two power cables and Ethernet cable to the MAC400 motor at this stage, others find it easier to make these connections once the winch is fully installed. Fit the cable glands over the connectors and tighten using the sleeve on the end of the cable. Wrap the connections in electrical tape for extra security, making sure to stretch the electrical tape slightly to ensure good contact.

Apply a small amount of Molykote to the gearbox input shaft with a cotton swab (be careful, it is very messy if it gets on clothing or skin). Filp the case so the open end is facing down.

The gearbox has a small setscrew that will tighten a clamp around the motor output shaft, located underneath a small rubber insert. Remove the over and take out the setscrew to open the input shaft.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE1.jpeg)

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE2.jpeg)

Remove the nuts from the ends of the bolts around the motor face, and carefully line up the motor output shaft with the gearbox input. Run the bolts through the bores on the gearbox face, using the hex key to keep them in place. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE3.jpeg)

Use a small wrench to secure the bolts with lock nuts, tightening evenly as you go. Coat the setscrew in Loctite and insert back into the gearbox to tighten the input shaft and secure the shaft connection. Set it in place with a hex key. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE4.jpeg)

When fitted properly, the motor and case should lie flush against the gearbox face, the motor should feel securely in place, and the motor will run smoothly. If any bolts are loose, the load can sag, putting strain on the motor or causing misalignment between the motor and gearbox.

**If the motor is properly fitted to the gearbox, and tuned appropriately, there should be no noise when the motor runs.**

Cover the slots on either side of the case with electrical tape to prevent water from leaking in. The motor and control module are both highly resistant to spraying or splashing water, but we want to keep it to a minimum.

Next, fit the spindle over the output shaft on the gearbox. Remove the setscrew from the spindle and set aside. Give the input on the spindle a coat of Blue Grease HT or Molykote to prevent grinding and galling, and apply some to the gearbox output shaft as well.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE5.jpeg)

Mount the spindle, and apply some Loctite to the setscrew before tightening it in place. If the spindle isn't properly fitted or is loose, there will be increased wear on the key from moving the winch up and down. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE6.jpg)

Remove the knob from the end of the output shaft and mount the spool. Be sure the spool has a teal nonslip pad on either side as you mount it. 

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Winch4.jpeg)

Fit the metal ring with the tab over the end of the output shaft, and slide the flat key through the slot. Run a small bolt through the key and tab, and tighten with a lock nut. This nut does not need to be tightened down completely, but to keep the key from slipping. Replace the knob on the end and tighten down. The knob presses into the key, securing the spool on the shaft. If the knob is not tight, the spool may slip on the shaft.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/Winch5.jpeg)

Apply the non-slip decking tape to the inside of the spool to increase the grip the line will have. If played out too far without enough friction on the spool, the line may simply rotate around the spool as it turns. Secure your line on the spool by running the line through a loop at the end around the center of the spool.

Secure the top plate and boom mount to the top of the gearbox. Coat the bolts in Aquashield to prevent corrosion before tightening in place.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE7.jpeg)

Once the boom mount is on, position the boom slats. Each slat should have a hole near the end, if using more than one, line them up and secure with the eye bolt and lock nut. Loosen the bolts holding the plates on top of the gearbox together, but note that you will need to use longer bolts than the standard ones Hiliner provides if you use more than two slats. Tighten the bolts down bit by bit in an X pattern, so the boom is under more or less even pressure on all sides while you adjust it. When properly aligned, the end of the boom should be in line with the center of the spool. Feed the line through the pulley secured on the end of the boom.

![Alt](https://github.com/WHOIGit/PhytO-ARM/blob/master/website/static/images/CONE8.jpeg)

Tune the winch before deploying. Check the alignment on the boom by placing a weight at the end of the line and raising and lowering it several times. The line should feed cleanly through the pulley and wrap across the spool evenly. If the boom (and therefore the line) is properly centered, the line will wrap to one side of the spool and then back across to the other. 

#### Deployment & Maintenance
When deploying, remove anything that could be damaged or shaken loose during transit. During breaks and once you arrive at the site, check all bolts and tighten as needed, they will slowly become loose while trailering.

At the site, check all cable connections and ensure you can control the motor. If you haven't already, secure the cables to the stand with zipties. The motor should run smoothly, and there should be little to no noise coming from the gearbox. If there is noise, check to make sure the input shaft is properly clamped around the motor output shaft. If the winch does not run smoothly after adjusting, or motion is jerky, check the motor filter settings.

Make sure the boom is properly aligned. When doing longer casts, the line may wrap differently than over short distances. Make sure the line feeds evenly from one side back to the other and does not favor one side. If this happens, the line will pile up against the side of the spool and slip, resulting in loops of line getting stuck underneath the line being fed onto the spool. This will result in the line jumping suddenly during a downcast, jerking the logger and preventing smooth, steady data collection.

  Bolts should also be examined for signs of corrosion; apply more Aquashield to the bolt threads if needed.
