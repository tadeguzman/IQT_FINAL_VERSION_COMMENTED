# IridiumSBD Arduino Library rev 2.0
The Rock 7 RockBLOCK is a fascinating communications module that gives TTL-level devices like Arduino access to the Iridium satellite network.  This is a big deal, because it means that your application can now easily and inexpensively communicate from any point on the surface of the globe, from the heart of the Amazon to the Siberian tundra.
This library, IridiumSBD, uses Iridium's SBD ("Short Burst Data") protocol to send and receive short messages to/from the Iridium hub.  SBD is a "text message"-like technology that supports the transmission of text or binary messages up to a certain maximum size (270 bytes received, 340 bytes transmitted).
Written by Mikal Hart with generous support from Rock 7 Mobile. For more information, visit the Rock 7 http://rock7mobile.com.

THEO DEGUZMAN - JHU/IQT24 modifications

This library has been modified, adding small functionality to allow for the ability to read "RING" signals without wiring the ring wire, to reduce electrical complexity. It does this by reading and recording "Ring" messages sent solely over the RS232 protocol, which is a built-in feature, but is not leveraged in the current code.  This is likely not used in the current code because most applications are not checking Satellite messages regularly, and instead use the "Ring" pin attached to an interrupt. 

