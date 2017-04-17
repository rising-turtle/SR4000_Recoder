Win32-VS Projects:
------------------
TCPUDPServer.vcproj: TCP-Server project for Windows
TCPUDPClient.vcproj: TCP-Client project for Windows

Makefiles:
----------
Makefile_bfin      : Embedded Makefile TCP-Server
Makefile_linux     : Linux-OS Makefile TCP-Client

Sources:
--------
TCPUDPClient.cpp   : TCP-Client sources
TCPUDPServer.cpp   : TCP-Server sources
protocolDefines.h  : simple protocol

How it works
------------

TCPUDPClient and TCPUDPServer on windows.
TCPUDPClient connects to 127.0.0.1
mesaserver is running on camera

+------------------------+              +------------------------------------------+
¦                        ¦              ¦                                          ¦
¦   Camera               ¦              ¦    PC                                    ¦
¦                        ¦              ¦                                          ¦
¦                        ¦              ¦                                          ¦
¦   mesaserver           ¦<------------>¦<--->TCPUDPServer<---->TCPUDPClient       ¦
¦   10.0.1.181           ¦              ¦                        127.0.0.1         ¦
¦                        ¦              ¦                                          ¦
¦                        ¦              ¦                                          ¦
+------------------------+              +------------------------------------------+


TCPUDPServer on blackfin (embedded), mesaserver killed.
TCPUDPClient on windows or linux.
TCPUDPClient connects to 10.0.1.181
+------------------------+              +-------------------+
¦                        ¦              ¦                   ¦
¦   Camera               ¦              ¦    PC             ¦
¦                        ¦              ¦                   ¦
¦                        ¦              ¦                   ¦
¦   TCPUDPServer         ¦<------------>¦<--->TCPUDPClient  ¦
¦   (libMesaSR)          ¦              ¦      10.0.1.181   ¦
¦   10.0.1.181           ¦              ¦                   ¦
¦                        ¦              ¦                   ¦
¦                        ¦              ¦                   ¦
+------------------------+              +-------------------+


