# invoke SourceDir generated makefile for tcpEchoCC3100.pem4f
tcpEchoCC3100.pem4f: .libraries,tcpEchoCC3100.pem4f
.libraries,tcpEchoCC3100.pem4f: package/cfg/tcpEchoCC3100_pem4f.xdl
	$(MAKE) -f C:\Users\Julio\Documents\GitHub\TrabajoSEPA/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Julio\Documents\GitHub\TrabajoSEPA/src/makefile.libs clean

