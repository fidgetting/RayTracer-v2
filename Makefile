

DIRS = obj src

all:
	$(MAKE) -C obj
	$(MAKE) -C util
	$(MAKE) -C src

debug:
	$(MAKE) -C obj  debug
	$(MAKE) -C util debug
	$(MAKE) -C src  debug

clean:
	$(MAKE) -C obj  clean
	$(MAKE) -C util clean
	$(MAKE) -C src  clean


