

all:
	$(MAKE) -C ./src all -j`cat /proc/cpuinfo | grep processor | wc -l`

new: clean all

single:
	$(MAKE) -C ./src all

clean:
	$(MAKE) -C ./src clean

