
BIN  = bin
DIRS = obj src

all: $(BIN)
	$(MAKE) -C obj
	$(MAKE) -C util
	$(MAKE) -C src

debug: $(BIN)
	$(MAKE) -C obj  debug
	$(MAKE) -C util debug
	$(MAKE) -C src  debug

$(BIN):
	mkdir $(BIN)

clean:
	$(MAKE) -C obj  clean
	$(MAKE) -C util clean
	$(MAKE) -C src  clean


