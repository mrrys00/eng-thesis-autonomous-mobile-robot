
.PHONY: init_submodules
init_submodules:
	git submodule init
	git submodule update

	mkdir -p hokuyo_lib
	cp -r hokuyo-python-lib/src/hokuyo/* hokuyo_lib/

.PHONY: update_submodules
update_submodules:
	git submodule update
