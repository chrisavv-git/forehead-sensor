# make test: every test (C++ core under sanitizers, the tool against fhsim)
# make firmware: the T-Watch S3 firmware (firmware/out/twatch/)
test:
	$(MAKE) -C firmware/host test
	.venv/bin/python -m pytest -q tool/tests

firmware:
	firmware/build.sh

.PHONY: test firmware
