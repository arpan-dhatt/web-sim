CC = clang++
CARGS = --target=wasm32 --no-standard-libraries -Wl,--export-all -Wl,--no-entry

test.wasm: test.cpp
	$(CC) $(CARGS) -o $@ $<
