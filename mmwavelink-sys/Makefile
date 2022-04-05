compile:
	intercept-build sh -c "clang -I$(pwd) -I/usr/lib/llvm-6.0/lib/clang/6.0.1/include -nostartfiles ti/control/mmwavelink/src/*.c"

transpile:
	c2rust transpile compile_commands.json
