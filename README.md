# Assembly Instrumenting tool

This tool can be used to instrument ARMv8-m33 C code for CFA.

## Produce instrumented assembly from C file

1) Write the application source code to `application.c` and `application.h`.

2) Modify `PROJ` in `pre-process.sh` to point to your STM32Cube project directory

3) In `application.c`, ensure any function with no internal function call has a call to `SECURE_new_log_entry`. This will ensure any added calls due to instrumentation save/return from the link register properly.

4) Run `pre-proces.sh` as a bash script with two input parameters. First: the C file name without the .c extension (e.g., for `application.c`, the first input should be `application`). Second: the desired name of assembly output (e.g., for `instrumented.s` the second input should be `instrumented`)

5) This should result in two files. First, `application.s` containing unmodified assembly of `application.c`. Second, `instrumented.s` (or whatever you set as output file name), containing an instrumented version of `application.s`. This script modifies each indirect call/jump to pass through `SECURE_log_indr_fwd`, each return to pass through `SECURE_log_ret`, and ensures the first instruction at the destination of conditional branches is a call to `SECURE_log_cond_br_taken` or `SECURE_log_cond_br_not_taken`. These trampolines are implemented in `secure_nsc.c`. This implementation ensures each control flow destination that occurred is logged.

6) Add `instrumented.s` and `application.h` to the `{PROJ}/NonSecure/Core/Src` directory of your STM32Cube project

7) Add `secure_nsc.c` to `{PROJ}/Secure/Core/Src` and `secure_nsc.h` to `{PROJ}/Secure_nsclib`

   
