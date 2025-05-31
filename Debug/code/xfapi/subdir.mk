################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/xfapi/asr_audio.c \
../code/xfapi/base64.c \
../code/xfapi/hmac_sha256.c \
../code/xfapi/sha1.c \
../code/xfapi/utf8.c \
../code/xfapi/websocket_client.c 

COMPILED_SRCS += \
code/xfapi/asr_audio.src \
code/xfapi/base64.src \
code/xfapi/hmac_sha256.src \
code/xfapi/sha1.src \
code/xfapi/utf8.src \
code/xfapi/websocket_client.src 

C_DEPS += \
code/xfapi/asr_audio.d \
code/xfapi/base64.d \
code/xfapi/hmac_sha256.d \
code/xfapi/sha1.d \
code/xfapi/utf8.d \
code/xfapi/websocket_client.d 

OBJS += \
code/xfapi/asr_audio.o \
code/xfapi/base64.o \
code/xfapi/hmac_sha256.o \
code/xfapi/sha1.o \
code/xfapi/utf8.o \
code/xfapi/websocket_client.o 


# Each subdirectory must supply rules for building sources it contributes
code/xfapi/asr_audio.src: ../code/xfapi/asr_audio.c code/xfapi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/ADS/ADS_TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=1 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/xfapi/asr_audio.o: code/xfapi/asr_audio.src code/xfapi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/xfapi/base64.src: ../code/xfapi/base64.c code/xfapi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/ADS/ADS_TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=1 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/xfapi/base64.o: code/xfapi/base64.src code/xfapi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/xfapi/hmac_sha256.src: ../code/xfapi/hmac_sha256.c code/xfapi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/ADS/ADS_TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=1 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/xfapi/hmac_sha256.o: code/xfapi/hmac_sha256.src code/xfapi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/xfapi/sha1.src: ../code/xfapi/sha1.c code/xfapi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/ADS/ADS_TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=1 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/xfapi/sha1.o: code/xfapi/sha1.src code/xfapi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/xfapi/utf8.src: ../code/xfapi/utf8.c code/xfapi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/ADS/ADS_TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=1 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/xfapi/utf8.o: code/xfapi/utf8.src code/xfapi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/xfapi/websocket_client.src: ../code/xfapi/websocket_client.c code/xfapi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/ADS/ADS_TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=1 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/xfapi/websocket_client.o: code/xfapi/websocket_client.src code/xfapi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-xfapi

clean-code-2f-xfapi:
	-$(RM) code/xfapi/asr_audio.d code/xfapi/asr_audio.o code/xfapi/asr_audio.src code/xfapi/base64.d code/xfapi/base64.o code/xfapi/base64.src code/xfapi/hmac_sha256.d code/xfapi/hmac_sha256.o code/xfapi/hmac_sha256.src code/xfapi/sha1.d code/xfapi/sha1.o code/xfapi/sha1.src code/xfapi/utf8.d code/xfapi/utf8.o code/xfapi/utf8.src code/xfapi/websocket_client.d code/xfapi/websocket_client.o code/xfapi/websocket_client.src

.PHONY: clean-code-2f-xfapi

