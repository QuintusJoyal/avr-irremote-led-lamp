FROM ubuntu

RUN apt update
RUN apt install \
  avrdude \
  gcc-avr \
  gdb-avr \
  avr-libc \
  binutils-avr