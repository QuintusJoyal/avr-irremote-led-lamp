FROM ubuntu

RUN apt-get update
RUN apt-get install -y\
  avrdude \
  gcc-avr \
  gdb-avr \
  avr-libc \
  binutils-avr \
  make

WORKDIR /app

COPY . /app

CMD ["/bin/bash"]