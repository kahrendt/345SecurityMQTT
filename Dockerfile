ARG ARCH=library
FROM $ARCH/debian:stretch AS build

# Install build tools and remove apt-cache afterwards
RUN apt-get -q update && apt-get install -yq --no-install-recommends \
	build-essential librtlsdr-dev rtl-sdr libmosquittopp-dev libyaml-cpp-dev git \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

# Switch into our apps working directory
WORKDIR /usr/src/app/345SecurityMQTT/src

COPY . /usr/src/app/345SecurityMQTT
WORKDIR /usr/src/app/345SecurityMQTT/src
RUN ./build.sh

FROM $ARCH/debian:stretch

RUN apt-get -q update && apt-get install -yq --no-install-recommends \
	librtlsdr-dev rtl-sdr libmosquittopp-dev libyaml-cpp-dev \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

COPY --from=build /usr/src/app/345SecurityMQTT/src/345toMqtt 345toMqtt

#switch on systemd init system in container
ENV INITSYSTEM on

# Run our binary on container startup
CMD ./345toMqtt
