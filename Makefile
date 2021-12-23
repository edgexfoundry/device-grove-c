
.PHONY: build test clean docker

MICROSERVICES=build/release/device-grove-c/device-grove-c
.PHONY: $(MICROSERVICES)

DOCKERS=docker_device_grove_c
.PHONY: $(DOCKERS)

VERSION=$(shell cat ./VERSION)
GIT_SHA=$(shell git rev-parse HEAD)

build: ./VERSION ${MICROSERVICES}

build/release/device-grove-c/device-grove-c:
	    scripts/build.sh

test:
	    @echo $(MICROSERVICES)

clean:
	    rm -f $(MICROSERVICES)

./VERSION:
	    @git describe --abbrev=0 | sed 's/^v//' > ./VERSION

version: ./VERSION
	    echo ${VERSION}

docker: ./VERSION $(DOCKERS)

docker_device_grove_c:
	    docker build \
	        -f scripts/Dockerfile.alpine \
	        --label "git_sha=$(GIT_SHA)" \
	        -t edgexfoundry/device-grove:${GIT_SHA} \
	        -t edgexfoundry/device-grove:${VERSION}-dev \
            .
