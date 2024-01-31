# C++

## build the project
```
cd root folder
cmake -B build .
cmake --build build
./build/bin/test_main
```

# Docker

## build dockerfile
```console
docker build -t jordan787878/solarsail-landing:tag .
```

## run dockerfile
``` 
docker run -it jordan787878/solarsail-landing bash
./build/bin/test_main
```
or run it on the docker volume: solarsail-volume
```
docker run -v solarsail-volume:/app/outputs -it jordan787878/solarsail-landing bash
```

## push dockerfile
```
docker push jordan787878/solarsail-landing
```

# Developing
## output filesystem for python visualization
Create a docker volume: solarsail-volume to store output files
Go to Docker Desktop/Volumes to access the output files

## Put source code into Github, and follow the Gitflow 
Need to think about the output file and python visual scripts
