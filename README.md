

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
docker run -v solarsail-volume:/app/outputs -it jordan787878/solarsail-landing bash

## push dockerfile
docker push jordan787878/solarsail-landing


# Developing
## Integrated the visual.py into this container
I copy the original python_workspace into the c_workspace,
rename it as: python_workspace_build.
It is not a good way; the image built is too large.
A better way could be saving the output to "docker Volume"

Create a docker volume: solarsail-volume to store output files
Go to Docker Desktop/Volumes to access the output files

## Put source code into Github, and follow the Gitflow 
Need to think about the output file and python visual scripts
