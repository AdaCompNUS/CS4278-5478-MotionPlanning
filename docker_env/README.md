# README

## Build

``` sh
docker build --tag adacomplab/ros_kinetic:latest .
```

## Run

Without mount:
``` sh
docker run --init --name lab1 -p 6080:80 -e RESOLUTION=1920x1080 -w="/home/ir" adacomplab/ros_kinetic:latest
```

To mount a directory, add the flag `-v /full/path/on/host:/full/path/in/container`.

## Stop & Resume A Container

``` sh
docker stop lab1
docker start lab1
```

The container presists as long as you do not remove it, even if you power off
you computer. If you want to remove the container, `commit` the container to a
Docker image.
