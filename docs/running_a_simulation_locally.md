# Running a simulation locally
## Host machine prerequisites
- docker engine (daemon)
- docker compose

On a Linux host, you will want to make sure to [install or upgrade](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) to the latest docker daemon and docker compose version. This setup was tested with:
- docker versions: 20.10.25, 24.0.6
- docker compose versions: 1.29.2, 2.21.0

It may work with earlier versions, but some features such as providing the `extra_hosts` service input in the docker compose file are only available in later versions. `extra_hosts` is used to map the address where px4 should send mavlink data to be received by QGroundControl.

## Configurations
### QGroundControl (QGC)
The `host.qgc` mapping under the px4 service is configured by default in the docker compose file to send QGC mavlink data to the host maching running the simulation containers. However, you may want to forward the QGC mavlink data to another IP on the network. If so, you can update the `extra_hosts` `host.qgc` configuration to point to your desired IP:

```yaml
px4:
  extra_hosts:
    - "host.qgc:<your.desired.ip.address>"
```

## Steps to run
Start the docker containers:
```bash
docker compose build
docker compose run vs-code bash
```

Note, the vs-code container is also set up to run as a vs code dev container as well. You can start
it as a dev container instead of using `docker run` above.

Start the air-leasing service - refer to the [README](https://github.com/DroneResponse/microservice-air-lease/blob/main/README.md).
Note: if a mission fails, it's likely that you'll need to restart the air-leasing service.

You should be able to connect to the gazebo simulation window via a browser at: 127.0.0.1:8080

Run the state machine from the vs-code container:
```bash
source /catkin_ws/devel/setup.bash
rosrun dr_onboard_autonomy state_machine.py _uav_name:=Orange _mqtt_host:=mqtt _local_mqtt_host:=mqtt_local
```

In another second terminal window, start a another dev container:
```bash
docker compose run --no-deps dev
```

From the second dev container, send a mission to the drone (`test-briar-travel.json` is one example):
```bash
cat /catkin_ws/src/dr_onboard_autonomy/missions/peppermint-rd-big-circle.json | mosquitto_pub -h mqtt -t 'drone/Orange/mission-spec' -s
```

---

To stop your containers:
```bash
docker compose down
```

Depending on your docker verison, you may need to use `docker stop` and `docker rm` to stop and 
remove any containers you started with `docker run` or `docker compose run`. Once all containers are
stopped run `docker compose down` again to rm the compose network. 