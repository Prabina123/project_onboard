# Multiple UAVs in Gazebo
## Prerequisites
See the Host machine prerequisites section in the [running_a_simulation_locally](./running_a_simulation_locally.md) doc.

## Configuration
With the current docker compose definitions, up to 10 drones can be spawned in the containerized gazebo simulation. This is because there are 10 mavros instances available. The number of drones spawned is set via the `NUMBER_OF_DRONES` environment variable for the `px4` service in the `docker-compose.yml`.

## Running the simulation
```bash
docker compose -f docker-compose.yml -f multi_drone.yml run dev bash
```

The first instance of the dr-onboard state machine can be run in the resulting container bash instance. 

- first source the ros workspace setup script 
```bash
source /catkin_ws/devel/setup.bash
```
- then run the dr onboard state machine:
```bash
rosrun dr_onboard_autonomy state_machine.py _uav_name:=<unique drone name> _mqtt_host:=mqtt _local_mqtt_host:=mqtt_local __ns:=<unique name matching one mavros namespace>
```
- it is key that the name space specified with rosrun above matches that of one of the mavros instances
    - the name spaces are formatted like: `uav0`, `uav1`, . . .
    - see the `multi_drone.yml` for specific details around each mavros service instance
- it is also key that the drone names are unique for each instance as you want to be able to communicate to them separately over mqtt

Each additional state machine will need a new dev container. In a new bash shell run:
```bash
docker compose run --no-deps dev
```
- repeat the commands above while updating (incrementing) the ros namespace and using a unique drone name

You can send a mission from yet another new dev container. After starting the new dev container instance, run:
```bash
cat </path/to/mission/spec.json> | mosquitto_pub -h mqtt -t 'drone/<unique drone name>/mission-spec' -s
```

NOTE: as with the single drone simulation, any necessary additional services such as the [air leasing service](https://github.com/DroneResponse/microservice-air-lease/blob/main/README.md) must be running

To shut down the simulation:
```bash
docker compose -f docker-compose.yml -f multi_drone.yml down
```

- simply running _docker compose down_ will result in an orphaned container since there is a new service specified in mac_multi_drone.yml: 
https://stackoverflow.com/questions/41494612/docker-compose-orphan-containers-when-overriding-services

If the docker network for the compose services fails to be removed, it is likely that one of the dev containers is still running. To resolve, remove any remaining containers associated with the network and run the same `docker compose down` command above. 


## Updating the configuration
The multi_drone.yml file does a couple key things to allow for communciation to two separate drones:
1) starts independent mavros instances each in their own ros name space using the ROS_NAMESPACE env variable. This allows each mavros service to communicate with a specific OnboardPilot node in the same name space
2) starts each mavros instance with a new udp port and incremented target system id as specified in the following documentation: https://docs.px4.io/v1.12/en/simulation/multi_vehicle_simulation_gazebo.html

Note that the mavlink parameters for px4 have also been updated as a step in the `px4` docker image creation. The updated configuration file `px4-rc.mavlink` is located under the `px4` directory. The updates include new mavlink port ranges for both the local and remote ports - using 16000+ and 15000+ instead of the defaults. This allows for more than 9 px4 onboard pilot instances to be created before running out of available mavlink ports. The updated configuration also looks up and specifies the ip of each mavros container instance. This is necessary as px4 + gazebo are running in a separate container from each mavros service. 

If you want to add more than 10 drones, then update the `multi_drone.yml` file to create additional mavros instances. Make sure to update the px4 service `depends_on` at the bottom of the file accordingly. 


