# Parameterized Map

This repo is under construction. For mini-project users, you can directly use it.

### usage 1: structured map generation


``
roslaunch param_env structure_map.launch

``

You can adjust the apprximate ratio of each element (overlapping is also counting now)

``
    <param name="map/cylinder_ratio" value="0.10" type="double"/>
    <param name="map/circle_ratio"   value="0.02" type="double"/>
    <param name="map/gate_ratio"     value="0.02" type="double"/>
    <param name="map/ellip_ratio"    value="0.02" type="double"/>
    <param name="map/poly_ratio"     value="0.01" type="double"/>

``


Examples:



![](docs/exp_all2.png)

![](docs/exp_cy2.png)

![](docs/exp_gate2.png)
