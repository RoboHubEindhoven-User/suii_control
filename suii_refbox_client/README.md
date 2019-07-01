# Suii Refbox Client

**Configuration with the Refbox**

The refbox client should be running on the same network as the refbox.

In launch/suii_refbox_client.launch, line 11:

```<param name="host_name" type="string" value="192.168.0.103"/>```

The value tag should be changed to the IP of the computer running the Refbox. 

Double check that the ports are correct on Line 18:

```<param name="public_port" type="int" value="5444"/>```

And line 27:

```<param name="team_port" type="int" value="5468"/>```

The two ports above are information that the Refbox will give to us.

**Configuration with Suii's Task Manager**

This package is the one publishing to Task Manager, please also check their ROS Master.
