# Crazyflies API Quickstart
This file is written by Nick Zhang to provide a short intro to some of Crazyflies' APIs

## CRTP
Crazyflies RealTime Protocol is for communication with crazyflies

### RX
A callback function should be registered with `crtpRegisterPortCB` for each and every port number. The packets can be further classified with link number, which are port dependent
```
/**
 * Register a callback to be called for a particular port.
 *
 * @param[in] port Crtp port for which the callback is set
 * @param[in] cb Callback that will be called when a packet is received on
 *            'port'.
 *
 * @note Only one callback can be registered per port! The last callback
 *       registered will be the one called
 */
int crtpReceivePacket(CRTPPort taskId, CRTPPacket *p);
```


### TX
To send a packet, construct a `CRTPPacket` struct and use `crtpSendPacket` to send

```
/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the oldest lowest priority packet is dropped
 *
 * @param[in] p CRTPPacket to send
 */
int crtpSendPacket(CRTPPacket *p);
```

## Modifications

### Stage 1 (recording)
on firmware:
new port number 13 -> link 1: motor thrust feedback
sendpacket -> stabilizer.c

on host pc:
script to control cf with joystick with setpoint, will connect to Tao's Optitrack stream and save state + thrust

### Stage 2 (direct control)
new port number 13 -> link 2: motor thrust command
new callback on crtp port 13
remove stabilizer/estimater etc

on host pc:
script to control cf with joystick with setpoint.


