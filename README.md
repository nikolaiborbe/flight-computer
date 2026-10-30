# Fossekall Flight Computer

## data organization
Groundstation is all the datatypes which configured correctly
to be sent down to gui/groundstation. This is because locally we have
to wrap everything in a seqlock which makes our data larger
and slower to send over the network. So if you want to change a reader or writer type
you go to data/GroundStation.hpp.

## Structure
The current project uses a singular entrypoint which is App.cpp. Here all the crtical systems the threads are dependent on
are initalized. This is to ensure that if we are able to pass App.cpp the system is functional and operational. The bugs that remain
will mainly be logic errors in the thread code. Currently there are some optional parts of the initalization faze mostly meant for launch
like waiting for a response for every pcan device to ensure that everything is working. Other then that it sets up network.

## Seqlock
Seqlock uses a descriptive reader/writer system. There can only be a singular writer
at any given time. And in practice you can use as many readers as you would like. We use seqlock as it is faster
then the standard mutex solution and gives us more flexibility in how data is handled. For example this has allowed us to 
detect stale data coming from the any sensor and ensure that both the kalman filter and fsm states are aware of it to
such to handle the data appropietly. 

Currently the writers are setup in Data/Cells.cpp. Where all the appropriate writers are setup. Which is then again setup in app.cpp
since the threads are dependent on the writer system to work correctly. Furthermore all writers are passed down to the threads with pointers
this is ensured to be legal by the static assert at the start of the app.cpp. This is just cleaner in my opinion then having them in global
namespace indicating that anybody can use them. This way they are localized, eventually we could expand this system by giving each thread more
spesific information.

Currently all seqlock writers have been defined and there is no need
to define any more. The gui is missing the thermocouple
data and status symbol

## What is still missing for launch?
1. testing that pressure, thermocouple and gnss boards communicate over can
2. testing solenoid and servos being able to be controlled over can.
3. finishing stim driver
4. Add thermocouple to gui

## Some nice to haves for flight
1. finish nandflash thread
2. Create stale data handlers in fsm for emergency situations
