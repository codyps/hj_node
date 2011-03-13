# half jackal #

The setup is like so: 1 Arduino drives _2_ of the following
motor+motor controller setups. Motor with dual phase encoders is attached
to a "simple h-bridge". This gives us 2 encoder signals and current
information for each motor.

Motor speed is set via the usart interface, packets are framed in a hdlc
like manner, and CRC-CCITT protected.

Due to the physical layout of the robot 1 motor controler (half_jackal/hj)
will controll a pair of left/right wheels such that a hj controls the 2
front wheels and a second controls the 2 rear wheels.

## Cloning ##

 My dear sir, upon examining your repository of code I noticed that the depth
 of submodules is like so:

 hj_node -> half_jackal -> muc -> penny.

 Surely, you realize the significant cruelty you impose upon your users?

### Recomendation ###

If cloning initially, do:
    git clone --recursive <repo>
If that fails to reach you in time, do:
    git clone <repo>
    git submodule init
    git submodule foreach --recursive git submodule init

When pulling at a top level, do:
    git pull --recurse-submodules
    git submodule update
    git submodule foreach --recursive git submodule update

If one of you poor blokes happens to be in as bad a state as myself and wishes
to quickly update their chain of submodules, do:
    git pull --recurse-submodules
    git submodule foreach --recursive git submodule init '&&' git submodule \
        update '&&' git checkout master '&&' git pull

Of note:
	adding the flag '--recurse-submodules' doesn`t really recurse.


