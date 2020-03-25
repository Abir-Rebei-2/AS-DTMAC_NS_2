#======================================================================
# Define DTMAC Parapmeters
# ======================================================================
Mac/DTMAC set frame_len_ 100 		
Mac/DTMAC set xmax_ 16000
Mac/DTMAC set ymax_ 16000
Mac/DTMAC set number_nodes_ 44
Mac/DTMAC set s0_ 34
Mac/DTMAC set s1_ 33
Mac/DTMAC set s2_ 33
Mac/DTMAC set PeriodicQ_lim_ 10000
Mac/DTMAC set EventQ_lim_ 10000
Mac/DTMAC set PeriodicQ_dropfront_ 1
Mac/DTMAC set EventQ_dropfront_ 1
Mac/DTMAC set EventQ_dropfront_ 1
Mac/DTMAC set seed_ 134586 ; # that is the default value   26876 10432 1230 187

#12 56 47 34 57 59 61 35 20 28 40 46 67 37 66 27 69 33 26 68

#

#======================================================================
# Define the Link Layer Parapmeters
# ======================================================================
LL set delay_  0 ;# The default is  25us

#======================================================================
# PBC Parapmeters
# ======================================================================

Agent/PBC set payloadSize 600
Agent/PBC set periodicBroadcastInterval 3 ;# don't use ms (i.e. use 0.025 not 25ms) 
Agent/PBC set periodicBroadcastVariance 0
Agent/PBC set modulationScheme 1
#$agent_($i) set Pt_ 1e-4


#======================================================================
# Wireless PHY Parapmeters
# ======================================================================
Phy/WirelessPhy set CPThresh_ 10.0
Phy/WirelessPhy set CSThresh_ 1.42681e-12
Phy/WirelessPhy set RXThresh_ 1.42681e-12
Phy/WirelessPhy set Rb_ 2*1e6

Phy/WirelessPhy set Pt_ 0.0334 

Phy/WirelessPhy set freq_ 914e+6 
Phy/WirelessPhy set L_ 1.0

# ======================================================================
# Define options
# ======================================================================
set val(chan)           Channel/WirelessChannel    ;# channel type
set val(prop)           Propagation/FreeSpace      ;# radio-propagation model
set val(netif)          Phy/WirelessPhy            ;# network interface type
set val(mac)            Mac/DTMAC                  ;# MAC type
set val(ifq)            Queue/DropTail             ;# interface queue type
set val(ll)             LL                         ;# link layer type
set val(ant)            Antenna/OmniAntenna        ;# antenna model
set val(ifqlen)         50                         ;# max packet in ifq
#set val(nn)             10                          ;# number of mobilenodes
set val(rtg)            DumbAgent
set val(x)              17050   	;# X dimension of the topography
set val(y)              4000   	;# Y dimension of the topography
#set val(stop)          79

#
# Initialize Global Variables
#

set val(nn) [lindex $argv 1];
set val(stop) [lindex $argv 2];
set val(seed) [lindex $argv 3]
global defaultRNG
$defaultRNG seed $val(seed)

set ns_		[new Simulator]
set topo	[new Topography]
set tracefd     [open simple.tr w]
$ns_ trace-all $tracefd
$ns_ use-newtrace

$topo load_flatgrid $val(x) $val(y)
set god_ [create-god $val(nn)]
$god_ off

set chan [new $val(chan)]
$ns_ node-config -adhocRouting $val(rtg) \
                 -llType $val(ll) \
                 -macType $val(mac) \
                 -ifqType $val(ifq) \
                 -ifqLen $val(ifqlen) \
                 -antType $val(ant) \
                 -propType $val(prop) \
                 -phyType $val(netif) \
                 -channel $chan \
                 -topoInstance $topo \
                 -agentTrace OFF \
                 -routerTrace OFF \
                 -macTrace OFF \
                 -phyTrace OFF




for {set i 0} {$i < $val(nn) } {incr i} {
    set node_($i) [$ns_ node]
  
    set agent_($i) [new Agent/PBC]
    $ns_ attach-agent $node_($i)  $agent_($i)
    $agent_($i) PeriodicBroadcast ON
    $ns_ at $val(stop).0001 "$node_($i) reset";
    # Note that we don't need to start the PBC since it is an agent,
    # not a traffic generator, and its timer is automatically started
    # if periodicBroadcast is ON (see Ln 245 in pbc.cc)
    
    
   

}

set opt(sc) [lindex $argv 0];	

if { $opt(sc) == "" } {
    puts "*** NOTE: no scenario file specified."
    set opt(sc) "none"
} else {
    puts "Loading scenario file...$opt(sc)"
    source $opt(sc)
    puts "Load complete..."
}



$ns_ at $val(stop).0002 "puts \"NS EXITING...\" ; $ns_ halt"
$ns_ at $val(stop).0003 "$ns_ flush-trace"
puts "Starting Simulation..."
$ns_ run


