
#ifndef ns_AS-DTMAC_h
#define ns_AS-DTMAC_h

// #define DEBUG
//#include <debug.h>
#include <cmath>
#include "marshall.h"
#include <delay.h>
#include <connector.h>
#include <packet.h>
#include <random.h>
#include <arp.h>
#include <ll.h>
#include <mac.h>
#include <math.h>
#include <mobilenode.h>
#include <queue.h>
#include "wireless-phy.h"
#include "cmu-trace.h"
#include <vector>
//#include <list>
#include <time.h>
#include <sstream>
#include <string> 
#include <iostream>
#include <stdlib.h> 


using namespace std;

/* ======================================================================
                        AS-DTMAC Header and Parameters Definition
   ====================================================================== 
*/
        
// The number of bits required to represent a node id
#define NODE_ID_LEN      (ceil(log(frame_len_)/log(2)))

// The number of bits required to represent a time slot used by a one-hop neighbour
#define SLOT_IDX_LEN      (ceil(log(frame_len_)/log(2)))

#define CURRENT_TIME    Scheduler::instance().clock() 

#define VEHICLE      0
#define RSU          1


#define NO_FUT_SLOT_SCHEDULED        -1 

#define NO_PREV_SLOT_ACCESSED        -1
     
#define TRANSMISSION_RANGE        250

#define BURST_LENGTH 9

#define DIRECTION_UPDATE_INTERVAL     0.001  //seconds

#define AREA_UPDATE_INTERVAL     0.001  //seconds

#define eps slot_duration_/10
//#define DEFAULT_PERIODIC_QLIM    inf
//#define DEFAULT_EVENT_QLIM       inf

int  nbr_veh_slot_access = 0;
int  nbr_already_accessing_slot = 0;
int  nbr_first_accessing_slot  = 0;
int  nbr_node_in_slot = 0;
int  nbr_slot_free = 0;
int  top = 0;
int  one_time = 0;


enum NodeDirection {
	LEFT,
	RIGHT,
	FIXED
};

enum AS-DTMAC_pkt_type {
	TYPE2,	
	TYPE1,
	BURST
};

struct hdr_AS-DTMAC {
	
	// The offset of this header
	static int offset_;

	// Define the function access(p)
	inline static hdr_AS-DTMAC* access(const Packet* p) {
		return (hdr_AS-DTMAC*) p->access(offset_);
	};

	//My current direction
	NodeDirection mydirection_;
	// The index of this fragment
	int frag_num_;
	int seti_;
	// The mac addresses of the source node and the intended one-hop destination node
	int src_mac_addr_;
	int dest_mac_addr_;
	int busrt_dec_nbr;
	int frame;
	

	// The type of the AS-DTMAC packet transmitted (i.e. type 1 or type 2)
	AS-DTMAC_pkt_type pkt_type_;
	
	// The set of IDs corresponding to the time slots occupied by the neighbours. 
	
	int* one_hop_;

};


// struct burst_hdr_AS-DTMAC {
	
// 	// The offset of this header
// 	static int offset_;

// 	// Define the function access(p)
// 	inline static burst_hdr_AS-DTMAC* access(const Packet* p) {
// 		return (hdr_AS-DTMAC*) p->access(offset_);
// 	};

// 	int src_mac_addr_;
// 	int busrt_dec_nbr;

// };

// Initialize the static variable offset_
int hdr_AS-DTMAC::offset_ = 0;

/* ======================================================================
                                 AS-DTMAC Timers
   ====================================================================== 
*/
class AS-DTMAC;
class AS-DTMAC_SlotTimer : public Handler {

public:
	AS-DTMAC_SlotTimer(AS-DTMAC*, double);
	virtual void start(Event* , double);
	virtual void handle(Event*);
	inline double timer_slot_duration(void) { 
	 	return timer_slot_duration_; 
	}

	AS-DTMAC 	*mac;
	double   timer_slot_duration_; // The slot duration
	
};

class Direction_UpdateTimer : public Handler {

public:
	Direction_UpdateTimer(AS-DTMAC*, double);
	virtual void start(Event* , double);
	virtual void handle(Event*);
	inline double timer_direction_update(void) { 
	 	return timer_direction_update_; 
	}
	
	
	AS-DTMAC 	*mac_;
	double   timer_direction_update_; // The direction update interval
	bool initialized_;
	
};

/*class AS-DTMAC_DirectionUpdateTimer : public Handler {

public:
	AS-DTMAC_DirectionUpdateTimer(AS-DTMAC*, double);
	virtual void start(Event* , double);
	virtual void handle(Event*);
	inline double direction_update_interval(void) { 
		return direction_update_interval_; 
	}
	
	AS-DTMAC 	*mac;
	double   direction_update_interval_; 
	bool initialized_;
	bool areaInitialized_;
	
};*/


class AS-DTMAC_AreaUpdateTimer : public Handler {

public:
	AS-DTMAC_AreaUpdateTimer(AS-DTMAC*, double);
	virtual void start(Event* , double);
	virtual void handle(Event*);
	inline double area_update_interval(void) { 
		return area_update_interval_; 
	}
	
	AS-DTMAC 	*mac;
	double   area_update_interval_;
	bool initialized1_; 
	
};


/*
   ======================================================================
                                 Vehicle Position
   ====================================================================== 
*/


class slotState {

public:
	int slots[90];
	
	void slotInitialiser()
	{
		for(int i=0;i<90;i++)
		slots[i]=0;
	}	

	void setSlotState (int i)
	{
		slots[i]=1;
	}

};



class Position {

public:
	double XPos;
	double YPos;
	Position ()
	{
		XPos=0.0;
		YPos=0.0;
	}
	Position (double x, double y)
	{
		XPos=x;
		YPos=y;
	}

};

/* ======================================================================
                                 AS-DTMAC Queues
   ====================================================================== 
*/
class AS-DTMACQueue {
	friend class AS-DTMAC;

public: 
	AS-DTMACQueue();
	~AS-DTMACQueue();

protected:
	void enque(Packet*);
	void enque_front(Packet*); // enque packet at the fron of the queue
	Packet* deque();
	int length();
	int length_bytes();
	PacketQueue *q_;
	bool drop_front_;	
	int qlim_;          // maximum allowed pkts in queue 
};

/* ======================================================================
                                The AS-DTMAC class
   ====================================================================== 
*/

class AS-DTMAC : public Mac {
	friend class AS-DTMAC_SlotTimer;
	friend class AS-DTMAC_Mini_SlotTimer;
	friend class AS-DTMAC_AreaUpdateTimer;

  
public:

	AS-DTMAC();
	~AS-DTMAC();
	// Packet handling functions
	void recv(Packet *p, Handler *h);
	void sendUp(Packet* p);
	void slot_handler(Event *e);
	void direction_update_handler(Event *e);
	bool send(int s);
	bool send_burst_signal(int mini_s);
	int  burst_convert(long long n);
	void generate_burst(int *tab);
	/*void black_node_list(int node);
	bool check_node(int node);
	void Empty_black_node_list();*/
	
	// Tracing function
	void printparam();

	// other funcions
	int hdr_len(Packet*); // To find the length of the AS-DTMAC header in bytes
	                      
	int hdr_len(int*); // The same as the previous function but is takes the set
		                   // of neighbours N_ as an input.
	

protected:

	
	int frame_len_;
	int left_slots_;
	int right_slots_;
   	int s0_,s1_,s2_;
	int set_;
	int xmax_,ymax_;
	

	// The seed for the random number generator
	int seed_;

	// Parameters required to get the node position
	MobileNode* node_;
	double Xnew,Xprev;
	double Ynew,Yprev;
	double Znew,Zprev;
	double speed;
	double Xnew1,Xprev1;
	double Ynew1,Yprev1;
	double Znew1,Zprev1;
	int currentframe;
	double Xintersec;
	double Yintersec;
	double Zintersec;
	double dist;
   	int  xi;
	int ai,bi;
	int nnb;
	// Define the periodic and event-driven drop-tail queues and their 
	// callback pointers
	AS-DTMACQueue PeriodicQ_;
	AS-DTMACQueue EventQ_;

	// Define the size of the queues and the Drop front object
	int PeriodicQ_lim_;
	int EventQ_lim_;
	int PeriodicQ_dropfront_;
	int EventQ_dropfront_;

	// The sources of the periodic and event driven safety messages
	packet_t periodic_packet_type_;
	packet_t event_packet_type_;

	// Pointer to the trace file
	static FILE *TraceFile_;
	//static FILE *Trace_;
	static FILE *Reception_;
	static FILE *AccessCollision_;
	static FILE *AccessCollision1_;
	//static FILE *SlotAccess_;
	//static FILE *SlotAccess1_;
	static FILE *MergingCollision_ ;
	//static FILE *NoAccessCollisionProb_;
	static FILE *Neighboring_;
	static FILE *DebitRelational_;
	static FILE *Time_Access;
	static FILE *Node_Acces_Slot;
	//static FILE *NOSlotFound_;

private:
	
	int command(int argc, const char*const* argv);

	
	static Position* NodePosition_;
	static int* NodeArea_;
	static int* SlotAcc_;
	static int* slotNum_;
	static int* NodeSend_;
	static slotState* tabSlotState;
	static NodeDirection* VehicleDirection_;
	static int* RoadMap_;
	static int* slotStatus_;
	static int* slotStatus1_;
	static int *Neigh_number_;
	static int *Neigs_number_;
	double ts = 0;
	double t = 0;
	double Time_Access_slot =0;
	int number_nodes_;
	// The node id. Note that the node will have a number of node_id_ equal to
	// the number of time slots that it access per frame.
	int* node_id_;



	// The node direction
	NodeDirection direction_;

	// The mac state
	MacState	macstate_; // state_is equal to MAC_SEND during the time slot when 
	                    // the node is transmitting a packet and is reset to 
	                    // MAC_IDLE after the transmission at the end of the time slot.

	int frame_number;

	// containing the indece of the time slots that the node is sucessfully accessing. If acc_slot_number = -1, 
	// that means that the node hasn't accessed its ith time slot yet or has detected a collision on its 
	// ith slot and didn't attempt to access a new one. 
	int acc_slot_number_;

	// containing the indece of the time slot that the node decides to access in the future
	// when it is attempting to acquire a time slot. If fut_slot_number != -1, that means that the node
	// is successfully acquiring its time slot or attempted to acquire its time slot and hasn't detected
	// a collision yet.
	int fut_slot_number_;



	//Burst sequence
    int burst_v_seq[BURST_LENGTH];
	int burst_value = 0;
    int burst_generation = 1;
    int node_out_of_competition = 1;
    //std::vector<int> node_out_of_competition;
	int start_ = 0;


    
	
	
	//  indicating whether the node successfully acquired its time slots or not. The entry succ_acc 
	// is originally 0 (false) and =1 (true) if the node successfully acquired its time slot.
	bool succ_acc_;

	// A boolean variable indicating whether or not the node has detected collision on its time slot. If
	// coll = true, this means that the node has detected collision on its time slot. The 
	// coll is reset to false in the function update_fut_slot once the node schedule to access a future 
	// time slot
	bool coll_;

	// A set containing the mac addresses of all neighbours. Note that, each node has  a unique mac address and the mac addresses start from 0.
	int* neigh_mac_addr_;
	
	// The slot timer
	AS-DTMAC_SlotTimer  timer_;

	// The mini slot timer
	//AS-DTMAC_DirectionUpdateTimer  direction_update_timer_;

	// The direction update timer
	Direction_UpdateTimer direction_update_timer_;

	// The area update timer
	AS-DTMAC_AreaUpdateTimer area_timer_;

	// The COMING slot idx and slot_duration_
	int slot_;
	int allnodesend;
	//The slot duration
	double slot_duration_;
	virtual void getSpeed();

	//Mini slot
	int mini_slot_;
	double Mini_slot_duration_;


	// A dummy event
	Event dummy0_,dummy1_,dummy2_;
	
	// AS-DTMAC functions
	//================
	void Route_to_Qs(Packet* p);        // To route the packet coming from the IFQ to the suitable Q
	void update_direction(Event* e);   // Update the direction of te node (i.e. left, right) 
	                                   // every DIRECTION_UPDATE_INTERVAL
	void update_area(Event* e);     // Update the area of te node (i.e. left, right) 
	                                   // every AREA_UPDATE_INTERVAL
	void getmovspeed(double *);
	void update_fut_slot();
	int detectTwoAccess(int nd);
 	// These two functions are used to set the variable coll_ if 
    // the packet p indicates that a collision is detected at time slot of the node
	void detect_collision(Packet* p);
	void detect_merging_collision(Packet* p);
	int nbr_node_send();
	
	double Euclidistance(int i, int j); // Calculate the euclidienne distance between two vehicles

	double AccessColissionProbability(); // Calculate the total access collision probability

	int numberOfNeighb(int i);
	
	
};

Position* AS-DTMAC::NodePosition_= new Position[450];
int* AS-DTMAC::NodeArea_= new int[450];
int* AS-DTMAC::SlotAcc_= new int[450];
int* AS-DTMAC::slotNum_= new int[450];
int* AS-DTMAC::NodeSend_= new int[450];
int* AS-DTMAC::slotStatus_= new int[100];
int* AS-DTMAC::slotStatus1_= new int[100];
int* AS-DTMAC::Neigh_number_=new int[450];
int* AS-DTMAC::Neigs_number_=new int[450];

slotState* AS-DTMAC::tabSlotState=new slotState[2000];

NodeDirection* AS-DTMAC::VehicleDirection_= new NodeDirection[450];
int* AS-DTMAC::RoadMap_= new int[450];

FILE* AS-DTMAC::TraceFile_ = fopen("TRACEFILE.txt","w");
//FILE* AS-DTMAC::NOSlotFound_ = fopen("NOSlotFound.txt","w");
FILE* AS-DTMAC::Reception_ = fopen("Reception.txt","w");
//FILE* AS-DTMAC::Trace_ = fopen("TRACE.txt","w");
//FILE* AS-DTMAC::AccessCollision_ = fopen("AccessCollision.txt","w");
FILE* AS-DTMAC::AccessCollision1_ = fopen("Collision.txt","w");
FILE* AS-DTMAC::DebitRelational_ = fopen("DebitRelational.txt","w");
FILE* AS-DTMAC::Time_Access = fopen("Time_Access.txt","w");
//FILE* AS-DTMAC::SlotAccess_ = fopen("SlotAccess.txt","w");
//FILE* AS-DTMAC::Neighboring_ = fopen("Neighboring.txt","w");
//FILE* AS-DTMAC::MergingCollision_ = fopen("MergingCollision.txt","w");
//FILE* AS-DTMAC::NoAccessCollisionProb_ = fopen("AccessCollisionProbability.txt","w");
//FILE* AS-DTMAC::SlotAccess1_ = fopen("SlotAccess.txt","w");
//FILE* AS-DTMAC::Time_Access = fopen("Time_Access.txt","w");
FILE* AS-DTMAC::Node_Acces_Slot = fopen("Vehicul_Access_slot.txt","w");
#endif  // AS-DTMAC.h
