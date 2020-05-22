
#include "AS-DTMAC.h"

/* ======================================================================
                      AS-DTMAC Slot Timer Function Definition
   ====================================================================== */

AS-DTMAC_SlotTimer::AS-DTMAC_SlotTimer(AS-DTMAC* m=0, double s=0) {
	mac = m;
	timer_slot_duration_ = s;
}


//Rajouter AS-DTMAC_S

void AS-DTMAC_SlotTimer::start(Event *e, double time) {
	Scheduler &s = Scheduler::instance();
        s.schedule(this, e, time);
}

void AS-DTMAC_SlotTimer::handle(Event *e) {
			mac->slot_handler(e); 
}

/* ======================================================================
                      AS-DTMAC Direction Update Timer Definition
   ====================================================================== */

Direction_UpdateTimer::Direction_UpdateTimer(AS-DTMAC* m=0, double s=0) {
	mac_ = m;
	timer_direction_update_ = s;
	initialized_ = 0;
}

void Direction_UpdateTimer::start(Event *e, double time) {
	Scheduler &s = Scheduler::instance();
	s.schedule(this, e, time);
}

void Direction_UpdateTimer::handle(Event *e) {
	mac_->direction_update_handler(e); 
}



/* ======================================================================
                      AS-DTMAC Direction Update Function Definition
   ====================================================================== */

/*AS-DTMAC_DirectionUpdateTimer::AS-DTMAC_DirectionUpdateTimer(AS-DTMAC* m=0, double s=0) {
	mac = m;
	direction_update_interval_ = s;
	initialized_ = 0;
	areaInitialized_ = 0;
}


void AS-DTMAC_DirectionUpdateTimer::start(Event *e, double time) {
	Scheduler &s = Scheduler::instance();
	s.schedule(this, e, time);
}

void AS-DTMAC_DirectionUpdateTimer::handle(Event *e) {
	mac->update_direction(e);  
}*/


/* ======================================================================
                      AS-DTMAC Area Update Function Definition
   ====================================================================== */


AS-DTMAC_AreaUpdateTimer::AS-DTMAC_AreaUpdateTimer(AS-DTMAC* m=0, double s=0) {
	mac = m;
	area_update_interval_ = s;
	initialized1_ = 0;
}

void AS-DTMAC_AreaUpdateTimer::start(Event *e, double time) {
	Scheduler &s = Scheduler::instance();
	s.schedule(this, e, time);
}

void AS-DTMAC_AreaUpdateTimer::handle(Event *e) {
	mac->update_area(e);   
}

/* ======================================================================
                      AS-DTMAC Queue Update Function Definition
   ====================================================================== */
AS-DTMACQueue::AS-DTMACQueue(){
	q_ = new PacketQueue; 
	drop_front_ = 1;
}

AS-DTMACQueue::~AS-DTMACQueue() {
	delete q_;
}

void AS-DTMACQueue::enque(Packet* p) {
	
	if ( (q_->length() + 1) >= qlim_) // i.e. the queue would overflow if we added this packet
	{
		if (drop_front_) // remove from head of queue  
		{ 
			q_->enque(p);
			Packet *pp = q_->deque();
			Packet::free(pp);
		} 
		else 
		{
			Packet::free(p);
		}
	} 
	else 
	{
		q_->enque(p);
	}
}

void AS-DTMACQueue::enque_front(Packet* p) {

	if ( (q_->length() + 1) >= qlim_) // i.e. the queue would overflow if we added this packet
	{
		if (drop_front_) // remove from head of queue  
		{ 
			Packet::free(p);
		} 
		else 
		{
			q_->remove(q_->tail());
			q_->enqueHead(p);
		}
	} 
	else 
	{
		q_->enqueHead(p);
	}
}

Packet* AS-DTMACQueue::deque() {
	return q_->deque();
}

int AS-DTMACQueue::length() {
	return q_->length();
}

int AS-DTMACQueue::length_bytes() {
	return q_->byteLength();
}
/* ========================================================================================
          Mapping the class AS-DTMAC and the struct hdr_AS-DTMAC to the TCL counterparts 
   ======================================================================================== */

// Create an instance of hdr_AS-DTMAC to obtain a reference for the static 
// variable hdr_AS-DTMAC::offset_
hdr_AS-DTMAC obj_to_create_offset;

static class AS-DTMACClass : public TclClass {
public:
	AS-DTMACClass() : TclClass("Mac/AS-DTMAC") {}
	TclObject* create(int, const char*const*) {
		return (new AS-DTMAC());
	}
} class_AS-DTMAC;


class AS-DTMACHeaderClass : public PacketHeaderClass {
public:
	AS-DTMACHeaderClass() : PacketHeaderClass("PacketHeader/AS-DTMAC", sizeof(hdr_AS-DTMAC)) {
		bind_offset(&hdr_AS-DTMAC::offset_);
     }
    void export_offsets() {
         field_offset("one_hop_", OFFSET(hdr_AS-DTMAC, one_hop_));
     };
} class_AS-DTMAChdr;


/* ======================================================================
                    The Class AS-DTMAC Function Definition 
   ====================================================================== */

AS-DTMAC::AS-DTMAC():Mac() {
	
	// Binding the corresponding variables in the TCL and C++ classes
	bind("frame_len_", &frame_len_);
	bind("xmax_", &xmax_);
	bind("ymax_",&ymax_);
	bind("number_nodes_",&number_nodes_);
	bind("s0_", &s0_);
	bind("s1_", &s1_);
	bind("s2_", &s2_);
	bind("seed_", &seed_);
	bind("PeriodicQ_lim_", &PeriodicQ_lim_);
	bind("EventQ_lim_", &EventQ_lim_);
	bind_bool("PeriodicQ_dropfront_", &PeriodicQ_dropfront_);
	bind_bool("EventQ_dropfront_", &EventQ_dropfront_);
	bind("periodic_packet_type_", &periodic_packet_type_);
	bind("event_packet_type_", &event_packet_type_);





	
	// Set the seed for the random number generator and open the necessary files 
	// for writing when the first node is created
 
    

	// Get a pointer to a MobileNode with address_ equal to index_
	node_ = (MobileNode*) Node::get_node_by_address(index_);
	
	// Define the qlim_ for the queues. Those are the default values and they
	// can be configured from tcl
	PeriodicQ_.qlim_ = PeriodicQ_lim_;
	EventQ_.qlim_ = EventQ_lim_;
	PeriodicQ_.drop_front_ = PeriodicQ_dropfront_; 
	EventQ_.drop_front_ = EventQ_dropfront_;

	
	neigh_mac_addr_ = new int[frame_len_];
	
	for(int i=0; i<frame_len_; i++) 
	{
		neigh_mac_addr_[i] = -1;
	}

	for(int i=0; i<number_nodes_; i++) 
	{
		RoadMap_[i] = 0;
	}
	for(int i=1; i<number_nodes_; i++) 
	{
		RoadMap_[i] = RoadMap_[i-1]+TRANSMISSION_RANGE;
	}
	
	for(int i=0; i<number_nodes_; i++) 
	{
		slotNum_[i] = -1;
	}

	for(int i=0; i<frame_len_; i++) 
	{
	    slotStatus_[i]=0;
	}

	/*for(int i=0;i<frame_len_;i++)
	{
	tabSlotState[0].slots[i]=0;
	}*/
	
	// Initialize the rest of the variables to zeros 
	//xi=0;
	//set_=0;
	ai = 0;
	frame_number = 0;
	bi = 0;
	acc_slot_number_ = NO_PREV_SLOT_ACCESSED;
	fut_slot_number_ = NO_FUT_SLOT_SCHEDULED;
	succ_acc_= 0;
	node_id_ = new int[frame_len_];
	SlotAcc_[index_] = 0;
	direction_ = FIXED;	
	coll_= 1;
	Xintersec = 0;
	Yintersec = 0;
	Zintersec = 0;	
	currentframe = -1;

	// Initialize the slot index 
	slot_ = 0; 
	
	//Moi : Initialize the mini slot index 
	mini_slot_ = 0; 
	
	// Initialize the macstate_
	macstate_ = MAC_IDLE;
	allnodesend = 0;
	for(int i = 0; i<number_nodes_; i++) 
	{
		NodeSend_[i] = 0;
	}

	for(int i=0;i<number_nodes_;i++)
	Neigh_number_[i] = 0;

	for(int i=0;i<number_nodes_;i++)
	Neigs_number_[i] = 0;

	

	// The slot duration in sec 
	slot_duration_ = 0.001;

	//Moi: The mini slot duration in sec 
	Mini_slot_duration_ = 0.000025;

	
		//  printf("Je suis vehicle %d burst ", index_);
	srand(time(NULL));
	
		
	// Initialize the slot timer and start it
	timer_.mac = this;
	timer_.timer_slot_duration_ = Mini_slot_duration_;
	timer_.start(&dummy0_,Mini_slot_duration_); // the first slot duration is left for initializing the direction
	                                       // i.e. slot#0 starts at time slot_duration_ (NOT time 0)

	
	// Initialize the direction timer
	direction_update_timer_.mac_ = this;
	direction_update_timer_.timer_direction_update_ = slot_duration_;
	direction_update_timer_.start(&dummy1_,0);
	
	
	
	// Initialize the direction timer
	/*dir_timer_.mac = this;
	dir_timer_.direction_update_interval_ = DIRECTION_UPDATE_INTERVAL;
	dir_timer_.start(&dummy2_,0);*/

	// Initialize the area timer
	area_timer_.mac = this;
	area_timer_.area_update_interval_ = slot_duration_;
	area_timer_.start(&dummy2_,0); 

	double xcord,ycord,zcord;
	node_->getLoc(&xcord,&ycord,&zcord);
	xi=xcord/TRANSMISSION_RANGE;

}


AS-DTMAC::~AS-DTMAC() {
	
		if (TraceFile_)
		{
			fclose (TraceFile_);
		}
		/*if (Trace_)
		{
			fclose (Trace_);
		}
		/*if (AccessCollision_)
		{
			fclose (AccessCollision_);
		}*/
		if (AccessCollision1_)
		{
			fclose (AccessCollision1_);
		}
		
		/*if (MergingCollision_)
		{
			fclose (MergingCollision_);
		}*/

		/*if (NoAccessCollisionProb_)
		{
			fclose (NoAccessCollisionProb_);
		}
		/*if (SlotAccess1_)
		{
			fclose (SlotAccess1_);
		}*/
		if (Reception_)
		{
			fclose (Reception_);
		}
		

		if (DebitRelational_)
		{
			fclose (DebitRelational_);
		}

			if (Time_Access)
		{
			fclose (Time_Access);
		}
		
		/*if (NOSlotFound_)
		{
			fclose (NOSlotFound_);
		}*/

}

/*void AS-DTMAC::printparam() {
	
	if ((NOW<54)&&(NOW>50))
	{
	fprintf(TraceFile_,"Simulation time: %g \n", NOW);
	
	fprintf(TraceFile_,"The start of time slot %d for node %d \r\n", slot_, index_);
	
	fprintf(TraceFile_,"The set neigh_mac_addr_ =");
	for(int i=0; i<frame_len_; i++)
	  fprintf(TraceFile_," %d, ", neigh_mac_addr_[i]);
	fprintf(TraceFile_,"\r\n");
	
	fprintf(TraceFile_,"The fut_slots =");
	fprintf(TraceFile_," %d, ", fut_slot_number_);
	fprintf(TraceFile_,"\r\n");

	fprintf(TraceFile_,"The acc_slots =");
	fprintf(TraceFile_," %d, ", acc_slot_number_);
	fprintf(TraceFile_,"\r\n");
	}

}*/

int AS-DTMAC::command(int argc, const char*const* argv) {
	if (argc == 2)
	{
		if(strcmp(argv[1],"attach-trace-file")==0)
		  {
			  if (!TraceFile_)
				  TraceFile_ = fopen(argv[2],"w");
			  return TCL_OK;
		  }
	}
	return Mac::command(argc, argv);
}

void AS-DTMAC::recv(Packet* p, Handler* h) {
	// Get a pointer to te common header 
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
	// Send to the upper layer the packet coming from the PHY layer
	if (ch->direction() == hdr_cmn::UP) {
		p_info packetinfo;
	//if((NOW>76)&&(NOW<83))
	//fprintf(BroadcastCoverage_,"r	%.3g	%d	%d\n",NOW, vh->src_mac_addr_,index_);
		sendUp(p);
	}
	// Send to the suitable queue the packet coming from the IFQ 
	else if (ch->direction() == hdr_cmn::DOWN) 
	{
		p_info packetinfo;
		// Route the packet to the suitable Q
		Route_to_Qs(p);
		// Unblock the IFQ
		h->handle((Event*) 0);
	

	}
vehicule=vh->src_mac_addr_ ;
\\determine the slot of the sender	
for (int i=0; i<frame_len_ ; i++)
{
if (neigh_mac_addr_[i] == vehicule)
{
s_init = i;

}
break;
}
//insert the test here and affect the value of relay slot
if (s_init < 34)
   {
	relay_slot=34;
//fprintf(Routage_,"La competition deroulera dans le slot 34 \n");
   }
   else if ( (34 < s_init) and (s_init < 67) )
   {
	relay_slot=67;
//fprintf(Routage_,"La competition deroulera dans le slot 67 \n");
   }
   

   else
   {
	relay_slot=0;
//fprintf(Routage_,"La competition deroulera dans le slot 0 \n");
   }
	
}

double AS-DTMAC::AccessColissionProbability()
{

	double Pnac=0;
	int Ai=0;
	int asi=0;


		for(int i=0;i<number_nodes_;i++)
		{
		 if ((NodeArea_[i]==xi)&&(SlotAcc_[i]==0))
		    {
			Ai++;
			//fprintf(AccessCollisionProb_, "%d ", i);
		    }
		}
		for(int i=ai;i<bi;i++)
		{
			if (neigh_mac_addr_[i]==-1)
				asi++;

		}
	//fprintf(AccessCollisionProb_, "\n");



		for(int i=0;i<asi;i++)
		{
		if((1-(double)1/(asi))!=0)
		 Pnac+=(double)1/(asi)*pow(1-(double)1/(asi),Ai-1);
		
		}

	// The non-access collision probability
	return Pnac;
	
}

void AS-DTMAC::Route_to_Qs(Packet* p) {

	struct hdr_cmn *ch = HDR_CMN(p);
	packet_t packet_type = ch->ptype();
	p_info packetinfo;
		

	if (packet_type == periodic_packet_type_) 
		{                                        
		//fprintf(TraceFile_,"Route_to_Qs\n");	                                   
			                                     
			PeriodicQ_.enque(p);
			

		//	fprintf(TraceFile_,"Route_to_Qs\n");
		/*	if (TraceFile_)
			{
				struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
				fprintf(TraceFile_,"%c %g %d %s %d %d \r\n",
					'+', NOW, index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_);
				if (PeriodicQ_.length() == 1) // i.e. the packet is the HOL
					fprintf(TraceFile_,"%c %g %d %s %d %d \r\n",
					    'h', NOW, index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_);
			}*/
		}
		else 
		{

			//fprintf(TraceFile_,"Route_to_Qs\n");
			EventQ_.enque(p);
			
		/*	if (TraceFile_)
			{
				struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
				fprintf(TraceFile_,"%c %g %d %s %d %d \r\n",
					'+', NOW, index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_);
				if (EventQ_.length() == 1) // i.e. the packet is the HOL
					fprintf(TraceFile_,"%c %g %d %s %d %d \r\n",
					    'h', NOW, index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_);
			}*/
		}
	
}

void AS-DTMAC::sendUp(Packet* p) {
	

	if (TraceFile_)
	{
		if (macstate_ != MAC_IDLE)
		{
			struct hdr_cmn* ch = HDR_CMN(p);
		    struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
			p_info packetinfo;
	     //  if (ch->error() == 1) // This packet is discarded due to a channel error rather than collision
			//    fprintf(TraceFile_,"%c %d %d %s %d %d %d %d\r\n",
			//	    'p', vh->src_mac_addr_, index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_, ch->size(), hdr_len(p));
		 //   else   // This packet is discarded due to collision rather than channel error     
			  //  fprintf(TraceFile_,"%c %d %d %s %d %d %d %d\r\n",
				//    'c', vh->src_mac_addr_, index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_, ch->size(), hdr_len(p));
		}
	}

	// If the node was transmitting during this time slot, send the received packet to 
	// the free list (the node cannot receive while transmitting)
	if (macstate_ == MAC_SEND)
	{
		
		Packet::free(p);
	}
	// If the node is not transmitting or receiving, that means this packet is 
	// the first packet that the node is receiving in the current slot. Hence, 
	// change the node state to MAC_RECV, move the packet p to pktRx and wait till
	// the end of the current time slot. The actual SendUp of the packet is done in
	// the function slot_handler based on whether or not more packets will be
	// received in the current slot and on the power of each extra packet received (i.e.
	// based on whether there is collision or capture)
	else if (macstate_ == MAC_IDLE)
	{
		pktRx_ = p;
		macstate_ = MAC_RECV;
	}
	else if (macstate_ == MAC_RECV)
	{
		// check the power between the packet being received (pktRx_) and the  packet which
		// has just arrived (p) to determine whether the node will perform capturing or not.
		// Note: it would be more reasonable to save all the packets received in the slot then 
		// compare the received power of pktRx with summation of the powers of all other packets,
		// however, the capturing is implemented in this way in the IEEE 802.11, so I didn't change
		// it for the comparison to be fair. This way in capturing assume that collisions can
		// only happen among two packets
		if(pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh)
		{
		
			Packet::free(p);
		}
		else // This means that the arriving packet will cause collision
		{
			macstate_ = MAC_COLL;
			Packet::free(p);		
		}
	}
	else if (macstate_ == MAC_COLL)
	{
		
		Packet::free(p);		
	}
}


bool AS-DTMAC::send(int s) { 
	// The parameter s is the number of the time slot 
	// that the node will be transmitting in.

	int overhead1 = hdr_len(neigh_mac_addr_);

				//the total number of neighboring vehilces 



    //fprintf(AccessCollision_,"At %g the number of frame is %d \n",NOW,frame_number);

	neigh_mac_addr_[slot_] = index_;
				
	// Set the macstate to send
	macstate_ = MAC_SEND;
				
	// Get a packet from the PeriodicQ
	pktTx_ = PeriodicQ_.deque();
			// If there is no packet which needs to be transitted, create a new packet, prepare
			// the header, pass it to the PHY layer.
	if (!pktTx_) 
	{
		printf("AS-DTMAC HAS CREATED A PERIODIC TYPE1 PACKET \n");
		// Create a new pointer p to Packet
		Packet* p = Packet::alloc();
				
		// Get a pointer to the Common header and AS-DTMAC header of pktTx_
		struct hdr_cmn* ch = HDR_CMN(p);
		struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
						
		// Update the AS-DTMAC header by including the  src_mac_addr, 
		// dest_mac_addr_, set_, direction, and the packet type. 
		vh->src_mac_addr_ = index_;
		vh->dest_mac_addr_ = BCAST_ADDR;  // BCAST_ADDR = -1 is defined in mac.h
		vh->pkt_type_ = TYPE1;
		vh->seti_ = set_;
		vh->mydirection_ = direction_;
		vh->one_hop_ = new int[frame_len_];

		for (int i=0; i<frame_len_; i++)   // include the set neigh_mac_addr_
			vh->one_hop_[i] = neigh_mac_addr_[i];
			ch->size() = hdr_len(p); // Update the size of the packet 
			// I am not sure if it should be changed to ch->size() += hdr_len(p);
					
			ch->ptype_ = PT_AS-DTMAC; // Set the type of the packet to show that it is generated by AS-DTMAC
					                // PT_AS-DTMAC is the number corresponding to te AS-DTMAC packet type.
			ch->uid() = -1; // We dont need a unique ID for every AS-DTMAC packets as they will not be considered
					                // in the calculation of delay or throughput, just the overhead.
			ch->timestamp() = NOW; // Set the time stamp of the packet
					
					

					// Update the trace file if defined
					/*if (TraceFile_)
					{
						p_info packetinfo;
						fprintf(TraceFile_,"%c %d %s %d %d %d %d \r\n",
							's', index_, packetinfo.name(ch->ptype()), ch->uid(), vh->frag_num_, ch->size(), hdr_len(p));
					}*/
					
			// Pass the packet to the PHY layer
			downtarget_->recv(p, this);
			return true;
	}
			
}

int AS-DTMAC::burst_convert(long long n)
{
    int decimalNumber = 0, i = 0, remainder;
    while (n!=0)
    {
        remainder = n%10;
        n /= 10;
        decimalNumber += remainder*pow(2,i);
        ++i;
    }
    return decimalNumber;
}

bool AS-DTMAC::send_burst_signal(int mini_slot_) { 
	
  //  neigh_mac_addr_[slot_] = index_;
				
	// Set the macstate to send
	macstate_ = MAC_SEND;

	// Get a packet from the PeriodicQ
	pktTx_ = PeriodicQ_.deque();
	// If there is no packet which needs to be transitted, create a new packet, prepare
	// the header, pass it to the PHY layer.


	 


	if (!pktTx_) 
	{
		printf("AS-DTMAC HAS CREATED A PERIODIC TYPE1 PACKET \n");
		// Create a new pointer p to Packet
		Packet* p = Packet::alloc();
				
		// Get a pointer to the Common header and AS-DTMAC header of pktTx_
		struct hdr_cmn* ch = HDR_CMN(p);
		struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);




		vh->dest_mac_addr_ = BCAST_ADDR;  // BCAST_ADDR = -1 is defined in mac.h
		vh->pkt_type_ = BURST;
		vh->src_mac_addr_ = index_;
        vh->busrt_dec_nbr = burst_value;
		vh->frame=frame_number;
       		 ch->ptype_ = PT_AS-DTMAC; // Set the type of the packet to show that it is generated by AS-DTMAC
					                // PT_AS-DTMAC is the number corresponding to te AS-DTMAC packet type.
		ch->uid() = -1; // We dont need a unique ID for every AS-DTMAC packets as they will not be considered
					                // in the calculation of delay or throughput, just the overhead.
		ch->timestamp() = NOW; // Set the time stamp of the packet
		
		downtarget_->recv(p, this);
		return true;
	}
			
}

int AS-DTMAC::hdr_len(Packet* p) { 
	// Note: This function ONLY considers the overhead of including the set one_hop_ in 
	// the header, i.e. the calculated length represents the number of bytes 
	// required to include the set one_hop_ and the time slot corresponding to each node_id_
	// in the set one_hop_ in the header of the packet p.
	
	// Get a pointer to the AS-DTMAC header in the packet p
	struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
	
	// If the packet is of type2, the len of the header is 0 (recall that we only consider 
	// the overhead caused by the set one_hop_)
	if (vh->pkt_type_ == TYPE2)
		return 0;

	// Declare a variable which indicates the number of neighbours included in the set N_
	int num_neighbours = 0; 
	
	// Calculate num_neighbours
	for (int i=0; i<frame_len_; i++)
		if (vh->one_hop_[i] != 0)
			num_neighbours++;

	return ceil(num_neighbours*(NODE_ID_LEN+SLOT_IDX_LEN)/8);
}

int AS-DTMAC::hdr_len(int* N) {
	// Declare a variable which indicates the number of neighbours included in the set N_
	int num_neighbours = 0; 
	
	// Calculate num_neighbours
	for (int i=0; i<frame_len_; i++)
		if (N[i] != 0)
			num_neighbours++;

	return ceil(num_neighbours*(NODE_ID_LEN+SLOT_IDX_LEN)/8);
}


/*void AS-DTMAC::black_node_list(int node){

	node_out_of_competition.push_back(node);
}

void AS-DTMAC::Empty_black_node_list(){

	/*for (int i = 0; i < (node_out_of_competition.size()-1); i++){

		node_out_of_competition.pop_back();
	}
	//fprintf(Time_Access,"je vide la black_node_list");

	while (!node_out_of_competition.empty())
	{

		node_out_of_competition.pop_back();		

	}
	fprintf(Time_Access,"the black_node_list is empty\n");

}

bool AS-DTMAC::check_node(int node){

	for (int i = 0; i < node_out_of_competition.size(); i++){

		if(node_out_of_competition[i] == node){

			return true;

		}

	}
	return false;
}*/

void AS-DTMAC::slot_handler(Event *e) {

	
	getSpeed();

	node_->getLoc(&Xnew1,&Ynew1,&Znew1);



	    if((frame_number == 0) && (slot_==99) && (one_time ==0)){
	    	one_time++;
            for(int i=0; i<frame_len_; i++){

	              if (slotStatus_[i]==0){
	        	     top++;
	              }


	        }
	    //fprintf(TraceFile_,"nbr de  node libre: %d \n", top);
        }



	if ((speed!=0)&&((Xnew1>=2000)&&(Xnew1<=4000)))
	{

		int prev_slot_idx = (slot_-1) + (frame_len_)*(slot_==0);

		if (macstate_ == MAC_IDLE)
		{
			if(mini_slot_ == 0)
			neigh_mac_addr_[prev_slot_idx] = -1;	
		    //fprintf(Reception_,"At %g: frame %d : slot %d : mini_slot_ %d : je suis vehicle %d,  j'ai detecté que le canal est libre \n", NOW, frame_number, slot_, mini_slot_, index_);
			
		}
		else if (macstate_ == MAC_SEND)
		{
			//fprintf(Reception_,"At %g: frame %d : slot %d : mini_slot_ %d : je suis vehicle %d,  send :\n", NOW, frame_number, slot_, mini_slot_, index_);
		}
		else if (macstate_ == MAC_COLL)
		{		
			
				if(mini_slot_ == 0)
				neigh_mac_addr_[prev_slot_idx] = -1;
				//fprintf(Reception_,"At %g: frame %d : slot %d : mini_slot_ %d : je suis vehicle %d,  j'ai detecté une collision \n", NOW, frame_number, slot_, mini_slot_, index_);
				
				if(mini_slot_ !=0)
				node_out_of_competition = 0;
				//black_node_list(index_);

			
				//fprintf(Reception_,"At %g: frame %d : slot %d : mini_slot_ %d : je suis vehicle %d,  je quite la compition \n", NOW, frame_number, slot_, mini_slot_, index_);

		}
		else if (macstate_ == MAC_RECV)  // i.e. the node has received one packet in the previous slot or has received more
			                             // than one packet but could capture one (will be stored in pktRx_). Note that, 
										 // when macstate_ = MAC_RECV, this does not mean that the packet pktRx_ is SUCCESSFULLY received
		{
			
			struct hdr_cmn* ch = HDR_CMN(pktRx_);
			struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(pktRx_);
			p_info packetinfo;
			if (vh->dest_mac_addr_ != BCAST_ADDR)
			{
				
				Packet::free(pktRx_);
			}

			else // i.e. a packet is correctly received
			{
				

				if(vh->pkt_type_ == BURST)
				{
					node_out_of_competition = 0;
					//update_fut_slot();
					/*
					if(mini_slot_<=8)
					fprintf(Reception_,"At %g: frame %d : slot %d : mini_slot_ %d : je suis vehicle %d, j'ai reçu un burst de vehicule %d de valeur  %d\n", NOW, frame_number, slot_, mini_slot_, index_,vh->src_mac_addr_,vh->busrt_dec_nbr);
					else
					fprintf(Reception_,"At %g: frame %d : slot %d : TX_ slot : je suis vehicle %d, j'ai reçu un burst de vehicule %d de valeur  %d\n", NOW, frame_number, slot_, index_,vh->src_mac_addr_,vh->busrt_dec_nbr);
					*/
				// Send the packet to the uptarget				
				}
				else
				{

					ch->size() -= hdr_len(pktRx_);
					ch->num_forwards() += 1;
					int k;
					bool already_included = 0;
					//printf("vh->src_mac_addr_ =%d\n",vh->src_mac_addr_);
					Neigs_number_[vh->src_mac_addr_]++;
					//fprintf(Reception_,"%d: %d ----> %d \n",frame_number,vh->src_mac_addr_,index_);
					for (int i=0; i<frame_len_; i++)
						if (vh->src_mac_addr_ == neigh_mac_addr_[i])
						{
							already_included = 1;
							k=i;
							break;
						}
				
				//if(vh->src_mac_addr_==38)
				//fprintf(Reception_, "At %g: frame %d slot %d mini slot %d: je suis vehicle %d j'reçu un message de véhicule %d de type %d \n", NOW, frame_number, slot_,mini_slot_, index_, vh->src_mac_addr_,vh->pkt_type_);
						
					if (already_included) // detect collision
						{
							detect_collision(pktRx_);
							//detect_merging_collision(pktRx_);
							neigh_mac_addr_[prev_slot_idx] = vh->src_mac_addr_;
							for(int i = ai;i < bi; i++)
							{
								if((neigh_mac_addr_[i]==-1)&&(vh->one_hop_[i]!=-1))
								neigh_mac_addr_[i]=vh->one_hop_[i];
							}				

							if (k!=prev_slot_idx)
							neigh_mac_addr_[k]=-1;
						}
			
					else
						{
				
							neigh_mac_addr_[prev_slot_idx] = vh->src_mac_addr_;
						    
						    for(int i=ai;i<bi;i++)
							{

							if((neigh_mac_addr_[i]==-1)&&(vh->one_hop_[i]!=-1))
							neigh_mac_addr_[i]=vh->one_hop_[i];
					
							}
						}
				/*if(index_==170)
					{
					for(int i=0;i<frame_len_;i++)
					fprintf(Reception_, "%d ", neigh_mac_addr_[i]);
					fprintf(Reception_, "\n");
					}			
				*/
				
				}
				
				uptarget_->recv(pktRx_, (Handler*) 0);
			}
		}


		macstate_ = MAC_IDLE;
		pktRx_ = 0;



				if (coll_==1)
				{
				//SlotAcc_[index_]=0;
				//if(acc_slot_number_!=NO_PREV_SLOT_ACCESSED)
				//slotStatus_[acc_slot_number_]=0;
				fut_slot_number_ = NO_FUT_SLOT_SCHEDULED;
				t = NOW;
				//neigh_mac_addr_[acc_slot_number_]=-1;
				node_out_of_competition = 1;
				acc_slot_number_ = NO_PREV_SLOT_ACCESSED;
				coll_= 0;
                                //fprintf(TraceFile_,"slot %d: at %g \n", slot_,NOW);

			    //update_fut_slot();
				}	








		/*if((slot_== 0) && (acc_slot_number_==NO_PREV_SLOT_ACCESSED) )
		{
		fut_slot_number_ = NO_FUT_SLOT_SCHEDULED;
		update_fut_slot();
		start_++;
		}*/		
		if((slot_== 1) && (acc_slot_number_==NO_PREV_SLOT_ACCESSED) && (mini_slot_==0) && (start_==0)){
		update_fut_slot();
		start_++;	
		}

		if ((slot_== 0) && (mini_slot_==0) && (acc_slot_number_==NO_PREV_SLOT_ACCESSED) && (start_>0)) {
		fut_slot_number_ = NO_FUT_SLOT_SCHEDULED;
		update_fut_slot();
		}
 			
			if ((acc_slot_number_ == slot_)&&(mini_slot_ > 11))
			{
				//fprintf(Reception_,"At %g: frame %d : slot %d :***********Avant********** %d \n",NOW, frame_number, slot_,index_);
		 		 bool sent = send(slot_);
				 nbr_already_accessing_slot++;
				
				acc_slot_number_ = slot_; 
                                
				//fprintf(Reception_,"At %g: frame %d : slot %d :**********Après************** %d \n", NOW, frame_number, slot_,index_);
			}





            /*if ((mini_slot_ > 3) && (index_== 6)){

                if (nbr_node_in_slot == 0){

                    nbr_slot_free++;
                    //fprintf(TraceFile_,"nbr de  node libre %d : slot %d: at %g \n", nbr_slot_free, slot_,NOW);


                }

                            if(frame_number == 0 && (slot_==99)){
                            fprintf(TraceFile_,"nbr de  node libre %d: slot %d: at %g \n", nbr_slot_free, slot_,NOW);
                            fprintf(AccessCollision1_,"n= %d \n",nbr_slot_free);
                            //tab
                            }
                nbr_node_in_slot=0;

            }*/
			
	
        if((node_out_of_competition==1)&&(acc_slot_number_==NO_PREV_SLOT_ACCESSED)&&(slot_>=ai)&&(slot_<bi)&&(neigh_mac_addr_[slot_]==-1)){

        	if(slot_ == fut_slot_number_){


                    nbr_node_in_slot++;

       		//fprintf(Reception_,"At %g: frame %d : TX_ slot : je suis vehicle %d en competition pour le slot :%d\n", NOW, frame_number, index_, fut_slot_number_);


        //		if(check_node(index_) == false){

        /*if(slot_==fut_slot_number_){*/

        	//fprintf(Reception_,"At %g: frame %d : TX_ slot : je suis vehicle %d en competition pour le slot :%d\n", NOW, frame_number, index_, slot_);
			
        
			if (mini_slot_ <= 11)
			{
               fprintf(Reception_,"At %g: frame %d : je suis vehicle %d en compétition pour le slot: %d\n", NOW, frame_number, index_, fut_slot_number_);

                                        
				if(burst_generation == 1)
					{
						
						burst_value = 0;
						generate_burst(burst_v_seq);
						
						burst_generation = 0;
					}
				
				if (burst_v_seq[mini_slot_] == 1)
				{
						
							//fprintf(Reception_, "At %g: frame %d : je suis vehicle %d j'ai envoyé mon signal burst de valeur %d \n", NOW, frame_number, index_,burst_value);
						send_burst_signal(mini_slot_);

						
				}				
					
			}		
			else 
			{




                                                       Time_Access_slot = NOW - t;
								
					    		fprintf(Time_Access,"%f\n", Time_Access_slot);

								nbr_first_accessing_slot++;

                                acc_slot_number_ = slot_;

                                bool sent = send(slot_);

                                //fprintf(Reception_,"At %g: frame %d: slot %d: TX_ slot: je suis vehicle %d le SLOT est à moi: xi: %d\n", NOW, frame_number, slot_, index_,xi);
								burst_generation = 1;
                                //acc_slot_number_=slot_;
                                

								//fut_slot_number_ = NO_FUT_SLOT_SCHEDULED;


				//if(index_==200)
				//	fprintf(Time_Access,"At: %g Frame: %d Slot: %d Tx_slot_\n", NOW, frame_number, slot_);

				
			}

		  }
		}
	}

	if (mini_slot_ <= 11)
	{
			
		mini_slot_ = (mini_slot_+1) % 13;
		timer_.start(e,Mini_slot_duration_);

	}
	else
	{

                slot_ = (slot_+1) % frame_len_;
		mini_slot_ = (mini_slot_+1) % 13;
        
		if(slot_== 0)
		{
			

			if(index_== 6)
			{
			
			nbr_veh_slot_access = nbr_already_accessing_slot + nbr_first_accessing_slot;
			fprintf(Node_Acces_Slot,"%d\n",nbr_veh_slot_access);
			nbr_already_accessing_slot=0; nbr_first_accessing_slot = 0;
		
			}
            

			frame_number++;

		}

		node_out_of_competition = 1;
		timer_.start(e,(slot_duration_- Mini_slot_duration_*12));
		
		//Empty_black_node_list();		
	}
	
}

void AS-DTMAC::direction_update_handler(Event *e)
{


	
	if (!direction_update_timer_.initialized_)
	{
		node_->getLoc(&Xprev,&Yprev,&Zprev);
		NodePosition_[index_].XPos=Xprev;
		NodePosition_[index_].YPos=Yprev;
		direction_update_timer_.initialized_ = 1;
		direction_update_timer_.start(e,slot_duration_);
	}
	else
	{	
			node_->getLoc(&Xnew,&Ynew,&Znew);
			NodePosition_[index_].XPos=Xnew;
			NodePosition_[index_].YPos=Ynew;
			double dX = Xnew - Xprev;
			dist=abs(dX);
			double dY = Ynew - Yprev;
			if (dX > 0)
			    {	direction_ = LEFT;
				VehicleDirection_[index_]= LEFT;
			    }

			else if (dX < 0) 
			    {    
				direction_ = RIGHT;
				VehicleDirection_[index_]= RIGHT;
			    }
			else if (dX == 0)
			{
				if (dY > 0)
				{	
				direction_ = LEFT;
				VehicleDirection_[index_]= LEFT;
			        }
				else if (dY < 0)
				{    
				direction_ = RIGHT;
				VehicleDirection_[index_]= RIGHT;
			        }
			}
			Xprev = Xnew;
			Yprev = Ynew;

		// The junction coordination
			if (direction_ == RIGHT)
			{
			      Xintersec=4000;
	                      Yintersec=15;
	                      Zintersec=0;
			}
			else
			{
			      Xintersec=0;
	                      Yintersec=5;
	                      Zintersec=0;
			}
		

		direction_update_timer_.start(e,8);
	}
	
}

void AS-DTMAC::getSpeed(){

	getmovspeed(&speed);
}

int AS-DTMAC::nbr_node_send(){
	int nbns=0;

	for(int i=0; i<number_nodes_; i++) 
		{
			if (NodeSend_[i]==1)
			  nbns++;
		}

	return nbns;
}


 void AS-DTMAC::generate_burst(int *tab)
{
		
	for(int i = 0; i <= BURST_LENGTH-1; i++)
		{
			tab[i]=rand()%2;
			burst_value+=tab[i]*pow(2,BURST_LENGTH-1-i);
		}

}


void AS-DTMAC::getmovspeed(double * sp)
{
	*sp=node_->speed();
}


void AS-DTMAC::update_fut_slot()
{

//fprintf(Reception_,"At %g: frame %d : TX_ slot : je suis ici\n", NOW, frame_number);


int V[frame_len_];
int Vsize,ran_num;


Vsize=0;
for (int i=ai; i<bi; i++)
	{
	if (neigh_mac_addr_[i]==-1)
        {
		
		V[Vsize]=i;
		Vsize++;
	   }

	}

if(Vsize==0)
{
if(bi!=0)
{

	if ((ai==0)&&(bi==s0_))
		{
            		
		  for (int i=bi; i<s0_+s1_; i++)
			{
			   if (neigh_mac_addr_[i]==-1)
                  {
		              ts=NOW;
           		      fut_slot_number_=i;
				      slotStatus_[i]=1;
					  //fprintf(TraceFile_, "At %g: frame %d : je suis vehicle %d :slot: %d \n", NOW, frame_number, index_,fut_slot_number_);
	   		      }

	                 }



                }

		

		if ((ai==s0_)&&(bi==s0_+s1_))
		{
            		
		  for (int i=bi; i<s0_+s1_+s2_; i++)
			{
			   if (neigh_mac_addr_[i]==-1)
                             {
		               ts=NOW;
           		      fut_slot_number_=i;
				      slotStatus_[i]=1;
					  //fprintf(TraceFile_, "At %g: frame %d : je suis vehicle %d :slot: %d \n", NOW, frame_number, index_,fut_slot_number_);
	   		      }

	                 }



                }


		if ((ai==s0_+s1_)&&(bi==s0_+s1_+s2_))
		{
            		
		  for (int i=0; i<s0_; i++)
			{
			   if (neigh_mac_addr_[i]==-1)
                             {
		               ts=NOW;
           		      fut_slot_number_=i;
			          slotStatus_[i]=1;
					  //fprintf(TraceFile_, "At %g: frame %d : je suis vehicle %d :slot: %d \n", NOW, frame_number, index_,fut_slot_number_);
	   		      }

	                 }


                }



//ouble xp,yp,zp;
//node_->getLoc(&xp,&yp,&zp);

//fprintf(NOSlotFound_,"%d	%d\n",frame_number, index_);
}
}
else
  {
		    ran_num = rand() % Vsize; //Remplace by the Active Signaling 
//fprintf(Trace_,"At %g: The node %d in area %d: Slot= %d \n",NOW, index_,xi,V[ran_num]);

  /*   if(1-AccessColissionProbability()>=0)
	fprintf(NoAccessCollisionProb_,"%d	%g \n", frame_number,1-AccessColissionProbability());
	else
	fprintf(NoAccessCollisionProb_,"%d	%g \n", frame_number,0.0);*/
	
	       ts=NOW;
           fut_slot_number_=V[ran_num];
	       slotStatus_[fut_slot_number_]=1;



		   //fprintf(TraceFile_, "At %g: frame %d : je suis vehicle %d :slot: %d \n", NOW, frame_number, index_,fut_slot_number_);
   }
}


int AS-DTMAC::detectTwoAccess(int nd) {

	for(int i=0;i<number_nodes_;i++)
		{
		  if((slotNum_[nd]==slotNum_[i])&&(i!=nd)&&(Euclidistance(nd,i)<2*TRANSMISSION_RANGE))
		   return 1;
		}
	return 0;

}

void AS-DTMAC::detect_collision(Packet* p) {
	struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
	
		if(acc_slot_number_ != NO_PREV_SLOT_ACCESSED)
		{
			if((index_ != vh->one_hop_[acc_slot_number_]))
			{
				//if(detectTwoAccess(index_)==1)
				coll_ = 1;
				succ_acc_ = false;
				neigh_mac_addr_[acc_slot_number_]=-1;
		//fprintf(Trace_,"Node %d has detected an actual collision on its time slot number %d : Node= %d \n", index_, acc_slot_number_,vh->src_mac_addr_);

		//for (int i=0; i<number_nodes_; i++)
		//fprintf(Trace_,"%g %d: x=%g  y=%g   %d\n",NOW, i, NodePosition_[i].XPos,NodePosition_[i].YPos, slotNum_[i]);

		//fprintf(AccessCollision_,"An actual access collision detected: NOW= %g   index= %d   acc_slot_number_= %d  vh->src_mac_addr_= %d  one_hop_[acc_slot_number_]= %d  xi= %d\n",NOW,index_,acc_slot_number_,vh->src_mac_addr_,vh->one_hop_[acc_slot_number_],xi);


	if((NOW-ts)<=3*frame_len_*slot_duration_)
	{
	
        fprintf(AccessCollision1_,"An actual Access collision detected: NOW= %g Frame= %d  index= %d   acc_slot_number_= %d  vh->src_mac_addr_= %d  one_hop_[acc_slot_number_]= %d  xi= %d \n",NOW,frame_number,index_,acc_slot_number_,vh->src_mac_addr_,vh->one_hop_[acc_slot_number_],xi);
	//tabSlotState[frame_number].slots[acc_slot_number_]=1;
	}
	else
	{
	fprintf(AccessCollision1_,"An actual Merging collision detected: NOW= %g  Frame= %d index= %d   acc_slot_number_= %d  vh->src_mac_addr_= %d  one_hop_[acc_slot_number_]= %d  xi= %d\n",NOW,frame_number,index_,acc_slot_number_,vh->src_mac_addr_,vh->one_hop_[acc_slot_number_],xi);
	//tabSlotState[frame_number].slots[acc_slot_number_]=2;
	}
	
			}
		
		}		

}

void AS-DTMAC::detect_merging_collision(Packet* p) {
	struct hdr_AS-DTMAC* vh = HDR_AS-DTMAC(p);
	
		if(acc_slot_number_ != NO_PREV_SLOT_ACCESSED)
		{
			if((index_ != vh->one_hop_[acc_slot_number_])) //&& ((acc_slot_number_<ai)||(acc_slot_number_>=bi))
			{
				coll_ = 1;
				succ_acc_ = false;
				neigh_mac_addr_[acc_slot_number_]=-1;
		//fprintf(Trace_,"Node %d has detected a merging collision on its time slot number %d : Node= %d \n", index_, acc_slot_number_,vh->src_mac_addr_);

			//fprintf(MergingCollision_,"NOW= %g   index= %d   acc_slot_number_= %d  vh->src_mac_addr_= %d  one_hop_[acc_slot_number_]= %d  xi= %d\n",NOW,index_,acc_slot_number_,vh->src_mac_addr_,vh->one_hop_[acc_slot_number_],xi);
			}
		
		}		

}

void AS-DTMAC::update_area(Event* e){

	if (direction_ == FIXED)
		{
			area_timer_.start(e,slot_duration_);
			
		}


	else
	{

	double distance;

	node_->getLoc(&Xnew1,&Ynew1,&Znew1);


	xi= Xnew1 / TRANSMISSION_RANGE;
	NodeArea_[index_]=xi;

	if (xi%3==0)
			{
			 ai=0;
			 bi=s0_;
			}
		else
		  {
	      		 if (xi%3==1)
			 {
			    ai=s0_;
			    bi=s0_+s1_;
			 }
			 else
			 {
			    ai=s0_+s1_;
			    bi=s0_+s1_+s2_;
			 }
			 
	          }

	 if (xi%3!=set_)
		{ 
		//fprintf(TraceFile_,"At %g: The node %d change its area \n",NOW, index_);
		//fprintf(Trace_,"At %g: The node %d change its slots set from  %d to %d \n",NOW, index_,set_,xi%3);
		fut_slot_number_ = NO_FUT_SLOT_SCHEDULED;
		//SlotAcc_[index_]=0;
		

		neigh_mac_addr_[acc_slot_number_]=-1;
		node_out_of_competition=1;
		t = NOW; 
		acc_slot_number_ = NO_PREV_SLOT_ACCESSED;	

		//if(acc_slot_number_!=NO_PREV_SLOT_ACCESSED)
		//slotStatus_[acc_slot_number_]=0;

		//slotNum_[index_] = NO_PREV_SLOT_ACCESSED;
		set_=xi%3;
		update_fut_slot();
		
		}


	/*
	}
	else
	{
		if (set_==0)
			{
			 ai=s0_+s1_+s2_;
			 bi=2*s0_+s1_+s2_;
			}
		else
		  {
	      		 if (set_==1)
			 {
			    ai=2*s0_+s1_+s2_;
			    bi=2*s0_+2*s1_+s2_;
			 }
			 else
			 {
			    ai=2*s0_+2*s1_+s2_;
			    bi=2*s0_+2*s1_+2*s2_;
			 }
			 
	          }


	}*/



	//fprintf(Trace_,"At %g: The node %d (%g, %g) is in the area X %d and moving in direction %d set=%d ai=%d and bi=%d distance  %g and acc_slot_number_=%d\n",NOW, index_,NodePosition_[index_].XPos,NodePosition_[index_].YPos,xi,direction_,set_,ai,bi,distance, acc_slot_number_);

	area_timer_.start(e,slot_duration_);
	}
}

int AS-DTMAC::numberOfNeighb(int j){
	int neighn=0;
	for(int i=0;i<number_nodes_;i++)
		{
		  if((Euclidistance(i,j)<=TRANSMISSION_RANGE)&&(i!=j)&&(NodePosition_[i].XPos>=2000)&&(NodePosition_[i].XPos<=4000))
			neighn++;

		}

	return neighn;
}

// The euclidienne distance between two vehicles
double AS-DTMAC::Euclidistance(int i, int j){
	  double EuDist=0;
	 
		EuDist = sqrt((NodePosition_[i].XPos-NodePosition_[j].XPos)*(NodePosition_[i].XPos-NodePosition_[j].XPos) + (NodePosition_[i].YPos-NodePosition_[j].YPos)*(NodePosition_[i].YPos-NodePosition_[j].YPos)); 
	    //printf(", %s\n", );

	  	return EuDist;

}

/*void AS-DTMAC::update_direction(Event *e) {
	if (!dir_timer_.initialized_)
	{
		node_->getLoc(&Xprev,&Yprev,&Zprev);
		NodePosition_[index_].XPos=Xprev;
		NodePosition_[index_].YPos=Yprev;
		dir_timer_.initialized_ = 1;
		dir_timer_.start(e,slot_duration_- eps);
	}
	else
	{	
			node_->getLoc(&Xnew,&Ynew,&Znew);
			NodePosition_[index_].XPos=Xnew;
			NodePosition_[index_].YPos=Ynew;
			double dX = Xnew - Xprev;
			dist=abs(dX);
			double dY = Ynew - Yprev;
			if (dX > 0)
			    {	direction_ = LEFT;
				VehicleDirection_[index_]= LEFT;
			    }

			else if (dX < 0) 
			    {    
				direction_ = RIGHT;
				VehicleDirection_[index_]= RIGHT;
			    }
			else if (dX == 0)
			{
				if (dY > 0)
				{	
				direction_ = LEFT;
				VehicleDirection_[index_]= LEFT;
			        }
				else if (dY < 0)
				{    
				direction_ = RIGHT;
				VehicleDirection_[index_]= RIGHT;
			        }
			}
			Xprev = Xnew;
			Yprev = Ynew;

		// The junction coordination
			if (direction_ == RIGHT)
			{
			      Xintersec=4000;
	                      Yintersec=15;
	                      Zintersec=0;
			}
			else
			{
			      Xintersec=0;
	                      Yintersec=5;
	                      Zintersec=0;
			}
		

		dir_timer_.start(e,DIRECTION_UPDATE_INTERVAL);
	}

}*/
	
	
		
		
		

