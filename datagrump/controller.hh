#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#define SCHEME (3)
#define INITIAL_CWND (15.0)
#define AI (1.0)
#define MD (0.5)
#define DELAY_THRES (100)
#define EWMA_ALPHA (0.7)
#define TIMEOUT (100)
#define TICK_LEN (20.0)
#define NUM_RATIOS ((int) 50)
#define TARGET_DELAY (100)
#define DELAY_TICKS ((int) (TARGET_DELAY / TICK_LEN))
#define MIN_CWND (10.0)
#define PERC (0.10)

#include <cstdint>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  uint64_t previous_ack_; /* Timestamp of receipt of most recent ack */
  double cwnd_; /* Window size */ 
  unsigned int scheme_; /* Flag specifying congestion control scheme */
  double rtt_; /* EWMA of RTT's */
  int last_tick_; /* Start timestamp of the previous tick */
  double lambda_; /* EWMA estimate of rate */
  double ratio_; /* Estimate of cumulative rate over next few ticks divided by current rate */
  uint64_t prev_firsts_ [NUM_RATIOS + DELAY_TICKS]; /* Circular array of sequence numbers of first acks in each tick */
  double prev_lambdas_ [NUM_RATIOS +  DELAY_TICKS]; /* Circular array of EWMA lambdas for previous ticks */
  int cursor_; /* Index into above circular arrays */

  /* Add member variables here */

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );
  
  /* Update estimate on ratio_ */
  void update_ratio( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );
};

#endif
